//! Corner support is an upstream spline-only policy, including in mosaics.

use std::collections::BTreeMap;

use resvg::{tiny_skia, usvg};
use vtracer::ir::PathCmd;
use vtracer::{Clustering, ColorImage, Config, FitMode, Hierarchical, PointF64};

fn features() -> ColorImage {
    let (width, height) = (48, 32);
    let mut pixels = Vec::new();
    for y in 0..height {
        for x in 0..width {
            let ring = (2..18).contains(&x) && (2..26).contains(&y)
                && !((6..14).contains(&x) && (7..20).contains(&y));
            let neighbour = (19..28).contains(&x) && (2..26).contains(&y);
            let small = (35..37).contains(&x) && (6..8).contains(&y);
            let thin = x == 34 && (14..27).contains(&y);
            let c = if ring || neighbour || small || thin { 0 } else { 255 };
            pixels.extend_from_slice(&[c, c, c, 255]);
        }
    }
    ColorImage { pixels, width, height }
}

fn config(hierarchical: Hierarchical) -> Config {
    Config {
        clustering: Clustering::Binary,
        hierarchical,
        filter_speckle: 0,
        optimize: 0,
        ..Config::default()
    }
}

#[test]
fn omitted_and_zero_are_exact_and_other_modes_ignore_support() {
    let image = features();
    for hierarchical in [Hierarchical::Stacked, Hierarchical::Cutout] {
        for simplify in [None, Some(0.0), Some(2.0)] {
            for mode in [FitMode::Spline, FitMode::Pixel, FitMode::Polygon] {
                let base = Config { mode, simplify, ..config(hierarchical) };
                let expected = base.build().unwrap().to_svg(&image).unwrap();
                let zero = Config { corner_support: 0.0, ..base.clone() };
                assert_eq!(expected, zero.build().unwrap().to_svg(&image).unwrap());
                if mode != FitMode::Spline {
                    let on = Config { corner_support: 20.0, ..base };
                    assert_eq!(expected, on.build().unwrap().to_svg(&image).unwrap());
                }
            }
        }
    }
}

#[test]
fn invalid_support_is_rejected() {
    for corner_support in [-1.0, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        assert!(Config { corner_support, ..Config::default() }.build().is_err());
    }
    for corner_support in [0.0, 0.5, f64::MAX] {
        assert!(Config { corner_support, ..Config::default() }.build().is_ok());
    }
}

fn component_count(mask: &[bool], width: usize, foreground: bool, interior_only: bool) -> usize {
    let height = mask.len() / width;
    let mut seen = vec![false; mask.len()];
    let mut count = 0;
    for start in 0..mask.len() {
        if seen[start] || mask[start] != foreground {
            continue;
        }
        seen[start] = true;
        let mut stack = vec![start];
        let mut border = false;
        while let Some(i) = stack.pop() {
            let (x, y) = (i % width, i / width);
            border |= x == 0 || y == 0 || x + 1 == width || y + 1 == height;
            for (dx, dy) in [(-1, 0), (1, 0), (0, -1), (0, 1)] {
                let (nx, ny) = (x as isize + dx, y as isize + dy);
                if nx < 0 || ny < 0 || nx >= width as isize || ny >= height as isize {
                    continue;
                }
                let j = ny as usize * width + nx as usize;
                if !seen[j] && mask[j] == foreground {
                    seen[j] = true;
                    stack.push(j);
                }
            }
        }
        count += usize::from(!interior_only || !border);
    }
    count
}

#[test]
fn holes_one_pixel_gap_thin_feature_and_small_ring_keep_topology() {
    let image = features();
    for hierarchical in [Hierarchical::Stacked, Hierarchical::Cutout] {
        for simplify in [None, Some(0.0), Some(2.0)] {
            let cfg = Config {
                corner_support: 100.0,
                simplify,
                ..config(hierarchical)
            };
            let svg = cfg.build().unwrap().to_svg(&image).unwrap();
            let tree = usvg::Tree::from_str(&svg, &usvg::Options::default()).unwrap();
            let mut pixmap = tiny_skia::Pixmap::new(48, 32).unwrap();
            resvg::render(&tree, tiny_skia::Transform::identity(), &mut pixmap.as_mut());
            let mask: Vec<bool> = pixmap.data().chunks_exact(4)
                .map(|rgba| rgba[3] > 128 && rgba[0] < 128).collect();
            assert_eq!(component_count(&mask, 48, true, false), 4, "{hierarchical:?}, {simplify:?}");
            assert_eq!(component_count(&mask, 48, false, true), 1, "hole must stay open");
            for y in 2..26 {
                assert!(!mask[y * 48 + 18], "one-pixel gap stays open at {y}");
            }
            assert!(mask[6 * 48 + 35], "small contour survives");
            assert!(mask[20 * 48 + 34], "one-pixel-wide feature survives");
        }
    }
}

fn bits(c: [PointF64; 4]) -> [u64; 8] {
    std::array::from_fn(|i| if i % 2 == 0 { c[i / 2].x.to_bits() } else { c[i / 2].y.to_bits() })
}

#[test]
fn mosaic_curved_boundary_is_bit_for_bit_shared_after_simplification() {
    let (width, height) = (64, 64);
    let mut pixels = Vec::new();
    for y in 0..height {
        let boundary = (32.0 + (y as f64 / 8.0).sin() * 8.0) as usize;
        for x in 0..width {
            let c = if x < boundary { [220, 30, 30, 255] } else { [30, 30, 220, 255] };
            pixels.extend_from_slice(&c);
        }
    }
    let image = ColorImage { pixels, width, height };
    for simplify in [None, Some(2.0)] {
        let cfg = Config {
            hierarchical: Hierarchical::Cutout,
            corner_support: 8.0,
            simplify,
            filter_speckle: 0,
            layer_difference: 0,
            color_precision: 8,
            optimize: 0,
            ..Config::default()
        };
        let doc = cfg.build().unwrap().run(&image).unwrap();
        assert_eq!(doc.shapes.len(), 2);
        let mut shared = BTreeMap::<[u64; 8], usize>::new();
        for shape in &doc.shapes {
            for sub in &shape.path.subpaths {
                let mut start = PointF64 { x: 0.0, y: 0.0 };
                for cmd in &sub.commands {
                    match *cmd {
                        PathCmd::MoveTo(p) | PathCmd::LineTo(p) => start = p,
                        PathCmd::CubicTo(a, b, end) => {
                            if [start, a, b, end].iter().all(|p| p.x > 0.0 && p.x < width as f64) {
                                let fwd = bits([start, a, b, end]);
                                let rev = bits([end, b, a, start]);
                                *shared.entry(fwd.min(rev)).or_default() += 1;
                            }
                            start = end;
                        }
                        PathCmd::Close => {}
                    }
                }
            }
        }
        assert!(!shared.is_empty(), "exercise the curved shared boundary");
        assert!(shared.values().all(|&count| count == 2), "each cubic appears in both faces");
    }
}
