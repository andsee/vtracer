use resvg::{tiny_skia, usvg};
use vtracer::ir::PathCmd;
use vtracer::{Clustering, ColorImage, Config, FitMode, Hierarchical, VectorDoc};

fn binary_image(width: usize, height: usize, black: impl Fn(usize, usize) -> bool) -> ColorImage {
    let mut pixels = Vec::with_capacity(width * height * 4);
    for y in 0..height {
        for x in 0..width {
            let value = if black(x, y) { 0 } else { 255 };
            pixels.extend_from_slice(&[value, value, value, 255]);
        }
    }
    ColorImage {
        pixels,
        width,
        height,
    }
}

fn snowman_bow() -> ColorImage {
    let text = include_str!("fixtures/snowman-bow.pbm");
    let tokens: Vec<_> = text
        .lines()
        .filter_map(|line| line.split('#').next())
        .flat_map(str::split_whitespace)
        .collect();
    assert_eq!(tokens[0], "P1");
    let (width, height) = (tokens[1].parse().unwrap(), tokens[2].parse().unwrap());
    let pixels: Vec<_> = tokens[3..]
        .iter()
        .map(|&bit| {
            let value = if bit == "1" { 0 } else { 255 };
            [value, value, value, 255]
        })
        .flatten()
        .collect();
    ColorImage {
        pixels,
        width,
        height,
    }
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

fn rendered_mask(svg: &str, width: u32, height: u32) -> Vec<bool> {
    let tree = usvg::Tree::from_str(svg, &usvg::Options::default()).unwrap();
    let mut pixmap = tiny_skia::Pixmap::new(width, height).unwrap();
    resvg::render(&tree, tiny_skia::Transform::identity(), &mut pixmap.as_mut());
    pixmap
        .data()
        .chunks_exact(4)
        .map(|rgba| rgba[3] > 128 && rgba[0] < 128)
        .collect()
}

fn boundary(mask: &[bool], width: usize, height: usize) -> Vec<(i32, i32)> {
    let mut result = Vec::new();
    for y in 0..height {
        for x in 0..width {
            let i = y * width + x;
            if mask[i]
                && ((x == 0 || !mask[i - 1])
                    || (x + 1 == width || !mask[i + 1])
                    || (y == 0 || !mask[i - width])
                    || (y + 1 == height || !mask[i + width]))
            {
                result.push((x as i32, y as i32));
            }
        }
    }
    result
}

fn hausdorff(a: &[(i32, i32)], b: &[(i32, i32)]) -> f64 {
    let directed = |from: &[(i32, i32)], to: &[(i32, i32)]| {
        from.iter()
            .map(|&(x, y)| {
                to.iter()
                    .map(|&(u, v)| ((x - u).pow(2) + (y - v).pow(2)) as f64)
                    .fold(f64::INFINITY, f64::min)
                    .sqrt()
            })
            .fold(0.0, f64::max)
    };
    directed(a, b).max(directed(b, a))
}

fn cubic_count(doc: &VectorDoc) -> usize {
    doc.shapes
        .iter()
        .flat_map(|shape| &shape.path.subpaths)
        .flat_map(|sub| &sub.commands)
        .filter(|command| matches!(command, PathCmd::CubicTo(_, _, _)))
        .count()
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
fn zero_support_is_legacy_and_other_fit_modes_ignore_it() {
    let image = snowman_bow();
    for hierarchical in [Hierarchical::Stacked, Hierarchical::Cutout] {
        for mode in [FitMode::Spline, FitMode::Pixel, FitMode::Polygon] {
            let base = Config {
                mode,
                ..config(hierarchical)
            };
            let expected = base.build().unwrap().to_svg(&image).unwrap();
            let zero = Config {
                corner_support: 0.0,
                ..base.clone()
            };
            assert_eq!(expected, zero.build().unwrap().to_svg(&image).unwrap());
            if mode != FitMode::Spline {
                let enabled = Config {
                    corner_support: 1.0,
                    ..base
                };
                assert_eq!(expected, enabled.build().unwrap().to_svg(&image).unwrap());
            }
        }
    }
}

#[test]
fn snowman_bow_wobble_is_removed_by_the_full_raster_to_svg_pipeline() {
    let image = snowman_bow();
    let source: Vec<bool> = image
        .pixels
        .chunks_exact(4)
        .map(|rgba| rgba[0] < 128)
        .collect();
    let (width, height) = (image.width, image.height);
    let source_boundary = boundary(&source, image.width, image.height);
    let base = config(Hierarchical::Stacked);
    let legacy = base.build().unwrap().run(&image).unwrap();
    let supported = Config {
        corner_support: 4.0,
        ..base
    };
    let fitted = supported.build().unwrap().run(&image).unwrap();
    let legacy_svg = config(Hierarchical::Stacked)
        .build()
        .unwrap()
        .to_svg(&image)
        .unwrap();
    let supported_svg = Config {
        corner_support: 4.0,
        ..config(Hierarchical::Stacked)
    }
    .build()
    .unwrap()
    .to_svg(&image)
    .unwrap();
    let legacy_render = rendered_mask(&legacy_svg, width as u32, height as u32);
    let fitted_render = rendered_mask(&supported_svg, width as u32, height as u32);
    let legacy_deviation = hausdorff(
        &source_boundary,
        &boundary(&legacy_render, width, height),
    );
    let supported_deviation = hausdorff(
        &source_boundary,
        &boundary(&fitted_render, width, height),
    );
    let geometry_before = cubic_count(&legacy);
    let geometry_after = cubic_count(&fitted);
    eprintln!(
        "Stacked: cubics {geometry_before} -> {geometry_after}, \
         source-boundary Hausdorff {legacy_deviation:.3}px -> {supported_deviation:.3}px"
    );
    assert_ne!(legacy_svg, supported_svg, "corner rejection must be active");
    assert!(geometry_after < geometry_before);
    assert!(supported_deviation <= 2.5);
    assert!(
        source[4 * width + 16]
            && legacy_render[4 * width + 16]
            && !fitted_render[4 * width + 16],
        "the raised hat-edge wobble is removed by size-aware protection"
    );
}

#[test]
fn larger_stacked_and_cutout_fixtures_keep_holes_gaps_and_small_rings() {
    let image = binary_image(64, 48, |x, y| {
        let body = (4..30).contains(&x) && (4..38).contains(&y);
        let short_bump = (14..18).contains(&x) && (2..4).contains(&y);
        let ring = (body || short_bump)
            && !((11..23).contains(&x) && (12..29).contains(&y));
        let neighbor = (31..45).contains(&x) && (4..38).contains(&y);
        let small_ring = (50..55).contains(&x)
            && (8..13).contains(&y)
            && !((51..54).contains(&x) && (9..12).contains(&y));
        let narrow = x == 47 && (20..40).contains(&y);
        ring || neighbor || small_ring || narrow
    });
    for hierarchical in [Hierarchical::Stacked, Hierarchical::Cutout] {
        let base = config(hierarchical);
        let legacy = base.build().unwrap().run(&image).unwrap();
        let cfg = Config {
            corner_support: 8.0,
            ..base.clone()
        };
        let fitted = cfg.build().unwrap().run(&image).unwrap();
        let legacy_svg = base.build().unwrap().to_svg(&image).unwrap();
        let svg = cfg.build().unwrap().to_svg(&image).unwrap();
        let rendered = rendered_mask(&svg, 64, 48);
        assert_ne!(legacy_svg, svg, "{hierarchical:?} corner rejection is active");
        assert!(
            cubic_count(&fitted) < cubic_count(&legacy),
            "{hierarchical:?} active corner rejection reduces fitted geometry"
        );
        assert_eq!(component_count(&rendered, 64, true, false), 4);
        assert_eq!(component_count(&rendered, 64, false, true), 2);
        for y in 4..38 {
            assert!(!rendered[y * 64 + 30], "one-pixel gap stays open at {y}");
        }
        assert!(rendered[10 * 64 + 50], "small ring remains present");
        assert!(rendered[30 * 64 + 47], "one-pixel-wide feature remains present");
    }
}

#[test]
fn invalid_corner_support_is_rejected() {
    for corner_support in [-1.0, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        assert!(Config {
            corner_support,
            ..Config::default()
        }
        .build()
        .is_err());
    }
}
