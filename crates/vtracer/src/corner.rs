//! Size-aware corner decisions made once on the pre-subdivision contour.

use visioncortex::{PathF64, PointF64, SubdivideSmooth};

use crate::fitter::{FitParams, FittedGeom};

fn turn(a: PointF64, b: PointF64, c: PointF64) -> f64 {
    let (u, v) = (b - a, c - b);
    (u.x * v.y - u.y * v.x).atan2(u.x * v.x + u.y * v.y)
}

/// Canonicalize straight runs before measuring or subdividing. Sampling a
/// straight edge more densely must not change either the policy or the fit.
fn canonical(path: PathF64, closed: bool) -> PathF64 {
    let mut points = path.path;
    points.dedup();
    if closed && points.len() > 1 && points.first() == points.last() {
        points.pop();
    }
    let n = points.len();
    let mut out = Vec::new();
    for i in 0..n {
        if !closed && (i == 0 || i + 1 == n) {
            out.push(points[i]);
        } else if n < 3 || turn(points[(i + n - 1) % n], points[i], points[(i + 1) % n]).abs() > 1e-12 {
            out.push(points[i]);
        }
    }
    if closed && !out.is_empty() {
        out.push(out[0]);
    }
    PathF64::from_points(out)
}

fn probe(
    points: &[PointF64],
    turns: &[f64],
    i: usize,
    forward: bool,
    support: f64,
    threshold: f64,
    closed: bool,
) -> PointF64 {
    let n = points.len();
    let mut j = i;
    let mut remaining = support;
    // At most one circuit, even for an enormous support setting.
    for _ in 0..n - 1 {
        if !closed && ((!forward && j == 0) || (forward && j + 1 == n)) {
            return points[j];
        }
        let next = if forward { (j + 1) % n } else { (j + n - 1) % n };
        let delta = points[next] - points[j];
        let length = delta.norm();
        if length >= remaining {
            return points[j] + delta * (remaining / length);
        }
        remaining -= length;
        j = next;
        // Do not look through another deliberate turn in the same direction.
        // In particular a tiny square is not a four-corner raster wobble.
        if turns[j].abs() >= threshold && turns[j] * turns[i] > 0.0 {
            return points[j];
        }
    }
    points[j]
}

fn classify(path: &PathF64, params: FitParams, closed: bool) -> (Vec<bool>, Vec<bool>) {
    let points = &path.path[..path.len() - usize::from(closed)];
    let n = points.len();
    let local = SubdivideSmooth::find_corners(path, params.corner_threshold, closed);
    if closed {
        let perimeter: f64 = path.path.windows(2).map(|w| (w[1] - w[0]).norm()).sum();
        // A whole small contour is not evidence of noise. Do not wrap either
        // support window through the other one to manufacture cancellation.
        if params.corner_support >= perimeter / 2.0 {
            return (local, vec![false; n]);
        }
    }
    let turns: Vec<f64> = (0..n)
        .map(|i| {
            if !closed && (i == 0 || i + 1 == n) {
                0.0
            } else {
                turn(points[(i + n - 1) % n], points[i], points[(i + 1) % n])
            }
        })
        .collect();
    let mut protected = local.clone();
    for i in 0..n {
        if !local[i] || (!closed && (i == 0 || i + 1 == n)) {
            continue;
        }
        // A cusp has no stable turn sign at atan2's ±π branch cut. Preserve
        // these narrow tips rather than mistaking a sign flip for cancellation.
        if turns[i].abs() >= std::f64::consts::PI - 1e-12 {
            continue;
        }
        let before = probe(points, &turns, i, false, params.corner_support, params.corner_threshold, closed);
        let after = probe(points, &turns, i, true, params.corner_support, params.corner_threshold, closed);
        let supported = turn(before, points[i], after);
        protected[i] = supported * turns[i] > 0.0
            && supported.abs() + 1e-12 >= params.corner_threshold;
    }
    let rejected = local.iter().zip(&protected).map(|(a, b)| *a && !*b).collect();
    (protected, rejected)
}

/// Shared opt-in spline pipeline. Rejected vertices stay in the fit samples,
/// but are neither subdivision corners nor pinned splice endpoints.
pub(crate) fn fit_supported(path: PathF64, params: FitParams, closed: bool) -> FittedGeom {
    let mut path = canonical(path, closed);
    if path.len() < if closed { 4 } else { 3 } {
        return FittedGeom::Polyline(path.path);
    }
    let (mut protected, rejected) = classify(&path, params, closed);
    let corners: Vec<PointF64> = protected.iter().enumerate()
        .filter_map(|(i, &keep)| keep.then_some(path.path[i])).collect();
    let rejected: Vec<PointF64> = rejected.iter().enumerate()
        .filter_map(|(i, &drop)| drop.then_some(path.path[i])).collect();
    for _ in 0..params.max_iterations {
        let (next, flags, done) = SubdivideSmooth::subdivide_keep_corners(
            &path, &protected, 8.0, params.length_threshold, closed,
        );
        path = next;
        protected = flags;
        if done {
            break;
        }
    }
    if closed {
        // Ring fitting needs two seams. If every available sample was rejected,
        // place seams inside edges instead of pinning those corners again.
        for _ in 0..2 {
            let available = path.path[..path.len() - 1].iter()
                .filter(|p| !rejected.contains(p)).count();
            if available >= 2 {
                break;
            }
            let i = (path.len() - 1) / 2;
            let midpoint = path.path[i] + (path.path[i + 1] - path.path[i]) * 0.5;
            path.path.insert(i + 1, midpoint);
            protected.insert(i + 1, false);
        }
    }
    let mut splice = SubdivideSmooth::find_splice_points(&path, params.splice_threshold, closed);
    for (i, cut) in splice.iter_mut().enumerate() {
        if rejected.contains(&path.path[i]) {
            *cut = false;
        }
        *cut |= protected[i];
    }
    let n = splice.len();
    let mut cuts: Vec<usize> = splice.iter().enumerate()
        .filter_map(|(i, &cut)| cut.then_some(i)).collect();
    if closed {
        if cuts.is_empty() {
            // Avoid making a rejected vertex the mandatory ring seam.
            cuts.push((0..n).find(|&i| !rejected.contains(&path.path[i])).unwrap_or(0));
        }
        if cuts.len() == 1 {
            let opposite = (cuts[0] + n / 2) % n;
            let other = (0..n).map(|offset| (opposite + offset) % n)
                .find(|&i| i != cuts[0] && !rejected.contains(&path.path[i]))
                .expect("ring has two non-rejected seam samples");
            cuts.push(other);
            cuts.sort_unstable();
        }
        cuts.push(cuts[0] + n);
    }
    let mut chain = Vec::new();
    for pair in cuts.windows(2) {
        let samples: Vec<PointF64> = (pair[0]..=pair[1]).map(|i| path.path[i % n]).collect();
        chain.extend(SubdivideSmooth::fit_points_with_beziers(&samples, 10.0));
    }
    if !closed {
        if let Some(first) = chain.first_mut() {
            first[0] = path.path[0];
        }
        if let Some(last) = chain.last_mut() {
            last[3] = *path.path.last().unwrap();
        }
    }
    FittedGeom::SupportedBeziers { chain, corners }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::simplify::{CurvePass, SimplifyCurves};

    fn p(x: f64, y: f64) -> PointF64 {
        PointF64 { x, y }
    }

    fn params() -> FitParams {
        FitParams { corner_support: 8.0, ..FitParams::default() }
    }

    fn wobble() -> PathF64 {
        // A 64-degree excursion, immediately cancelled, on a long edge.
        PathF64::from_points(vec![
            p(-30.0, 0.0), p(0.0, 0.0), p(0.5, 1.0), p(1.0, 0.0), p(30.0, 0.0),
        ])
    }

    fn parts(geom: FittedGeom) -> (Vec<[PointF64; 4]>, Vec<PointF64>) {
        match geom {
            FittedGeom::SupportedBeziers { chain, corners } => (chain, corners),
            _ => panic!("expected supported spline"),
        }
    }

    #[test]
    fn wobble_is_not_protected_and_fit_spans_it() {
        let path = wobble();
        let local = SubdivideSmooth::find_corners(&path, params().corner_threshold, false);
        assert_eq!(local, vec![true; 5]);
        let (protected, _) = classify(&path, params(), false);
        assert_eq!(protected, vec![true, false, false, false, true]);
        let (chain, corners) = parts(fit_supported(path, params(), false));
        assert_eq!(corners, vec![p(-30.0, 0.0), p(30.0, 0.0)]);
        for q in [p(0.0, 0.0), p(0.5, 1.0), p(1.0, 0.0)] {
            assert!(!chain.iter().any(|c| c[0] == q || c[3] == q),
                "rejected corner {q:?} must not pin a fitted slice");
        }
        assert_eq!(chain[0][0], p(-30.0, 0.0));
        assert_eq!(chain.last().unwrap()[3], p(30.0, 0.0));
    }

    #[test]
    fn square_v_and_short_deliberate_corners_survive() {
        for side in [1.0, 40.0] {
            let path = PathF64::from_points(vec![
                p(0.0, 0.0), p(side, 0.0), p(side, side), p(0.0, side), p(0.0, 0.0),
            ]);
            assert_eq!(classify(&path, params(), true).0, vec![true; 4]);
            let (chain, corners) = parts(fit_supported(path, params(), true));
            assert_eq!(corners.len(), 4);
            assert_eq!(chain[0][0], chain.last().unwrap()[3]);
            for corner in corners {
                assert!(chain.iter().any(|c| c[0] == corner));
            }
        }
        for size in [1.0, 30.0] {
            let path = PathF64::from_points(vec![p(-size, size), p(0.0, 0.0), p(size, size)]);
            assert_eq!(classify(&path, params(), false).0, vec![true; 3]);
        }
        // A short chamfer between long legs turns in the same direction twice.
        let path = PathF64::from_points(vec![
            p(-30.0, 0.0), p(0.0, 0.0), p(1.0, 2.0), p(-15.0, 20.0),
        ]);
        assert!(classify(&path, params(), false).0[1]);
    }

    #[test]
    fn support_does_not_consume_a_whole_small_contour() {
        let path = PathF64::from_points(vec![
            p(0.0, 0.0), p(3.0, 0.0), p(1.0, 1.0), p(3.0, 3.0),
            p(0.0, 3.0), p(0.0, 0.0),
        ]);
        let local = SubdivideSmooth::find_corners(&path, params().corner_threshold, true);
        assert_eq!(classify(&path, params(), true).0, local);
        let (chain, corners) = parts(fit_supported(path, params(), true));
        assert!(!chain.is_empty());
        for point in corners {
            assert!(chain.iter().any(|c| c[0] == point));
        }
    }

    #[test]
    fn narrow_cusps_survive_either_turn_sign() {
        for dy in [-1e-15, 0.0, 1e-15] {
            let path = PathF64::from_points(vec![
                p(-20.0, 0.0), p(0.0, 0.0), p(-0.25, dy), p(-0.25, 30.0),
            ]);
            assert!(classify(&path, params(), false).0[1]);
            let (chain, corners) = parts(fit_supported(path, params(), false));
            assert!(corners.contains(&p(0.0, 0.0)));
            assert!(chain.iter().any(|c| c[0] == p(0.0, 0.0)));
        }
    }

    #[test]
    fn ring_simplifier_does_not_pin_a_rejected_seam() {
        let mut points = wobble().path;
        points.extend([p(30.0, 30.0), p(-30.0, 30.0), p(-30.0, 0.0)]);
        let mut chain: Vec<_> = points.windows(2)
            .map(|w| [w[0], w[0], w[1], w[1]]).collect();
        chain.rotate_left(2); // Start at the wobble's sharpest (rejected) turn.
        let corners = vec![p(-30.0, 0.0), p(30.0, 0.0), p(30.0, 30.0), p(-30.0, 30.0)];
        let pass = SimplifyCurves { tolerance: 2.0, corner_threshold: params().corner_threshold };
        let (chain, _) = parts(pass.ring(FittedGeom::SupportedBeziers {
            chain, corners: corners.clone(),
        }));
        assert_eq!(chain[0][0], chain.last().unwrap()[3]);
        assert_eq!(chain.len(), 4);
        for c in chain {
            assert!(corners.contains(&c[0]), "only deliberate corners remain as anchors");
        }
    }

    #[test]
    fn collinear_sampling_and_duplicates_do_not_change_fit() {
        let path = wobble();
        let mut dense = Vec::new();
        for w in path.path.windows(2) {
            dense.push(w[0]);
            dense.push(w[0]);
            dense.push(w[0] + (w[1] - w[0]) * 0.25);
            dense.push(w[0] + (w[1] - w[0]) * 0.5);
        }
        dense.push(*path.path.last().unwrap());
        let a = parts(fit_supported(path, params(), false));
        let b = parts(fit_supported(PathF64::from_points(dense), params(), false));
        assert_eq!(a, b);
    }

    #[test]
    fn ring_support_wraps_through_the_seam() {
        let mut points = wobble().path;
        points.extend([p(30.0, 30.0), p(-30.0, 30.0)]);
        for seam in 0..points.len() {
            let mut ring = points.clone();
            ring.rotate_left(seam);
            ring.push(ring[0]);
            let path = PathF64::from_points(ring);
            let (protected, _) = classify(&path, params(), true);
            let kept: Vec<_> = protected.iter().enumerate()
                .filter_map(|(i, &yes)| yes.then_some(path.path[i])).collect();
            assert_eq!(kept.len(), 4, "only the rectangle corners survive, seam {seam}");
            assert!(!kept.contains(&p(0.5, 1.0)));
        }
    }

    #[test]
    fn simplifier_uses_source_decisions_instead_of_tangent_breaks() {
        // A residual tangent break deliberately left by an earlier fitter.
        let points = wobble().path;
        let chain: Vec<_> = points.windows(2)
            .map(|w| [w[0], w[0], w[1], w[1]]).collect();
        let pass = SimplifyCurves { tolerance: 2.0, corner_threshold: params().corner_threshold };
        let legacy = pass.open(FittedGeom::Beziers(chain.clone()));
        assert!(matches!(legacy, FittedGeom::Beziers(ref c) if c.len() == 4));
        let corners = vec![points[0], *points.last().unwrap()];
        let geom = FittedGeom::SupportedBeziers { chain: chain.clone(), corners: corners.clone() };
        let (simplified, kept) = parts(pass.open(geom.clone()));
        assert!(simplified.len() < chain.len());
        assert_eq!(kept, corners);
        assert_eq!(simplified[0][0], points[0]);
        assert_eq!(simplified.last().unwrap()[3], *points.last().unwrap());
        let off = SimplifyCurves { tolerance: 0.0, ..pass };
        assert_eq!(parts(off.open(geom)), (chain, corners));
    }
}
