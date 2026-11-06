//! Size-aware corner decisions made on the source contour.

use std::collections::HashSet;

use visioncortex::{PathF64, PointF64, SubdivideSmooth};

use crate::fitter::{FitParams, FittedGeom};

fn point_set(points: impl Iterator<Item = PointF64>) -> HashSet<PointKey> {
    points.map(PointKey::from).collect()
}

#[derive(Clone, Copy, Eq, Hash, PartialEq)]
struct PointKey(u64, u64);

impl From<PointF64> for PointKey {
    fn from(p: PointF64) -> Self {
        Self(
            if p.x == 0.0 { 0.0 } else { p.x }.to_bits(),
            if p.y == 0.0 { 0.0 } else { p.y }.to_bits(),
        )
    }
}

fn turn(a: PointF64, b: PointF64, c: PointF64) -> f64 {
    let (u, v) = (b - a, c - b);
    (u.x * v.y - u.y * v.x).atan2(u.x * v.x + u.y * v.y)
}

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
        } else if n < 3
            || turn(
                points[(i + n - 1) % n],
                points[i],
                points[(i + 1) % n],
            )
            .abs()
                > 1e-12
        {
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
    for _ in 0..n.saturating_sub(1) {
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
        if params.corner_support >= perimeter / 2.0 {
            return (local, vec![false; n]);
        }
    }
    let turns: Vec<f64> = (0..n)
        .map(|i| {
            if !closed && (i == 0 || i + 1 == n) {
                0.0
            } else {
                turn(
                    points[(i + n - 1) % n],
                    points[i],
                    points[(i + 1) % n],
                )
            }
        })
        .collect();
    let mut protected = local.clone();
    for i in 0..n {
        if !local[i] || (!closed && (i == 0 || i + 1 == n)) {
            continue;
        }
        if turns[i].abs() >= std::f64::consts::PI - 1e-12 {
            continue;
        }
        let before = probe(
            points,
            &turns,
            i,
            false,
            params.corner_support,
            params.corner_threshold,
            closed,
        );
        let after = probe(
            points,
            &turns,
            i,
            true,
            params.corner_support,
            params.corner_threshold,
            closed,
        );
        let supported = turn(before, points[i], after);
        protected[i] = supported * turns[i] > 0.0
            && supported.abs() + 1e-12 >= params.corner_threshold;
    }
    let rejected = local
        .iter()
        .zip(&protected)
        .map(|(was_corner, keep)| *was_corner && !*keep)
        .collect();
    (protected, rejected)
}

pub(crate) fn has_rejected_candidate(path: &PathF64, params: FitParams, closed: bool) -> bool {
    let path = canonical(path.clone(), closed);
    if path.len() < if closed { 4 } else { 3 } {
        return false;
    }
    classify(&path, params, closed).1.into_iter().any(|rejected| rejected)
}

fn fitting_corner_flags(
    path: &PathF64,
    params: FitParams,
    closed: bool,
) -> (Vec<bool>, Vec<PointF64>, HashSet<PointKey>) {
    let classification_path = canonical(path.clone(), closed);
    let (_, rejected) = classify(&classification_path, params, closed);
    let rejected = point_set(
        rejected
            .iter()
            .enumerate()
            .filter_map(|(i, &drop)| drop.then_some(classification_path.path[i])),
    );
    let mut corners = SubdivideSmooth::find_corners(path, params.corner_threshold, closed);
    for (i, corner) in corners.iter_mut().enumerate() {
        *corner &= !rejected.contains(&PointKey::from(path.path[i]));
    }
    let source_corners = corners
        .iter()
        .enumerate()
        .filter_map(|(i, &keep)| keep.then_some(path.path[i]))
        .collect();
    (corners, source_corners, rejected)
}

pub(crate) fn fit_supported(mut path: PathF64, params: FitParams, closed: bool) -> FittedGeom {
    let classification_path = canonical(path.clone(), closed);
    if classification_path.len() < if closed { 4 } else { 3 } {
        return FittedGeom::Polyline(path.path);
    }
    let (mut protected, corners, rejected) = fitting_corner_flags(&path, params, closed);
    for _ in 0..params.max_iterations {
        let (next, flags, done) = SubdivideSmooth::subdivide_keep_corners(
            &path,
            &protected,
            8.0,
            params.length_threshold,
            closed,
        );
        path = next;
        protected = flags;
        if done {
            break;
        }
    }
    if closed {
        for _ in 0..2 {
            let available = path.path[..path.len() - 1]
                .iter()
                .filter(|&&p| !rejected.contains(&PointKey::from(p)))
                .count();
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
        *cut &= !rejected.contains(&PointKey::from(path.path[i]));
        *cut |= protected[i];
    }
    let n = splice.len();
    let mut cuts: Vec<usize> = splice
        .iter()
        .enumerate()
        .filter_map(|(i, &cut)| cut.then_some(i))
        .collect();
    if closed {
        if cuts.is_empty() {
            cuts.push(
                (0..n)
                    .find(|&i| !rejected.contains(&PointKey::from(path.path[i])))
                    .unwrap_or(0),
            );
        }
        if cuts.len() == 1 {
            let opposite = (cuts[0] + n / 2) % n;
            let other = (0..n)
                .map(|offset| (opposite + offset) % n)
                .find(|&i| i != cuts[0] && !rejected.contains(&PointKey::from(path.path[i])))
                .expect("closed contour has two distinct spline seams");
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

    fn p(x: f64, y: f64) -> PointF64 {
        PointF64 { x, y }
    }

    fn params() -> FitParams {
        FitParams {
            corner_support: 8.0,
            ..FitParams::default()
        }
    }

    fn wobble() -> PathF64 {
        PathF64::from_points(vec![
            p(-30.0, 0.0),
            p(0.0, 0.0),
            p(0.5, 1.0),
            p(1.0, 0.0),
            p(30.0, 0.0),
        ])
    }

    #[test]
    fn supported_corners_reject_a_short_wobble() {
        let path = wobble();
        let local = SubdivideSmooth::find_corners(&path, params().corner_threshold, false);
        assert_eq!(local, vec![true; 5]);
        let (protected, rejected) = classify(&path, params(), false);
        assert_eq!(protected, vec![true, false, false, false, true]);
        assert_eq!(rejected, vec![false, true, true, true, false]);
        let geom = fit_supported(path, params(), false);
        match geom {
            FittedGeom::SupportedBeziers { chain, corners } => {
                assert_eq!(corners, vec![p(-30.0, 0.0), p(30.0, 0.0)]);
                assert_eq!(chain[0][0], p(-30.0, 0.0));
                assert_eq!(chain.last().unwrap()[3], p(30.0, 0.0));
                for q in [p(0.0, 0.0), p(0.5, 1.0), p(1.0, 0.0)] {
                    assert!(!chain.iter().any(|c| c[0] == q || c[3] == q));
                }
            }
            _ => panic!("expected supported spline"),
        }
    }

    #[test]
    fn small_ring_bypass_and_sustained_corners_are_preserved() {
        for side in [1.0, 40.0] {
            let path = PathF64::from_points(vec![
                p(0.0, 0.0),
                p(side, 0.0),
                p(side, side),
                p(0.0, side),
                p(0.0, 0.0),
            ]);
            assert_eq!(classify(&path, params(), true).0, vec![true; 4]);
        }
        let v = PathF64::from_points(vec![p(-30.0, 30.0), p(0.0, 0.0), p(30.0, 30.0)]);
        assert_eq!(classify(&v, params(), false).0, vec![true; 3]);
    }

    #[test]
    fn subthreshold_concave_vertex_is_not_a_rejection_candidate() {
        let path = PathF64::from_points(vec![
            p(4.0, 4.0),
            p(60.0, 4.0),
            p(60.0, 55.0),
            p(50.0, 55.0),
            p(29.0, 43.0),
            p(5.0, 55.0),
            p(4.0, 55.0),
            p(4.0, 4.0),
        ]);
        let vertex = 4;
        assert!(
            (turn(path.path[vertex - 1], path.path[vertex], path.path[vertex + 1])
                .to_degrees()
                .abs()
                - 56.31)
                .abs()
                < 0.1
        );
        let (_, rejected) = classify(&path, params(), true);
        assert!(!rejected[vertex], "sub-threshold turn is not a local candidate");
        assert!(!has_rejected_candidate(&path, params(), true));
    }

    #[test]
    fn fitting_changes_only_the_corner_flags_of_rejected_candidates() {
        let path = PathF64::from_points(vec![
            p(4.0, 4.0),
            p(20.0, 4.0),
            p(20.0, 5.0),
            p(21.0, 4.0),
            p(60.0, 4.0),
            p(60.0, 55.0),
            p(50.0, 55.0),
            p(29.0, 43.0),
            p(5.0, 55.0),
            p(4.0, 55.0),
            p(4.0, 4.0),
        ]);
        let junction = 7;
        let baseline = SubdivideSmooth::find_corners(&path, params().corner_threshold, true);
        let (supported, _, rejected) = fitting_corner_flags(&path, params(), true);
        assert!(has_rejected_candidate(&path, params(), true));
        assert_eq!(supported[junction], baseline[junction]);
        for i in 0..baseline.len() {
            if baseline[i] != supported[i] {
                assert!(rejected.contains(&PointKey::from(path.path[i])));
            }
        }
    }

    #[test]
    fn classification_is_invariant_to_collinear_samples() {
        let path = wobble();
        let mut dense = Vec::new();
        for pair in path.path.windows(2) {
            dense.push(pair[0]);
            dense.push(pair[0] + (pair[1] - pair[0]) * 0.5);
        }
        dense.push(*path.path.last().unwrap());
        assert_eq!(
            classify(&canonical(path, false), params(), false),
            classify(&canonical(PathF64::from_points(dense), false), params(), false)
        );
    }

    #[test]
    fn dense_noise_rejection_membership_scales_with_input() {
        let elapsed = |count| {
            let points: Vec<_> = (0..count)
                .map(|i| p(i as f64, (i % 2) as f64))
                .collect();
            let start = std::time::Instant::now();
            let rejected = point_set(points.iter().copied());
            assert!(points
                .iter()
                .all(|point| rejected.contains(&PointKey::from(*point))));
            start.elapsed()
        };
        let small = elapsed(8_000);
        let large = elapsed(32_000);
        assert!(
            large < small * 16 + std::time::Duration::from_millis(10),
            "4x dense input took {small:?} then {large:?}"
        );
    }

}
