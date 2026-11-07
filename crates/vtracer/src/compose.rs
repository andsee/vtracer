//! Compositing: turn a [`Segmentation`] into a [`VectorDoc`].
//!
//! * **Stacked** — each layer is traced independently into closed outlines and
//!   stacked in paint order (painter's algorithm).
//! * **Mosaic** — a seam-free gapless tessellation with shared boundary
//!   geometry (see [`crate::mosaic`]).
//!
//! Both compositors run the pipeline's [`CurvePass`]es over every fitted
//! contour before assembling paths — geometry passes have to happen here, on
//! the fitted geometry, so that in mosaic mode each shared boundary segment
//! is transformed exactly once for both of its faces.

use crate::error::Error;
use crate::fitter::{CurveFitter, FittedGeom};
use crate::ir::{MultiPath, Segmentation, Shape, VectorDoc};
use crate::mosaic::{compose_mosaic, SegmentFitter};
use crate::progress::{Ctx, Phase};
use crate::simplify::CurvePass;

/// Which compositing strategy the pipeline uses. Each variant owns its fitter.
pub enum Compositing {
    /// Independent per-region closed outlines, stacked bottom-to-top.
    Stacked(Box<dyn CurveFitter>),
    /// Seam-free gapless tessellation via a shared boundary graph.
    Mosaic {
        fitter: Box<dyn SegmentFitter>,
        /// Merge flattened neighbours whose colors are within this diff —
        /// rejoins regions the stacked gradient layering had split. Usually
        /// the clustering gradient step; `0` still merges identical-color
        /// neighbours, negative disables merging entirely.
        merge_diff: i32,
    },
}

impl Compositing {
    /// Run the selected compositor over a segmentation, applying `passes` to
    /// every fitted contour before paths are assembled.
    pub fn compose(&self, seg: &Segmentation, passes: &[Box<dyn CurvePass>]) -> VectorDoc {
        match self {
            Compositing::Stacked(fitter) => compose_stacked(seg, fitter.as_ref(), passes),
            Compositing::Mosaic { fitter, merge_diff } => {
                compose_mosaic(seg, fitter.as_ref(), *merge_diff, passes)
            }
        }
    }

    /// Progress- and cancellation-aware compositing.
    ///
    /// Stacked mode reports per-layer progress and can be cancelled between
    /// layers. Mosaic builds its boundary graph in one pass, so it reports
    /// coarsely (start/end) and is cancellable only at the boundaries — the
    /// dominant cost is upstream in clustering, which cancels finely.
    pub fn compose_with(
        &self,
        seg: &Segmentation,
        passes: &[Box<dyn CurvePass>],
        ctx: &mut Ctx,
    ) -> Result<VectorDoc, Error> {
        match self {
            Compositing::Stacked(fitter) => compose_stacked_with(seg, fitter.as_ref(), passes, ctx),
            Compositing::Mosaic { fitter, merge_diff } => {
                ctx.check()?;
                ctx.report(Phase::Compose, 0.0);
                let doc = compose_mosaic(seg, fitter.as_ref(), *merge_diff, passes);
                ctx.check()?;
                ctx.report(Phase::Compose, 1.0);
                Ok(doc)
            }
        }
    }
}

/// Run closed-ring passes with cross-layer context, then assemble each layer.
fn fit_regions(
    mut regions: Vec<Vec<FittedGeom>>,
    passes: &[Box<dyn CurvePass>],
) -> Vec<MultiPath> {
    let counts: Vec<usize> = regions.iter().map(Vec::len).collect();
    let mut geoms: Vec<FittedGeom> = regions.drain(..).flatten().collect();
    for pass in passes {
        geoms = pass.rings(geoms);
    }
    let mut geoms = geoms.into_iter();
    counts
        .into_iter()
        .map(|count| {
            let mut path = MultiPath::new();
            for geom in geoms.by_ref().take(count) {
                path.push(geom.into_closed_subpath());
            }
            path
        })
        .collect()
}

fn fit_stacked_regions(
    seg: &Segmentation,
    fitter: &dyn CurveFitter,
    passes: &[Box<dyn CurvePass>],
) -> Vec<MultiPath> {
    let regions = seg
        .layers
        .iter()
        .map(|layer| fitter.fit_region(&layer.mask))
        .collect();
    fit_regions(regions, passes)
}

fn append_stacked(
    seg: &Segmentation,
    paths: impl IntoIterator<Item = MultiPath>,
    doc: &mut VectorDoc,
) {
    for (layer, path) in seg.layers.iter().zip(paths) {
        if !path.is_empty() {
            doc.shapes.push(Shape {
                paint: layer.paint,
                path,
            });
        }
    }
}

/// Progress-aware [`compose_stacked`]: reports after each layer and checks for
/// cancellation between them.
fn compose_stacked_with(
    seg: &Segmentation,
    fitter: &dyn CurveFitter,
    passes: &[Box<dyn CurvePass>],
    ctx: &mut Ctx,
) -> Result<VectorDoc, Error> {
    let mut doc = VectorDoc::new(seg.width, seg.height);
    let total = seg.layers.len().max(1);
    let mut regions = Vec::with_capacity(seg.layers.len());
    for (i, layer) in seg.layers.iter().enumerate() {
        ctx.check()?;
        regions.push(fitter.fit_region(&layer.mask));
        ctx.report(Phase::Compose, (i + 1) as f32 / total as f32);
    }
    let paths = fit_regions(regions, passes);
    append_stacked(seg, paths, &mut doc);
    Ok(doc)
}

/// Trace every layer's closed outline and stack the shapes in paint order.
pub fn compose_stacked(
    seg: &Segmentation,
    fitter: &dyn CurveFitter,
    passes: &[Box<dyn CurvePass>],
) -> VectorDoc {
    let mut doc = VectorDoc::new(seg.width, seg.height);
    let paths = fit_stacked_regions(seg, fitter, passes);
    append_stacked(seg, paths, &mut doc);
    doc
}
