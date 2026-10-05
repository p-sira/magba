/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

/// Estimated work at which distributing a collection's direct children is
/// expected to repay Rayon scheduling and reduction overhead.
#[cfg(any(feature = "rayon", test))]
pub(crate) const COLLECTION_RAYON_THRESHOLD: usize = 40_000;

#[inline]
#[cfg(any(feature = "rayon", test))]
pub(crate) const fn use_outer_parallel(
    direct_child_count: usize,
    observer_count: usize,
    relative_complexity: usize,
) -> bool {
    direct_child_count > 1
        && observer_count.saturating_mul(relative_complexity) > COLLECTION_RAYON_THRESHOLD
}

macro_rules! impl_group_compute_B {
    () => {
        #[inline]
        fn compute_B(&self, point: Point3<T>) -> Vector3<T> {
            self.components().fold(Vector3::zeros(), |acc, source| {
                acc + source.compute_B(point)
            })
        }

        fn relative_complexity(&self) -> usize {
            self.components().fold(0usize, |complexity, source| {
                complexity.saturating_add(source.relative_complexity())
            })
        }

        #[inline]
        fn compute_B_batch(&self, points: &[Point3<T>]) -> Vec<Vector3<T>> {
            #[cfg(feature = "rayon")]
            {
                use rayon::prelude::*;

                let auto = crate::collections::utils::use_outer_parallel(
                    self.nodes.len(),
                    points.len(),
                    self.relative_complexity(),
                );

                #[cfg(feature = "threshold-calibration")]
                let use_parallel = crate::threshold_calibration::should_parallel_collection(
                    auto,
                    self.nodes.len() > 1,
                );
                #[cfg(not(feature = "threshold-calibration"))]
                let use_parallel = auto;

                if use_parallel {
                    // Iterate over nodes directly to avoid collecting into a Vec.
                    self.nodes
                        .par_iter()
                        .map(|node| node.component().compute_B_batch(points))
                        .reduce(
                            || vec![Vector3::zeros(); points.len()],
                            |mut acc, child_batch| {
                                acc.iter_mut()
                                    .zip(child_batch)
                                    .for_each(|(sum, b)| *sum += b);
                                acc
                            },
                        )
                } else {
                    self.components()
                        .fold(vec![Vector3::zeros(); points.len()], |mut acc, source| {
                            let child_batch = source.compute_B_batch(points);
                            acc.iter_mut()
                                .zip(child_batch)
                                .for_each(|(sum, b)| *sum += b);
                            acc
                        })
                }
            }

            #[cfg(not(feature = "rayon"))]
            {
                // Standard sequential fold
                self.components()
                    .fold(vec![Vector3::zeros(); points.len()], |mut acc, source| {
                        let child_batch = source.compute_B_batch(points);
                        acc.iter_mut()
                            .zip(child_batch)
                            .for_each(|(sum, b)| *sum += b);
                        acc
                    })
            }
        }
    };
}
pub(crate) use impl_group_compute_B;

pub(crate) fn write_tree<'a, I: 'a>(
    f: &mut core::fmt::Formatter<'_>,
    leafs: impl IntoIterator<Item = &'a I>,
    indent: &str,
    mut format_leaf: impl FnMut(&'a I, &mut core::fmt::Formatter<'_>, &str) -> core::fmt::Result,
) -> core::fmt::Result {
    let mut iter = leafs.into_iter().enumerate().peekable();

    while let Some((i, leaf)) = iter.next() {
        let is_last = iter.peek().is_none();
        let branch = if is_last { "└── " } else { "├── " };

        write!(f, " {}{}{}: ", indent, branch, i)?;

        let extension = if is_last { "    " } else { "│   " };
        let next_indent = format!("{}{}", indent, extension);

        // Delegate to the provided closure
        format_leaf(leaf, f, &next_indent)?;

        if !is_last {
            writeln!(f)?;
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn collection_threshold_is_strict_and_saturating() {
        assert!(!use_outer_parallel(2, COLLECTION_RAYON_THRESHOLD - 1, 1));
        assert!(!use_outer_parallel(2, COLLECTION_RAYON_THRESHOLD, 1));
        assert!(use_outer_parallel(2, COLLECTION_RAYON_THRESHOLD + 1, 1));
        assert!(use_outer_parallel(2, usize::MAX, usize::MAX));
    }

    #[test]
    fn collection_requires_multiple_direct_children() {
        assert!(!use_outer_parallel(0, usize::MAX, usize::MAX));
        assert!(!use_outer_parallel(1, usize::MAX, usize::MAX));
    }
}
