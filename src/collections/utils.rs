/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use crate::{Float, Source};

macro_rules! impl_group_compute_B {
    () => {
        #[inline]
        fn compute_B(&self, point: Point3<T>) -> Vector3<T> {
            self.components().fold(Vector3::zeros(), |acc, source| {
                acc + source.compute_B(point)
            })
        }

        #[inline]
        fn compute_B_batch(&self, points: &[Point3<T>]) -> Vec<Vector3<T>> {
            #[cfg(feature = "rayon")]
            {
                use rayon::prelude::*;

                // Parallel iterate over nodes directly to avoid collecting into a Vec
                self.nodes
                    .par_iter()
                    .map(|node| node.component.compute_B_batch(points))
                    .reduce(
                        || vec![Vector3::zeros(); points.len()],
                        |mut acc, child_batch| {
                            acc.iter_mut()
                                .zip(child_batch)
                                .for_each(|(sum, b)| *sum += b);
                            acc
                        },
                    )
            }

            #[cfg(not(feature = "rayon"))]
            {
                // Standard sequential fold
                self.nodes()
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

pub(crate) fn write_tree<'a, T: Float, S: Source<T> + 'a>(
    f: &mut core::fmt::Formatter<'_>,
    leafs: impl IntoIterator<Item = &'a S>,
    indent: &str,
) -> core::fmt::Result {
    let mut iter = leafs.into_iter().enumerate().peekable();

    while let Some((i, leaf)) = iter.next() {
        let is_last = iter.peek().is_none();
        let branch = if is_last { "└── " } else { "├── " };

        write!(f, " {}{}{}: ", indent, branch, i)?;

        let extension = if is_last { "    " } else { "│   " };
        let next_indent = format!("{}{}", indent, extension);
        leaf.format(f, &next_indent)?;

        if !is_last {
            writeln!(f)?;
        }
    }
    Ok(())
}
