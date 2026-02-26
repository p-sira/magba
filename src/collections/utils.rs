/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use crate::{Float, Source};

macro_rules! impl_group_compute_B {
    () => {
        #[inline]
        fn compute_B(&self, point: Point3<T>) -> Vector3<T> {
            let mut net_field = Vector3::zeros();
            for source in &self.children {
                net_field += source.compute_B(point);
            }
            net_field
        }

        #[inline]
        fn compute_B_batch(&self, points: &[Point3<T>]) -> Vec<Vector3<T>> {
            let mut net_field = vec![Vector3::zeros(); points.len()];

            #[cfg(feature = "rayon")]
            {
                use rayon::prelude::*;

                let b_fields: Vec<_> = self
                    .children
                    .par_iter()
                    .map(|source| source.compute_B_batch(points))
                    .collect();
                b_fields.iter().for_each(|child_b_field| {
                    net_field
                        .iter_mut()
                        .zip(child_b_field)
                        .for_each(|(sum, b)| *sum += b)
                });
            }

            #[cfg(not(feature = "rayon"))]
            {
                for source in &self.children {
                    let b_fields = source.compute_B_batch(points);
                    net_field
                        .iter_mut()
                        .zip(b_fields)
                        .for_each(|(sum, b)| *sum += b);
                }
            }
            net_field
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
