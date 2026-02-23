/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use crate::{Float, Source};

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