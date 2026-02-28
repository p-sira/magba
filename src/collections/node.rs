/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Node pairs a component with its local offset for synchronized hierarchy storage.

use crate::{base::Float, geometry::Pose};

/// A node pairing a component with its local pose offset.
///
/// Used by [crate::Collection] and [crate::SourceArray] to keep each child
/// and its offset in one place so they stay synchronized.
#[derive(Debug, Clone)]
pub struct Node<S, T: Float = f64> {
    pub component: S,
    pub local_offset: Pose<T>,
}

impl<S, T: Float> Node<S, T> {
    pub fn new(component: S, local_offset: Pose<T>) -> Self {
        Self {
            component,
            local_offset,
        }
    }
}

impl<S: Default, T: Float> Default for Node<S, T> {
    fn default() -> Self {
        Self {
            component: S::default(),
            local_offset: Pose::default(),
        }
    }
}

impl<S: PartialEq, T: Float> PartialEq for Node<S, T> {
    fn eq(&self, other: &Self) -> bool {
        self.component == other.component && self.local_offset == other.local_offset
    }
}

impl<S: Eq, T: Float> Eq for Node<S, T> {}
