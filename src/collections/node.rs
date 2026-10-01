/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

//! Node pairs a component with its local offset for synchronized hierarchy storage.

use crate::base::{Float, Pose};

/// A node pairing a component with its local pose offset.
///
/// Used by [SourceAssembly](crate::collections::SourceAssembly) and [SourceArray](crate::collections::SourceArray) to keep each child
/// and its offset in one place so they stay synchronized.
#[derive(Debug, Clone)]
pub struct Node<S, T: Float = f64> {
    component: S,
    local_offset: Pose<T>,
    dirty: bool,
}

impl<S, T: Float> Node<S, T> {
    pub fn new(component: S, local_offset: Pose<T>) -> Self {
        Self {
            component,
            local_offset,
            dirty: false,
        }
    }

    pub fn component(&self) -> &S {
        &self.component
    }

    pub fn component_mut(&mut self) -> &mut S {
        self.dirty = true;
        &mut self.component
    }

    pub fn into_component(self) -> S {
        self.component
    }

    pub fn local_offset(&self) -> &Pose<T> {
        &self.local_offset
    }

    pub(crate) fn sync_local_offset(
        &mut self,
        parent_pose: &Pose<T>,
        parent_inverse: &nalgebra::Isometry3<T>,
    ) where
        S: crate::base::Transform<T>,
    {
        if self.dirty {
            let expected_pose = parent_pose.as_isometry() * self.local_offset.as_isometry();
            if self.component.pose().as_isometry() != &expected_pose {
                self.local_offset = (parent_inverse * self.component.pose().as_isometry()).into();
            }
            self.dirty = false;
        }
    }

    pub(crate) fn apply_parent_pose(&mut self, parent_pose: &Pose<T>)
    where
        S: crate::base::Transform<T>,
    {
        let global_isometry = parent_pose.as_isometry() * self.local_offset.as_isometry();
        self.component.set_pose(global_isometry.into());
    }

    #[cfg(test)]
    pub(crate) fn is_dirty(&self) -> bool {
        self.dirty
    }
}

impl<S: Default, T: Float> Default for Node<S, T> {
    fn default() -> Self {
        Self {
            component: S::default(),
            local_offset: Pose::default(),
            dirty: false,
        }
    }
}

impl<S: PartialEq, T: Float> PartialEq for Node<S, T> {
    fn eq(&self, other: &Self) -> bool {
        self.component == other.component && self.local_offset == other.local_offset
    }
}

impl<S: Eq, T: Float> Eq for Node<S, T> {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_node_methods() {
        let node1 = Node::<String, f64>::default();
        let node2 = Node::<String, f64>::default();
        assert_eq!(node1, node2);
        assert!(!node1.is_dirty());
        assert_eq!(node1.component(), "");
        assert_eq!(node1.local_offset(), &Pose::default());
        assert_eq!(node1.into_component(), "");
    }
}
