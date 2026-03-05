/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use core::ops::{Index, IndexMut};
use std::fmt::Display;

use nalgebra::{Point3, Translation3, UnitQuaternion};

use crate::{
    base::{
        Float, Observer, Pose, SensorOutput, Source, Transform,
        transform::{impl_group_transform, impl_transform},
    },
    collections::{Node, ObserverArray, ObserverComponent},
};

// MARK: Base

/// Heap-allocated data structure for grouping [ObserverComponent].
#[derive(Debug, Clone)]
pub struct ObserverAssembly<T: Float = f64> {
    pose: Pose<T>,
    nodes: Vec<Node<ObserverComponent<T>, T>>,
}

impl<T: Float> ObserverAssembly<T> {
    /// Constructs a [ObserverAssembly], keeping the components' coordinates as GLOBAL.
    pub fn new(
        position: Point3<T>,
        orientation: UnitQuaternion<T>,
        components: impl IntoIterator<Item = impl Into<ObserverComponent<T>>>,
    ) -> Self {
        let pose = Pose::new(position, orientation);
        let pose_inv = pose.as_isometry().inverse();

        let nodes = components
            .into_iter()
            .map(|c| {
                let component: ObserverComponent<T> = c.into();
                let local_offset = (pose_inv * component.pose().as_isometry()).into();
                Node::new(component, local_offset)
            })
            .collect();

        Self { pose, nodes }
    }

    pub fn components(&self) -> impl Iterator<Item = &ObserverComponent<T>> {
        self.nodes.iter().map(|n| &n.component)
    }

    pub fn iter(&self) -> impl Iterator<Item = &ObserverComponent<T>> {
        self.components()
    }

    /// Acquires a reading from all sensors in the assembly given a magnetic source.
    pub fn read_all(&self, source: &dyn Source<T>) -> Vec<SensorOutput<T>> {
        self.nodes
            .iter()
            .map(|node| node.component.read(source))
            .collect()
    }
}

impl<T: Float> Default for ObserverAssembly<T> {
    fn default() -> Self {
        Self {
            pose: Pose::default(),
            nodes: Vec::new(),
        }
    }
}

// MARK: With builders

impl<T: Float> ObserverAssembly<T> {
    pub fn with(mut self, component: impl Into<ObserverComponent<T>>) -> Self {
        self.push(component);
        self
    }

    pub fn with_position(mut self, position: impl Into<Translation3<T>>) -> Self {
        self.set_position(position);
        self
    }

    pub fn with_orientation(mut self, orientation: UnitQuaternion<T>) -> Self {
        self.set_orientation(orientation);
        self
    }

    pub fn with_pose(mut self, pose: impl Into<Pose<T>>) -> Self {
        self.set_pose(pose);
        self
    }
}

// MARK: From, Into

impl<T: Float> FromIterator<ObserverComponent<T>> for ObserverAssembly<T> {
    fn from_iter<I: IntoIterator<Item = ObserverComponent<T>>>(iter: I) -> Self {
        let nodes = iter
            .into_iter()
            .map(|c| {
                let local_offset = *c.pose();
                Node::new(c, local_offset)
            })
            .collect();

        Self {
            pose: Pose::default(),
            nodes,
        }
    }
}

impl<T: Float, I: Into<ObserverComponent<T>>, const N: usize> From<[I; N]> for ObserverAssembly<T> {
    fn from(components: [I; N]) -> Self {
        components.into_iter().map(Into::into).collect()
    }
}

impl<T: Float, I: Into<ObserverComponent<T>>> From<Vec<I>> for ObserverAssembly<T> {
    fn from(components: Vec<I>) -> Self {
        components.into_iter().map(Into::into).collect()
    }
}

impl<T: Float> From<&[ObserverComponent<T>]> for ObserverAssembly<T> {
    fn from(components: &[ObserverComponent<T>]) -> Self {
        components.iter().cloned().collect()
    }
}

impl<'a, T: Float> IntoIterator for &'a ObserverAssembly<T> {
    type Item = &'a ObserverComponent<T>;
    type IntoIter = std::iter::Map<
        std::slice::Iter<'a, Node<ObserverComponent<T>, T>>,
        fn(&'a Node<ObserverComponent<T>, T>) -> &'a ObserverComponent<T>,
    >;

    fn into_iter(self) -> Self::IntoIter {
        self.nodes.iter().map(|n| &n.component)
    }
}

impl<S, T: Float, const N: usize> From<ObserverArray<S, N, T>> for ObserverAssembly<T>
where
    S: Observer<T> + Into<ObserverComponent<T>>,
{
    fn from(array: ObserverArray<S, N, T>) -> Self {
        ObserverAssembly::new(array.position(), array.orientation(), array)
    }
}

// MARK: Extend, Push

impl<T: Float> ObserverAssembly<T> {
    pub fn push(&mut self, component: impl Into<ObserverComponent<T>>) {
        let component: ObserverComponent<T> = component.into();
        let local_offset =
            (self.pose.as_isometry().inverse() * component.pose().as_isometry()).into();
        self.nodes.push(Node::new(component, local_offset));
    }
}

impl<T: Float> Extend<ObserverComponent<T>> for ObserverAssembly<T> {
    fn extend<I: IntoIterator<Item = ObserverComponent<T>>>(&mut self, iter: I) {
        for component in iter {
            self.push(component);
        }
    }
}

// MARK: Index

impl<T: Float> Index<usize> for ObserverAssembly<T> {
    type Output = ObserverComponent<T>;

    fn index(&self, index: usize) -> &Self::Output {
        &self.nodes[index].component
    }
}

impl<T: Float> IndexMut<usize> for ObserverAssembly<T> {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        &mut self.nodes[index].component
    }
}

// MARK: Transform

impl_transform!(ObserverAssembly<T> where T: Float);
impl_group_transform!(ObserverAssembly<T> where T: Float);

// MARK: Display

impl<T: Float> ObserverAssembly<T> {
    pub fn format(&self, f: &mut std::fmt::Formatter<'_>, indent: &str) -> std::fmt::Result {
        writeln!(
            f,
            "ObserverAssembly ({} children) at {}",
            self.nodes.len(),
            self.pose()
        )?;

        crate::collections::utils::write_tree(f, self.components(), indent, |leaf, f, ind| {
            leaf.format(f, ind)
        })
    }
}

impl<T: Float> Display for ObserverAssembly<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        self.format(f, "")
    }
}

// MARK: PartialEq

impl<T: Float> PartialEq for ObserverAssembly<T> {
    fn eq(&self, other: &Self) -> bool {
        if self.position() != other.position()
            || self.orientation() != other.orientation()
            || self.nodes.len() != other.nodes.len()
        {
            return false;
        }

        let mut matched = vec![false; other.nodes.len()];
        for node in &self.nodes {
            let found =
                other.nodes.iter().enumerate().find(|(idx, other_node)| {
                    !matched[*idx] && node.component == other_node.component
                });

            match found {
                Some((idx, _)) => matched[idx] = true,
                None => return false,
            }
        }
        true
    }
}

#[cfg(test)]
mod tests {
    // TODO
    #[test]
    fn test_todo() {
        use crate::collections::observers;
        use crate::prelude::*;
        let _: ObserverAssembly = observers!();
    }
}
