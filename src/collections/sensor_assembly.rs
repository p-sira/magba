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
    collections::{Node, SensorArray, SensorComponent},
};

// MARK: Base

/// Heap-allocated data structure for grouping [Sensor].
#[derive(Debug, Clone)]
pub struct SensorAssembly<T: Float = f64> {
    pose: Pose<T>,
    nodes: Vec<Node<SensorComponent<T>, T>>,
}

impl<T: Float> SensorAssembly<T> {
    /// Constructs a [SensorAssembly], keeping the components' coordinates as GLOBAL.
    pub fn new(
        position: Point3<T>,
        orientation: UnitQuaternion<T>,
        components: impl IntoIterator<Item = impl Into<SensorComponent<T>>>,
    ) -> Self {
        let pose = Pose::new(position, orientation);
        let pose_inv = pose.as_isometry().inverse();

        let nodes = components
            .into_iter()
            .map(|c| {
                let component: SensorComponent<T> = c.into();
                let local_offset = (pose_inv * component.pose().as_isometry()).into();
                Node::new(component, local_offset)
            })
            .collect();

        Self { pose, nodes }
    }

    pub fn components(&self) -> impl Iterator<Item = &SensorComponent<T>> {
        self.nodes.iter().map(|n| &n.component)
    }

    pub fn iter(&self) -> impl Iterator<Item = &SensorComponent<T>> {
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

impl<T: Float> Default for SensorAssembly<T> {
    fn default() -> Self {
        Self {
            pose: Pose::default(),
            nodes: Vec::new(),
        }
    }
}

// MARK: With builders

impl<T: Float> SensorAssembly<T> {
    pub fn with(mut self, component: impl Into<SensorComponent<T>>) -> Self {
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

impl<T: Float> FromIterator<SensorComponent<T>> for SensorAssembly<T> {
    fn from_iter<I: IntoIterator<Item = SensorComponent<T>>>(iter: I) -> Self {
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

impl<T: Float, I: Into<SensorComponent<T>>, const N: usize> From<[I; N]> for SensorAssembly<T> {
    fn from(components: [I; N]) -> Self {
        components.into_iter().map(Into::into).collect()
    }
}

impl<T: Float, I: Into<SensorComponent<T>>> From<Vec<I>> for SensorAssembly<T> {
    fn from(components: Vec<I>) -> Self {
        components.into_iter().map(Into::into).collect()
    }
}

impl<T: Float> From<&[SensorComponent<T>]> for SensorAssembly<T> {
    fn from(components: &[SensorComponent<T>]) -> Self {
        components.iter().cloned().collect()
    }
}

impl<'a, T: Float> IntoIterator for &'a SensorAssembly<T> {
    type Item = &'a SensorComponent<T>;
    type IntoIter = std::iter::Map<
        std::slice::Iter<'a, Node<SensorComponent<T>, T>>,
        fn(&'a Node<SensorComponent<T>, T>) -> &'a SensorComponent<T>,
    >;

    fn into_iter(self) -> Self::IntoIter {
        self.nodes.iter().map(|n| &n.component)
    }
}

impl<S, T: Float, const N: usize> From<SensorArray<S, N, T>> for SensorAssembly<T>
where
    S: Observer<T> + Into<SensorComponent<T>>,
{
    fn from(array: SensorArray<S, N, T>) -> Self {
        SensorAssembly::new(array.position(), array.orientation(), array)
    }
}

// MARK: Extend, Push

impl<T: Float> SensorAssembly<T> {
    pub fn push(&mut self, component: impl Into<SensorComponent<T>>) {
        let component: SensorComponent<T> = component.into();
        let local_offset =
            (self.pose.as_isometry().inverse() * component.pose().as_isometry()).into();
        self.nodes.push(Node::new(component, local_offset));
    }
}

impl<T: Float> Extend<SensorComponent<T>> for SensorAssembly<T> {
    fn extend<I: IntoIterator<Item = SensorComponent<T>>>(&mut self, iter: I) {
        for component in iter {
            self.push(component);
        }
    }
}

// MARK: Index

impl<T: Float> Index<usize> for SensorAssembly<T> {
    type Output = SensorComponent<T>;

    fn index(&self, index: usize) -> &Self::Output {
        &self.nodes[index].component
    }
}

impl<T: Float> IndexMut<usize> for SensorAssembly<T> {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        &mut self.nodes[index].component
    }
}

// MARK: Transform

impl_transform!(SensorAssembly<T> where T: Float);
impl_group_transform!(SensorAssembly<T> where T: Float);

// MARK: Display

impl<T: Float> SensorAssembly<T> {
    pub fn format(&self, f: &mut std::fmt::Formatter<'_>, indent: &str) -> std::fmt::Result {
        writeln!(
            f,
            "SensorAssembly ({} children) at {}",
            self.nodes.len(),
            self.pose()
        )?;

        crate::collections::utils::write_tree(f, self.components(), indent, |leaf, f, ind| {
            leaf.format(f, ind)
        })
    }
}

impl<T: Float> Display for SensorAssembly<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        self.format(f, "")
    }
}

// MARK: PartialEq

impl<T: Float> PartialEq for SensorAssembly<T> {
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
        use crate::collections::sensors;
        use crate::prelude::*;
        let _: SensorAssembly = sensors!();
    }
}
