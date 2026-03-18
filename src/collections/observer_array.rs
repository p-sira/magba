/*
 * Magba is licensed under The 3-Clause BSD, see LICENSE.
 * Copyright 2025 Sira Pornsiriprasert <code@psira.me>
 */

use core::fmt::Display;
use core::ops::{Index, IndexMut};

use nalgebra::{Point3, Translation3, UnitQuaternion};

use crate::{
    base::{
        Float, Observer, Pose, SensorOutput, Source, Transform,
        transform::{impl_group_transform, impl_transform},
    },
    collections::node::Node,
};

/// Stack-allocated data structure for grouping [Observer].
#[derive(Debug, Clone)]
pub struct ObserverArray<S: Observer<T>, const N: usize, T: Float = f64> {
    pose: Pose<T>,
    nodes: [Node<S, T>; N],
}

impl<S: Observer<T>, const N: usize, T: Float> ObserverArray<S, N, T> {
    pub fn new(
        position: impl Into<Point3<T>>,
        orientation: UnitQuaternion<T>,
        sensors: [S; N],
    ) -> Self {
        let pose = Pose::new(position.into(), orientation);
        let pose_inv = pose.as_isometry().inverse();

        let mut into_iter = sensors.into_iter();
        let nodes = core::array::from_fn(|_| {
            let component = into_iter.next().unwrap();
            let local_offset = (pose_inv * component.pose().as_isometry()).into();
            Node::new(component, local_offset)
        });

        Self { pose, nodes }
    }

    pub fn components(&self) -> impl Iterator<Item = &S> {
        self.nodes.iter().map(|n| &n.component)
    }

    pub fn iter(&self) -> impl Iterator<Item = &S> {
        self.components()
    }

    /// Acquires a reading from all sensors in the array given a magnetic source.
    pub fn read_all(&self, source: &dyn Source<T>) -> [SensorOutput<T>; N] {
        core::array::from_fn(|i| self.nodes[i].component.read(source))
    }
}

impl<S: Observer<T> + Default, T: Float, const N: usize> Default for ObserverArray<S, N, T> {
    fn default() -> Self {
        Self {
            pose: Pose::default(),
            nodes: core::array::from_fn(|_| Node::default()),
        }
    }
}

// MARK: Transform

impl_transform!(ObserverArray<S, N, T> where S: Observer<T>, const N: usize, T: Float);
impl_group_transform!(ObserverArray<S, N, T> where S: Observer<T>, const N: usize, T: Float);

// MARK: Index

impl<S: Observer<T>, const N: usize, T: Float> Index<usize> for ObserverArray<S, N, T> {
    type Output = S;

    fn index(&self, index: usize) -> &Self::Output {
        &self.nodes[index].component
    }
}

impl<S: Observer<T>, const N: usize, T: Float> IndexMut<usize> for ObserverArray<S, N, T> {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        &mut self.nodes[index].component
    }
}

// MARK: From, Into

impl<S: Observer<T>, const N: usize, T: Float> From<[S; N]> for ObserverArray<S, N, T> {
    fn from(sensors: [S; N]) -> Self {
        let mut into_iter = sensors.into_iter();
        let nodes = core::array::from_fn(|_| {
            let component = into_iter.next().unwrap();
            let local_offset = *component.pose();
            Node::new(component, local_offset)
        });

        Self {
            pose: Pose::default(),
            nodes,
        }
    }
}

impl<S: Observer<T>, const N: usize, T: Float> FromIterator<S> for ObserverArray<S, N, T> {
    fn from_iter<I: IntoIterator<Item = S>>(iter: I) -> Self {
        let mut iter = iter.into_iter();
        let sensors = core::array::from_fn(|_| {
            iter.next()
                .expect("ObserverArray::from_iter: iterator yielded fewer than N items")
        });

        if iter.next().is_some() {
            panic!("ObserverArray::from_iter: iterator yielded more than N items");
        }

        Self::from(sensors)
    }
}

impl<'a, S: Observer<T>, const N: usize, T: Float> IntoIterator for &'a ObserverArray<S, N, T> {
    type Item = &'a S;
    type IntoIter = std::iter::Map<std::slice::Iter<'a, Node<S, T>>, fn(&'a Node<S, T>) -> &'a S>;

    fn into_iter(self) -> Self::IntoIter {
        self.nodes.iter().map(|n| &n.component)
    }
}

impl<S: Observer<T>, const N: usize, T: Float> IntoIterator for ObserverArray<S, N, T> {
    type Item = S;
    type IntoIter = std::iter::Map<std::array::IntoIter<Node<S, T>, N>, fn(Node<S, T>) -> S>;

    fn into_iter(self) -> Self::IntoIter {
        self.nodes.into_iter().map(|n| n.component)
    }
}

// MARK: PartialEq

impl<S: Observer<T> + PartialEq, T: Float, const N: usize> PartialEq for ObserverArray<S, N, T> {
    fn eq(&self, other: &Self) -> bool {
        if self.position() != other.position() || self.orientation() != other.orientation() {
            return false;
        }

        let mut matched = [false; N];
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

// MARK: Display

impl<S: Observer<T>, const N: usize, T: Float> Display for ObserverArray<S, N, T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(
            f,
            "ObserverArray ({} children) at {}",
            self.nodes.len(),
            self.pose()
        )?;

        crate::collections::utils::write_tree(f, self.components(), "", |leaf, f, ind| {
            leaf.format(f, ind)
        })
    }
}

// MARK: Tests

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sensors::hall_effect::LinearHallSensor;

    #[test]
    fn test_observer_array_from_iter() {
        let sensors = (0..3).map(|i| {
            LinearHallSensor::new(
                [i as f64, 0.0, 0.0],
                UnitQuaternion::identity(),
                [0.0, 0.0, 1.0],
                30.0,
                3.3,
            )
        });

        let array: ObserverArray<LinearHallSensor, 3> = sensors.collect();
        assert_eq!(array.nodes.len(), 3);
        assert_eq!(array.nodes[0].component.pose().position().x, 0.0);
        assert_eq!(array.nodes[1].component.pose().position().x, 1.0);
        assert_eq!(array.nodes[2].component.pose().position().x, 2.0);
    }

    #[test]
    #[should_panic]
    fn test_observer_array_from_iter_too_short() {
        let sensors = (0..2).map(|i| {
            LinearHallSensor::new(
                [i as f64, 0.0, 0.0],
                UnitQuaternion::identity(),
                [0.0, 0.0, 1.0],
                30.0,
                3.3,
            )
        });

        let _: ObserverArray<LinearHallSensor, 3> = sensors.collect();
    }

    #[test]
    #[should_panic]
    fn test_observer_array_from_iter_too_long() {
        let sensors = (0..4).map(|i| {
            LinearHallSensor::new(
                [i as f64, 0.0, 0.0],
                UnitQuaternion::identity(),
                [0.0, 0.0, 1.0],
                30.0,
                3.3,
            )
        });

        let _: ObserverArray<LinearHallSensor, 3> = sensors.collect();
    }
}
