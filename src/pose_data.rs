use nalgebra::{Quaternion, Translation3, Unit, UnitQuaternion, Vector3};
use std::{
    convert::{AsMut, AsRef},
    ops::{Deref, DerefMut},
};

#[derive(Debug)]
pub struct PoseData(pub realsense_sys::rs2_pose);

impl PoseData {
    pub fn translation(&self) -> Translation3<f32> {
        let realsense_sys::rs2_vector { x, y, z } = self.0.translation;
        Translation3::new(x, y, z)
    }

    pub fn velocity(&self) -> Vector3<f32> {
        let realsense_sys::rs2_vector { x, y, z } = self.0.velocity;
        Vector3::new(x, y, z)
    }

    pub fn acceleration(&self) -> Vector3<f32> {
        let realsense_sys::rs2_vector { x, y, z } = self.0.acceleration;
        Vector3::new(x, y, z)
    }

    pub fn rotation(&self) -> UnitQuaternion<f32> {
        let realsense_sys::rs2_quaternion { x, y, z, w } = self.0.rotation;
        Unit::new_unchecked(Quaternion::new(w, x, z, y))
    }

    pub fn angular_velocity(&self) -> Vector3<f32> {
        let realsense_sys::rs2_vector { x, y, z } = self.0.angular_velocity;
        Vector3::new(x, y, z)
    }

    pub fn angular_acceleration(&self) -> Vector3<f32> {
        let realsense_sys::rs2_vector { x, y, z } = self.0.angular_acceleration;
        Vector3::new(x, y, z)
    }

    pub fn tracker_confidence(&self) -> u32 {
        self.0.tracker_confidence as u32
    }

    pub fn mapper_confidence(&self) -> u32 {
        self.0.mapper_confidence as u32
    }
}

impl Deref for PoseData {
    type Target = realsense_sys::rs2_pose;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for PoseData {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl AsRef<realsense_sys::rs2_pose> for PoseData {
    fn as_ref(&self) -> &realsense_sys::rs2_pose {
        &self.0
    }
}

impl AsMut<realsense_sys::rs2_pose> for PoseData {
    fn as_mut(&mut self) -> &mut realsense_sys::rs2_pose {
        &mut self.0
    }
}
