//! Marker traits and types for [Sensor](crate::sensor::Sensor).

use crate::{common::*, kind::Extension};

/// The marker traits of all kinds of sensor.
pub trait SensorKind {}

/// The marker traits of all kinds of sensor except [Any](Any).
pub trait NonAnySensorKind
where
    Self: SensorKind,
{
    const EXTENSION: Extension;
}

#[derive(Debug)]
pub struct Any;
impl SensorKind for Any {}

#[derive(Debug)]
pub struct Tm2;
impl SensorKind for Tm2 {}
impl NonAnySensorKind for Tm2 {
    const EXTENSION: Extension = Extension::Tm2Sensor;
}

#[derive(Debug)]
pub struct Pose;
impl SensorKind for Pose {}
impl NonAnySensorKind for Pose {
    const EXTENSION: Extension = Extension::PoseSensor;
}

#[derive(Debug)]
pub struct Color;
impl SensorKind for Color {}
impl NonAnySensorKind for Color {
    const EXTENSION: Extension = Extension::ColorSensor;
}

#[derive(Debug)]
pub struct Depth;
impl SensorKind for Depth {}
impl NonAnySensorKind for Depth {
    const EXTENSION: Extension = Extension::DepthSensor;
}

#[derive(Debug)]
pub struct Motion;
impl SensorKind for Motion {}
impl NonAnySensorKind for Motion {
    const EXTENSION: Extension = Extension::MotionSensor;
}

#[derive(Debug)]
pub struct FishEye;
impl SensorKind for FishEye {}
impl NonAnySensorKind for FishEye {
    const EXTENSION: Extension = Extension::FishEyeSensor;
}

#[derive(Debug)]
pub struct Software;
impl SensorKind for Software {}
impl NonAnySensorKind for Software {
    const EXTENSION: Extension = Extension::SoftwareSensor;
}

#[derive(Debug)]
pub struct L500Depth;
impl SensorKind for L500Depth {}
impl NonAnySensorKind for L500Depth {
    const EXTENSION: Extension = Extension::L500DepthSensor;
}

#[derive(Debug)]
pub struct DepthStereo;
impl SensorKind for DepthStereo {}
impl NonAnySensorKind for DepthStereo {
    const EXTENSION: Extension = Extension::DepthStereoSensor;
}
