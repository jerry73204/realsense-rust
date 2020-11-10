//! Marker traits and types for [StreamProfile](crate::stream_profile::StreamProfile).

use crate::{common::*, kind::Extension};

/// The marker traits of all kinds of StreamProfile.
pub trait StreamProfileKind {}

/// The marker traits of all kinds of StreamProfile except [Any](Any).
pub trait NonAnyStreamProfileKind
where
    Self: StreamProfileKind,
{
    const EXTENSION: Extension;
}

#[derive(Debug)]
pub struct Any;
impl StreamProfileKind for Any {}

#[derive(Debug)]
pub struct Video;
impl StreamProfileKind for Video {}
impl NonAnyStreamProfileKind for Video {
    const EXTENSION: Extension = Extension::VideoProfile;
}

#[derive(Debug)]
pub struct Motion;
impl StreamProfileKind for Motion {}
impl NonAnyStreamProfileKind for Motion {
    const EXTENSION: Extension = Extension::MotionProfile;
}

#[derive(Debug)]
pub struct Pose;
impl StreamProfileKind for Pose {}
impl NonAnyStreamProfileKind for Pose {
    const EXTENSION: Extension = Extension::PoseProfile;
}
