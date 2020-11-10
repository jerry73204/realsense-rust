//! Marker traits and types for [Pipeline](crate::pipeline::Pipeline).

use crate::{common::*, config::Config, pipeline_profile::PipelineProfile};

/// Marker trait for pipeline marker types.
pub trait PipelineState {
    /// Clone the state with the underlying pointer. It is intended for internal use only.
    ///
    /// # Safety
    /// You can to prevent [Drop::drop] to be called twice by calling this method.
    unsafe fn unsafe_clone(&self) -> Self;
}

/// A marker type indicating the [Pipeline](crate::pipeline::Pipeline) is started.
#[derive(Debug)]
pub struct Active {
    pub profile: PipelineProfile,
    pub config: Option<Config>,
}

impl PipelineState for Active {
    unsafe fn unsafe_clone(&self) -> Self {
        Self {
            profile: self.profile.unsafe_clone(),
            config: self.config.as_ref().map(|config| config.unsafe_clone()),
        }
    }
}

/// A marker type indicating the [Pipeline](crate::pipeline::Pipeline) is stopped.
#[derive(Debug)]
pub struct Inactive;

impl PipelineState for Inactive {
    unsafe fn unsafe_clone(&self) -> Self {
        Self
    }
}
