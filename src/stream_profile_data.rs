use crate::kind::{Format, StreamKind};

/// Represents the specification of a stream.
#[derive(Debug)]
pub struct StreamProfileData {
    pub stream: StreamKind,
    pub format: Format,
    pub index: usize,
    pub unique_id: i32,
    pub framerate: i32,
}
