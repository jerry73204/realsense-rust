pub mod base;
mod common;
pub mod config;
pub mod context;
pub mod device;
pub mod device_hub;
pub mod device_list;
pub mod error;
pub mod frame;
pub mod frame_kind;
pub mod frame_queue;
pub mod kind;
pub mod options;
pub mod pipeline;
pub mod pipeline_kind;
pub mod pipeline_profile;
pub mod processing_block;
pub mod processing_block_kind;
pub mod processing_block_list;
pub mod sensor;
pub mod sensor_kind;
pub mod sensor_list;
pub mod stream_profile;
pub mod stream_profile_kind;
pub mod stream_profile_list;

/// The mod collects common used traits from this crate.
pub mod prelude {
    pub use crate::frame::{DepthFrameEx, DisparityFrameEx, GenericFrameEx, VideoFrameEx};
}

#[cfg(feature = "with-image")]
pub use base::Rs2Image;
pub use base::{Extrinsics, Intrinsics, MotionIntrinsics, PoseData, Resolution, StreamProfileData};
pub use config::Config;
pub use context::Context;
pub use device::Device;
pub use device_hub::DeviceHub;
pub use device_list::{DeviceList, DeviceListIntoIter};
pub use error::{Error, Result};
pub use frame::{
    CompositeFrameIntoIter, DepthFrame, DepthFrameEx, DisparityFrame, DisparityFrameEx,
    ExtendedFrame, Frame, GenericFrameEx, VideoFrame, VideoFrameEx,
};
pub use frame_queue::FrameQueue;
pub use kind::{
    CameraInfo, ColorScheme, Extension, Format, FrameMetaDataValue, HoleFillingMode,
    PersistenceControl, Rs2Option, StreamKind, TimestampDomain,
};
pub use options::{OptionHandle, ToOptions};
pub use pipeline::{ActivePipeline, InactivePipeline, Pipeline};
pub use pipeline_profile::PipelineProfile;
pub use processing_block::{
    Align, AnyProcessingBlock, Colorizer, DecimationFilter, DisparityFilter, HoleFillingFilter,
    HuffmanDepthDecompress, PointCloud, ProcessingBlock, RatesPrinter, SpatialFilter, Syncer,
    TemporalFilter, ThresholdFilter, UnitsTransform, YuyDecoder, ZeroOrderFilter,
};
pub use processing_block_list::{ProcessingBlockList, ProcessingBlockListIntoIter};
pub use sensor::{
    AnySensor, ColorSensor, DepthSensor, DepthStereoSensor, ExtendedSensor, FishEyeSensor,
    L500DepthSensor, MotionSensor, PoseSensor, Sensor, SoftwareSensor, Tm2Sensor,
};
pub use sensor_list::{SensorList, SensorListIntoIter};
pub use stream_profile::{
    AnyStreamProfile, MotionStreamProfile, PoseStreamProfile, StreamProfile, VideoStreamProfile,
};
pub use stream_profile_list::{StreamProfileList, StreamProfileListIntoIter};
