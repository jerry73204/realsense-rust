//! Defines the frame type including sensor data.

#[cfg(feature = "with-image")]
use crate::base::Rs2Image;
use crate::{
    base::{PoseData, Resolution, StreamProfileData},
    common::*,
    error::{ErrorChecker, Result as RsResult},
    kind::{Extension, Format, FrameMetaDataValue, StreamKind, TimestampDomain},
    sensor::{marker as sensor_marker, Sensor},
    stream_profile::{marker as stream_marker, StreamProfile},
};

/// Marker types and traits for [Frame].
pub mod marker {
    use super::*;

    /// The marker trait for frame kinds.
    pub trait FrameKind {}

    /// The marker traits for frame kinds except [Any](Any).
    pub trait NonAnyFrameKind
    where
        Self: FrameKind,
    {
        const EXTENSION: Extension;
    }

    #[derive(Debug)]
    pub struct Composite;

    impl FrameKind for Composite {}
    impl NonAnyFrameKind for Composite {
        const EXTENSION: Extension = Extension::CompositeFrame;
    }

    #[derive(Debug)]
    pub struct Any;

    impl FrameKind for Any {}

    #[derive(Debug)]
    pub struct Video;

    impl FrameKind for Video {}
    impl NonAnyFrameKind for Video {
        const EXTENSION: Extension = Extension::VideoFrame;
    }

    #[derive(Debug)]
    pub struct Motion;

    impl FrameKind for Motion {}
    impl NonAnyFrameKind for Motion {
        const EXTENSION: Extension = Extension::MotionFrame;
    }

    #[derive(Debug)]
    pub struct Depth;

    impl FrameKind for Depth {}
    impl NonAnyFrameKind for Depth {
        const EXTENSION: Extension = Extension::DepthFrame;
    }

    #[derive(Debug)]
    pub struct Disparity;

    impl FrameKind for Disparity {}
    impl NonAnyFrameKind for Disparity {
        const EXTENSION: Extension = Extension::DisparityFrame;
    }

    #[derive(Debug)]
    pub struct Pose;

    impl FrameKind for Pose {}
    impl NonAnyFrameKind for Pose {
        const EXTENSION: Extension = Extension::PoseFrame;
    }

    #[derive(Debug)]
    pub struct Points;

    impl FrameKind for Points {}
    impl NonAnyFrameKind for Points {
        const EXTENSION: Extension = Extension::Points;
    }
}

/// The trait provides common methods on frames of all kinds.
pub trait GenericFrame
where
    Self: Sized,
{
    /// Obtains the metadata of frame.
    fn metadata(&self, kind: FrameMetaDataValue) -> RsResult<u64> {
        unsafe {
            let mut checker = ErrorChecker::new();
            let val = realsense_sys::rs2_get_frame_metadata(
                self.ptr().as_ptr(),
                kind as realsense_sys::rs2_frame_metadata_value,
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            Ok(val as u64)
        }
    }

    /// Gets frame number.
    fn number(&self) -> RsResult<u64> {
        unsafe {
            let mut checker = ErrorChecker::new();
            let val =
                realsense_sys::rs2_get_frame_number(self.ptr().as_ptr(), checker.inner_mut_ptr());
            checker.check()?;
            Ok(val as u64)
        }
    }

    /// Gets raw data size in bytes.
    fn data_size(&self) -> RsResult<usize> {
        unsafe {
            let mut checker = ErrorChecker::new();
            let val = realsense_sys::rs2_get_frame_data_size(
                self.ptr().as_ptr(),
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            Ok(val as usize)
        }
    }

    /// Gets the timestamp.
    fn timestamp(&self) -> RsResult<f64> {
        unsafe {
            let mut checker = ErrorChecker::new();
            let val = realsense_sys::rs2_get_frame_timestamp(
                self.ptr().as_ptr(),
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            Ok(val as f64)
        }
    }

    /// Gets the domain of timestamp.
    fn timestamp_domain(&self) -> RsResult<TimestampDomain> {
        let val = unsafe {
            let mut checker = ErrorChecker::new();
            let val = realsense_sys::rs2_get_frame_timestamp_domain(
                self.ptr().as_ptr(),
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            val
        };
        let domain = TimestampDomain::from_u32(val).unwrap();
        Ok(domain)
    }

    /// Gets raw data bytes in frame.
    fn data(&self) -> RsResult<&[u8]> {
        let size = self.data_size()?;
        let slice = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr =
                realsense_sys::rs2_get_frame_data(self.ptr().as_ptr(), checker.inner_mut_ptr());
            checker.check()?;
            std::slice::from_raw_parts::<u8>(ptr.cast::<u8>(), size)
        };
        Ok(slice)
    }

    /// Gets the relating sensor instance.
    fn sensor(&self) -> RsResult<Sensor<sensor_marker::Any>> {
        let sensor = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr =
                realsense_sys::rs2_get_frame_sensor(self.ptr().as_ptr(), checker.inner_mut_ptr());
            checker.check()?;
            Sensor::from_ptr(NonNull::new(ptr).unwrap())
        };
        Ok(sensor)
    }

    /// Gets the relating stream profile.
    fn stream_profile(&self) -> RsResult<StreamProfile<stream_marker::Any>> {
        let profile = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_get_frame_stream_profile(
                self.ptr().as_ptr(),
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            StreamProfile::from_parts(NonNull::new(ptr as *mut _).unwrap(), false)
        };
        Ok(profile)
    }

    fn try_clone(&self) -> RsResult<Self> {
        unsafe {
            // add reference
            let mut checker = ErrorChecker::new();
            realsense_sys::rs2_frame_add_ref(self.ptr().as_ptr(), checker.inner_mut_ptr());
            checker.check()?;
            Ok(Self::from_ptr(self.ptr()))
        }
    }

    fn ptr(&self) -> NonNull<realsense_sys::rs2_frame>;

    /// Destruct and return the raw pointer. It is intended for internal use only.
    ///
    /// # Safety
    /// You have to manage the lifetime of internal pointer by calling this method.
    unsafe fn take(self) -> NonNull<realsense_sys::rs2_frame>;

    /// Construct from a raw pointer. It is intended for internal use only.
    ///
    /// # Safety
    /// You have to ensure the pointer is valid.
    unsafe fn from_ptr(ptr: NonNull<realsense_sys::rs2_frame>) -> Self;
}

/// The trait provides methods on frames with video data.
pub trait VideoFrame
where
    Self: GenericFrame,
{
    /// Gets image resolution.
    fn resolution(&self) -> RsResult<Resolution> {
        let width = self.width()?;
        let height = self.height()?;
        let resolution = Resolution { width, height };
        Ok(resolution)
    }

    /// Gets image width in pixels.
    fn width(&self) -> RsResult<usize> {
        unsafe {
            let mut checker = ErrorChecker::new();
            let val =
                realsense_sys::rs2_get_frame_width(self.ptr().as_ptr(), checker.inner_mut_ptr());
            checker.check()?;
            Ok(val as usize)
        }
    }

    /// Gets image height in pixels.
    fn height(&self) -> RsResult<usize> {
        unsafe {
            let mut checker = ErrorChecker::new();
            let val =
                realsense_sys::rs2_get_frame_height(self.ptr().as_ptr(), checker.inner_mut_ptr());
            checker.check()?;
            Ok(val as usize)
        }
    }

    /// Gets image row stride in bytes.
    fn stride_in_bytes(&self) -> RsResult<usize> {
        unsafe {
            let mut checker = ErrorChecker::new();
            let val = realsense_sys::rs2_get_frame_stride_in_bytes(
                self.ptr().as_ptr(),
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            Ok(val as usize)
        }
    }

    /// Gets the size of pixel in bits.
    fn bits_per_pixel(&self) -> RsResult<usize> {
        unsafe {
            let mut checker = ErrorChecker::new();
            let val = realsense_sys::rs2_get_frame_bits_per_pixel(
                self.ptr().as_ptr(),
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            Ok(val as usize)
        }
    }

    /// Gets color image buffer referencing underlying raw data.
    #[cfg(feature = "with-image")]
    fn image(&self) -> RsResult<Rs2Image> {
        let StreamProfileData { format, .. } = self.stream_profile()?.get_data()?;
        let raw_data = self.data()?;
        let Resolution { width, height } = self.resolution()?;
        let stride_in_bytes = self.stride_in_bytes()?;
        debug_assert_eq!(raw_data.len() % stride_in_bytes, 0, "please report bug");

        let image = match format {
            Format::Bgr8 => {
                let channels = 3;

                let sample_size = std::mem::size_of::<u8>();
                debug_assert_eq!(stride_in_bytes % sample_size, 0, "please report bug");

                let stride_in_samples = stride_in_bytes / sample_size;
                debug_assert_eq!(
                    raw_data.len(),
                    stride_in_samples * height,
                    "please report bug"
                );
                debug_assert!(width * channels <= stride_in_samples, "please report bug");

                let flat = FlatSamples {
                    samples: raw_data,
                    layout: SampleLayout {
                        channels: channels as u8,
                        width: width as u32,
                        height: height as u32,
                        channel_stride: 1,
                        width_stride: channels,
                        height_stride: stride_in_samples,
                    },
                    color_hint: Some(ColorType::Bgr8),
                };
                let image = flat.try_into_buffer().unwrap();
                Rs2Image::Bgr8(image)
            }
            Format::Bgra8 => {
                let channels = 4;

                let sample_size = std::mem::size_of::<u8>();
                debug_assert_eq!(stride_in_bytes % sample_size, 0, "please report bug");

                let stride_in_samples = stride_in_bytes / sample_size;
                debug_assert_eq!(
                    raw_data.len(),
                    stride_in_samples * height,
                    "please report bug"
                );
                debug_assert!(width * channels <= stride_in_samples, "please report bug");

                let flat = FlatSamples {
                    samples: raw_data,
                    layout: SampleLayout {
                        channels: channels as u8,
                        width: width as u32,
                        height: height as u32,
                        channel_stride: 1,
                        width_stride: channels,
                        height_stride: stride_in_samples,
                    },
                    color_hint: Some(ColorType::Bgra8),
                };
                let image = flat.try_into_buffer().unwrap();
                Rs2Image::Bgra8(image)
            }
            Format::Rgb8 => {
                let channels = 3;

                let sample_size = std::mem::size_of::<u8>();
                debug_assert_eq!(stride_in_bytes % sample_size, 0, "please report bug");

                let stride_in_samples = stride_in_bytes / sample_size;
                debug_assert_eq!(
                    raw_data.len(),
                    stride_in_samples * height,
                    "please report bug"
                );
                debug_assert!(width * channels <= stride_in_samples, "please report bug");

                let flat = FlatSamples {
                    samples: raw_data,
                    layout: SampleLayout {
                        channels: channels as u8,
                        width: width as u32,
                        height: height as u32,
                        channel_stride: 1,
                        width_stride: channels,
                        height_stride: stride_in_samples,
                    },
                    color_hint: Some(ColorType::Rgb8),
                };
                let image = flat.try_into_buffer().unwrap();
                Rs2Image::Rgb8(image)
            }
            Format::Rgba8 => {
                let channels = 4;

                let sample_size = std::mem::size_of::<u8>();
                debug_assert_eq!(stride_in_bytes % sample_size, 0, "please report bug");

                let stride_in_samples = stride_in_bytes / sample_size;
                debug_assert_eq!(
                    raw_data.len(),
                    stride_in_samples * height,
                    "please report bug"
                );
                debug_assert!(width * channels <= stride_in_samples, "please report bug");

                let flat = FlatSamples {
                    samples: raw_data,
                    layout: SampleLayout {
                        channels: channels as u8,
                        width: width as u32,
                        height: height as u32,
                        channel_stride: 1,
                        width_stride: channels,
                        height_stride: stride_in_samples,
                    },
                    color_hint: Some(ColorType::Rgba8),
                };
                let image = flat.try_into_buffer().unwrap();
                Rs2Image::Rgba8(image)
            }
            Format::Z16 => {
                let sample_size = std::mem::size_of::<u16>();
                debug_assert_eq!(stride_in_bytes % sample_size, 0, "please report bug");

                let depth_data: &[u16] =
                    safe_transmute::transmute_many::<u16, PedanticGuard>(raw_data).unwrap();

                let stride_in_samples = stride_in_bytes / sample_size;
                debug_assert_eq!(
                    depth_data.len(),
                    stride_in_samples * height,
                    "please report bug"
                );
                debug_assert!(width <= stride_in_samples, "please report bug");

                let flat = FlatSamples {
                    samples: depth_data,
                    layout: SampleLayout {
                        channels: 1,
                        width: width as u32,
                        height: height as u32,
                        channel_stride: 1,
                        width_stride: 1,
                        height_stride: stride_in_bytes / sample_size,
                    },
                    color_hint: Some(ColorType::L16),
                };
                let image = flat.try_into_buffer().unwrap();
                Rs2Image::Luma16(image)
            }
            _ => unreachable!("unsupported format. please report bug"),
        };

        Ok(image)
    }
}

/// The trait provides methods on frames with depth data.
///
/// Frame types with this trait also implements [VideoFrame](VideoFrame) trait.
pub trait DepthFrame
where
    Self: VideoFrame,
{
    /// Gets distance at given coordinates.
    fn distance(&self, x: usize, y: usize) -> RsResult<f32> {
        let distance = unsafe {
            let mut checker = ErrorChecker::new();
            let distance = realsense_sys::rs2_depth_frame_get_distance(
                self.ptr().as_ptr(),
                x as c_int,
                y as c_int,
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            distance
        };
        Ok(distance)
    }

    /// Gets the length in meter per distance unit.
    fn depth_units(&self) -> RsResult<f32> {
        let sensor = self.sensor()?;
        let sensor = sensor.try_extend_to::<sensor_marker::Depth>()?.unwrap();
        let depth_units = sensor.depth_units()?;
        Ok(depth_units)
    }
}

/// The trait provides methods on frames with disparity data.
///
/// Frame types with this trait also implements [DepthFrame](DepthFrame) trait.
pub trait DisparityFrame
where
    Self: DepthFrame,
{
    /// Retrieves the distance between the two IR sensors.
    fn baseline(&self) -> RsResult<f32> {
        unsafe {
            let mut checker = ErrorChecker::new();
            let baseline = realsense_sys::rs2_depth_stereo_frame_get_baseline(
                self.ptr().as_ptr(),
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            Ok(baseline)
        }
    }
}

/// The type returned by [Frame::<Any>::try_extend](Frame::try_extend).
///
/// It enumerates all possible frame extensions. If the frame failed to
/// extend any one of the kind, it falls back to [ExtendedFrame::Other](ExtendedFrame::Other) variant.
#[derive(Debug)]
pub enum ExtendedFrame {
    Points(Frame<marker::Points>),
    Composite(Frame<marker::Composite>),
    Video(Frame<marker::Video>),
    Depth(Frame<marker::Depth>),
    Disparity(Frame<marker::Disparity>),
    Motion(Frame<marker::Motion>),
    Pose(Frame<marker::Pose>),
    Other(Frame<marker::Any>),
}

/// Represents a collection of sensor data.
#[derive(Debug)]
pub struct Frame<Kind>
where
    Kind: marker::FrameKind,
{
    pub(crate) ptr: NonNull<realsense_sys::rs2_frame>,
    _phantom: PhantomData<Kind>,
}

impl<Kind> GenericFrame for Frame<Kind>
where
    Kind: marker::FrameKind,
{
    fn ptr(&self) -> NonNull<realsense_sys::rs2_frame> {
        self.ptr
    }

    unsafe fn take(self) -> NonNull<realsense_sys::rs2_frame> {
        let ptr = self.ptr;
        std::mem::forget(self);
        ptr
    }

    unsafe fn from_ptr(ptr: NonNull<realsense_sys::rs2_frame>) -> Self {
        Self {
            ptr,
            _phantom: PhantomData,
        }
    }
}

impl VideoFrame for Frame<marker::Video> {}

impl VideoFrame for Frame<marker::Depth> {}

impl DepthFrame for Frame<marker::Depth> {}

impl VideoFrame for Frame<marker::Disparity> {}

impl DepthFrame for Frame<marker::Disparity> {}

impl DisparityFrame for Frame<marker::Disparity> {}

impl<Kind> Frame<Kind> where Kind: marker::FrameKind {}

impl Frame<marker::Any> {
    pub fn is_extendable_to<Kind>(&self) -> RsResult<bool>
    where
        Kind: marker::NonAnyFrameKind,
    {
        unsafe {
            let mut checker = ErrorChecker::new();
            let val = realsense_sys::rs2_is_frame_extendable_to(
                self.ptr.as_ptr(),
                Kind::EXTENSION as realsense_sys::rs2_extension,
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            Ok(val != 0)
        }
    }

    pub fn try_extend_to<Kind>(self) -> RsResult<Result<Frame<Kind>, Self>>
    where
        Kind: marker::NonAnyFrameKind,
    {
        unsafe {
            let is_extendable = self.is_extendable_to::<Kind>()?;
            if is_extendable {
                let ptr = self.take();
                let frame = Frame {
                    ptr,
                    _phantom: PhantomData,
                };
                Ok(Ok(frame))
            } else {
                Ok(Err(self))
            }
        }
    }

    pub fn try_extend(self) -> RsResult<ExtendedFrame> {
        let frame_any = self;

        let frame_any = match frame_any.try_extend_to::<marker::Points>()? {
            Ok(frame) => return Ok(ExtendedFrame::Points(frame)),
            Err(frame) => frame,
        };

        let frame_any = match frame_any.try_extend_to::<marker::Composite>()? {
            Ok(frame) => return Ok(ExtendedFrame::Composite(frame)),
            Err(frame) => frame,
        };

        let frame_any = match frame_any.try_extend_to::<marker::Motion>()? {
            Ok(frame) => return Ok(ExtendedFrame::Motion(frame)),
            Err(frame) => frame,
        };

        let frame_any = match frame_any.try_extend_to::<marker::Pose>()? {
            Ok(frame) => return Ok(ExtendedFrame::Pose(frame)),
            Err(frame) => frame,
        };

        let frame_any = match frame_any.try_extend_to::<marker::Disparity>()? {
            Ok(frame) => return Ok(ExtendedFrame::Disparity(frame)),
            Err(frame) => frame,
        };

        let frame_any = match frame_any.try_extend_to::<marker::Depth>()? {
            Ok(frame) => return Ok(ExtendedFrame::Depth(frame)),
            Err(frame) => frame,
        };

        let frame_any = match frame_any.try_extend_to::<marker::Video>()? {
            Ok(frame) => return Ok(ExtendedFrame::Video(frame)),
            Err(frame) => frame,
        };

        Ok(ExtendedFrame::Other(frame_any))
    }
}

impl Frame<marker::Composite> {
    /// Gets the number of frames included in the composite frame.
    pub fn len(&self) -> RsResult<usize> {
        let len = unsafe {
            let mut checker = ErrorChecker::new();
            let len = realsense_sys::rs2_embedded_frames_count(
                self.ptr.as_ptr(),
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            len as usize
        };
        Ok(len)
    }

    /// Checks if the composite-frame contains no sub-frames.
    pub fn is_empty(&self) -> RsResult<bool> {
        Ok(self.len()? == 0)
    }

    /// Gets the frame in frameset by index.
    ///
    /// The method throws error if index is out of bound given by [Frame::len].
    pub fn get(&self, index: usize) -> RsResult<Option<Frame<marker::Any>>> {
        let len = self.len()?;
        if index >= len {
            return Ok(None);
        }

        let frame = unsafe {
            // extract frame
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_extract_frame(
                self.ptr.as_ptr(),
                index as c_int,
                checker.inner_mut_ptr(),
            );
            checker.check()?;

            Frame::from_ptr(NonNull::new(ptr).unwrap())
        };
        Ok(Some(frame))
    }

    /// Unpacks the set of frames and turns into iterable [CompositeFrameIntoIter] instance.
    pub fn try_into_iter(self) -> RsResult<CompositeFrameIntoIter> {
        let len = self.len()?;
        let ptr = unsafe { self.take() };
        let iter = CompositeFrameIntoIter {
            index: 0,
            len,
            ptr,
            fused: len == 0,
        };
        Ok(iter)
    }

    pub fn try_iter(&self) -> RsResult<CompositeFrameIter> {
        let len = self.len()?;
        let iter = CompositeFrameIter {
            index: 0,
            len,
            ptr: self.ptr,
            fused: len == 0,
        };
        Ok(iter)
    }

    pub fn first_of<Kind>(&self, stream: StreamKind) -> RsResult<Option<Frame<Kind>>>
    where
        Kind: marker::NonAnyFrameKind,
    {
        for result in self.try_iter()? {
            let frame_any = result?;
            if let Ok(frame) = frame_any.try_extend_to::<Kind>()? {
                if frame.stream_profile()?.get_data()?.stream == stream {
                    return Ok(Some(frame));
                }
            }
        }
        Ok(None)
    }

    pub fn color_frame(&self) -> RsResult<Option<Frame<marker::Video>>> {
        self.first_of::<marker::Video>(StreamKind::Color)
    }

    pub fn depth_frame(&self) -> RsResult<Option<Frame<marker::Depth>>> {
        self.first_of::<marker::Depth>(StreamKind::Depth)
    }

    pub fn pose_frame(&self) -> RsResult<Option<Frame<marker::Pose>>> {
        self.first_of::<marker::Pose>(StreamKind::Pose)
    }
}

impl Frame<marker::Pose> {
    /// Gets the pose data.
    pub fn pose(&self) -> RsResult<PoseData> {
        let pose_data = unsafe {
            let mut checker = ErrorChecker::new();
            let mut pose_data = MaybeUninit::uninit();
            realsense_sys::rs2_pose_frame_get_pose_data(
                self.ptr.as_ptr(),
                pose_data.as_mut_ptr(),
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            pose_data.assume_init()
        };

        let pose = PoseData(pose_data);
        Ok(pose)
    }
}

/// UV texture coordinates.
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct texture_coordinate {
    pub u: f32,
    pub v: f32,
}

impl Frame<marker::Points> {
    /// Gets vertices of point cloud.
    pub fn vertices<'a>(&'a self) -> RsResult<&'a [realsense_sys::rs2_vertex]> {
        let n_points = self.points_count()?;
        unsafe {
            let mut checker = ErrorChecker::new();
            let ptr =
                realsense_sys::rs2_get_frame_vertices(self.ptr.as_ptr(), checker.inner_mut_ptr());
            checker.check()?;
            let slice = std::slice::from_raw_parts::<realsense_sys::rs2_vertex>(ptr, n_points);
            Ok(slice)
        }
    }

    /// Gets texture coordinates of each point of point cloud.
    pub fn texture_coordinates<'a>(&'a self) -> RsResult<&'a [texture_coordinate]> {
        unsafe {
            let n_points = self.points_count()?;
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_get_frame_texture_coordinates(
                self.ptr.as_ptr(),
                checker.inner_mut_ptr(),
            ) as *const texture_coordinate;
            checker.check()?;
            let slice = std::slice::from_raw_parts::<texture_coordinate>(ptr, n_points);
            Ok(slice)
        }
    }

    /// Gets number of points in frame.
    pub fn points_count(&self) -> RsResult<usize> {
        unsafe {
            let mut checker = ErrorChecker::new();
            let val = realsense_sys::rs2_get_frame_points_count(
                self.ptr.as_ptr(),
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            Ok(val as usize)
        }
    }
}

impl Frame<marker::Motion> {
    /// Gets motion data.
    pub fn motion(&self) -> RsResult<[f32; 3]> {
        let slice = safe_transmute::transmute_many::<f32, PedanticGuard>(self.data()?).unwrap();
        match *slice {
            [x, y, z] => Ok([x, y, z]),
            _ => unreachable!("please report bug"),
        }
    }
}

impl IntoIterator for Frame<marker::Composite> {
    type Item = RsResult<Frame<marker::Any>>;
    type IntoIter = CompositeFrameIntoIter;

    /// The method internally calls [Frame::try_into_iter](Frame::try_into_iter).
    ///
    /// # Panics
    /// This method panics if [Frame::try_into_iter](Frame::try_into_iter) returns [Err](Result::Err).
    ///
    fn into_iter(self) -> Self::IntoIter {
        self.try_into_iter().unwrap()
    }
}

impl<Kind> Clone for Frame<Kind>
where
    Kind: marker::FrameKind,
{
    fn clone(&self) -> Self {
        self.try_clone().unwrap()
    }
}

impl<Kind> Drop for Frame<Kind>
where
    Kind: marker::FrameKind,
{
    fn drop(&mut self) {
        unsafe {
            realsense_sys::rs2_release_frame(self.ptr.as_ptr());
        }
    }
}

unsafe impl<Kind> Send for Frame<Kind> where Kind: marker::FrameKind {}
unsafe impl<Kind> Sync for Frame<Kind> where Kind: marker::FrameKind {}

/// The iterator type returned by [Frame::try_into_iter](Frame::try_into_iter).
#[derive(Debug)]
pub struct CompositeFrameIntoIter {
    len: usize,
    index: usize,
    ptr: NonNull<realsense_sys::rs2_frame>,
    fused: bool,
}

impl Iterator for CompositeFrameIntoIter {
    type Item = RsResult<Frame<marker::Any>>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.fused {
            return None;
        }

        let ptr = unsafe {
            // extract frame
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_extract_frame(
                self.ptr.as_ptr(),
                self.index as c_int,
                checker.inner_mut_ptr(),
            );
            if let Err(err) = checker.check() {
                self.fused = true;
                return Some(Err(err));
            }
            ptr
        };

        self.index += 1;
        if self.index >= self.len {
            self.fused = true;
        }

        let frame = unsafe { Frame::from_ptr(NonNull::new(ptr).unwrap()) };
        Some(Ok(frame))
    }
}

impl FusedIterator for CompositeFrameIntoIter {}

impl Drop for CompositeFrameIntoIter {
    fn drop(&mut self) {
        unsafe {
            realsense_sys::rs2_release_frame(self.ptr.as_ptr());
        }
    }
}

/// The iterator type returned by [Frame::try_iter](Frame::try_iter).
#[derive(Debug)]
pub struct CompositeFrameIter {
    len: usize,
    index: usize,
    ptr: NonNull<realsense_sys::rs2_frame>,
    fused: bool,
}

impl Iterator for CompositeFrameIter {
    type Item = RsResult<Frame<marker::Any>>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.fused {
            return None;
        }

        let ptr = unsafe {
            // extract frame
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_extract_frame(
                self.ptr.as_ptr(),
                self.index as c_int,
                checker.inner_mut_ptr(),
            );
            if let Err(err) = checker.check() {
                self.fused = true;
                return Some(Err(err));
            }
            ptr
        };

        self.index += 1;
        if self.index >= self.len {
            self.fused = true;
        }

        let frame = unsafe { Frame::from_ptr(NonNull::new(ptr).unwrap()) };
        Some(Ok(frame))
    }
}

impl FusedIterator for CompositeFrameIter {}
