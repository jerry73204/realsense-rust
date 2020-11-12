//! Defines the frame type including sensor data.

#[cfg(feature = "with-image")]
use crate::base::Rs2Image;
use crate::{
    base::{PoseData, Resolution, StreamProfileData},
    common::*,
    error::{ErrorChecker, Result},
    frame_kind,
    kind::{Format, FrameMetaDataValue, StreamKind, TimestampDomain},
    sensor::{AnySensor, DepthSensor},
    stream_profile::{AnyStreamProfile, StreamProfile},
};

/// The trait provides common methods on frames of all kinds.
pub trait GenericFrameEx
where
    Self: Sized,
{
    /// Obtains the metadata of frame.
    fn metadata(&self, kind: FrameMetaDataValue) -> Result<u64> {
        unsafe {
            let mut checker = ErrorChecker::new();
            let val = sys::rs2_get_frame_metadata(
                self.ptr().as_ptr(),
                kind as sys::rs2_frame_metadata_value,
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            Ok(val as u64)
        }
    }

    /// Gets frame number.
    fn number(&self) -> Result<u64> {
        unsafe {
            let mut checker = ErrorChecker::new();
            let val = sys::rs2_get_frame_number(self.ptr().as_ptr(), checker.inner_mut_ptr());
            checker.check()?;
            Ok(val as u64)
        }
    }

    /// Gets raw data size in bytes.
    fn data_size(&self) -> Result<usize> {
        unsafe {
            let mut checker = ErrorChecker::new();
            let val = sys::rs2_get_frame_data_size(self.ptr().as_ptr(), checker.inner_mut_ptr());
            checker.check()?;
            Ok(val as usize)
        }
    }

    /// Gets the timestamp.
    fn timestamp(&self) -> Result<f64> {
        unsafe {
            let mut checker = ErrorChecker::new();
            let val = sys::rs2_get_frame_timestamp(self.ptr().as_ptr(), checker.inner_mut_ptr());
            checker.check()?;
            Ok(val as f64)
        }
    }

    /// Gets the domain of timestamp.
    fn timestamp_domain(&self) -> Result<TimestampDomain> {
        let val = unsafe {
            let mut checker = ErrorChecker::new();
            let val =
                sys::rs2_get_frame_timestamp_domain(self.ptr().as_ptr(), checker.inner_mut_ptr());
            checker.check()?;
            val
        };
        let domain = TimestampDomain::from_u32(val).unwrap();
        Ok(domain)
    }

    /// Gets raw data bytes in frame.
    fn data(&self) -> Result<&[u8]> {
        let size = self.data_size()?;
        let slice = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = sys::rs2_get_frame_data(self.ptr().as_ptr(), checker.inner_mut_ptr());
            checker.check()?;
            slice::from_raw_parts::<u8>(ptr.cast::<u8>(), size)
        };
        Ok(slice)
    }

    /// Gets the relating sensor instance.
    fn sensor(&self) -> Result<AnySensor> {
        let sensor = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = sys::rs2_get_frame_sensor(self.ptr().as_ptr(), checker.inner_mut_ptr());
            checker.check()?;
            AnySensor::from_raw(ptr)
        };
        Ok(sensor)
    }

    /// Gets the relating stream profile.
    fn stream_profile(&self) -> Result<AnyStreamProfile> {
        let profile = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr =
                sys::rs2_get_frame_stream_profile(self.ptr().as_ptr(), checker.inner_mut_ptr());
            checker.check()?;
            StreamProfile::from_raw_parts(ptr as *mut _, false)
        };
        Ok(profile)
    }

    fn try_clone(&self) -> Result<Self> {
        unsafe {
            // add reference
            let mut checker = ErrorChecker::new();
            sys::rs2_frame_add_ref(self.ptr().as_ptr(), checker.inner_mut_ptr());
            checker.check()?;
            Ok(Self::from_raw(self.ptr().as_ptr()))
        }
    }

    #[doc(hidden)]
    fn ptr(&self) -> NonNull<sys::rs2_frame>;

    /// Destruct and return the raw pointer. It is intended for internal use only.
    ///
    /// # Safety
    /// You have to manage the lifetime of internal pointer by calling this method.
    fn into_raw(self) -> *mut sys::rs2_frame;

    /// Construct from a raw pointer. It is intended for internal use only.
    ///
    /// # Safety
    /// You have to ensure the pointer is valid.
    unsafe fn from_raw(ptr: *mut sys::rs2_frame) -> Self;
}

/// The trait provides methods on frames with video data.
pub trait VideoFrameEx
where
    Self: GenericFrameEx,
{
    /// Gets image resolution.
    fn resolution(&self) -> Result<Resolution> {
        let width = self.width()?;
        let height = self.height()?;
        let resolution = Resolution { width, height };
        Ok(resolution)
    }

    /// Gets image width in pixels.
    fn width(&self) -> Result<usize> {
        unsafe {
            let mut checker = ErrorChecker::new();
            let val = sys::rs2_get_frame_width(self.ptr().as_ptr(), checker.inner_mut_ptr());
            checker.check()?;
            Ok(val as usize)
        }
    }

    /// Gets image height in pixels.
    fn height(&self) -> Result<usize> {
        unsafe {
            let mut checker = ErrorChecker::new();
            let val = sys::rs2_get_frame_height(self.ptr().as_ptr(), checker.inner_mut_ptr());
            checker.check()?;
            Ok(val as usize)
        }
    }

    /// Gets image row stride in bytes.
    fn stride_in_bytes(&self) -> Result<usize> {
        unsafe {
            let mut checker = ErrorChecker::new();
            let val =
                sys::rs2_get_frame_stride_in_bytes(self.ptr().as_ptr(), checker.inner_mut_ptr());
            checker.check()?;
            Ok(val as usize)
        }
    }

    /// Gets the size of pixel in bits.
    fn bits_per_pixel(&self) -> Result<usize> {
        unsafe {
            let mut checker = ErrorChecker::new();
            let val =
                sys::rs2_get_frame_bits_per_pixel(self.ptr().as_ptr(), checker.inner_mut_ptr());
            checker.check()?;
            Ok(val as usize)
        }
    }

    /// Gets color image buffer referencing underlying raw data.
    #[cfg(feature = "with-image")]
    fn ref_image(&self) -> Result<Rs2Image> {
        let StreamProfileData { format, .. } = self.stream_profile()?.get_data()?;
        let raw_data = self.data()?;
        let Resolution { width, height } = self.resolution()?;
        let stride_in_bytes = self.stride_in_bytes()?;
        debug_assert_eq!(raw_data.len() % stride_in_bytes, 0, "please report bug");

        let image = match format {
            Format::Bgr8 => {
                let channels = 3;

                let sample_size = mem::size_of::<u8>();
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

                let sample_size = mem::size_of::<u8>();
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

                let sample_size = mem::size_of::<u8>();
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

                let sample_size = mem::size_of::<u8>();
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
                let sample_size = mem::size_of::<u16>();
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

    /// Builds an owned image. Note that it incurs memory copy.
    #[cfg(feature = "with-image")]
    fn owned_image(&self) -> Result<DynamicImage> {
        Ok(self.ref_image()?.into())
    }
}

/// The trait provides methods on frames with depth data.
///
/// Frame types with this trait also implements [VideoFrame](VideoFrame) trait.
pub trait DepthFrameEx
where
    Self: VideoFrameEx,
{
    /// Gets distance at given coordinates.
    fn distance(&self, x: usize, y: usize) -> Result<f32> {
        let distance = unsafe {
            let mut checker = ErrorChecker::new();
            let distance = sys::rs2_depth_frame_get_distance(
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
    fn depth_units(&self) -> Result<f32> {
        let sensor = self.sensor()?;
        let sensor: DepthSensor = sensor.try_extend_to()?.unwrap();
        let depth_units = sensor.depth_units()?;
        Ok(depth_units)
    }
}

/// The trait provides methods on frames with disparity data.
///
/// Frame types with this trait also implements [DepthFrame](DepthFrame) trait.
pub trait DisparityFrameEx
where
    Self: DepthFrameEx,
{
    /// Retrieves the distance between the two IR sensors.
    fn baseline(&self) -> Result<f32> {
        unsafe {
            let mut checker = ErrorChecker::new();
            let baseline = sys::rs2_depth_stereo_frame_get_baseline(
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
    Points(PointsFrame),
    Composite(CompositeFrame),
    Video(VideoFrame),
    Depth(DepthFrame),
    Disparity(DisparityFrame),
    Motion(MotionFrame),
    Pose(PoseFrame),
    Other(AnyFrame),
}

pub type PointsFrame = Frame<frame_kind::Points>;
pub type CompositeFrame = Frame<frame_kind::Composite>;
pub type VideoFrame = Frame<frame_kind::Video>;
pub type DepthFrame = Frame<frame_kind::Depth>;
pub type DisparityFrame = Frame<frame_kind::Disparity>;
pub type MotionFrame = Frame<frame_kind::Motion>;
pub type PoseFrame = Frame<frame_kind::Pose>;
pub type AnyFrame = Frame<frame_kind::Any>;

/// Represents a collection of sensor data.
#[derive(Debug)]
pub struct Frame<Kind>
where
    Kind: frame_kind::FrameKind,
{
    pub(crate) ptr: NonNull<sys::rs2_frame>,
    _phantom: PhantomData<Kind>,
}

impl<Kind> GenericFrameEx for Frame<Kind>
where
    Kind: frame_kind::FrameKind,
{
    fn ptr(&self) -> NonNull<sys::rs2_frame> {
        self.ptr
    }

    fn into_raw(self) -> *mut sys::rs2_frame {
        let ptr = self.ptr;
        mem::forget(self);
        ptr.as_ptr()
    }

    unsafe fn from_raw(ptr: *mut sys::rs2_frame) -> Self {
        Self {
            ptr: NonNull::new(ptr).unwrap(),
            _phantom: PhantomData,
        }
    }
}

impl VideoFrameEx for VideoFrame {}

impl VideoFrameEx for DepthFrame {}

impl DepthFrameEx for DepthFrame {}

impl VideoFrameEx for DisparityFrame {}

impl DepthFrameEx for DisparityFrame {}

impl DisparityFrameEx for DisparityFrame {}

impl<Kind> Frame<Kind> where Kind: frame_kind::FrameKind {}

impl AnyFrame {
    pub fn is_extendable_to<Kind>(&self) -> Result<bool>
    where
        Kind: frame_kind::NonAnyFrameKind,
    {
        unsafe {
            let mut checker = ErrorChecker::new();
            let val = sys::rs2_is_frame_extendable_to(
                self.ptr.as_ptr(),
                Kind::EXTENSION as sys::rs2_extension,
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            Ok(val != 0)
        }
    }

    pub fn try_extend_to<Kind>(self) -> Result<result::Result<Frame<Kind>, Self>>
    where
        Kind: frame_kind::NonAnyFrameKind,
    {
        let is_extendable = self.is_extendable_to::<Kind>()?;
        if is_extendable {
            let ptr = self.into_raw();
            let frame = Frame {
                ptr: NonNull::new(ptr).unwrap(),
                _phantom: PhantomData,
            };
            Ok(Ok(frame))
        } else {
            Ok(Err(self))
        }
    }

    pub fn try_extend(self) -> Result<ExtendedFrame> {
        let frame_any = self;

        let frame_any = match frame_any.try_extend_to::<frame_kind::Points>()? {
            Ok(frame) => return Ok(ExtendedFrame::Points(frame)),
            Err(frame) => frame,
        };

        let frame_any = match frame_any.try_extend_to::<frame_kind::Composite>()? {
            Ok(frame) => return Ok(ExtendedFrame::Composite(frame)),
            Err(frame) => frame,
        };

        let frame_any = match frame_any.try_extend_to::<frame_kind::Motion>()? {
            Ok(frame) => return Ok(ExtendedFrame::Motion(frame)),
            Err(frame) => frame,
        };

        let frame_any = match frame_any.try_extend_to::<frame_kind::Pose>()? {
            Ok(frame) => return Ok(ExtendedFrame::Pose(frame)),
            Err(frame) => frame,
        };

        let frame_any = match frame_any.try_extend_to::<frame_kind::Disparity>()? {
            Ok(frame) => return Ok(ExtendedFrame::Disparity(frame)),
            Err(frame) => frame,
        };

        let frame_any = match frame_any.try_extend_to::<frame_kind::Depth>()? {
            Ok(frame) => return Ok(ExtendedFrame::Depth(frame)),
            Err(frame) => frame,
        };

        let frame_any = match frame_any.try_extend_to::<frame_kind::Video>()? {
            Ok(frame) => return Ok(ExtendedFrame::Video(frame)),
            Err(frame) => frame,
        };

        Ok(ExtendedFrame::Other(frame_any))
    }
}

impl CompositeFrame {
    /// Gets the number of frames included in the composite frame.
    pub fn len(&self) -> Result<usize> {
        let len = unsafe {
            let mut checker = ErrorChecker::new();
            let len = sys::rs2_embedded_frames_count(self.ptr.as_ptr(), checker.inner_mut_ptr());
            checker.check()?;
            len as usize
        };
        Ok(len)
    }

    /// Checks if the composite-frame contains no sub-frames.
    pub fn is_empty(&self) -> Result<bool> {
        Ok(self.len()? == 0)
    }

    /// Gets the frame in frameset by index.
    ///
    /// The method throws error if index is out of bound given by [Frame::len].
    pub fn get(&self, index: usize) -> Result<Option<AnyFrame>> {
        let len = self.len()?;
        if index >= len {
            return Ok(None);
        }

        let frame = unsafe {
            // extract frame
            let mut checker = ErrorChecker::new();
            let ptr =
                sys::rs2_extract_frame(self.ptr.as_ptr(), index as c_int, checker.inner_mut_ptr());
            checker.check()?;

            Frame::from_raw(ptr)
        };
        Ok(Some(frame))
    }

    /// Unpacks the set of frames and turns into iterable [CompositeFrameIntoIter] instance.
    pub fn try_into_iter(self) -> Result<CompositeFrameIntoIter> {
        let len = self.len()?;
        let ptr = self.into_raw();
        let iter = CompositeFrameIntoIter {
            index: 0,
            len,
            ptr: NonNull::new(ptr).unwrap(),
            fused: len == 0,
        };
        Ok(iter)
    }

    pub fn try_iter(&self) -> Result<CompositeFrameIter> {
        let len = self.len()?;
        let iter = CompositeFrameIter {
            index: 0,
            len,
            ptr: self.ptr,
            fused: len == 0,
        };
        Ok(iter)
    }

    pub fn first_of<Kind>(&self, stream: StreamKind) -> Result<Option<Frame<Kind>>>
    where
        Kind: frame_kind::NonAnyFrameKind,
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

    pub fn color_frame(&self) -> Result<Option<VideoFrame>> {
        self.first_of::<frame_kind::Video>(StreamKind::Color)
    }

    pub fn depth_frame(&self) -> Result<Option<DepthFrame>> {
        self.first_of::<frame_kind::Depth>(StreamKind::Depth)
    }

    pub fn pose_frame(&self) -> Result<Option<PoseFrame>> {
        self.first_of::<frame_kind::Pose>(StreamKind::Pose)
    }
}

impl PoseFrame {
    /// Gets the pose data.
    pub fn pose(&self) -> Result<PoseData> {
        let pose_data = unsafe {
            let mut checker = ErrorChecker::new();
            let mut pose_data = MaybeUninit::uninit();
            sys::rs2_pose_frame_get_pose_data(
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
pub struct TextureCoordinate {
    // SAFETY: See safe_transmute::TriviallyTransmutable trait implementation for this type.
    pub u: f32,
    pub v: f32,
}

// SAFETY: TextureCoordinate is a POD type.
unsafe impl safe_transmute::TriviallyTransmutable for TextureCoordinate {}

impl PointsFrame {
    /// Gets vertices of point cloud.
    pub fn vertices<'a>(&'a self) -> Result<&'a [sys::rs2_vertex]> {
        let n_points = self.points_count()?;
        unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = sys::rs2_get_frame_vertices(self.ptr.as_ptr(), checker.inner_mut_ptr());
            checker.check()?;
            let slice = slice::from_raw_parts::<sys::rs2_vertex>(ptr, n_points);
            Ok(slice)
        }
    }

    /// Gets texture coordinates of each point of point cloud.
    pub fn texture_coordinates<'a>(&'a self) -> Result<&'a [TextureCoordinate]> {
        unsafe {
            let n_points = self.points_count()?;
            let mut checker = ErrorChecker::new();
            let ptr =
                sys::rs2_get_frame_texture_coordinates(self.ptr.as_ptr(), checker.inner_mut_ptr());
            checker.check()?;

            // SAFETY:
            // The librealsense2 C++ API directly casts the rs2_pixel* returned from
            // rs2_get_frame_texture_coordinates() into a texture_coordinate*, thereby
            // re-interpreting [[c_int; 2]; N] as [[c_float; 2]; N] values.
            // Note that C does not generally guarantee that sizeof(int) == sizeof(float).
            let slice = slice::from_raw_parts::<sys::rs2_pixel>(ptr, n_points);
            let bytes =
                slice::from_raw_parts::<u8>(slice.as_ptr() as *const u8, mem::size_of_val(slice));
            let tcs =
                safe_transmute::transmute_many::<TextureCoordinate, PedanticGuard>(bytes).unwrap();
            debug_assert_eq!(tcs.len(), n_points);
            Ok(tcs)
        }
    }

    /// Gets number of points in frame.
    pub fn points_count(&self) -> Result<usize> {
        unsafe {
            let mut checker = ErrorChecker::new();
            let val = sys::rs2_get_frame_points_count(self.ptr.as_ptr(), checker.inner_mut_ptr());
            checker.check()?;
            Ok(val as usize)
        }
    }
}

impl MotionFrame {
    /// Gets motion data.
    pub fn motion(&self) -> Result<[f32; 3]> {
        let slice = safe_transmute::transmute_many::<f32, PedanticGuard>(self.data()?).unwrap();
        match *slice {
            [x, y, z] => Ok([x, y, z]),
            _ => unreachable!("please report bug"),
        }
    }
}

impl IntoIterator for CompositeFrame {
    type Item = Result<AnyFrame>;
    type IntoIter = CompositeFrameIntoIter;

    /// The method internally calls [Frame::try_into_iter](Frame::try_into_iter).
    ///
    /// # Panics
    /// This method panics if [Frame::try_into_iter](Frame::try_into_iter) returns error.
    ///
    fn into_iter(self) -> Self::IntoIter {
        self.try_into_iter().unwrap()
    }
}

impl<Kind> Clone for Frame<Kind>
where
    Kind: frame_kind::FrameKind,
{
    fn clone(&self) -> Self {
        self.try_clone().unwrap()
    }
}

impl<Kind> Drop for Frame<Kind>
where
    Kind: frame_kind::FrameKind,
{
    fn drop(&mut self) {
        unsafe {
            sys::rs2_release_frame(self.ptr.as_ptr());
        }
    }
}

unsafe impl<Kind> Send for Frame<Kind> where Kind: frame_kind::FrameKind {}
unsafe impl<Kind> Sync for Frame<Kind> where Kind: frame_kind::FrameKind {}

/// The iterator type returned by [Frame::try_into_iter](Frame::try_into_iter).
#[derive(Debug)]
pub struct CompositeFrameIntoIter {
    len: usize,
    index: usize,
    ptr: NonNull<sys::rs2_frame>,
    fused: bool,
}

impl Iterator for CompositeFrameIntoIter {
    type Item = Result<AnyFrame>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.fused {
            return None;
        }

        let ptr = unsafe {
            // extract frame
            let mut checker = ErrorChecker::new();
            let ptr = sys::rs2_extract_frame(
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

        let frame = unsafe { Frame::from_raw(ptr) };
        Some(Ok(frame))
    }
}

impl FusedIterator for CompositeFrameIntoIter {}

impl Drop for CompositeFrameIntoIter {
    fn drop(&mut self) {
        unsafe {
            sys::rs2_release_frame(self.ptr.as_ptr());
        }
    }
}

/// The iterator type returned by [Frame::try_iter](Frame::try_iter).
#[derive(Debug)]
pub struct CompositeFrameIter {
    len: usize,
    index: usize,
    ptr: NonNull<sys::rs2_frame>,
    fused: bool,
}

impl Iterator for CompositeFrameIter {
    type Item = Result<AnyFrame>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.fused {
            return None;
        }

        let ptr = unsafe {
            // extract frame
            let mut checker = ErrorChecker::new();
            let ptr = sys::rs2_extract_frame(
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

        let frame = unsafe { Frame::from_raw(ptr) };
        Some(Ok(frame))
    }
}

impl FusedIterator for CompositeFrameIter {}
