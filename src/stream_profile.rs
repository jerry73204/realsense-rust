//! Defines the profile type of streams.

use crate::{
    base::{Extrinsics, Intrinsics, MotionIntrinsics, Resolution, StreamProfileData},
    error::{ErrorChecker, Result as RsResult},
    kind::{Extension, Format, StreamKind},
};
use num_traits::FromPrimitive;
use std::{borrow::Borrow, marker::PhantomData, mem::MaybeUninit, ptr::NonNull};

/// Marker traits and types for [StreamProfile].
pub mod marker {
    use super::*;

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
}

/// The enumeration of extended stream profile type returned by [StreamProfile::try_extend](StreamProfile::try_extend).
#[derive(Debug)]
pub enum ExtendedStreamProfile {
    Video(StreamProfile<marker::Video>),
    Motion(StreamProfile<marker::Motion>),
    Pose(StreamProfile<marker::Pose>),
    Other(StreamProfile<marker::Any>),
}

/// The profile of stream.
#[derive(Debug)]
pub struct StreamProfile<Kind>
where
    Kind: marker::StreamProfileKind,
{
    ptr: NonNull<realsense_sys::rs2_stream_profile>,
    from_clone: bool,
    _phantom: PhantomData<Kind>,
}

impl<Kind> StreamProfile<Kind>
where
    Kind: marker::StreamProfileKind,
{
    /// Check whether the profile is default or not.
    pub fn is_default(&self) -> RsResult<bool> {
        unsafe {
            let mut checker = ErrorChecker::new();
            let val = realsense_sys::rs2_is_stream_profile_default(
                self.ptr.as_ptr(),
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            Ok(val != 0)
        }
    }

    /// Gets the attributes of stream.
    pub fn get_data(&self) -> RsResult<StreamProfileData> {
        unsafe {
            let mut checker = ErrorChecker::new();
            let mut stream = MaybeUninit::uninit();
            let mut format = MaybeUninit::uninit();
            let mut index = MaybeUninit::uninit();
            let mut unique_id = MaybeUninit::uninit();
            let mut framerate = MaybeUninit::uninit();

            realsense_sys::rs2_get_stream_profile_data(
                self.ptr.as_ptr(),
                stream.as_mut_ptr(),
                format.as_mut_ptr(),
                index.as_mut_ptr(),
                unique_id.as_mut_ptr(),
                framerate.as_mut_ptr(),
                checker.inner_mut_ptr(),
            );
            checker.check()?;

            let data = StreamProfileData {
                stream: StreamKind::from_u32(stream.assume_init()).unwrap(),
                format: Format::from_u32(format.assume_init()).unwrap(),
                index: index.assume_init() as usize,
                unique_id: unique_id.assume_init(),
                framerate: framerate.assume_init(),
            };
            Ok(data)
        }
    }

    // /// Sets the attributes of stream.
    // pub fn set_data(&mut self) {
    //     todo!();
    // }

    /// Compute the extrinsic parameters to another stream.
    pub fn extrinsics_to<P, K>(&self, other: P) -> RsResult<Extrinsics>
    where
        P: Borrow<StreamProfile<K>>,
        K: marker::StreamProfileKind,
    {
        unsafe {
            let mut extrinsics = MaybeUninit::<realsense_sys::rs2_extrinsics>::uninit();
            let mut checker = ErrorChecker::new();
            realsense_sys::rs2_get_extrinsics(
                self.ptr.as_ptr(),
                other.borrow().ptr.as_ptr(),
                extrinsics.as_mut_ptr(),
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            Ok(Extrinsics(extrinsics.assume_init()))
        }
    }

    pub(crate) unsafe fn take(mut self) -> (NonNull<realsense_sys::rs2_stream_profile>, bool) {
        let ptr = std::mem::replace(&mut self.ptr, MaybeUninit::uninit().assume_init());
        let from_clone = self.from_clone;
        std::mem::forget(self);
        (ptr, from_clone)
    }

    pub(crate) unsafe fn from_parts(
        ptr: NonNull<realsense_sys::rs2_stream_profile>,
        from_clone: bool,
    ) -> Self {
        Self {
            ptr,
            from_clone,
            _phantom: PhantomData,
        }
    }
}

impl StreamProfile<marker::Any> {
    /// Check if the stream is extendable to the given extension.
    pub fn is_extendable_to<Kind>(&self) -> RsResult<bool>
    where
        Kind: marker::NonAnyStreamProfileKind,
    {
        unsafe {
            let mut checker = ErrorChecker::new();
            let val = realsense_sys::rs2_stream_profile_is(
                self.ptr.as_ptr(),
                Kind::EXTENSION as realsense_sys::rs2_extension,
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            Ok(val != 0)
        }
    }

    /// Extends to a specific stream profile subtype.
    pub fn try_extend_to<Kind>(self) -> RsResult<Result<StreamProfile<Kind>, Self>>
    where
        Kind: marker::NonAnyStreamProfileKind,
    {
        if self.is_extendable_to::<Kind>()? {
            let (ptr, from_clone) = unsafe { self.take() };
            let profile = StreamProfile {
                ptr,
                from_clone,
                _phantom: PhantomData,
            };
            Ok(Ok(profile))
        } else {
            Ok(Err(self))
        }
    }

    /// Extends to one of a stream profile subtype.
    pub fn try_extend(self) -> RsResult<ExtendedStreamProfile> {
        let profile_any = self;

        let profile_any = match profile_any.try_extend_to::<marker::Video>()? {
            Ok(profile) => return Ok(ExtendedStreamProfile::Video(profile)),
            Err(profile) => profile,
        };

        let profile_any = match profile_any.try_extend_to::<marker::Motion>()? {
            Ok(profile) => return Ok(ExtendedStreamProfile::Motion(profile)),
            Err(profile) => profile,
        };

        let profile_any = match profile_any.try_extend_to::<marker::Pose>()? {
            Ok(profile) => return Ok(ExtendedStreamProfile::Pose(profile)),
            Err(profile) => profile,
        };

        Ok(ExtendedStreamProfile::Other(profile_any))
    }
}

impl StreamProfile<marker::Video> {
    /// Gets the resolution of stream.
    pub fn resolution(&self) -> RsResult<Resolution> {
        let mut width = MaybeUninit::uninit();
        let mut height = MaybeUninit::uninit();
        let resolution = unsafe {
            let mut checker = ErrorChecker::new();
            realsense_sys::rs2_get_video_stream_resolution(
                self.ptr.as_ptr(),
                width.as_mut_ptr(),
                height.as_mut_ptr(),
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            let resolution = Resolution {
                width: width.assume_init() as usize,
                height: height.assume_init() as usize,
            };
            resolution
        };
        Ok(resolution)
    }

    /// Gets the intrinsic parameters.
    pub fn intrinsics(&self) -> RsResult<Intrinsics> {
        unsafe {
            let mut checker = ErrorChecker::new();
            let mut intrinsics = MaybeUninit::<realsense_sys::rs2_intrinsics>::uninit();
            realsense_sys::rs2_get_video_stream_intrinsics(
                self.ptr.as_ptr(),
                intrinsics.as_mut_ptr(),
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            Ok(Intrinsics(intrinsics.assume_init()))
        }
    }
}

impl StreamProfile<marker::Motion> {
    /// Gets the motion intrinsic parameters.
    pub fn motion_intrinsics(&self) -> RsResult<MotionIntrinsics> {
        unsafe {
            let mut checker = ErrorChecker::new();
            let mut intrinsics =
                MaybeUninit::<realsense_sys::rs2_motion_device_intrinsic>::uninit();
            realsense_sys::rs2_get_motion_intrinsics(
                self.ptr.as_ptr(),
                intrinsics.as_mut_ptr(),
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            Ok(MotionIntrinsics(intrinsics.assume_init()))
        }
    }
}

impl<Kind> Drop for StreamProfile<Kind>
where
    Kind: marker::StreamProfileKind,
{
    fn drop(&mut self) {
        unsafe {
            if self.from_clone {
                realsense_sys::rs2_delete_stream_profile(self.ptr.as_ptr());
            }
        }
    }
}

unsafe impl<Kind> Send for StreamProfile<Kind> where Kind: marker::StreamProfileKind {}
