//! Defines the profile type of streams.

use crate::{
    base::{Extrinsics, Intrinsics, MotionIntrinsics, Resolution, StreamProfileData},
    common::*,
    error::{ErrorChecker, Result},
    kind::{Format, StreamKind},
    stream_profile_kind,
};

/// The enumeration of extended stream profile type returned by [StreamProfile::try_extend](StreamProfile::try_extend).
#[derive(Debug)]
pub enum ExtendedStreamProfile {
    Video(VideoStreamProfile),
    Motion(MotionStreamProfile),
    Pose(PoseStreamProfile),
    Other(AnyStreamProfile),
}

/// The profile of stream.
#[derive(Debug)]
pub struct StreamProfile<Kind>
where
    Kind: stream_profile_kind::StreamProfileKind,
{
    ptr: NonNull<realsense_sys::rs2_stream_profile>,
    from_clone: bool,
    _phantom: PhantomData<Kind>,
}

// type aliases

pub type VideoStreamProfile = StreamProfile<stream_profile_kind::Video>;
pub type MotionStreamProfile = StreamProfile<stream_profile_kind::Motion>;
pub type PoseStreamProfile = StreamProfile<stream_profile_kind::Pose>;
pub type AnyStreamProfile = StreamProfile<stream_profile_kind::Any>;

impl<Kind> StreamProfile<Kind>
where
    Kind: stream_profile_kind::StreamProfileKind,
{
    /// Check whether the profile is default or not.
    pub fn is_default(&self) -> Result<bool> {
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
    pub fn get_data(&self) -> Result<StreamProfileData> {
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

    /// Gets the extrinsic parameters to another stream.
    pub fn get_extrinsics<P, K>(&self, to_stream: P) -> Result<Extrinsics>
    where
        P: Borrow<StreamProfile<K>>,
        K: stream_profile_kind::StreamProfileKind,
    {
        unsafe {
            let mut extrinsics = MaybeUninit::<realsense_sys::rs2_extrinsics>::uninit();
            let mut checker = ErrorChecker::new();
            realsense_sys::rs2_get_extrinsics(
                self.ptr.as_ptr(),
                to_stream.borrow().ptr.as_ptr(),
                extrinsics.as_mut_ptr(),
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            Ok(Extrinsics(extrinsics.assume_init()))
        }
    }

    /// Sets the extrinsic parameters to another stream.
    pub fn set_extrinsics<P, K>(&self, to_stream: P, extrinsics: Extrinsics) -> Result<()>
    where
        P: Borrow<StreamProfile<K>>,
        K: stream_profile_kind::StreamProfileKind,
    {
        unsafe {
            let mut checker = ErrorChecker::new();
            realsense_sys::rs2_register_extrinsics(
                self.ptr.as_ptr(),
                to_stream.borrow().ptr.as_ptr(),
                *extrinsics,
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            Ok(())
        }
    }

    pub(crate) unsafe fn take(self) -> (NonNull<realsense_sys::rs2_stream_profile>, bool) {
        let ptr = self.ptr;
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

impl AnyStreamProfile {
    /// Check if the stream is extendable to the given extension.
    pub fn is_extendable_to<Kind>(&self) -> Result<bool>
    where
        Kind: stream_profile_kind::NonAnyStreamProfileKind,
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
    pub fn try_extend_to<Kind>(self) -> Result<std::result::Result<StreamProfile<Kind>, Self>>
    where
        Kind: stream_profile_kind::NonAnyStreamProfileKind,
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
    pub fn try_extend(self) -> Result<ExtendedStreamProfile> {
        let profile_any = self;

        let profile_any = match profile_any.try_extend_to::<stream_profile_kind::Video>()? {
            Ok(profile) => return Ok(ExtendedStreamProfile::Video(profile)),
            Err(profile) => profile,
        };

        let profile_any = match profile_any.try_extend_to::<stream_profile_kind::Motion>()? {
            Ok(profile) => return Ok(ExtendedStreamProfile::Motion(profile)),
            Err(profile) => profile,
        };

        let profile_any = match profile_any.try_extend_to::<stream_profile_kind::Pose>()? {
            Ok(profile) => return Ok(ExtendedStreamProfile::Pose(profile)),
            Err(profile) => profile,
        };

        Ok(ExtendedStreamProfile::Other(profile_any))
    }
}

impl VideoStreamProfile {
    /// Gets the resolution of stream.
    pub fn resolution(&self) -> Result<Resolution> {
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

            Resolution {
                width: width.assume_init() as usize,
                height: height.assume_init() as usize,
            }
        };
        Ok(resolution)
    }

    /// Gets the intrinsic parameters.
    pub fn intrinsics(&self) -> Result<Intrinsics> {
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

impl MotionStreamProfile {
    /// Gets the motion intrinsic parameters.
    pub fn motion_intrinsics(&self) -> Result<MotionIntrinsics> {
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
    Kind: stream_profile_kind::StreamProfileKind,
{
    fn drop(&mut self) {
        unsafe {
            if self.from_clone {
                realsense_sys::rs2_delete_stream_profile(self.ptr.as_ptr());
            }
        }
    }
}

unsafe impl<Kind> Send for StreamProfile<Kind> where Kind: stream_profile_kind::StreamProfileKind {}
