use crate::{
    base::Resolution,
    error::{ErrorChecker, Result as RsResult},
    kind::{Extension, Format, StreamKind},
    stream_profile_data::StreamProfileData,
};
use nalgebra::{Isometry3, MatrixMN, Translation3, UnitQuaternion, U3};
use num_traits::FromPrimitive;
use std::{borrow::Borrow, mem::MaybeUninit, ptr::NonNull};

#[derive(Debug)]
pub struct StreamProfile {
    ptr: NonNull<realsense_sys::rs2_stream_profile>,
    from_clone: bool,
}

impl StreamProfile {
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

    pub fn extend(&mut self, extension: Extension) -> RsResult<()> {
        unsafe {
            let mut checker = ErrorChecker::new();
            realsense_sys::rs2_stream_profile_is(
                self.ptr.as_ptr(),
                extension as realsense_sys::rs2_extension,
                checker.inner_mut_ptr(),
            );
            checker.check()?;
        }
        Ok(())
    }

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

    pub fn set_data(&mut self) {
        todo!();
    }

    pub fn extrinsics_to<P>(&self, other: P) -> RsResult<Isometry3<f32>>
    where
        P: Borrow<StreamProfile>,
    {
        let mut extrinsics = MaybeUninit::<realsense_sys::rs2_extrinsics>::uninit();
        let (raw_rotation, raw_translation) = unsafe {
            let mut checker = ErrorChecker::new();
            realsense_sys::rs2_get_extrinsics(
                self.ptr.as_ptr(),
                other.borrow().ptr.as_ptr(),
                extrinsics.as_mut_ptr(),
                checker.inner_mut_ptr(),
            );
            checker.check()?;

            let realsense_sys::rs2_extrinsics {
                rotation,
                translation,
            } = extrinsics.assume_init();
            (rotation, translation)
        };

        let rotation = {
            let matrix =
                MatrixMN::<f32, U3, U3>::from_iterator(raw_rotation.iter().map(|val| *val));
            UnitQuaternion::from_matrix(&matrix)
        };

        let translation =
            { Translation3::new(raw_translation[0], raw_translation[1], raw_translation[2]) };

        let isometry = Isometry3::from_parts(translation, rotation);

        Ok(isometry)
    }

    pub(crate) unsafe fn from_parts(
        ptr: NonNull<realsense_sys::rs2_stream_profile>,
        from_clone: bool,
    ) -> Self {
        Self { ptr, from_clone }
    }
}

impl Drop for StreamProfile {
    fn drop(&mut self) {
        unsafe {
            if self.from_clone {
                realsense_sys::rs2_delete_stream_profile(self.ptr.as_ptr());
            }
        }
    }
}

unsafe impl Send for StreamProfile {}
