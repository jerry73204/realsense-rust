use crate::{
    error::{ErrorChecker, Result as RsResult},
    stream_profile::StreamProfile,
};
use std::{iter::FusedIterator, mem::MaybeUninit, os::raw::c_int, ptr::NonNull};

#[derive(Debug)]
pub struct StreamProfileList {
    ptr: NonNull<realsense_sys::rs2_stream_profile_list>,
}

impl StreamProfileList {
    pub fn get(&mut self, index: usize) -> RsResult<StreamProfile> {
        let profile = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_get_stream_profile(
                self.ptr.as_ptr(),
                index as c_int,
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            StreamProfile::from_parts(NonNull::new(ptr as *mut _).unwrap(), false)
        };
        Ok(profile)
    }

    pub fn len(&mut self) -> RsResult<usize> {
        unsafe {
            let mut checker = ErrorChecker::new();
            let len = realsense_sys::rs2_get_stream_profiles_count(
                self.ptr.as_ptr(),
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            Ok(len as usize)
        }
    }

    pub fn try_into_iter(mut self) -> RsResult<StreamProfileListIntoIter> {
        let len = self.len()?;
        let ptr = unsafe { self.take() };
        let iter = StreamProfileListIntoIter {
            len,
            index: 0,
            ptr,
            fused: false,
        };
        Ok(iter)
    }

    pub(crate) unsafe fn take(mut self) -> NonNull<realsense_sys::rs2_stream_profile_list> {
        let ptr = std::mem::replace(&mut self.ptr, { MaybeUninit::uninit().assume_init() });
        std::mem::forget(self);
        ptr
    }

    pub(crate) unsafe fn from_ptr(ptr: NonNull<realsense_sys::rs2_stream_profile_list>) -> Self {
        Self { ptr }
    }
}

impl IntoIterator for StreamProfileList {
    type Item = RsResult<StreamProfile>;
    type IntoIter = StreamProfileListIntoIter;

    fn into_iter(self) -> Self::IntoIter {
        self.try_into_iter().unwrap()
    }
}

impl Drop for StreamProfileList {
    fn drop(&mut self) {
        unsafe {
            realsense_sys::rs2_delete_stream_profiles_list(self.ptr.as_ptr());
        }
    }
}

#[derive(Debug)]
pub struct StreamProfileListIntoIter {
    len: usize,
    index: usize,
    ptr: NonNull<realsense_sys::rs2_stream_profile_list>,
    fused: bool,
}

impl Iterator for StreamProfileListIntoIter {
    type Item = RsResult<StreamProfile>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.fused {
            return None;
        }

        let ptr = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_get_stream_profile(
                self.ptr.as_ptr(),
                self.index as c_int,
                checker.inner_mut_ptr(),
            );
            match checker.check() {
                Ok(()) => ptr,

                Err(err) => {
                    self.fused = true;
                    return Some(Err(err));
                }
            }
        };

        self.index += 1;
        if self.index >= self.len {
            self.fused = true;
        }

        let profile =
            unsafe { StreamProfile::from_parts(NonNull::new(ptr as *mut _).unwrap(), false) };
        Some(Ok(profile))
    }
}

impl FusedIterator for StreamProfileListIntoIter {}

unsafe impl Send for StreamProfileList {}

impl Drop for StreamProfileListIntoIter {
    fn drop(&mut self) {
        unsafe {
            realsense_sys::rs2_delete_stream_profiles_list(self.ptr.as_ptr());
        }
    }
}
