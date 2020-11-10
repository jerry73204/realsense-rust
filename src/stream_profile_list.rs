//! Defines the iterable list of stream profiles.

use crate::{
    common::*,
    error::{ErrorChecker, Result},
    stream_profile::AnyStreamProfile,
};

/// An iterable list of streams.
#[derive(Debug)]
pub struct StreamProfileList {
    ptr: NonNull<realsense_sys::rs2_stream_profile_list>,
}

impl StreamProfileList {
    /// Gets the stream profile at given index.
    ///
    /// The method returns error if the index is out of bound given by [StreamProfileList::len].
    pub fn get(&mut self, index: usize) -> Result<AnyStreamProfile> {
        let profile = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_get_stream_profile(
                self.ptr.as_ptr(),
                index as c_int,
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            AnyStreamProfile::from_parts(NonNull::new(ptr as *mut _).unwrap(), false)
        };
        Ok(profile)
    }

    /// Gets the length of list.
    pub fn len(&mut self) -> Result<usize> {
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

    /// Checks if the profile list is empty.
    pub fn is_empty(&mut self) -> Result<bool> {
        Ok(self.len()? == 0)
    }

    /// Turns into iterable [StreamProfileListIntoIter] instance.
    pub fn try_into_iter(mut self) -> Result<StreamProfileListIntoIter> {
        let len = self.len()?;
        let ptr = unsafe { self.take() };
        let iter = StreamProfileListIntoIter {
            len,
            index: 0,
            ptr,
            fused: len == 0,
        };
        Ok(iter)
    }

    pub(crate) unsafe fn take(self) -> NonNull<realsense_sys::rs2_stream_profile_list> {
        let ptr = self.ptr;
        std::mem::forget(self);
        ptr
    }

    pub(crate) unsafe fn from_ptr(ptr: NonNull<realsense_sys::rs2_stream_profile_list>) -> Self {
        Self { ptr }
    }
}

impl IntoIterator for StreamProfileList {
    type Item = Result<AnyStreamProfile>;
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
    type Item = Result<AnyStreamProfile>;

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
            unsafe { AnyStreamProfile::from_parts(NonNull::new(ptr as *mut _).unwrap(), false) };
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
