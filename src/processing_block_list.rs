use crate::{
    error::{ErrorChecker, Result as RsResult},
    processing_block::{marker as processing_block_marker, ProcessingBlock},
};
use std::{iter::FusedIterator, mem::MaybeUninit, os::raw::c_int, ptr::NonNull};

#[derive(Debug)]
pub struct ProcessingBlockList {
    ptr: NonNull<realsense_sys::rs2_processing_block_list>,
}

impl ProcessingBlockList {
    pub fn get(&mut self, index: usize) -> RsResult<ProcessingBlock<processing_block_marker::Any>> {
        let block = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_get_processing_block(
                self.ptr.as_ptr(),
                index as c_int,
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            ProcessingBlock::from_ptr(NonNull::new(ptr).unwrap())
        };
        Ok(block)
    }

    pub fn len(&mut self) -> RsResult<usize> {
        unsafe {
            let mut checker = ErrorChecker::new();
            let val = realsense_sys::rs2_get_recommended_processing_blocks_count(
                self.ptr.as_ptr(),
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            Ok(val as usize)
        }
    }

    pub fn try_into_iter(mut self) -> RsResult<ProcessingBlockListIter> {
        let len = self.len()?;
        let ptr = unsafe { self.take() };
        let iter = ProcessingBlockListIter { len, index: 0, ptr };
        Ok(iter)
    }

    pub(crate) unsafe fn take(mut self) -> NonNull<realsense_sys::rs2_processing_block_list> {
        let ptr = std::mem::replace(&mut self.ptr, { MaybeUninit::uninit().assume_init() });
        std::mem::forget(self);
        ptr
    }

    pub(crate) unsafe fn from_ptr(ptr: NonNull<realsense_sys::rs2_processing_block_list>) -> Self {
        Self { ptr }
    }
}

impl IntoIterator for ProcessingBlockList {
    type Item = RsResult<ProcessingBlock<processing_block_marker::Any>>;
    type IntoIter = ProcessingBlockListIter;

    fn into_iter(self) -> Self::IntoIter {
        self.try_into_iter().unwrap()
    }
}

impl Drop for ProcessingBlockList {
    fn drop(&mut self) {
        unsafe {
            realsense_sys::rs2_delete_recommended_processing_blocks(self.ptr.as_ptr());
        }
    }
}

pub struct ProcessingBlockListIter {
    len: usize,
    index: usize,
    ptr: NonNull<realsense_sys::rs2_processing_block_list>,
}

impl Iterator for ProcessingBlockListIter {
    type Item = RsResult<ProcessingBlock<processing_block_marker::Any>>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.index < self.len {
            let block = unsafe {
                let mut checker = ErrorChecker::new();
                let ptr = realsense_sys::rs2_get_processing_block(
                    self.ptr.as_ptr(),
                    self.index as c_int,
                    checker.inner_mut_ptr(),
                );
                match checker.check() {
                    Ok(()) => ProcessingBlock::from_ptr(NonNull::new(ptr).unwrap()),
                    Err(err) => return Some(Err(err)),
                }
            };
            self.index += 1;
            Some(Ok(block))
        } else {
            None
        }
    }
}

impl FusedIterator for ProcessingBlockListIter {}

impl Drop for ProcessingBlockListIter {
    fn drop(&mut self) {
        unsafe {
            realsense_sys::rs2_delete_recommended_processing_blocks(self.ptr.as_ptr());
        }
    }
}
