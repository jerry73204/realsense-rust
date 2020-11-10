//! Defines the iterable list of processing blocks.

use crate::{
    common::*,
    error::{ErrorChecker, Result},
    processing_block::AnyProcessingBlock,
};

/// The iterable list of [AnyProcessingBlock](AnyProcessingBlock)s.
#[derive(Debug)]
pub struct ProcessingBlockList {
    ptr: NonNull<sys::rs2_processing_block_list>,
}

impl ProcessingBlockList {
    /// Retrieves the [AnyProcessingBlock](AnyProcessingBlock) instance at index.
    pub fn get(&mut self, index: usize) -> Result<AnyProcessingBlock> {
        let block = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = sys::rs2_get_processing_block(
                self.ptr.as_ptr(),
                index as c_int,
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            AnyProcessingBlock::new_from_raw(NonNull::new(ptr).unwrap())?
        };
        Ok(block)
    }

    /// Returns the length of list.
    pub fn len(&mut self) -> Result<usize> {
        unsafe {
            let mut checker = ErrorChecker::new();
            let val = sys::rs2_get_recommended_processing_blocks_count(
                self.ptr.as_ptr(),
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            Ok(val as usize)
        }
    }

    /// Checks if the list is empty.
    pub fn is_empty(&mut self) -> Result<bool> {
        Ok(self.len()? == 0)
    }

    /// Converts to iterator type.
    pub fn try_into_iter(mut self) -> Result<ProcessingBlockListIntoIter> {
        let len = self.len()?;
        let ptr = self.into_raw();
        let iter = ProcessingBlockListIntoIter {
            len,
            index: 0,
            ptr: NonNull::new(ptr).unwrap(),
        };
        Ok(iter)
    }

    pub fn into_raw(self) -> *mut sys::rs2_processing_block_list {
        let ptr = self.ptr;
        mem::forget(self);
        ptr.as_ptr()
    }

    pub unsafe fn from_raw(ptr: *mut sys::rs2_processing_block_list) -> Self {
        Self {
            ptr: NonNull::new(ptr).unwrap(),
        }
    }
}

impl IntoIterator for ProcessingBlockList {
    type Item = Result<AnyProcessingBlock>;
    type IntoIter = ProcessingBlockListIntoIter;

    /// The method calls [ProcessingBlockList::try_into_iter](ProcessingBlockList::try_into_iter).
    ///
    /// # Panics
    /// It panics if [ProcessingBlockList::try_into_iter](ProcessingBlockList::try_into_iter) returns error.
    fn into_iter(self) -> Self::IntoIter {
        self.try_into_iter().unwrap()
    }
}

impl Drop for ProcessingBlockList {
    fn drop(&mut self) {
        unsafe {
            sys::rs2_delete_recommended_processing_blocks(self.ptr.as_ptr());
        }
    }
}

/// The iterator type returned by [ProcessingBlockList::try_into_iter](ProcessingBlockList::try_into_iter).
pub struct ProcessingBlockListIntoIter {
    len: usize,
    index: usize,
    ptr: NonNull<sys::rs2_processing_block_list>,
}

impl Iterator for ProcessingBlockListIntoIter {
    type Item = Result<AnyProcessingBlock>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.index < self.len {
            let result = unsafe {
                let mut checker = ErrorChecker::new();
                let ptr = sys::rs2_get_processing_block(
                    self.ptr.as_ptr(),
                    self.index as c_int,
                    checker.inner_mut_ptr(),
                );
                match checker.check() {
                    Ok(()) => AnyProcessingBlock::new_from_raw(NonNull::new(ptr).unwrap()),
                    Err(err) => return Some(Err(err)),
                }
            };
            self.index += 1;
            Some(result)
        } else {
            None
        }
    }
}

impl FusedIterator for ProcessingBlockListIntoIter {}

impl Drop for ProcessingBlockListIntoIter {
    fn drop(&mut self) {
        unsafe {
            sys::rs2_delete_recommended_processing_blocks(self.ptr.as_ptr());
        }
    }
}
