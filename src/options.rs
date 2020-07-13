use crate::{
    common::*,
    error::{ErrorChecker, Result as RsResult},
    kind::Rs2Option,
};

pub trait ToOptions {
    fn to_options(&self) -> RsResult<HashMap<Rs2Option, OptionHandle>> {
        let options_ptr = self.get_options_ptr();
        unsafe {
            let list_ptr = {
                let mut checker = ErrorChecker::new();
                let list_ptr = realsense_sys::rs2_get_options_list(
                    options_ptr.as_ptr(),
                    checker.inner_mut_ptr(),
                );
                checker.check()?;
                list_ptr
            };

            let len = {
                let mut checker = ErrorChecker::new();
                let len =
                    realsense_sys::rs2_get_options_list_size(list_ptr, checker.inner_mut_ptr());
                checker.check()?;
                len
            };

            let handles = (0..len)
                .into_iter()
                .map(|index| {
                    let mut checker = ErrorChecker::new();
                    let val = realsense_sys::rs2_get_option_from_list(
                        list_ptr,
                        index,
                        checker.inner_mut_ptr(),
                    );
                    checker.check()?;
                    let option = Rs2Option::from_u32(val).unwrap();
                    let handle = OptionHandle {
                        ptr: options_ptr.clone(),
                        option,
                    };

                    RsResult::Ok((option, handle))
                })
                .collect::<RsResult<HashMap<_, _>>>()?;
            Ok(handles)
        }
    }

    fn get_options_ptr(&self) -> NonNull<realsense_sys::rs2_options>;
}

#[derive(Debug, Clone)]
pub struct OptionHandle {
    ptr: NonNull<realsense_sys::rs2_options>,
    option: Rs2Option,
}

impl OptionHandle {
    pub fn get_value(&self) -> RsResult<f32> {
        unsafe {
            let mut checker = ErrorChecker::new();
            let val = realsense_sys::rs2_get_option(
                self.ptr.as_ptr(),
                self.option as realsense_sys::rs2_option,
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            Ok(val)
        }
    }

    pub fn set_value(&self, value: f32) -> RsResult<()> {
        unsafe {
            let mut checker = ErrorChecker::new();
            realsense_sys::rs2_set_option(
                self.ptr.as_ptr(),
                self.option as realsense_sys::rs2_option,
                value,
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            Ok(())
        }
    }

    pub fn is_read_only(&self) -> RsResult<bool> {
        unsafe {
            let mut checker = ErrorChecker::new();
            let val = realsense_sys::rs2_is_option_read_only(
                self.ptr.as_ptr(),
                self.option as realsense_sys::rs2_option,
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            Ok(val != 0)
        }
    }

    pub fn name<'a>(&'a self) -> RsResult<&'a str> {
        unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_get_option_name(
                self.ptr.as_ptr(),
                self.option as realsense_sys::rs2_option,
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            let desc = CStr::from_ptr(ptr).to_str().unwrap();
            Ok(desc)
        }
    }

    pub fn option_description<'a>(&'a self) -> RsResult<&'a str> {
        unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_get_option_description(
                self.ptr.as_ptr(),
                self.option as realsense_sys::rs2_option,
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            let desc = CStr::from_ptr(ptr).to_str().unwrap();
            Ok(desc)
        }
    }

    pub fn value_description<'a>(&'a self, value: f32) -> RsResult<&'a str> {
        unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_get_option_value_description(
                self.ptr.as_ptr(),
                self.option as realsense_sys::rs2_option,
                value,
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            let desc = CStr::from_ptr(ptr).to_str().unwrap();
            Ok(desc)
        }
    }
}
