//! Defines the device types.

use crate::{
    error::{ErrorChecker, Result as RsResult},
    kind::CameraInfo,
    sensor_list::SensorList,
};
use std::{ffi::CStr, ptr::NonNull};

/// Represents a device instance.
#[derive(Debug)]
pub struct Device {
    pub(crate) ptr: NonNull<realsense_sys::rs2_device>,
}

impl Device {
    /// Discover available sensors on device.
    pub fn query_sensors(&self) -> RsResult<SensorList> {
        let list = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_query_sensors(self.ptr.as_ptr(), checker.inner_mut_ptr());
            checker.check()?;
            SensorList::from_ptr(NonNull::new(ptr).unwrap())
        };
        Ok(list)
    }

    pub fn name(&self) -> RsResult<Option<&str>> {
        self.info(CameraInfo::Name)
    }

    pub fn serial_number(&self) -> RsResult<Option<&str>> {
        self.info(CameraInfo::SerialNumber)
    }

    pub fn recommended_firmware_version(&self) -> RsResult<Option<&str>> {
        self.info(CameraInfo::RecommendedFirmwareVersion)
    }

    pub fn physical_port(&self) -> RsResult<Option<&str>> {
        self.info(CameraInfo::PhysicalPort)
    }

    pub fn debug_op_code(&self) -> RsResult<Option<&str>> {
        self.info(CameraInfo::DebugOpCode)
    }

    pub fn advanced_mode(&self) -> RsResult<Option<&str>> {
        self.info(CameraInfo::AdvancedMode)
    }

    pub fn product_id(&self) -> RsResult<Option<&str>> {
        self.info(CameraInfo::ProductId)
    }

    pub fn camera_locked(&self) -> RsResult<Option<&str>> {
        self.info(CameraInfo::CameraLocked)
    }

    pub fn usb_type_descriptor(&self) -> RsResult<Option<&str>> {
        self.info(CameraInfo::UsbTypeDescriptor)
    }

    pub fn product_line(&self) -> RsResult<Option<&str>> {
        self.info(CameraInfo::ProductLine)
    }

    pub fn asic_serial_number(&self) -> RsResult<Option<&str>> {
        self.info(CameraInfo::AsicSerialNumber)
    }

    pub fn firmware_update_id(&self) -> RsResult<Option<&str>> {
        self.info(CameraInfo::FirmwareUpdateId)
    }

    pub fn count(&self) -> RsResult<Option<&str>> {
        self.info(CameraInfo::Count)
    }

    pub fn info(&self, kind: CameraInfo) -> RsResult<Option<&str>> {
        if !self.is_info_supported(kind)? {
            return Ok(None);
        }

        let ptr = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = realsense_sys::rs2_get_device_info(
                self.ptr.as_ptr(),
                kind as realsense_sys::rs2_camera_info,
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            ptr
        };

        // TODO: deallicate this CStr?
        let string = unsafe { CStr::from_ptr(ptr).to_str().unwrap() };
        Ok(Some(string))
    }

    pub fn is_info_supported(&self, kind: CameraInfo) -> RsResult<bool> {
        let val = unsafe {
            let mut checker = ErrorChecker::new();
            let val = realsense_sys::rs2_supports_device_info(
                self.ptr.as_ptr(),
                kind as realsense_sys::rs2_camera_info,
                checker.inner_mut_ptr(),
            );
            checker.check()?;
            val
        };
        Ok(val != 0)
    }

    pub(crate) unsafe fn from_ptr(ptr: NonNull<realsense_sys::rs2_device>) -> Self {
        Self { ptr }
    }
}

impl Drop for Device {
    fn drop(&mut self) {
        unsafe {
            realsense_sys::rs2_delete_device(self.ptr.as_ptr());
        }
    }
}

unsafe impl Send for Device {}
