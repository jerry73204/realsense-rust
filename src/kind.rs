//! Defines the common used enums.

/// Collection of enum types.
use num_derive::FromPrimitive;
use std::ffi::CStr;

/// The enumeration of options.
#[repr(u32)]
#[derive(FromPrimitive, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Rs2Option {
    BacklightCompensation = realsense_sys::rs2_option_RS2_OPTION_BACKLIGHT_COMPENSATION,
    Brightness = realsense_sys::rs2_option_RS2_OPTION_BRIGHTNESS,
    Contrast = realsense_sys::rs2_option_RS2_OPTION_CONTRAST,
    Exposure = realsense_sys::rs2_option_RS2_OPTION_EXPOSURE,
    Gain = realsense_sys::rs2_option_RS2_OPTION_GAIN,
    Gamma = realsense_sys::rs2_option_RS2_OPTION_GAMMA,
    Hue = realsense_sys::rs2_option_RS2_OPTION_HUE,
    Saturation = realsense_sys::rs2_option_RS2_OPTION_SATURATION,
    Sharpness = realsense_sys::rs2_option_RS2_OPTION_SHARPNESS,
    WhiteBalance = realsense_sys::rs2_option_RS2_OPTION_WHITE_BALANCE,
    EnableAutoExposure = realsense_sys::rs2_option_RS2_OPTION_ENABLE_AUTO_EXPOSURE,
    EnableAutoWhiteBalance = realsense_sys::rs2_option_RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE,
    VisualPreset = realsense_sys::rs2_option_RS2_OPTION_VISUAL_PRESET,
    LaserPower = realsense_sys::rs2_option_RS2_OPTION_LASER_POWER,
    Accuracy = realsense_sys::rs2_option_RS2_OPTION_ACCURACY,
    MotionRange = realsense_sys::rs2_option_RS2_OPTION_MOTION_RANGE,
    FilterOption = realsense_sys::rs2_option_RS2_OPTION_FILTER_OPTION,
    ConfidenceThreshold = realsense_sys::rs2_option_RS2_OPTION_CONFIDENCE_THRESHOLD,
    EmitterEnabled = realsense_sys::rs2_option_RS2_OPTION_EMITTER_ENABLED,
    FramesQueueSize = realsense_sys::rs2_option_RS2_OPTION_FRAMES_QUEUE_SIZE,
    TotalFrameDrops = realsense_sys::rs2_option_RS2_OPTION_TOTAL_FRAME_DROPS,
    AutoExposureMode = realsense_sys::rs2_option_RS2_OPTION_AUTO_EXPOSURE_MODE,
    PowerLineFrequency = realsense_sys::rs2_option_RS2_OPTION_POWER_LINE_FREQUENCY,
    AsicTemperature = realsense_sys::rs2_option_RS2_OPTION_ASIC_TEMPERATURE,
    ErrorPollingEnabled = realsense_sys::rs2_option_RS2_OPTION_ERROR_POLLING_ENABLED,
    ProjectorTemperature = realsense_sys::rs2_option_RS2_OPTION_PROJECTOR_TEMPERATURE,
    OutputTriggerEnabled = realsense_sys::rs2_option_RS2_OPTION_OUTPUT_TRIGGER_ENABLED,
    MotionModuleTemperature = realsense_sys::rs2_option_RS2_OPTION_MOTION_MODULE_TEMPERATURE,
    DepthUnits = realsense_sys::rs2_option_RS2_OPTION_DEPTH_UNITS,
    EnableMotionCorrection = realsense_sys::rs2_option_RS2_OPTION_ENABLE_MOTION_CORRECTION,
    AutoExposurePriority = realsense_sys::rs2_option_RS2_OPTION_AUTO_EXPOSURE_PRIORITY,
    ColorScheme = realsense_sys::rs2_option_RS2_OPTION_COLOR_SCHEME,
    HistogramEqualizationEnabled =
        realsense_sys::rs2_option_RS2_OPTION_HISTOGRAM_EQUALIZATION_ENABLED,
    MinDistance = realsense_sys::rs2_option_RS2_OPTION_MIN_DISTANCE,
    MaxDistance = realsense_sys::rs2_option_RS2_OPTION_MAX_DISTANCE,
    TextureSource = realsense_sys::rs2_option_RS2_OPTION_TEXTURE_SOURCE,
    FilterMagnitude = realsense_sys::rs2_option_RS2_OPTION_FILTER_MAGNITUDE,
    FilterSmoothAlpha = realsense_sys::rs2_option_RS2_OPTION_FILTER_SMOOTH_ALPHA,
    FilterSmoothDelta = realsense_sys::rs2_option_RS2_OPTION_FILTER_SMOOTH_DELTA,
    HolesFill = realsense_sys::rs2_option_RS2_OPTION_HOLES_FILL,
    StereoBaseline = realsense_sys::rs2_option_RS2_OPTION_STEREO_BASELINE,
    AutoExposureConvergeStep = realsense_sys::rs2_option_RS2_OPTION_AUTO_EXPOSURE_CONVERGE_STEP,
    InterCamSyncMode = realsense_sys::rs2_option_RS2_OPTION_INTER_CAM_SYNC_MODE,
    StreamFilter = realsense_sys::rs2_option_RS2_OPTION_STREAM_FILTER,
    StreamFormatFilter = realsense_sys::rs2_option_RS2_OPTION_STREAM_FORMAT_FILTER,
    StreamIndexFilter = realsense_sys::rs2_option_RS2_OPTION_STREAM_INDEX_FILTER,
    EmitterOnOff = realsense_sys::rs2_option_RS2_OPTION_EMITTER_ON_OFF,
    ZeroOrderPointX = realsense_sys::rs2_option_RS2_OPTION_ZERO_ORDER_POINT_X,
    ZeroOrderPointY = realsense_sys::rs2_option_RS2_OPTION_ZERO_ORDER_POINT_Y,
    LldTemperature = realsense_sys::rs2_option_RS2_OPTION_LLD_TEMPERATURE,
    McTemperature = realsense_sys::rs2_option_RS2_OPTION_MC_TEMPERATURE,
    MaTemperature = realsense_sys::rs2_option_RS2_OPTION_MA_TEMPERATURE,
    HardwarePreset = realsense_sys::rs2_option_RS2_OPTION_HARDWARE_PRESET,
    GlobalTimeEnabled = realsense_sys::rs2_option_RS2_OPTION_GLOBAL_TIME_ENABLED,
    ApdTemperature = realsense_sys::rs2_option_RS2_OPTION_APD_TEMPERATURE,
    EnableMapping = realsense_sys::rs2_option_RS2_OPTION_ENABLE_MAPPING,
    EnableRelocalization = realsense_sys::rs2_option_RS2_OPTION_ENABLE_RELOCALIZATION,
    EnablePoseJumping = realsense_sys::rs2_option_RS2_OPTION_ENABLE_POSE_JUMPING,
    EnableDynamicCalibration = realsense_sys::rs2_option_RS2_OPTION_ENABLE_DYNAMIC_CALIBRATION,
    DepthOffset = realsense_sys::rs2_option_RS2_OPTION_DEPTH_OFFSET,
    LedPower = realsense_sys::rs2_option_RS2_OPTION_LED_POWER,
    ZeroOrderEnabled = realsense_sys::rs2_option_RS2_OPTION_ZERO_ORDER_ENABLED,
    EnableMapPreservation = realsense_sys::rs2_option_RS2_OPTION_ENABLE_MAP_PRESERVATION,
    Count = realsense_sys::rs2_option_RS2_OPTION_COUNT,
}

/// The enumeration of timestamp domains.
#[repr(u32)]
#[derive(FromPrimitive, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum TimestampDomain {
    HardwareClock = realsense_sys::rs2_timestamp_domain_RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK,
    SystemTime = realsense_sys::rs2_timestamp_domain_RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME,
    GlobalTime = realsense_sys::rs2_timestamp_domain_RS2_TIMESTAMP_DOMAIN_GLOBAL_TIME,
    Count = realsense_sys::rs2_timestamp_domain_RS2_TIMESTAMP_DOMAIN_COUNT,
}

impl TimestampDomain {
    pub fn as_cstr(&self) -> &'static CStr {
        unsafe {
            let ptr = realsense_sys::rs2_timestamp_domain_to_string(
                *self as realsense_sys::rs2_timestamp_domain,
            );
            CStr::from_ptr(ptr)
        }
    }

    pub fn as_str(&self) -> &'static str {
        self.as_cstr().to_str().unwrap()
    }
}

impl ToString for TimestampDomain {
    fn to_string(&self) -> String {
        self.as_str().to_owned()
    }
}

/// The enumeration of metadata kinds of a frame.
#[repr(u32)]
#[derive(FromPrimitive, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum FrameMetaDataValue {
    FrameCounter = realsense_sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_FRAME_COUNTER,
    FrameTimestamp = realsense_sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_FRAME_TIMESTAMP,
    SensorTimestamp = realsense_sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_SENSOR_TIMESTAMP,
    ActualExposure = realsense_sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_ACTUAL_EXPOSURE,
    GainLevel = realsense_sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_GAIN_LEVEL,
    AutoExposure = realsense_sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_AUTO_EXPOSURE,
    WhiteBalance = realsense_sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_WHITE_BALANCE,
    TimeOfArrival = realsense_sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_TIME_OF_ARRIVAL,
    Temperature = realsense_sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_TEMPERATURE,
    BackendTimestamp = realsense_sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_BACKEND_TIMESTAMP,
    ActualFps = realsense_sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_ACTUAL_FPS,
    FrameLaserPower = realsense_sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_FRAME_LASER_POWER,
    FrameLaserPowerMode =
        realsense_sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_FRAME_LASER_POWER_MODE,
    ExposurePriority = realsense_sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_EXPOSURE_PRIORITY,
    ExposureRoiLeft = realsense_sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_EXPOSURE_ROI_LEFT,
    ExposureRoiRight =
        realsense_sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_EXPOSURE_ROI_RIGHT,
    ExposureRoiTop = realsense_sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_EXPOSURE_ROI_TOP,
    ExposureRoiBottom =
        realsense_sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_EXPOSURE_ROI_BOTTOM,
    Brightness = realsense_sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_BRIGHTNESS,
    Contrast = realsense_sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_CONTRAST,
    Saturation = realsense_sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_SATURATION,
    Sharpness = realsense_sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_SHARPNESS,
    AutoWhiteBalanceTemperature =
        realsense_sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_AUTO_WHITE_BALANCE_TEMPERATURE,
    BacklightCompensation =
        realsense_sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_BACKLIGHT_COMPENSATION,
    Hue = realsense_sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_HUE,
    Gamma = realsense_sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_GAMMA,
    ManualWhiteBalance =
        realsense_sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_MANUAL_WHITE_BALANCE,
    PowerLineFrequency =
        realsense_sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_POWER_LINE_FREQUENCY,
    LowLightCompensation =
        realsense_sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_LOW_LIGHT_COMPENSATION,
    FrameEmitterMode =
        realsense_sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_FRAME_EMITTER_MODE,
    FrameLedPower = realsense_sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_FRAME_LED_POWER,
    Count = realsense_sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_COUNT,
}

/// The enumeration of extensions.
#[repr(u32)]
#[derive(FromPrimitive, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Extension {
    // sensor
    ColorSensor = realsense_sys::rs2_extension_RS2_EXTENSION_COLOR_SENSOR,
    MotionSensor = realsense_sys::rs2_extension_RS2_EXTENSION_MOTION_SENSOR,
    FishEyeSensor = realsense_sys::rs2_extension_RS2_EXTENSION_FISHEYE_SENSOR,
    DepthSensor = realsense_sys::rs2_extension_RS2_EXTENSION_DEPTH_SENSOR,
    DepthStereoSensor = realsense_sys::rs2_extension_RS2_EXTENSION_DEPTH_STEREO_SENSOR,
    SoftwareSensor = realsense_sys::rs2_extension_RS2_EXTENSION_SOFTWARE_SENSOR,
    PoseSensor = realsense_sys::rs2_extension_RS2_EXTENSION_POSE_SENSOR,
    L500DepthSensor = realsense_sys::rs2_extension_RS2_EXTENSION_L500_DEPTH_SENSOR,
    Tm2Sensor = realsense_sys::rs2_extension_RS2_EXTENSION_TM2_SENSOR,
    // frame
    VideoFrame = realsense_sys::rs2_extension_RS2_EXTENSION_VIDEO_FRAME,
    MotionFrame = realsense_sys::rs2_extension_RS2_EXTENSION_MOTION_FRAME,
    CompositeFrame = realsense_sys::rs2_extension_RS2_EXTENSION_COMPOSITE_FRAME,
    DepthFrame = realsense_sys::rs2_extension_RS2_EXTENSION_DEPTH_FRAME,
    DisparityFrame = realsense_sys::rs2_extension_RS2_EXTENSION_DISPARITY_FRAME,
    PoseFrame = realsense_sys::rs2_extension_RS2_EXTENSION_POSE_FRAME,
    Points = realsense_sys::rs2_extension_RS2_EXTENSION_POINTS,
    // filter
    DecimationFilter = realsense_sys::rs2_extension_RS2_EXTENSION_DECIMATION_FILTER,
    ThresholdFilter = realsense_sys::rs2_extension_RS2_EXTENSION_THRESHOLD_FILTER,
    DisparityFilter = realsense_sys::rs2_extension_RS2_EXTENSION_DISPARITY_FILTER,
    SpatialFilter = realsense_sys::rs2_extension_RS2_EXTENSION_SPATIAL_FILTER,
    TemporalFilter = realsense_sys::rs2_extension_RS2_EXTENSION_TEMPORAL_FILTER,
    HoleFillingFilter = realsense_sys::rs2_extension_RS2_EXTENSION_HOLE_FILLING_FILTER,
    ZeroOrderFilter = realsense_sys::rs2_extension_RS2_EXTENSION_ZERO_ORDER_FILTER,
    RecommendedFilters = realsense_sys::rs2_extension_RS2_EXTENSION_RECOMMENDED_FILTERS,
    // profile
    VideoProfile = realsense_sys::rs2_extension_RS2_EXTENSION_VIDEO_PROFILE,
    MotionProfile = realsense_sys::rs2_extension_RS2_EXTENSION_MOTION_PROFILE,
    PoseProfile = realsense_sys::rs2_extension_RS2_EXTENSION_POSE_PROFILE,
    // device
    SoftwareDevice = realsense_sys::rs2_extension_RS2_EXTENSION_SOFTWARE_DEVICE,
    UpdateDevice = realsense_sys::rs2_extension_RS2_EXTENSION_UPDATE_DEVICE,
    AutoCalibratedDevice = realsense_sys::rs2_extension_RS2_EXTENSION_AUTO_CALIBRATED_DEVICE,
    // misc
    AdvancedMode = realsense_sys::rs2_extension_RS2_EXTENSION_ADVANCED_MODE,
    Record = realsense_sys::rs2_extension_RS2_EXTENSION_RECORD,
    Playback = realsense_sys::rs2_extension_RS2_EXTENSION_PLAYBACK,
    Pose = realsense_sys::rs2_extension_RS2_EXTENSION_POSE,
    WheelOdometer = realsense_sys::rs2_extension_RS2_EXTENSION_WHEEL_ODOMETER,
    GlobalTimer = realsense_sys::rs2_extension_RS2_EXTENSION_GLOBAL_TIMER,
    Updatable = realsense_sys::rs2_extension_RS2_EXTENSION_UPDATABLE,
    Count = realsense_sys::rs2_extension_RS2_EXTENSION_COUNT,
    Tm2 = realsense_sys::rs2_extension_RS2_EXTENSION_TM2,
    Unknown = realsense_sys::rs2_extension_RS2_EXTENSION_UNKNOWN,
    Debug = realsense_sys::rs2_extension_RS2_EXTENSION_DEBUG,
    Info = realsense_sys::rs2_extension_RS2_EXTENSION_INFO,
    Motion = realsense_sys::rs2_extension_RS2_EXTENSION_MOTION,
    Options = realsense_sys::rs2_extension_RS2_EXTENSION_OPTIONS,
    Video = realsense_sys::rs2_extension_RS2_EXTENSION_VIDEO,
    Roi = realsense_sys::rs2_extension_RS2_EXTENSION_ROI,
}

/// The enumeration of sensor information.
#[repr(u32)]
#[derive(FromPrimitive, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CameraInfo {
    Name = realsense_sys::rs2_camera_info_RS2_CAMERA_INFO_NAME,
    SerialNumber = realsense_sys::rs2_camera_info_RS2_CAMERA_INFO_SERIAL_NUMBER,
    FirmwareVersion = realsense_sys::rs2_camera_info_RS2_CAMERA_INFO_FIRMWARE_VERSION,
    RecommendedFirmwareVersion =
        realsense_sys::rs2_camera_info_RS2_CAMERA_INFO_RECOMMENDED_FIRMWARE_VERSION,
    PhysicalPort = realsense_sys::rs2_camera_info_RS2_CAMERA_INFO_PHYSICAL_PORT,
    DebugOpCode = realsense_sys::rs2_camera_info_RS2_CAMERA_INFO_DEBUG_OP_CODE,
    AdvancedMode = realsense_sys::rs2_camera_info_RS2_CAMERA_INFO_ADVANCED_MODE,
    ProductId = realsense_sys::rs2_camera_info_RS2_CAMERA_INFO_PRODUCT_ID,
    CameraLocked = realsense_sys::rs2_camera_info_RS2_CAMERA_INFO_CAMERA_LOCKED,
    UsbTypeDescriptor = realsense_sys::rs2_camera_info_RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR,
    ProductLine = realsense_sys::rs2_camera_info_RS2_CAMERA_INFO_PRODUCT_LINE,
    AsicSerialNumber = realsense_sys::rs2_camera_info_RS2_CAMERA_INFO_ASIC_SERIAL_NUMBER,
    FirmwareUpdateId = realsense_sys::rs2_camera_info_RS2_CAMERA_INFO_FIRMWARE_UPDATE_ID,
    Count = realsense_sys::rs2_camera_info_RS2_CAMERA_INFO_COUNT,
}

/// The enumeration of all categories of stream.
#[repr(u32)]
#[derive(FromPrimitive, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum StreamKind {
    Any = realsense_sys::rs2_stream_RS2_STREAM_ANY,
    Depth = realsense_sys::rs2_stream_RS2_STREAM_DEPTH,
    Color = realsense_sys::rs2_stream_RS2_STREAM_COLOR,
    Infrared = realsense_sys::rs2_stream_RS2_STREAM_INFRARED,
    Fisheye = realsense_sys::rs2_stream_RS2_STREAM_FISHEYE,
    Gyro = realsense_sys::rs2_stream_RS2_STREAM_GYRO,
    Accel = realsense_sys::rs2_stream_RS2_STREAM_ACCEL,
    Gpio = realsense_sys::rs2_stream_RS2_STREAM_GPIO,
    Pose = realsense_sys::rs2_stream_RS2_STREAM_POSE,
    Confidence = realsense_sys::rs2_stream_RS2_STREAM_CONFIDENCE,
    Count = realsense_sys::rs2_stream_RS2_STREAM_COUNT,
}

/// The enumeration of frame data format.
#[repr(u32)]
#[derive(FromPrimitive, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Format {
    Any = realsense_sys::rs2_format_RS2_FORMAT_ANY,
    Yuyv = realsense_sys::rs2_format_RS2_FORMAT_YUYV,
    Uyvy = realsense_sys::rs2_format_RS2_FORMAT_UYVY,
    MotionRaw = realsense_sys::rs2_format_RS2_FORMAT_MOTION_RAW,
    GpioRaw = realsense_sys::rs2_format_RS2_FORMAT_GPIO_RAW,
    Distance = realsense_sys::rs2_format_RS2_FORMAT_DISTANCE,
    Mjpeg = realsense_sys::rs2_format_RS2_FORMAT_MJPEG,
    Inzi = realsense_sys::rs2_format_RS2_FORMAT_INZI,
    Invi = realsense_sys::rs2_format_RS2_FORMAT_INVI,
    Count = realsense_sys::rs2_format_RS2_FORMAT_COUNT,
    _6Dof = realsense_sys::rs2_format_RS2_FORMAT_6DOF,
    Bgr8 = realsense_sys::rs2_format_RS2_FORMAT_BGR8,
    Bgra8 = realsense_sys::rs2_format_RS2_FORMAT_BGRA8,
    Disparity16 = realsense_sys::rs2_format_RS2_FORMAT_DISPARITY16,
    Disparity32 = realsense_sys::rs2_format_RS2_FORMAT_DISPARITY32,
    MotionXyz32F = realsense_sys::rs2_format_RS2_FORMAT_MOTION_XYZ32F,
    Raw8 = realsense_sys::rs2_format_RS2_FORMAT_RAW8,
    Raw10 = realsense_sys::rs2_format_RS2_FORMAT_RAW10,
    Raw16 = realsense_sys::rs2_format_RS2_FORMAT_RAW16,
    Rgb8 = realsense_sys::rs2_format_RS2_FORMAT_RGB8,
    Rgba8 = realsense_sys::rs2_format_RS2_FORMAT_RGBA8,
    W10 = realsense_sys::rs2_format_RS2_FORMAT_W10,
    Xyz32F = realsense_sys::rs2_format_RS2_FORMAT_XYZ32F,
    Y8 = realsense_sys::rs2_format_RS2_FORMAT_Y8,
    Y8I = realsense_sys::rs2_format_RS2_FORMAT_Y8I,
    Y10Bpack = realsense_sys::rs2_format_RS2_FORMAT_Y10BPACK,
    Y12I = realsense_sys::rs2_format_RS2_FORMAT_Y12I,
    Y16 = realsense_sys::rs2_format_RS2_FORMAT_Y16,
    Z16 = realsense_sys::rs2_format_RS2_FORMAT_Z16,
}
