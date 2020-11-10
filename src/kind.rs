//! Defines the common used enums.

use crate::common::*;

/// The enumeration of options.
#[repr(u32)]
#[derive(FromPrimitive, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Rs2Option {
    BacklightCompensation = sys::rs2_option_RS2_OPTION_BACKLIGHT_COMPENSATION,
    Brightness = sys::rs2_option_RS2_OPTION_BRIGHTNESS,
    Contrast = sys::rs2_option_RS2_OPTION_CONTRAST,
    Exposure = sys::rs2_option_RS2_OPTION_EXPOSURE,
    Gain = sys::rs2_option_RS2_OPTION_GAIN,
    Gamma = sys::rs2_option_RS2_OPTION_GAMMA,
    Hue = sys::rs2_option_RS2_OPTION_HUE,
    Saturation = sys::rs2_option_RS2_OPTION_SATURATION,
    Sharpness = sys::rs2_option_RS2_OPTION_SHARPNESS,
    WhiteBalance = sys::rs2_option_RS2_OPTION_WHITE_BALANCE,
    EnableAutoExposure = sys::rs2_option_RS2_OPTION_ENABLE_AUTO_EXPOSURE,
    EnableAutoWhiteBalance = sys::rs2_option_RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE,
    VisualPreset = sys::rs2_option_RS2_OPTION_VISUAL_PRESET,
    LaserPower = sys::rs2_option_RS2_OPTION_LASER_POWER,
    Accuracy = sys::rs2_option_RS2_OPTION_ACCURACY,
    MotionRange = sys::rs2_option_RS2_OPTION_MOTION_RANGE,
    FilterOption = sys::rs2_option_RS2_OPTION_FILTER_OPTION,
    ConfidenceThreshold = sys::rs2_option_RS2_OPTION_CONFIDENCE_THRESHOLD,
    EmitterEnabled = sys::rs2_option_RS2_OPTION_EMITTER_ENABLED,
    FramesQueueSize = sys::rs2_option_RS2_OPTION_FRAMES_QUEUE_SIZE,
    TotalFrameDrops = sys::rs2_option_RS2_OPTION_TOTAL_FRAME_DROPS,
    AutoExposureMode = sys::rs2_option_RS2_OPTION_AUTO_EXPOSURE_MODE,
    PowerLineFrequency = sys::rs2_option_RS2_OPTION_POWER_LINE_FREQUENCY,
    AsicTemperature = sys::rs2_option_RS2_OPTION_ASIC_TEMPERATURE,
    ErrorPollingEnabled = sys::rs2_option_RS2_OPTION_ERROR_POLLING_ENABLED,
    ProjectorTemperature = sys::rs2_option_RS2_OPTION_PROJECTOR_TEMPERATURE,
    OutputTriggerEnabled = sys::rs2_option_RS2_OPTION_OUTPUT_TRIGGER_ENABLED,
    MotionModuleTemperature = sys::rs2_option_RS2_OPTION_MOTION_MODULE_TEMPERATURE,
    DepthUnits = sys::rs2_option_RS2_OPTION_DEPTH_UNITS,
    EnableMotionCorrection = sys::rs2_option_RS2_OPTION_ENABLE_MOTION_CORRECTION,
    AutoExposurePriority = sys::rs2_option_RS2_OPTION_AUTO_EXPOSURE_PRIORITY,
    ColorScheme = sys::rs2_option_RS2_OPTION_COLOR_SCHEME,
    HistogramEqualizationEnabled = sys::rs2_option_RS2_OPTION_HISTOGRAM_EQUALIZATION_ENABLED,
    MinDistance = sys::rs2_option_RS2_OPTION_MIN_DISTANCE,
    MaxDistance = sys::rs2_option_RS2_OPTION_MAX_DISTANCE,
    TextureSource = sys::rs2_option_RS2_OPTION_TEXTURE_SOURCE,
    FilterMagnitude = sys::rs2_option_RS2_OPTION_FILTER_MAGNITUDE,
    FilterSmoothAlpha = sys::rs2_option_RS2_OPTION_FILTER_SMOOTH_ALPHA,
    FilterSmoothDelta = sys::rs2_option_RS2_OPTION_FILTER_SMOOTH_DELTA,
    HolesFill = sys::rs2_option_RS2_OPTION_HOLES_FILL,
    StereoBaseline = sys::rs2_option_RS2_OPTION_STEREO_BASELINE,
    AutoExposureConvergeStep = sys::rs2_option_RS2_OPTION_AUTO_EXPOSURE_CONVERGE_STEP,
    InterCamSyncMode = sys::rs2_option_RS2_OPTION_INTER_CAM_SYNC_MODE,
    StreamFilter = sys::rs2_option_RS2_OPTION_STREAM_FILTER,
    StreamFormatFilter = sys::rs2_option_RS2_OPTION_STREAM_FORMAT_FILTER,
    StreamIndexFilter = sys::rs2_option_RS2_OPTION_STREAM_INDEX_FILTER,
    EmitterOnOff = sys::rs2_option_RS2_OPTION_EMITTER_ON_OFF,
    ZeroOrderPointX = sys::rs2_option_RS2_OPTION_ZERO_ORDER_POINT_X,
    ZeroOrderPointY = sys::rs2_option_RS2_OPTION_ZERO_ORDER_POINT_Y,
    LldTemperature = sys::rs2_option_RS2_OPTION_LLD_TEMPERATURE,
    McTemperature = sys::rs2_option_RS2_OPTION_MC_TEMPERATURE,
    MaTemperature = sys::rs2_option_RS2_OPTION_MA_TEMPERATURE,
    HardwarePreset = sys::rs2_option_RS2_OPTION_HARDWARE_PRESET,
    GlobalTimeEnabled = sys::rs2_option_RS2_OPTION_GLOBAL_TIME_ENABLED,
    ApdTemperature = sys::rs2_option_RS2_OPTION_APD_TEMPERATURE,
    EnableMapping = sys::rs2_option_RS2_OPTION_ENABLE_MAPPING,
    EnableRelocalization = sys::rs2_option_RS2_OPTION_ENABLE_RELOCALIZATION,
    EnablePoseJumping = sys::rs2_option_RS2_OPTION_ENABLE_POSE_JUMPING,
    EnableDynamicCalibration = sys::rs2_option_RS2_OPTION_ENABLE_DYNAMIC_CALIBRATION,
    DepthOffset = sys::rs2_option_RS2_OPTION_DEPTH_OFFSET,
    LedPower = sys::rs2_option_RS2_OPTION_LED_POWER,
    ZeroOrderEnabled = sys::rs2_option_RS2_OPTION_ZERO_ORDER_ENABLED,
    EnableMapPreservation = sys::rs2_option_RS2_OPTION_ENABLE_MAP_PRESERVATION,
    Count = sys::rs2_option_RS2_OPTION_COUNT,
}

impl Rs2Option {
    pub fn to_cstr(&self) -> &'static CStr {
        unsafe {
            let ptr = sys::rs2_option_to_string(*self as sys::rs2_option);
            CStr::from_ptr(ptr)
        }
    }

    pub fn to_str(&self) -> &'static str {
        self.to_cstr().to_str().unwrap()
    }
}

impl ToString for Rs2Option {
    fn to_string(&self) -> String {
        self.to_str().to_owned()
    }
}

/// The enumeration of timestamp domains.
#[repr(u32)]
#[derive(FromPrimitive, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum TimestampDomain {
    HardwareClock = sys::rs2_timestamp_domain_RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK,
    SystemTime = sys::rs2_timestamp_domain_RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME,
    GlobalTime = sys::rs2_timestamp_domain_RS2_TIMESTAMP_DOMAIN_GLOBAL_TIME,
    Count = sys::rs2_timestamp_domain_RS2_TIMESTAMP_DOMAIN_COUNT,
}

impl TimestampDomain {
    pub fn as_cstr(&self) -> &'static CStr {
        unsafe {
            let ptr = sys::rs2_timestamp_domain_to_string(*self as sys::rs2_timestamp_domain);
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
    FrameCounter = sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_FRAME_COUNTER,
    FrameTimestamp = sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_FRAME_TIMESTAMP,
    SensorTimestamp = sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_SENSOR_TIMESTAMP,
    ActualExposure = sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_ACTUAL_EXPOSURE,
    GainLevel = sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_GAIN_LEVEL,
    AutoExposure = sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_AUTO_EXPOSURE,
    WhiteBalance = sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_WHITE_BALANCE,
    TimeOfArrival = sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_TIME_OF_ARRIVAL,
    Temperature = sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_TEMPERATURE,
    BackendTimestamp = sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_BACKEND_TIMESTAMP,
    ActualFps = sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_ACTUAL_FPS,
    FrameLaserPower = sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_FRAME_LASER_POWER,
    FrameLaserPowerMode = sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_FRAME_LASER_POWER_MODE,
    ExposurePriority = sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_EXPOSURE_PRIORITY,
    ExposureRoiLeft = sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_EXPOSURE_ROI_LEFT,
    ExposureRoiRight = sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_EXPOSURE_ROI_RIGHT,
    ExposureRoiTop = sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_EXPOSURE_ROI_TOP,
    ExposureRoiBottom = sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_EXPOSURE_ROI_BOTTOM,
    Brightness = sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_BRIGHTNESS,
    Contrast = sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_CONTRAST,
    Saturation = sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_SATURATION,
    Sharpness = sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_SHARPNESS,
    AutoWhiteBalanceTemperature =
        sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_AUTO_WHITE_BALANCE_TEMPERATURE,
    BacklightCompensation = sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_BACKLIGHT_COMPENSATION,
    Hue = sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_HUE,
    Gamma = sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_GAMMA,
    ManualWhiteBalance = sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_MANUAL_WHITE_BALANCE,
    PowerLineFrequency = sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_POWER_LINE_FREQUENCY,
    LowLightCompensation = sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_LOW_LIGHT_COMPENSATION,
    FrameEmitterMode = sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_FRAME_EMITTER_MODE,
    FrameLedPower = sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_FRAME_LED_POWER,
    Count = sys::rs2_frame_metadata_value_RS2_FRAME_METADATA_COUNT,
}

/// The enumeration of extensions.
#[repr(u32)]
#[derive(FromPrimitive, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Extension {
    // sensor
    ColorSensor = sys::rs2_extension_RS2_EXTENSION_COLOR_SENSOR,
    MotionSensor = sys::rs2_extension_RS2_EXTENSION_MOTION_SENSOR,
    FishEyeSensor = sys::rs2_extension_RS2_EXTENSION_FISHEYE_SENSOR,
    DepthSensor = sys::rs2_extension_RS2_EXTENSION_DEPTH_SENSOR,
    DepthStereoSensor = sys::rs2_extension_RS2_EXTENSION_DEPTH_STEREO_SENSOR,
    SoftwareSensor = sys::rs2_extension_RS2_EXTENSION_SOFTWARE_SENSOR,
    PoseSensor = sys::rs2_extension_RS2_EXTENSION_POSE_SENSOR,
    L500DepthSensor = sys::rs2_extension_RS2_EXTENSION_L500_DEPTH_SENSOR,
    Tm2Sensor = sys::rs2_extension_RS2_EXTENSION_TM2_SENSOR,
    // frame
    VideoFrame = sys::rs2_extension_RS2_EXTENSION_VIDEO_FRAME,
    MotionFrame = sys::rs2_extension_RS2_EXTENSION_MOTION_FRAME,
    CompositeFrame = sys::rs2_extension_RS2_EXTENSION_COMPOSITE_FRAME,
    DepthFrame = sys::rs2_extension_RS2_EXTENSION_DEPTH_FRAME,
    DisparityFrame = sys::rs2_extension_RS2_EXTENSION_DISPARITY_FRAME,
    PoseFrame = sys::rs2_extension_RS2_EXTENSION_POSE_FRAME,
    Points = sys::rs2_extension_RS2_EXTENSION_POINTS,
    // filter
    DecimationFilter = sys::rs2_extension_RS2_EXTENSION_DECIMATION_FILTER,
    ThresholdFilter = sys::rs2_extension_RS2_EXTENSION_THRESHOLD_FILTER,
    DisparityFilter = sys::rs2_extension_RS2_EXTENSION_DISPARITY_FILTER,
    SpatialFilter = sys::rs2_extension_RS2_EXTENSION_SPATIAL_FILTER,
    TemporalFilter = sys::rs2_extension_RS2_EXTENSION_TEMPORAL_FILTER,
    HoleFillingFilter = sys::rs2_extension_RS2_EXTENSION_HOLE_FILLING_FILTER,
    ZeroOrderFilter = sys::rs2_extension_RS2_EXTENSION_ZERO_ORDER_FILTER,
    RecommendedFilters = sys::rs2_extension_RS2_EXTENSION_RECOMMENDED_FILTERS,
    // profile
    VideoProfile = sys::rs2_extension_RS2_EXTENSION_VIDEO_PROFILE,
    MotionProfile = sys::rs2_extension_RS2_EXTENSION_MOTION_PROFILE,
    PoseProfile = sys::rs2_extension_RS2_EXTENSION_POSE_PROFILE,
    // device
    SoftwareDevice = sys::rs2_extension_RS2_EXTENSION_SOFTWARE_DEVICE,
    UpdateDevice = sys::rs2_extension_RS2_EXTENSION_UPDATE_DEVICE,
    AutoCalibratedDevice = sys::rs2_extension_RS2_EXTENSION_AUTO_CALIBRATED_DEVICE,
    // misc
    AdvancedMode = sys::rs2_extension_RS2_EXTENSION_ADVANCED_MODE,
    Record = sys::rs2_extension_RS2_EXTENSION_RECORD,
    Playback = sys::rs2_extension_RS2_EXTENSION_PLAYBACK,
    Pose = sys::rs2_extension_RS2_EXTENSION_POSE,
    WheelOdometer = sys::rs2_extension_RS2_EXTENSION_WHEEL_ODOMETER,
    GlobalTimer = sys::rs2_extension_RS2_EXTENSION_GLOBAL_TIMER,
    Updatable = sys::rs2_extension_RS2_EXTENSION_UPDATABLE,
    Count = sys::rs2_extension_RS2_EXTENSION_COUNT,
    Tm2 = sys::rs2_extension_RS2_EXTENSION_TM2,
    Unknown = sys::rs2_extension_RS2_EXTENSION_UNKNOWN,
    Debug = sys::rs2_extension_RS2_EXTENSION_DEBUG,
    Info = sys::rs2_extension_RS2_EXTENSION_INFO,
    Motion = sys::rs2_extension_RS2_EXTENSION_MOTION,
    Options = sys::rs2_extension_RS2_EXTENSION_OPTIONS,
    Video = sys::rs2_extension_RS2_EXTENSION_VIDEO,
    Roi = sys::rs2_extension_RS2_EXTENSION_ROI,
}

/// The enumeration of sensor information.
#[repr(u32)]
#[derive(FromPrimitive, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CameraInfo {
    Name = sys::rs2_camera_info_RS2_CAMERA_INFO_NAME,
    SerialNumber = sys::rs2_camera_info_RS2_CAMERA_INFO_SERIAL_NUMBER,
    FirmwareVersion = sys::rs2_camera_info_RS2_CAMERA_INFO_FIRMWARE_VERSION,
    RecommendedFirmwareVersion = sys::rs2_camera_info_RS2_CAMERA_INFO_RECOMMENDED_FIRMWARE_VERSION,
    PhysicalPort = sys::rs2_camera_info_RS2_CAMERA_INFO_PHYSICAL_PORT,
    DebugOpCode = sys::rs2_camera_info_RS2_CAMERA_INFO_DEBUG_OP_CODE,
    AdvancedMode = sys::rs2_camera_info_RS2_CAMERA_INFO_ADVANCED_MODE,
    ProductId = sys::rs2_camera_info_RS2_CAMERA_INFO_PRODUCT_ID,
    CameraLocked = sys::rs2_camera_info_RS2_CAMERA_INFO_CAMERA_LOCKED,
    UsbTypeDescriptor = sys::rs2_camera_info_RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR,
    ProductLine = sys::rs2_camera_info_RS2_CAMERA_INFO_PRODUCT_LINE,
    AsicSerialNumber = sys::rs2_camera_info_RS2_CAMERA_INFO_ASIC_SERIAL_NUMBER,
    FirmwareUpdateId = sys::rs2_camera_info_RS2_CAMERA_INFO_FIRMWARE_UPDATE_ID,
    Count = sys::rs2_camera_info_RS2_CAMERA_INFO_COUNT,
}

/// The enumeration of all categories of stream.
#[repr(u32)]
#[derive(FromPrimitive, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum StreamKind {
    Any = sys::rs2_stream_RS2_STREAM_ANY,
    Depth = sys::rs2_stream_RS2_STREAM_DEPTH,
    Color = sys::rs2_stream_RS2_STREAM_COLOR,
    Infrared = sys::rs2_stream_RS2_STREAM_INFRARED,
    Fisheye = sys::rs2_stream_RS2_STREAM_FISHEYE,
    Gyro = sys::rs2_stream_RS2_STREAM_GYRO,
    Accel = sys::rs2_stream_RS2_STREAM_ACCEL,
    Gpio = sys::rs2_stream_RS2_STREAM_GPIO,
    Pose = sys::rs2_stream_RS2_STREAM_POSE,
    Confidence = sys::rs2_stream_RS2_STREAM_CONFIDENCE,
    Count = sys::rs2_stream_RS2_STREAM_COUNT,
}

/// The enumeration of frame data format.
#[repr(u32)]
#[derive(FromPrimitive, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Format {
    Any = sys::rs2_format_RS2_FORMAT_ANY,
    Yuyv = sys::rs2_format_RS2_FORMAT_YUYV,
    Uyvy = sys::rs2_format_RS2_FORMAT_UYVY,
    MotionRaw = sys::rs2_format_RS2_FORMAT_MOTION_RAW,
    GpioRaw = sys::rs2_format_RS2_FORMAT_GPIO_RAW,
    Distance = sys::rs2_format_RS2_FORMAT_DISTANCE,
    Mjpeg = sys::rs2_format_RS2_FORMAT_MJPEG,
    Inzi = sys::rs2_format_RS2_FORMAT_INZI,
    Invi = sys::rs2_format_RS2_FORMAT_INVI,
    Count = sys::rs2_format_RS2_FORMAT_COUNT,
    _6Dof = sys::rs2_format_RS2_FORMAT_6DOF,
    Bgr8 = sys::rs2_format_RS2_FORMAT_BGR8,
    Bgra8 = sys::rs2_format_RS2_FORMAT_BGRA8,
    Disparity16 = sys::rs2_format_RS2_FORMAT_DISPARITY16,
    Disparity32 = sys::rs2_format_RS2_FORMAT_DISPARITY32,
    MotionXyz32F = sys::rs2_format_RS2_FORMAT_MOTION_XYZ32F,
    Raw8 = sys::rs2_format_RS2_FORMAT_RAW8,
    Raw10 = sys::rs2_format_RS2_FORMAT_RAW10,
    Raw16 = sys::rs2_format_RS2_FORMAT_RAW16,
    Rgb8 = sys::rs2_format_RS2_FORMAT_RGB8,
    Rgba8 = sys::rs2_format_RS2_FORMAT_RGBA8,
    W10 = sys::rs2_format_RS2_FORMAT_W10,
    Xyz32F = sys::rs2_format_RS2_FORMAT_XYZ32F,
    Y8 = sys::rs2_format_RS2_FORMAT_Y8,
    Y8I = sys::rs2_format_RS2_FORMAT_Y8I,
    Y10Bpack = sys::rs2_format_RS2_FORMAT_Y10BPACK,
    Y12I = sys::rs2_format_RS2_FORMAT_Y12I,
    Y16 = sys::rs2_format_RS2_FORMAT_Y16,
    Z16 = sys::rs2_format_RS2_FORMAT_Z16,
}

/// The enumeration of color schemes.
#[repr(usize)]
#[derive(FromPrimitive, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ColorScheme {
    Jet = 0,
    Classic = 1,
    WhiteToBlack = 2,
    BlackToWhite = 3,
    Bio = 4,
    Cold = 5,
    Warm = 6,
    Quantized = 7,
    Pattern = 8,
    Hue = 9,
}

/// The enumeration of persistence controls.
#[repr(usize)]
#[derive(FromPrimitive, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum PersistenceControl {
    Disabled = 0,
    Valid8OutOf8 = 1,
    Valid2OutOf3 = 2,
    Valid2OutOf4 = 3,
    Valid2OutOf8 = 4,
    Valid1OutOf2 = 5,
    Valid1OutOf5 = 6,
    Valid1OutOf8 = 7,
    Indefinitely = 8,
}

/// The enumeration of persistence controls.
#[repr(usize)]
#[derive(FromPrimitive, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum HoleFillingMode {
    FillFromLeft = 0,
    FarestFromAround = 1,
    NearestFromAround = 2,
}
