#![no_std]

use core::panic::PanicInfo;

// External C functions from NuttX
extern "C" {
    fn syslog(priority: i32, format: *const u8, ...);
    // NuttX message queue functions
    fn mq_send(mqdes: i32, msg: *const u8, msglen: usize, prio: u32) -> i32;
    fn mq_receive(mqdes: i32, msg: *mut u8, msglen: usize, prio: *mut u32) -> isize;
    fn mq_getattr(mqdes: i32, mq_stat: *mut MqAttr) -> i32;
}

// Message queue attributes structure (mirrors NuttX mq_attr)
#[repr(C)]
pub struct MqAttr {
    pub mq_maxmsg: i32,
    pub mq_msgsize: i32,
    pub mq_flags: i32,
    pub mq_curmsgs: i32,
}

// Syslog priority levels (from NuttX syslog.h)
const LOG_INFO: i32 = 6;
const LOG_WARNING: i32 = 4;
const LOG_ERR: i32 = 3;

// Message types for queue communication
#[repr(C)]
#[derive(Clone, Copy)]
pub enum MessageType {
    SensorData = 1,
    ProcessedData = 2,
    HealthCheck = 3,
    StringProcess = 4,
    ArrayProcess = 5,
}

// Message structure for queue communication
#[repr(C)]
#[derive(Clone, Copy)]
pub struct QueueMessage {
    pub msg_type: u32,
    pub timestamp: u64,
    pub data: MessageData,
}

// Union-like structure for different message data types
#[repr(C)]
#[derive(Clone, Copy)]
pub union MessageData {
    pub sensor: SharedSensorData,
    pub processed: RustProcessedData,
    pub health: HealthData,
    pub string_result: StringProcessResult,
    pub array_result: ArrayProcessResult,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct HealthData {
    pub score: u32,
    pub issues: u32,
    pub battery_status: u32,
    pub temperature_status: u32,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct StringProcessResult {
    pub checksum: u32,
    pub vowel_count: u32,
    pub length: u32,
    pub status: u32,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct ArrayProcessResult {
    pub average: i32,
    pub min_val: i32,
    pub max_val: i32,
    pub count: u32,
}

// Shared data structure for inter-thread communication
#[repr(C)]
#[derive(Clone, Copy)]
pub struct SharedSensorData {
    pub temperature: f32,
    pub humidity: f32,
    pub pressure: f32,
    pub timestamp: u64,
    pub battery_voltage: f32,
    pub status_flags: u32,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct RustProcessedData {
    pub avg_temperature: f32,
    pub trend_direction: i32,  // -1: decreasing, 0: stable, 1: increasing
    pub alert_level: u32,      // 0: normal, 1: warning, 2: critical
    pub data_quality_score: u32, // 0-100
}

// Static storage for shared data (thread-safe access through queues)
static mut SENSOR_DATA: SharedSensorData = SharedSensorData {
    temperature: 0.0,
    humidity: 0.0,
    pressure: 0.0,
    timestamp: 0,
    battery_voltage: 0.0,
    status_flags: 0,
};

static mut PROCESSED_DATA: RustProcessedData = RustProcessedData {
    avg_temperature: 0.0,
    trend_direction: 0,
    alert_level: 0,
    data_quality_score: 100,
};

// Temperature history for trend analysis (circular buffer)
static mut TEMP_HISTORY: [f32; 10] = [0.0; 10];
static mut TEMP_INDEX: usize = 0;

// Message queue descriptors (set by C code)
static mut RUST_TO_C_QUEUE: i32 = -1;
static mut C_TO_RUST_QUEUE: i32 = -1;

// Panic handler (required for no_std)
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}

// Drone-specific data structures for IMU and control systems
#[repr(C)]
#[derive(Clone, Copy)]
pub struct ImuRawData {
    pub accel_x: f32,    // Accelerometer X-axis (m/s²)
    pub accel_y: f32,    // Accelerometer Y-axis (m/s²)
    pub accel_z: f32,    // Accelerometer Z-axis (m/s²)
    pub gyro_x: f32,     // Gyroscope X-axis (rad/s)
    pub gyro_y: f32,     // Gyroscope Y-axis (rad/s)
    pub gyro_z: f32,     // Gyroscope Z-axis (rad/s)
    pub mag_x: f32,      // Magnetometer X-axis (μT)
    pub mag_y: f32,      // Magnetometer Y-axis (μT)
    pub mag_z: f32,      // Magnetometer Z-axis (μT)
    pub timestamp: u64,  // Timestamp in microseconds
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct ImuFilteredData {
    pub roll: f32,       // Roll angle in radians
    pub pitch: f32,      // Pitch angle in radians
    pub yaw: f32,        // Yaw angle in radians
    pub roll_rate: f32,  // Roll rate in rad/s
    pub pitch_rate: f32, // Pitch rate in rad/s
    pub yaw_rate: f32,   // Yaw rate in rad/s
    pub confidence: f32, // Filter confidence (0.0 - 1.0)
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct PidController {
    pub kp: f32,         // Proportional gain
    pub ki: f32,         // Integral gain
    pub kd: f32,         // Derivative gain
    pub integral: f32,   // Integral accumulator
    pub last_error: f32, // Previous error for derivative
    pub output_limit: f32, // Output saturation limit
}

// Static storage for drone control systems
static mut IMU_RAW_DATA: ImuRawData = ImuRawData {
    accel_x: 0.0, accel_y: 0.0, accel_z: 0.0,
    gyro_x: 0.0, gyro_y: 0.0, gyro_z: 0.0,
    mag_x: 0.0, mag_y: 0.0, mag_z: 0.0,
    timestamp: 0,
};

static mut IMU_FILTERED_DATA: ImuFilteredData = ImuFilteredData {
    roll: 0.0, pitch: 0.0, yaw: 0.0,
    roll_rate: 0.0, pitch_rate: 0.0, yaw_rate: 0.0,
    confidence: 1.0,
};

// Gaussian filter buffers for IMU data smoothing
static mut ACCEL_FILTER_BUFFER_X: [f32; 5] = [0.0; 5];
static mut ACCEL_FILTER_BUFFER_Y: [f32; 5] = [0.0; 5];
static mut ACCEL_FILTER_BUFFER_Z: [f32; 5] = [0.0; 5];
static mut GYRO_FILTER_BUFFER_X: [f32; 5] = [0.0; 5];
static mut GYRO_FILTER_BUFFER_Y: [f32; 5] = [0.0; 5];
static mut GYRO_FILTER_BUFFER_Z: [f32; 5] = [0.0; 5];
static mut FILTER_INDEX: usize = 0;

// Complementary filter state
static mut COMP_FILTER_ANGLE_X: f32 = 0.0;
static mut COMP_FILTER_ANGLE_Y: f32 = 0.0;
static mut LAST_FILTER_TIME: u64 = 0;

// PID controllers for drone stabilization
static mut ROLL_PID: PidController = PidController {
    kp: 2.0, ki: 0.1, kd: 0.05, integral: 0.0, last_error: 0.0, output_limit: 45.0
};
static mut PITCH_PID: PidController = PidController {
    kp: 2.0, ki: 0.1, kd: 0.05, integral: 0.0, last_error: 0.0, output_limit: 45.0
};
static mut YAW_PID: PidController = PidController {
    kp: 1.5, ki: 0.05, kd: 0.02, integral: 0.0, last_error: 0.0, output_limit: 30.0
};

/// Gaussian filter for IMU data smoothing - replaces basic calculation
#[no_mangle]
pub extern "C" fn rust_gaussian_filter_imu(accel_x: f32, accel_y: f32, accel_z: f32,
                                          gyro_x: f32, gyro_y: f32, gyro_z: f32) -> i32 {
    unsafe {
        // Store raw IMU data in circular buffers for Gaussian filtering
        ACCEL_FILTER_BUFFER_X[FILTER_INDEX] = accel_x;
        ACCEL_FILTER_BUFFER_Y[FILTER_INDEX] = accel_y;
        ACCEL_FILTER_BUFFER_Z[FILTER_INDEX] = accel_z;
        GYRO_FILTER_BUFFER_X[FILTER_INDEX] = gyro_x;
        GYRO_FILTER_BUFFER_Y[FILTER_INDEX] = gyro_y;
        GYRO_FILTER_BUFFER_Z[FILTER_INDEX] = gyro_z;

        FILTER_INDEX = (FILTER_INDEX + 1) % 5;

        // Apply Gaussian weights: [0.06, 0.24, 0.40, 0.24, 0.06] (approximation)
        let weights = [0.06, 0.24, 0.40, 0.24, 0.06];

        // Calculate filtered accelerometer values
        let mut filtered_accel_x = 0.0;
        let mut filtered_accel_y = 0.0;
        let mut filtered_accel_z = 0.0;

        for i in 0..5 {
            filtered_accel_x += ACCEL_FILTER_BUFFER_X[i] * weights[i];
            filtered_accel_y += ACCEL_FILTER_BUFFER_Y[i] * weights[i];
            filtered_accel_z += ACCEL_FILTER_BUFFER_Z[i] * weights[i];
        }

        // Store filtered data
        IMU_RAW_DATA.accel_x = filtered_accel_x;
        IMU_RAW_DATA.accel_y = filtered_accel_y;
        IMU_RAW_DATA.accel_z = filtered_accel_z;

        // Convert to fixed-point for syslog
        let ax_fp = (filtered_accel_x * 100.0) as i32;
        let ay_fp = (filtered_accel_y * 100.0) as i32;
        let az_fp = (filtered_accel_z * 100.0) as i32;

        syslog(LOG_INFO, b"[Rust App IMU Gaussian Filter] Filtered IMU: ax=%d.%02d, ay=%d.%02d, az=%d.%02d\n\0".as_ptr(),
               ax_fp / 100, ax_fp % 100, ay_fp / 100, ay_fp % 100, az_fp / 100, az_fp % 100);
    }
    0
}

/// Complementary filter for attitude estimation - replaces advanced math
#[no_mangle]
pub extern "C" fn rust_complementary_filter(dt_us: u64, filter_type: u8) -> i32 {
    unsafe {
        let dt_sec = dt_us as f32 / 1_000_000.0; // Convert microseconds to seconds

        syslog(LOG_INFO, b"[Rust App Complementary Filter] Input: dt=%d us, type=%d\n\0".as_ptr(),
               dt_us as i32, filter_type as i32);

        let result = match filter_type {
            0 => { // Roll angle estimation
                // Accelerometer angle (noisy but no drift)
                let accel_angle = libm::atan2f(IMU_RAW_DATA.accel_y, IMU_RAW_DATA.accel_z);

                // Gyroscope integration (smooth but drifts)
                COMP_FILTER_ANGLE_X += IMU_RAW_DATA.gyro_x * dt_sec;

                // Complementary filter: 98% gyro + 2% accelerometer
                COMP_FILTER_ANGLE_X = 0.98 * COMP_FILTER_ANGLE_X + 0.02 * accel_angle;
                IMU_FILTERED_DATA.roll = COMP_FILTER_ANGLE_X;
                IMU_FILTERED_DATA.roll_rate = IMU_RAW_DATA.gyro_x;

                (COMP_FILTER_ANGLE_X * 1000.0) as i32 // Return angle in milliradians
            },
            1 => { // Pitch angle estimation
                let accel_angle = libm::atan2f(-IMU_RAW_DATA.accel_x,
                    libm::sqrtf(IMU_RAW_DATA.accel_y * IMU_RAW_DATA.accel_y + IMU_RAW_DATA.accel_z * IMU_RAW_DATA.accel_z));

                COMP_FILTER_ANGLE_Y += IMU_RAW_DATA.gyro_y * dt_sec;
                COMP_FILTER_ANGLE_Y = 0.98 * COMP_FILTER_ANGLE_Y + 0.02 * accel_angle;
                IMU_FILTERED_DATA.pitch = COMP_FILTER_ANGLE_Y;
                IMU_FILTERED_DATA.pitch_rate = IMU_RAW_DATA.gyro_y;

                (COMP_FILTER_ANGLE_Y * 1000.0) as i32
            },
            2 => { // Yaw rate (gyroscope only, no drift correction without magnetometer)
                IMU_FILTERED_DATA.yaw_rate = IMU_RAW_DATA.gyro_z;
                (IMU_RAW_DATA.gyro_z * 1000.0) as i32
            },
            3 => { // Calculate filter confidence based on accelerometer magnitude
                let accel_magnitude = libm::sqrtf(
                    IMU_RAW_DATA.accel_x * IMU_RAW_DATA.accel_x +
                    IMU_RAW_DATA.accel_y * IMU_RAW_DATA.accel_y +
                    IMU_RAW_DATA.accel_z * IMU_RAW_DATA.accel_z
                );

                // Confidence is higher when accelerometer magnitude is close to 1g (9.81 m/s²)
                let gravity = 9.81;
                let magnitude_error = libm::fabsf(accel_magnitude - gravity) / gravity;
                IMU_FILTERED_DATA.confidence = if magnitude_error < 0.1 { 1.0 }
                                              else if magnitude_error < 0.3 { 0.7 }
                                              else { 0.3 };

                (IMU_FILTERED_DATA.confidence * 1000.0) as i32
            },
            4 => { // Update IMU raw data with gyro values
                IMU_RAW_DATA.gyro_x = GYRO_FILTER_BUFFER_X.iter().sum::<f32>() / 5.0;
                IMU_RAW_DATA.gyro_y = GYRO_FILTER_BUFFER_Y.iter().sum::<f32>() / 5.0;
                IMU_RAW_DATA.gyro_z = GYRO_FILTER_BUFFER_Z.iter().sum::<f32>() / 5.0;

                let gyro_magnitude = libm::sqrtf(
                    IMU_RAW_DATA.gyro_x * IMU_RAW_DATA.gyro_x +
                    IMU_RAW_DATA.gyro_y * IMU_RAW_DATA.gyro_y +
                    IMU_RAW_DATA.gyro_z * IMU_RAW_DATA.gyro_z
                );
                (gyro_magnitude * 1000.0) as i32
            },
            5 => { // Low-pass filter for accelerometer noise reduction
                let alpha = 0.1; // Filter coefficient
                static mut PREV_ACCEL_X: f32 = 0.0;
                static mut PREV_ACCEL_Y: f32 = 0.0;
                static mut PREV_ACCEL_Z: f32 = 0.0;

                PREV_ACCEL_X = alpha * IMU_RAW_DATA.accel_x + (1.0 - alpha) * PREV_ACCEL_X;
                PREV_ACCEL_Y = alpha * IMU_RAW_DATA.accel_y + (1.0 - alpha) * PREV_ACCEL_Y;
                PREV_ACCEL_Z = alpha * IMU_RAW_DATA.accel_z + (1.0 - alpha) * PREV_ACCEL_Z;

                IMU_RAW_DATA.accel_x = PREV_ACCEL_X;
                IMU_RAW_DATA.accel_y = PREV_ACCEL_Y;
                IMU_RAW_DATA.accel_z = PREV_ACCEL_Z;

                let filtered_magnitude = libm::sqrtf(PREV_ACCEL_X * PREV_ACCEL_X +
                                                   PREV_ACCEL_Y * PREV_ACCEL_Y +
                                                   PREV_ACCEL_Z * PREV_ACCEL_Z);
                (filtered_magnitude * 100.0) as i32
            },
            _ => { // Default: Return timestamp difference for debugging
                let time_diff = dt_us.saturating_sub(LAST_FILTER_TIME);
                LAST_FILTER_TIME = dt_us;
                time_diff as i32
            }
        };

        syslog(LOG_INFO, b"[Rust App Complementary Filter] Result: %d\n\0".as_ptr(), result);
        result
    }
}

/// PID Controller for drone stabilization
#[no_mangle]
pub extern "C" fn rust_pid_controller(setpoint: f32, measured_value: f32, dt_ms: u32, controller_type: u8) -> i32 {
    unsafe {
        let dt_sec = dt_ms as f32 / 1000.0; // Convert milliseconds to seconds
        let error = setpoint - measured_value;

        let controller = match controller_type {
            0 => &mut ROLL_PID,  // Roll PID controller
            1 => &mut PITCH_PID, // Pitch PID controller
            2 => &mut YAW_PID,   // Yaw PID controller
            _ => &mut ROLL_PID,  // Default to roll
        };

        // Proportional term
        let p_term = controller.kp * error;

        // Integral term (with windup protection)
        controller.integral += error * dt_sec;
        // Limit integral to prevent windup
        let integral_limit = controller.output_limit / controller.ki;
        if controller.integral > integral_limit {
            controller.integral = integral_limit;
        } else if controller.integral < -integral_limit {
            controller.integral = -integral_limit;
        }
        let i_term = controller.ki * controller.integral;

        // Derivative term
        let d_term = if dt_sec > 0.0 {
            controller.kd * (error - controller.last_error) / dt_sec
        } else {
            0.0
        };
        controller.last_error = error;

        // Calculate PID output
        let mut output = p_term + i_term + d_term;

        // Apply output saturation
        if output > controller.output_limit {
            output = controller.output_limit;
        } else if output < -controller.output_limit {
            output = -controller.output_limit;
        }

        let output_fp = (output * 100.0) as i32;
        let error_fp = (error * 100.0) as i32;
        let p_fp = (p_term * 100.0) as i32;
        let i_fp = (i_term * 100.0) as i32;
        let d_fp = (d_term * 100.0) as i32;

        syslog(LOG_INFO, b"[Rust App PID Controller] PID[%d]: err=%d.%02d, P=%d.%02d, I=%d.%02d, D=%d.%02d, out=%d.%02d\n\0".as_ptr(),
               controller_type as i32,
               error_fp / 100, error_fp.abs() % 100,
               p_fp / 100, p_fp.abs() % 100,
               i_fp / 100, i_fp.abs() % 100,
               d_fp / 100, d_fp.abs() % 100,
               output_fp / 100, output_fp.abs() % 100);

        (output * 1000.0) as i32 // Return output in milli-units
    }
}

/// Store sensor data from C thread (simulated sensor readings)
#[no_mangle]
pub extern "C" fn rust_store_sensor_data(temp: f32, humidity: f32, pressure: f32, timestamp: u64, battery: f32) -> i32 {
    unsafe {
        SENSOR_DATA.temperature = temp;
        SENSOR_DATA.humidity = humidity;
        SENSOR_DATA.pressure = pressure;
        SENSOR_DATA.timestamp = timestamp;
        SENSOR_DATA.battery_voltage = battery;

        // Update temperature history for trend analysis
        TEMP_HISTORY[TEMP_INDEX] = temp;
        TEMP_INDEX = (TEMP_INDEX + 1) % 10;

        // Convert floats to fixed-point integers for syslog (multiply by 100 to preserve 2 decimal places)
        let temp_fp = (temp * 100.0) as i32;
        let humidity_fp = (humidity * 100.0) as i32;
        let pressure_fp = (pressure * 100.0) as i32;
        let battery_fp = (battery * 100.0) as i32;

        syslog(LOG_INFO, b"[Rust App Sensor Data Store] Stored: T=%d.%02d, H=%d.%02d, P=%d.%02d, Bat=%d.%02d\n\0".as_ptr(),
               temp_fp / 100, temp_fp % 100,
               humidity_fp / 100, humidity_fp % 100,
               pressure_fp / 100, pressure_fp % 100,
               battery_fp / 100, battery_fp % 100);
    }
    0
}

/// Process stored sensor data and generate analytics
#[no_mangle]
pub extern "C" fn rust_process_sensor_data() -> *const RustProcessedData {
    unsafe {
        // Calculate average temperature from history
        let mut sum = 0.0;
        let mut count = 0;
        for &temp in &TEMP_HISTORY {
            if temp != 0.0 {
                sum += temp;
                count += 1;
            }
        }

        if count > 0 {
            PROCESSED_DATA.avg_temperature = sum / count as f32;
        }

        // Determine trend direction
        if count >= 3 {
            let recent_avg = (TEMP_HISTORY[(TEMP_INDEX + 7) % 10] +
                             TEMP_HISTORY[(TEMP_INDEX + 8) % 10] +
                             TEMP_HISTORY[(TEMP_INDEX + 9) % 10]) / 3.0;
            let older_avg = (TEMP_HISTORY[(TEMP_INDEX + 4) % 10] +
                            TEMP_HISTORY[(TEMP_INDEX + 5) % 10] +
                            TEMP_HISTORY[(TEMP_INDEX + 6) % 10]) / 3.0;

            if recent_avg > older_avg + 1.0 {
                PROCESSED_DATA.trend_direction = 1;  // Increasing
            } else if recent_avg < older_avg - 1.0 {
                PROCESSED_DATA.trend_direction = -1; // Decreasing
            } else {
                PROCESSED_DATA.trend_direction = 0;  // Stable
            }
        }

        // Determine alert level based on temperature and battery
        PROCESSED_DATA.alert_level = if SENSOR_DATA.temperature > 40.0 || SENSOR_DATA.battery_voltage < 3.0 {
            2  // Critical
        } else if SENSOR_DATA.temperature > 35.0 || SENSOR_DATA.battery_voltage < 3.3 {
            1  // Warning
        } else {
            0  // Normal
        };

        // Calculate data quality score based on valid sensor ranges and battery level
        let mut quality_score = 100u32;

        // Check if sensor data is within valid ranges
        if SENSOR_DATA.temperature < -40.0 || SENSOR_DATA.temperature > 80.0 {
            quality_score -= 30;
        }
        if SENSOR_DATA.humidity < 0.0 || SENSOR_DATA.humidity > 100.0 {
            quality_score -= 30;
        }
        if SENSOR_DATA.pressure < 300.0 || SENSOR_DATA.pressure > 1200.0 {
            quality_score -= 30;
        }

        // Battery voltage quality impact
        if SENSOR_DATA.battery_voltage <= 0.0 {
            quality_score = 0; // No battery data
        } else if SENSOR_DATA.battery_voltage < 3.0 {
            quality_score = quality_score.saturating_sub(50);
        } else if SENSOR_DATA.battery_voltage < 3.3 {
            quality_score = quality_score.saturating_sub(20);
        }

        PROCESSED_DATA.data_quality_score = quality_score;

        // Convert floats to fixed-point integers for syslog
        let avg_temp_fp = (PROCESSED_DATA.avg_temperature * 100.0) as i32;

        syslog(LOG_INFO, b"[Rust App Data Analytics] Processed: AvgT=%d.%02d, Trend=%d, Alert=%d, Quality=%d\n\0".as_ptr(),
               avg_temp_fp / 100, avg_temp_fp % 100,
               PROCESSED_DATA.trend_direction,
               PROCESSED_DATA.alert_level,
               PROCESSED_DATA.data_quality_score);

        &PROCESSED_DATA as *const RustProcessedData
    }
}

/// Get current sensor data (for C thread to read)
#[no_mangle]
pub extern "C" fn rust_get_sensor_data() -> *const SharedSensorData {
    unsafe {
        &SENSOR_DATA as *const SharedSensorData
    }
}

/// Validate and filter sensor data
#[no_mangle]
pub extern "C" fn rust_validate_sensor_data(temp: f32, humidity: f32, pressure: f32) -> i32 {
    let is_valid = temp >= -50.0 && temp <= 85.0 &&
                   humidity >= 0.0 && humidity <= 100.0 &&
                   pressure >= 300.0 && pressure <= 1200.0;

    unsafe {
        if is_valid {
            syslog(LOG_INFO, b"[Rust App Sensor Validator] Validation: PASSED\n\0".as_ptr());
        } else {
            syslog(LOG_WARNING, b"[Rust App Sensor Validator] Validation: FAILED\n\0".as_ptr());
        }
    }

    is_valid as i32
}

/// Generate system health report
#[no_mangle]
pub extern "C" fn rust_system_health_check() -> u32 {
    unsafe {
        let mut health_score = 100u32;
        let mut issues = 0u32;

        // Check battery level
        if SENSOR_DATA.battery_voltage < 3.0 {
            health_score -= 30;
            issues |= 0x01; // Battery critical
        } else if SENSOR_DATA.battery_voltage < 3.3 {
            health_score -= 15;
            issues |= 0x02; // Battery low
        }

        // Check temperature range
        if SENSOR_DATA.temperature > 60.0 || SENSOR_DATA.temperature < -20.0 {
            health_score -= 25;
            issues |= 0x04; // Temperature out of range
        }

        // Check data freshness (simplified check)
        if PROCESSED_DATA.data_quality_score < 70 {
            health_score -= 20;
            issues |= 0x08; // Data quality poor
        }

        syslog(LOG_INFO, b"[Rust App System Health Check] Score=%d, Issues=0x%08X\n\0".as_ptr(),
               health_score, issues);

        (health_score << 16) | issues
    }
}

/// String processing function (C to Rust string handling)
#[no_mangle]
pub extern "C" fn rust_process_string(input: *const u8, len: usize) -> u32 {
    if input.is_null() || len == 0 {
        return 0;
    }

    unsafe {
        let slice = core::slice::from_raw_parts(input, len);
        let mut checksum = 0u32;
        let mut vowel_count = 0u32;

        for &byte in slice {
            checksum = checksum.wrapping_add(byte as u32);
            match byte {
                b'a' | b'e' | b'i' | b'o' | b'u' |
                b'A' | b'E' | b'I' | b'O' | b'U' => vowel_count += 1,
                _ => {}
            }
        }

        syslog(LOG_INFO, b"[Rust App String Processor] Processed: len=%d, checksum=%d, vowels=%d\n\0".as_ptr(),
               len as i32, checksum, vowel_count);

        (checksum << 16) | vowel_count
    }
}

/// Array processing function (C array to Rust)
#[no_mangle]
pub extern "C" fn rust_process_array(data: *const i32, len: usize) -> i32 {
    if data.is_null() || len == 0 {
        return -1;
    }

    unsafe {
        let slice = core::slice::from_raw_parts(data, len);
        let sum: i64 = slice.iter().map(|&x| x as i64).sum();
        let avg = sum / len as i64;

        syslog(LOG_INFO, b"[Rust App Array Processor] Processed: len=%d, sum=%ld, avg=%ld\n\0".as_ptr(),
               len as i32, sum as i32, avg as i32);

        avg as i32
    }
}

/// Simulate realistic IMU data for testing (replaces fibonacci)
fn simulate_imu_data(cycle: u32) -> (f32, f32, f32, f32, f32, f32) {
    let time = cycle as f32 * 0.01; // 10ms cycle time

    // Simulate drone movement with some realistic patterns
    let accel_x = 0.5 * libm::sinf(time * 0.5) + 0.1 * libm::sinf(time * 3.0); // ±0.6g
    let accel_y = 0.3 * libm::cosf(time * 0.7) + 0.05 * libm::sinf(time * 5.0); // ±0.35g
    let accel_z = 9.81 + 0.2 * libm::sinf(time * 1.2); // ~1g with small variations

    // Simulate angular rates (rad/s)
    let gyro_x = 0.1 * libm::sinf(time * 0.8) + 0.02 * libm::sinf(time * 10.0); // Roll rate
    let gyro_y = 0.08 * libm::cosf(time * 0.6) + 0.015 * libm::cosf(time * 8.0); // Pitch rate
    let gyro_z = 0.05 * libm::sinf(time * 0.3) + 0.01 * libm::sinf(time * 12.0); // Yaw rate

    (accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z)
}

/// Validate IMU data ranges for drone safety (replaces prime_check)
fn validate_imu_ranges(accel_x: f32, accel_y: f32, accel_z: f32, gyro_x: f32, gyro_y: f32, gyro_z: f32) -> bool {
    // Check accelerometer ranges (reasonable for drone: ±16g)
    let accel_valid = accel_x.abs() <= 156.8 && accel_y.abs() <= 156.8 && accel_z.abs() <= 156.8;

    // Check gyroscope ranges (reasonable for drone: ±2000 deg/s = ±34.9 rad/s)
    let gyro_valid = gyro_x.abs() <= 34.9 && gyro_y.abs() <= 34.9 && gyro_z.abs() <= 34.9;

    // Check for NaN or infinite values
    let no_nan = !accel_x.is_nan() && !accel_y.is_nan() && !accel_z.is_nan() &&
                 !gyro_x.is_nan() && !gyro_y.is_nan() && !gyro_z.is_nan() &&
                 accel_x.is_finite() && accel_y.is_finite() && accel_z.is_finite() &&
                 gyro_x.is_finite() && gyro_y.is_finite() && gyro_z.is_finite();

    accel_valid && gyro_valid && no_nan
}

/// Calculate motor mixing for quadcopter (replaces gcd)
fn calculate_motor_mixing(roll_output: f32, pitch_output: f32, yaw_output: f32, throttle: f32) -> (u32, u32, u32, u32) {
    // Standard X-configuration quadcopter motor mixing
    // Motors: Front-Left, Front-Right, Rear-Left, Rear-Right

    let motor_fl = throttle + pitch_output + roll_output - yaw_output; // Front-Left
    let motor_fr = throttle + pitch_output - roll_output + yaw_output; // Front-Right
    let motor_rl = throttle - pitch_output + roll_output + yaw_output; // Rear-Left
    let motor_rr = throttle - pitch_output - roll_output - yaw_output; // Rear-Right

    // Constrain motor outputs to 0-100% range and convert to integer
    let constrain = |value: f32| -> u32 {
        let clamped = if value < 0.0 { 0.0 } else if value > 100.0 { 100.0 } else { value };
        (clamped * 10.0) as u32 // Return as percentage * 10 (0-1000)
    };

    (constrain(motor_fl), constrain(motor_fr), constrain(motor_rl), constrain(motor_rr))
}

/// Initialize Rust subsystem with message queue descriptors
#[no_mangle]
pub extern "C" fn rust_system_init() -> i32 {
    unsafe {
        syslog(LOG_INFO, b"[Rust App System Init] Board-level Rust drone control system initialized!\n\0".as_ptr());
        syslog(LOG_INFO, b"[Rust App System Init] Available functions: IMU filtering, attitude estimation, PID control\n\0".as_ptr());
        syslog(LOG_INFO, b"[Rust App System Init] Thread-safe message queue communication enabled\n\0".as_ptr());
    }
    0
}

/// Set message queue descriptors for thread-safe communication
#[no_mangle]
pub extern "C" fn rust_set_queues(rust_to_c: i32, c_to_rust: i32) -> i32 {
    unsafe {
        RUST_TO_C_QUEUE = rust_to_c;
        C_TO_RUST_QUEUE = c_to_rust;
        syslog(LOG_INFO, b"[Rust App Message Queue] Queues set: R->C=%d, C->R=%d\n\0".as_ptr(), rust_to_c, c_to_rust);
    }
    0
}

/// Thread-safe function to send sensor data via message queue
#[no_mangle]
pub extern "C" fn rust_send_sensor_data(temp: f32, humidity: f32, pressure: f32, timestamp: u64, battery: f32) -> i32 {
    unsafe {
        if RUST_TO_C_QUEUE < 0 {
            syslog(LOG_ERR, b"[Rust App Queue Sender] Error: Message queue not initialized\n\0".as_ptr());
            return -1;
        }

        // Update local data
        SENSOR_DATA.temperature = temp;
        SENSOR_DATA.humidity = humidity;
        SENSOR_DATA.pressure = pressure;
        SENSOR_DATA.timestamp = timestamp;
        SENSOR_DATA.battery_voltage = battery;

        // Update temperature history for trend analysis
        TEMP_HISTORY[TEMP_INDEX] = temp;
        TEMP_INDEX = (TEMP_INDEX + 1) % 10;

        // Create message
        let msg = QueueMessage {
            msg_type: MessageType::SensorData as u32,
            timestamp,
            data: MessageData { sensor: SENSOR_DATA },
        };

        // Send via message queue
        let result = mq_send(RUST_TO_C_QUEUE, &msg as *const _ as *const u8,
                           core::mem::size_of::<QueueMessage>(), 0);

        if result < 0 {
            syslog(LOG_ERR, b"[Rust App Queue Sender] Failed to send sensor data to queue\n\0".as_ptr());
            return -1;
        }

        // Convert floats to fixed-point integers for syslog
        let temp_fp = (temp * 100.0) as i32;
        let humidity_fp = (humidity * 100.0) as i32;
        let pressure_fp = (pressure * 100.0) as i32;
        let battery_fp = (battery * 100.0) as i32;

        syslog(LOG_INFO, b"[Rust App Queue Sender] Sent sensor data: T=%d.%02d, H=%d.%02d, P=%d.%02d, Bat=%d.%02d\n\0".as_ptr(),
               temp_fp / 100, temp_fp % 100,
               humidity_fp / 100, humidity_fp % 100,
               pressure_fp / 100, pressure_fp % 100,
               battery_fp / 100, battery_fp % 100);
    }
    0
}

/// Thread-safe function to send processed data via message queue
#[no_mangle]
pub extern "C" fn rust_send_processed_data() -> i32 {
    unsafe {
        if RUST_TO_C_QUEUE < 0 {
            return -1;
        }

        // Process data first
        rust_process_sensor_data();

        // Create message
        let msg = QueueMessage {
            msg_type: MessageType::ProcessedData as u32,
            timestamp: SENSOR_DATA.timestamp,
            data: MessageData { processed: PROCESSED_DATA },
        };

        // Send via message queue
        let result = mq_send(RUST_TO_C_QUEUE, &msg as *const _ as *const u8,
                           core::mem::size_of::<QueueMessage>(), 0);

        if result < 0 {
            syslog(LOG_ERR, b"[Rust App Queue Sender] Failed to send processed data to queue\n\0".as_ptr());
            return -1;
        }

        // Convert floats to fixed-point integers for syslog
        let avg_temp_fp = (PROCESSED_DATA.avg_temperature * 100.0) as i32;

        syslog(LOG_INFO, b"[Rust App Queue Sender] Sent processed data: AvgT=%d.%02d, Trend=%d, Alert=%d\n\0".as_ptr(),
               avg_temp_fp / 100, avg_temp_fp % 100,
               PROCESSED_DATA.trend_direction, PROCESSED_DATA.alert_level);
    }
    0
}

/// Thread-safe function to send health check data via message queue
#[no_mangle]
pub extern "C" fn rust_send_health_check() -> i32 {
    unsafe {
        if RUST_TO_C_QUEUE < 0 {
            return -1;
        }

        let health_result = rust_system_health_check();
        let health_data = HealthData {
            score: (health_result >> 16) & 0xFFFF,
            issues: health_result & 0xFFFF,
            battery_status: if SENSOR_DATA.battery_voltage < 3.0 { 2 }
                           else if SENSOR_DATA.battery_voltage < 3.3 { 1 }
                           else { 0 },
            temperature_status: if SENSOR_DATA.temperature > 60.0 || SENSOR_DATA.temperature < -20.0 { 2 }
                               else if SENSOR_DATA.temperature > 35.0 { 1 }
                               else { 0 },
        };

        let msg = QueueMessage {
            msg_type: MessageType::HealthCheck as u32,
            timestamp: SENSOR_DATA.timestamp,
            data: MessageData { health: health_data },
        };

        let result = mq_send(RUST_TO_C_QUEUE, &msg as *const _ as *const u8,
                           core::mem::size_of::<QueueMessage>(), 0);

        if result < 0 {
            syslog(LOG_ERR, b"[Rust App Queue Sender] Failed to send health check to queue\n\0".as_ptr());
            return -1;
        }

        syslog(LOG_INFO, b"[Rust App Queue Sender] Sent health check: Score=%d, Issues=0x%08X\n\0".as_ptr(),
               health_data.score, health_data.issues);
    }
    0
}

/// Thread-safe function to receive messages from C thread
#[no_mangle]
pub extern "C" fn rust_receive_message() -> i32 {
    unsafe {
        if C_TO_RUST_QUEUE < 0 {
            return -1;
        }

        let mut msg: QueueMessage = core::mem::zeroed();
        let mut prio: u32 = 0;

        let result = mq_receive(C_TO_RUST_QUEUE, &mut msg as *mut _ as *mut u8,
                               core::mem::size_of::<QueueMessage>(), &mut prio);

        if result < 0 {
            // No message available (non-blocking)
            return 0;
        }

        // Process received message
        match msg.msg_type {
            1 => { // SensorData message from C
                syslog(LOG_INFO, b"[Rust App Message Receiver] Received sensor data from C thread\n\0".as_ptr());
                SENSOR_DATA = msg.data.sensor;
            },
            4 => { // String processing request
                syslog(LOG_INFO, b"[Rust App Message Receiver] Received string processing request from C thread\n\0".as_ptr());
            },
            5 => { // Array processing request
                syslog(LOG_INFO, b"[Rust App Message Receiver] Received array processing request from C thread\n\0".as_ptr());
            },
            _ => {
                syslog(LOG_WARNING, b"[Rust App Message Receiver] Unknown message type received: %d\n\0".as_ptr(), msg.msg_type);
            }
        }
    }
    1
}