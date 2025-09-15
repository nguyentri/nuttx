#![no_std]

use core::panic::PanicInfo;

// External C functions from NuttX
extern "C" {
    fn syslog(priority: i32, format: *const u8, ...);
    // NuttX message queue functions
    #[allow(dead_code)]
    fn mq_send(mqdes: i32, msg: *const u8, msglen: usize, prio: u32) -> i32;
    fn mq_receive(mqdes: i32, msg: *mut u8, msglen: usize, prio: *mut u32) -> isize;
    //fn mq_getattr(mqdes: i32, mq_stat: *mut MqAttr) -> i32;
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
#[allow(dead_code)]
const LOG_ERR: i32 = 3;

// Message types for Rust low-priority tasks (matching C definitions)
#[repr(C)]
#[derive(Clone, Copy)]
pub enum MessageType {
    DataLogging = 1,      // Flight data logging to Rust
    DataAnalytics = 2,    // Statistical analysis requests
    LogCompression = 3,   // Compress old flight logs
    HealthAnalysis = 4,   // System health trend analysis
    FlightStats = 5,      // Calculate flight statistics
    FlightData = 6,       // Raw flight data from C flight control
    Telemetry = 7,        // Telemetry packets from C for external transmission
}

// Flight control data structures (matching C definitions)
#[repr(C)]
#[derive(Clone, Copy)]
pub struct ImuData {
    pub accel_x: f32,
    pub accel_y: f32,
    pub accel_z: f32,
    pub gyro_x: f32,
    pub gyro_y: f32,
    pub gyro_z: f32,
    pub mag_x: f32,
    pub mag_y: f32,
    pub mag_z: f32,
    pub timestamp_us: u64,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct AttitudeData {
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
    pub roll_rate: f32,
    pub pitch_rate: f32,
    pub yaw_rate: f32,
    pub confidence: f32,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct ControlData {
    pub roll_output: f32,
    pub pitch_output: f32,
    pub yaw_output: f32,
    pub throttle: f32,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct MotorData {
    pub motor_fl: u32,
    pub motor_fr: u32,
    pub motor_rl: u32,
    pub motor_rr: u32,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct FlightData {
    pub attitude: AttitudeData,
    pub control: ControlData,
    pub imu_raw: ImuData,
    pub motors: MotorData,
    pub timestamp_us: u64,
    pub cycle_count: u32,
}

// Data logging and analytics structures
#[repr(C)]
#[derive(Clone, Copy)]
pub struct FlightLogEntry {
    pub timestamp_us: u64,
    pub attitude: AttitudeData,
    pub control: ControlData,
    pub motor_commands: MotorData,
    pub battery_voltage: f32,
    pub system_load: u32,
}

// Rust low-priority task data structures (matching C definitions)
#[repr(C)]
#[derive(Clone, Copy)]
pub struct DataLoggingRequest {
    pub request_id: u32,
    pub data_type: u32,     // 1=flight_logs, 2=sensor_data, 3=health_data
    pub priority: u32,      // 1=low, 2=normal, 3=high
    pub timestamp_us: u64,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct AnalyticsRequest {
    pub analysis_type: u32, // 1=trend, 2=statistics, 3=anomaly_detection
    pub time_window: u32,   // Analysis window in seconds
    pub data_points: u32,   // Number of data points to analyze
    pub threshold: f32,     // Analysis threshold value
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct CompressionRequest {
    pub compression_level: u32, // 1=fast, 2=balanced, 3=max
    pub age_threshold: u32,     // Compress data older than X seconds
    pub size_limit: u32,        // Maximum size before compression
    pub preserve_recent: u32,   // Keep recent X entries uncompressed
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct TelemetryPacket {
    pub packet_id: u32,
    pub timestamp_us: u64,
    pub roll_deg: f32,
    pub pitch_deg: f32,
    pub yaw_rate_deg: f32,
    pub altitude_est: f32,
    pub battery_percent: u32,
    pub system_status: u32,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct SystemHealthStatus {
    pub overall_score: u32,
    pub imu_health: u32,
    pub control_health: u32,
    pub motor_health: u32,
    pub battery_health: u32,
    pub memory_usage: u32,
    pub cpu_load: u32,
}

// Legacy structures for compatibility
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
    pub trend_direction: i32,
    pub alert_level: u32,
    pub data_quality_score: u32,
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

// Message structure for queue communication
#[repr(C)]
#[derive(Clone, Copy)]
pub struct QueueMessage {
    pub msg_type: u32,
    pub timestamp: u64,
    pub data: MessageData,
}

// Union-like structure for different message data types (matching C definitions)
#[repr(C)]
#[derive(Clone, Copy)]
pub union MessageData {
    pub logging_req: DataLoggingRequest,
    pub analytics_req: AnalyticsRequest,
    pub compression_req: CompressionRequest,
    pub health_analysis: HealthData,
    pub flight_data: FlightData,
    pub telemetry: TelemetryPacket,  // Telemetry sent from Rust to C
}

// Static storage for data logging and analytics
static mut FLIGHT_LOG_BUFFER: [FlightLogEntry; 100] = [FlightLogEntry {
    timestamp_us: 0,
    attitude: AttitudeData { roll: 0.0, pitch: 0.0, yaw: 0.0, roll_rate: 0.0, pitch_rate: 0.0, yaw_rate: 0.0, confidence: 0.0 },
    control: ControlData { roll_output: 0.0, pitch_output: 0.0, yaw_output: 0.0, throttle: 0.0 },
    motor_commands: MotorData { motor_fl: 0, motor_fr: 0, motor_rl: 0, motor_rr: 0 },
    battery_voltage: 0.0,
    system_load: 0,
}; 100];

static mut LOG_INDEX: usize = 0;
static mut TELEMETRY_PACKET_ID: u32 = 0;
static mut SYSTEM_UPTIME_MS: u64 = 0;

// Data analytics storage
static mut ATTITUDE_HISTORY: [AttitudeData; 50] = [AttitudeData {
    roll: 0.0, pitch: 0.0, yaw: 0.0, roll_rate: 0.0, pitch_rate: 0.0, yaw_rate: 0.0, confidence: 0.0
}; 50];
static mut ATTITUDE_INDEX: usize = 0;

// Legacy sensor data (for compatibility)
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

// Message queue descriptors (set by C code)
static mut RUST_TO_C_QUEUE: i32 = -1;
static mut C_TO_RUST_QUEUE: i32 = -1;

// Panic handler (required for no_std)
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}

/// Data Logging Service - Store flight data for analysis
#[no_mangle]
pub extern "C" fn rust_log_flight_data(flight_data: *const FlightData) -> i32 {
    if flight_data.is_null() {
        return -1;
    }

    unsafe {
        let data = *flight_data;

        // Create log entry
        let log_entry = FlightLogEntry {
            timestamp_us: data.timestamp_us,
            attitude: data.attitude,
            control: data.control,
            motor_commands: data.motors,
            battery_voltage: 4.2f32 - (data.cycle_count as f32 * 0.0001f32), // Simulate battery drain
            system_load: (data.cycle_count % 100) as u32, // Simulate system load
        };

        // Store in circular buffer
        FLIGHT_LOG_BUFFER[LOG_INDEX] = log_entry;
        LOG_INDEX = (LOG_INDEX + 1) % 100;

        // Store attitude for trend analysis
        ATTITUDE_HISTORY[ATTITUDE_INDEX] = data.attitude;
        ATTITUDE_INDEX = (ATTITUDE_INDEX + 1) % 50;

        // Log every 10th data point (10Hz when input is 100Hz)
        if data.cycle_count % 10 == 0 {
            let roll_deg = (data.attitude.roll * 180.0 / 3.14159) as i32;
            let pitch_deg = (data.attitude.pitch * 180.0 / 3.14159) as i32;
            let conf_percent = (data.attitude.confidence * 100.0) as i32;

            syslog(LOG_INFO, b"[Rust Data Logger] Flight Log: Roll=%ddegC, Pitch=%ddegC, Conf=%d%%, Motors=[%d,%d,%d,%d]\n\0".as_ptr(),
                   roll_deg, pitch_deg, conf_percent,
                   data.motors.motor_fl, data.motors.motor_fr,
                   data.motors.motor_rl, data.motors.motor_rr);
        }
    }

    0
}

/// Telemetry Service - Generate telemetry packets for transmission
#[no_mangle]
pub extern "C" fn rust_generate_telemetry() -> *const TelemetryPacket {
    unsafe {
        TELEMETRY_PACKET_ID += 1;
        SYSTEM_UPTIME_MS += 100; // Called every 100ms

        // Get latest attitude data
        let latest_attitude = if ATTITUDE_INDEX > 0 {
            ATTITUDE_HISTORY[ATTITUDE_INDEX - 1]
        } else {
            ATTITUDE_HISTORY[49]
        };

        static mut TELEMETRY_PACKET: TelemetryPacket = TelemetryPacket {
            packet_id: 0,
            timestamp_us: 0,
            roll_deg: 0.0,
            pitch_deg: 0.0,
            yaw_rate_deg: 0.0,
            altitude_est: 0.0,
            battery_percent: 100,
            system_status: 0,
        };

        TELEMETRY_PACKET.packet_id = TELEMETRY_PACKET_ID;
        TELEMETRY_PACKET.timestamp_us = SYSTEM_UPTIME_MS * 1000;
        TELEMETRY_PACKET.roll_deg = latest_attitude.roll * 180.0 / 3.14159;
        TELEMETRY_PACKET.pitch_deg = latest_attitude.pitch * 180.0 / 3.14159;
        TELEMETRY_PACKET.yaw_rate_deg = latest_attitude.yaw_rate * 180.0 / 3.14159;
        TELEMETRY_PACKET.altitude_est = 10.0 + libm::sinf(SYSTEM_UPTIME_MS as f32 * 0.001) * 2.0; // Simulated altitude
        TELEMETRY_PACKET.battery_percent = ((4.2 - SYSTEM_UPTIME_MS as f32 * 0.00001) / 4.2 * 100.0) as u32;
        TELEMETRY_PACKET.system_status = if latest_attitude.confidence > 0.8 { 1 } else { 2 }; // 1=OK, 2=WARNING

        &raw const TELEMETRY_PACKET as *const TelemetryPacket
    }
}

/// System Health Monitoring - Analyze flight data for anomalies
#[no_mangle]
pub extern "C" fn rust_analyze_system_health() -> *const SystemHealthStatus {
    unsafe {
        static mut HEALTH_STATUS: SystemHealthStatus = SystemHealthStatus {
            overall_score: 100,
            imu_health: 100,
            control_health: 100,
            motor_health: 100,
            battery_health: 100,
            memory_usage: 50,
            cpu_load: 30,
        };

        let mut total_score = 0u32;
        let mut sample_count = 0u32;

        // Analyze attitude stability over recent history
        for i in 0..50 {
            let attitude = ATTITUDE_HISTORY[i];
            if attitude.confidence > 0.0 {
                sample_count += 1;

                // Check for reasonable attitude values
                if attitude.roll.abs() < 0.5 && attitude.pitch.abs() < 0.5 {
                    total_score += 100;
                } else if attitude.roll.abs() < 1.0 && attitude.pitch.abs() < 1.0 {
                    total_score += 75;
                } else {
                    total_score += 25;
                }
            }
        }

        if sample_count > 0 {
            HEALTH_STATUS.imu_health = total_score / sample_count;
            HEALTH_STATUS.control_health = if HEALTH_STATUS.imu_health > 80 { 95 } else { 60 };
        }

        // Simulate other health metrics
        HEALTH_STATUS.battery_health = if SYSTEM_UPTIME_MS < 300000 { 100 } else { 75 }; // Degrade after 5 minutes
        HEALTH_STATUS.motor_health = 90 + (SYSTEM_UPTIME_MS % 20) as u32; // Simulate minor variations
        HEALTH_STATUS.memory_usage = 40 + (SYSTEM_UPTIME_MS / 1000 % 30) as u32; // Slowly increasing memory usage
        HEALTH_STATUS.cpu_load = 25 + (SYSTEM_UPTIME_MS % 50) as u32; // Variable CPU load

        HEALTH_STATUS.overall_score = (HEALTH_STATUS.imu_health + HEALTH_STATUS.control_health +
                                       HEALTH_STATUS.motor_health + HEALTH_STATUS.battery_health) / 4;

        syslog(LOG_INFO, b"[Rust System Health] Overall: %d%%, IMU: %d%%, Battery: %d%%, Memory: %d%%\n\0".as_ptr(),
               HEALTH_STATUS.overall_score, HEALTH_STATUS.imu_health,
               HEALTH_STATUS.battery_health, HEALTH_STATUS.memory_usage);

        &raw const HEALTH_STATUS as *const SystemHealthStatus
    }
}

/// Data Compression Service - Compress old flight logs
#[no_mangle]
pub extern "C" fn rust_compress_flight_logs() -> i32 {
    unsafe {
        // Simple data compression simulation
        let mut compressed_entries = 0u32;

        for i in 0..100 {
            let entry = FLIGHT_LOG_BUFFER[i];
            if entry.timestamp_us > 0 {
                // In a real implementation, this would compress and store to filesystem
                compressed_entries += 1;
            }
        }

        syslog(LOG_INFO, b"[Rust Data Compression] Compressed %d flight log entries\n\0".as_ptr(), compressed_entries);
        compressed_entries as i32
    }
}

/// Data Analytics - Calculate flight statistics
#[no_mangle]
pub extern "C" fn rust_calculate_flight_stats() -> i32 {
    unsafe {
        let mut max_roll = 0.0f32;
        let mut max_pitch = 0.0f32;
        let mut avg_confidence = 0.0f32;
        let mut valid_samples = 0u32;

        for i in 0..50 {
            let attitude = ATTITUDE_HISTORY[i];
            if attitude.confidence > 0.0 {
                valid_samples += 1;
                if attitude.roll.abs() > max_roll.abs() {
                    max_roll = attitude.roll;
                }
                if attitude.pitch.abs() > max_pitch.abs() {
                    max_pitch = attitude.pitch;
                }
                avg_confidence += attitude.confidence;
            }
        }

        if valid_samples > 0 {
            avg_confidence /= valid_samples as f32;

            let max_roll_deg = (max_roll * 180.0 / 3.14159) as i32;
            let max_pitch_deg = (max_pitch * 180.0 / 3.14159) as i32;
            let avg_conf_percent = (avg_confidence * 100.0) as i32;

            syslog(LOG_INFO, b"[Rust Flight Analytics] Max Roll: %degC, Max Pitch: %ddegC, Avg Confidence: %d%%\n\0".as_ptr(),
                   max_roll_deg, max_pitch_deg, avg_conf_percent);
        }        valid_samples as i32
    }
}

/// Initialize Rust data services subsystem
#[no_mangle]
pub extern "C" fn rust_system_init() -> i32 {
    unsafe {
        syslog(LOG_INFO, b"[Rust Data Services] Initializing Rust Data Services Subsystem\n\0".as_ptr());
        TELEMETRY_PACKET_ID = 0;
        SYSTEM_UPTIME_MS = 0;
        LOG_INDEX = 0;
        ATTITUDE_INDEX = 0;
    }
    0
}

/// Set message queue descriptors for thread-safe communication
#[no_mangle]
pub extern "C" fn rust_set_queues(rust_to_c: i32, c_to_rust: i32) -> i32 {
    unsafe {
        RUST_TO_C_QUEUE = rust_to_c;
        C_TO_RUST_QUEUE = c_to_rust;
        syslog(LOG_INFO, b"[Rust Data Services] Message queues configured: R2C=%d, C2R=%d\n\0".as_ptr(),
               rust_to_c, c_to_rust);
    }
    0
}

/// Thread-safe function to receive low-priority task requests from C thread
#[no_mangle]
pub extern "C" fn rust_receive_flight_data() -> i32 {
    unsafe {
        if C_TO_RUST_QUEUE < 0 {
            return -1;
        }

        let mut msg: QueueMessage = core::mem::zeroed();
        let result = mq_receive(C_TO_RUST_QUEUE, &mut msg as *mut _ as *mut u8,
                               core::mem::size_of::<QueueMessage>(), core::ptr::null_mut());

        if result > 0 {
            // Debug: Log all received messages
            //syslog(LOG_INFO, b"[Rust Message Debug] Received message type: //%u\n\0".as_ptr(), msg.msg_type);

            match msg.msg_type {
                1 => { // MSG_TYPE_DATA_LOGGING
                    //syslog(LOG_INFO, b"[Rust Data Services] Processing flight data message\n\0".as_ptr());
                    let flight_data = &msg.data.flight_data;
                    rust_log_flight_data(flight_data as *const FlightData);
                    return 1;
                },
                2 => { // MSG_TYPE_DATA_ANALYTICS
                    let analytics_req = &msg.data.analytics_req;
                    syslog(LOG_INFO, b"[Rust Data Services] Received analytics request: type=%d, window=%ds\n\0".as_ptr(),
                           analytics_req.analysis_type, analytics_req.time_window);
                    // Perform the requested analytics
                    rust_calculate_flight_stats();
                    return 1;
                },
                3 => { // MSG_TYPE_LOG_COMPRESSION
                    let compression_req = &msg.data.compression_req;
                    syslog(LOG_INFO, b"[Rust Data Services] Received compression request: level=%d, age_threshold=%ds\n\0".as_ptr(),
                           compression_req.compression_level, compression_req.age_threshold);
                    // Perform log compression
                    rust_compress_flight_logs();
                    return 1;
                },
                4 => { // MSG_TYPE_HEALTH_ANALYSIS
                    //syslog(LOG_INFO, b"[Rust Data Services] Received health analysis request\n\0".as_ptr());
                    rust_analyze_system_health();
                    return 1;
                },
                5 => { // MSG_TYPE_FLIGHT_STATS
                    //syslog(LOG_INFO, b"[Rust Data Services] Received flight statistics request\n\0".as_ptr());
                    rust_calculate_flight_stats();
                    return 1;
                },
                6 => { // MSG_TYPE_FLIGHT_DATA
                    //syslog(LOG_INFO, b"[Rust Data Services] Processing flight data message\n\0".as_ptr());
                    let flight_data = &msg.data.flight_data;
                    rust_log_flight_data(flight_data as *const FlightData);
                    return 1;
                },
                7 => { // MSG_TYPE_TELEMETRY
                    //syslog(LOG_INFO, b"[Rust Data Services] Processing telemetry message\n\0".as_ptr());
                    let telemetry = &msg.data.telemetry;
                    rust_transmit_telemetry(telemetry as *const TelemetryPacket);
                    return 1;
                },
                _ => {
                    syslog(LOG_INFO, b"[Rust Data Services] Received unknown message type: %d\n\0".as_ptr(), msg.msg_type);
                }
            }
        }
    }
    0
}

/// External Telemetry Transmission Service - Transmit telemetry generated by C flight control
#[no_mangle]
pub extern "C" fn rust_transmit_telemetry(telemetry: *const TelemetryPacket) -> i32 {
    unsafe {
        if telemetry.is_null() {
            return -1;
        }

        let data = &*telemetry;

        // Simulate external telemetry transmission (radio, network, etc.)
        syslog(LOG_INFO, b"[Rust Telemetry TX] External transmission: Packet #%u\n\0".as_ptr(),
               data.packet_id);
        syslog(LOG_INFO, b"[Rust Telemetry TX] Attitude: Roll=%.1fdeg, Pitch=%.1fdeg, YawRate=%.1fdeg/s\n\0".as_ptr(),
               data.roll_deg as i32, data.pitch_deg as i32, data.yaw_rate_deg as i32);
        syslog(LOG_INFO, b"[Rust Telemetry TX] Status: Altitude=%.1fm, Battery=%u%%, System=%u\n\0".as_ptr(),
               data.altitude_est as i32, data.battery_percent, data.system_status);

        // In a real implementation, this would:
        // - Format telemetry into radio protocol (e.g., MAVLink)
        // - Transmit via radio/network interface
        // - Handle transmission acknowledgments
        // - Queue retransmissions for failed packets

        0 // Success
    }
}

/// Legacy sensor data processing
#[no_mangle]
pub extern "C" fn rust_store_sensor_data(temp: f32, humidity: f32, pressure: f32, timestamp: u64, battery: f32) -> i32 {
    unsafe {
        SENSOR_DATA.temperature = temp;
        SENSOR_DATA.humidity = humidity;
        SENSOR_DATA.pressure = pressure;
        SENSOR_DATA.timestamp = timestamp;
        SENSOR_DATA.battery_voltage = battery;

        // Simple data validation
        if temp < -50.0 || temp > 85.0 || humidity < 0.0 || humidity > 100.0 {
            syslog(LOG_WARNING, b"[Rust Data Services] Invalid sensor data detected\n\0".as_ptr());
            return -1;
        }

        syslog(LOG_INFO, b"[Rust Sensor Service] Stored legacy sensor data\n\0".as_ptr());
    }
    0
}

/// Generate system health report (simplified)
#[no_mangle]
pub extern "C" fn rust_system_health_check() -> u32 {
    unsafe {
        let health_status = rust_analyze_system_health();
        let overall_score = (*health_status).overall_score;
        let issues = if overall_score < 70 { 0x01 } else { 0x00 };

        (overall_score << 16) | issues
    }
}

/// String processing function (lightweight data service)
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

        syslog(LOG_INFO, b"[Rust Data Services] String processing: checksum=%d, vowels=%d\n\0".as_ptr(),
               checksum, vowel_count);

        (checksum << 16) | vowel_count
    }
}

/// Array processing function (lightweight analytics)
#[no_mangle]
pub extern "C" fn rust_process_array(data: *const i32, len: usize) -> i32 {
    if data.is_null() || len == 0 {
        return 0;
    }

    unsafe {
        let slice = core::slice::from_raw_parts(data, len);
        let mut sum = 0i64;

        for &value in slice {
            sum += value as i64;
        }

        let average = if len > 0 { sum / len as i64 } else { 0 };

        syslog(LOG_INFO, b"[Rust Data Services] Array analytics: avg=%d from %d elements\n\0".as_ptr(),
               average as i32, len);

        average as i32
    }
}

/// Get legacy sensor data (for compatibility)
#[no_mangle]
pub extern "C" fn rust_get_sensor_data() -> *const SharedSensorData {
    unsafe {
        &raw const SENSOR_DATA as *const SharedSensorData
    }
}

/// Legacy processed data (simplified)
#[no_mangle]
pub extern "C" fn rust_process_sensor_data() -> *const RustProcessedData {
    unsafe {
        PROCESSED_DATA.avg_temperature = SENSOR_DATA.temperature;
        PROCESSED_DATA.trend_direction = 0; // Stable
        PROCESSED_DATA.alert_level = if SENSOR_DATA.battery_voltage < 3.3 { 1 } else { 0 };
        PROCESSED_DATA.data_quality_score = 95;

        &raw const PROCESSED_DATA as *const RustProcessedData
    }
}

/// Legacy validation (simplified)
#[no_mangle]
pub extern "C" fn rust_validate_sensor_data(temp: f32, humidity: f32, pressure: f32) -> i32 {
    let is_valid = temp >= -50.0 && temp <= 85.0 &&
                   humidity >= 0.0 && humidity <= 100.0 &&
                   pressure >= 300.0 && pressure <= 1200.0;

    unsafe {
        if is_valid {
            syslog(LOG_INFO, b"[Rust Sensor Validation Service] Result: PASSED\n\0".as_ptr());
        } else {
            syslog(LOG_WARNING, b"[Rust Sensor Validation Service] Result: FAILED\n\0".as_ptr());
        }
    }

    is_valid as i32
}

/// Legacy thread-safe functions (simplified stubs)
#[no_mangle]
pub extern "C" fn rust_send_sensor_data(temp: f32, humidity: f32, pressure: f32, timestamp: u64, battery: f32) -> i32 {
    rust_store_sensor_data(temp, humidity, pressure, timestamp, battery)
}

#[no_mangle]
pub extern "C" fn rust_send_processed_data() -> i32 {
    unsafe {
        syslog(LOG_INFO, b"[Rust Data Services] Processed data ready\n\0".as_ptr());
    }
    0
}

#[no_mangle]
pub extern "C" fn rust_send_health_check() -> i32 {
    let _health = rust_system_health_check();
    0
}

#[no_mangle]
pub extern "C" fn rust_receive_message() -> i32 {
    rust_receive_flight_data()
}
