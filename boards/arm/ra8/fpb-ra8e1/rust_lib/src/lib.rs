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
#[repr(C, packed)]
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

/// Enhanced Rust calculation function for cyclic logger
#[no_mangle]
pub extern "C" fn rust_calculate(a: i32, b: i32) -> i32 {
    let result = a * b + 42;
    unsafe {
        syslog(LOG_INFO, b"[RUST Application] rust_calculate: %d * %d + 42 = %d\n\0".as_ptr(), a, b, result);
    }
    result
}

/// Advanced math function with different operations
#[no_mangle]
pub extern "C" fn rust_advanced_math(x: i32, y: i32, operation: u8) -> i32 {
    unsafe {
        syslog(LOG_INFO, b"[RUST Application] rust_advanced_math: x=%d, y=%d, op=%d\n\0".as_ptr(), x, y, operation as i32);
    }

    let result = match operation {
        0 => x + y,           // Addition
        1 => x - y,           // Subtraction
        2 => x * y,           // Multiplication
        3 => if y != 0 { x / y } else { 0 }, // Division (safe)
        4 => fibonacci(x.abs() as u32) as i32, // Fibonacci of x
        5 => prime_check(x.abs() as u32) as i32, // Check if prime
        6 => gcd(x.abs() as u32, y.abs() as u32) as i32, // Greatest Common Divisor
        _ => x * x + y * y,   // Default: sum of squares
    };

    unsafe {
        syslog(LOG_INFO, b"[RUST Application] rust_advanced_math result: %d\n\0".as_ptr(), result);
    }

    result
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

        syslog(LOG_INFO, b"[RUST Application] Stored sensor data: T=%.1f, H=%.1f, P=%.1f, Bat=%.2f\n\0".as_ptr(),
               temp as i32, humidity as i32, pressure as i32, battery as i32);
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

        // Calculate data quality score
        PROCESSED_DATA.data_quality_score = if SENSOR_DATA.battery_voltage > 3.5 {
            100
        } else if SENSOR_DATA.battery_voltage > 3.2 {
            80
        } else {
            60
        };

        syslog(LOG_INFO, b"[RUST Application] Processed data: AvgT=%.1f, Trend=%d, Alert=%d, Quality=%d\n\0".as_ptr(),
               PROCESSED_DATA.avg_temperature as i32,
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
            syslog(LOG_INFO, b"[RUST Application] Sensor data validation: PASSED\n\0".as_ptr());
        } else {
            syslog(LOG_WARNING, b"[RUST Application] Sensor data validation: FAILED\n\0".as_ptr());
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

        syslog(LOG_INFO, b"[RUST Application] System health: Score=%d, Issues=0x%08X\n\0".as_ptr(),
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

        syslog(LOG_INFO, b"[RUST Application] String processed: len=%d, checksum=%d, vowels=%d\n\0".as_ptr(),
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

        syslog(LOG_INFO, b"[RUST Application] Array processed: len=%d, sum=%ld, avg=%ld\n\0".as_ptr(),
               len as i32, sum as i32, avg as i32);

        avg as i32
    }
}

/// Calculate Fibonacci number (iterative approach for efficiency)
fn fibonacci(n: u32) -> u32 {
    if n <= 1 {
        return n;
    }

    let mut a = 0;
    let mut b = 1;

    for _ in 2..=n {
        let temp = a + b;
        a = b;
        b = temp;
    }

    b
}

/// Check if a number is prime
fn prime_check(n: u32) -> bool {
    if n < 2 {
        return false;
    }
    if n == 2 {
        return true;
    }
    if n % 2 == 0 {
        return false;
    }

    let mut i = 3;
    while i * i <= n {
        if n % i == 0 {
            return false;
        }
        i += 2;
    }
    true
}

/// Calculate Greatest Common Divisor
fn gcd(mut a: u32, mut b: u32) -> u32 {
    while b != 0 {
        let temp = b;
        b = a % b;
        a = temp;
    }
    a
}

/// Initialize Rust subsystem with message queue descriptors
#[no_mangle]
pub extern "C" fn rust_system_init() -> i32 {
    unsafe {
        syslog(LOG_INFO, b"[RUST Application] Board-level Rust subsystem initialized!\n\0".as_ptr());
        syslog(LOG_INFO, b"[RUST Application] Available functions: calculations, data processing, validation\n\0".as_ptr());
        syslog(LOG_INFO, b"[RUST Application] Thread-safe message queue communication enabled\n\0".as_ptr());
    }
    0
}

/// Set message queue descriptors for thread-safe communication
#[no_mangle]
pub extern "C" fn rust_set_queues(rust_to_c: i32, c_to_rust: i32) -> i32 {
    unsafe {
        RUST_TO_C_QUEUE = rust_to_c;
        C_TO_RUST_QUEUE = c_to_rust;
        syslog(LOG_INFO, b"[RUST Application] Message queues set: R->C=%d, C->R=%d\n\0".as_ptr(), rust_to_c, c_to_rust);
    }
    0
}

/// Thread-safe function to send sensor data via message queue
#[no_mangle]
pub extern "C" fn rust_send_sensor_data(temp: f32, humidity: f32, pressure: f32, timestamp: u64, battery: f32) -> i32 {
    unsafe {
        if RUST_TO_C_QUEUE < 0 {
            syslog(LOG_ERR, b"[RUST Application] Error: Message queue not initialized\n\0".as_ptr());
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
            syslog(LOG_ERR, b"[RUST Application] Failed to send sensor data to queue\n\0".as_ptr());
            return -1;
        }

        syslog(LOG_INFO, b"[RUST Application] Sent sensor data via queue: T=%.1f, H=%.1f, P=%.1f, Bat=%.2f\n\0".as_ptr(),
               temp as i32, humidity as i32, pressure as i32, battery as i32);
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
            syslog(LOG_ERR, b"[RUST Application] Failed to send processed data to queue\n\0".as_ptr());
            return -1;
        }

        syslog(LOG_INFO, b"[RUST Application] Sent processed data via queue: AvgT=%.1f, Trend=%d, Alert=%d\n\0".as_ptr(),
               PROCESSED_DATA.avg_temperature as i32, PROCESSED_DATA.trend_direction, PROCESSED_DATA.alert_level);
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
            syslog(LOG_ERR, b"[RUST Application] Failed to send health check to queue\n\0".as_ptr());
            return -1;
        }

        syslog(LOG_INFO, b"[RUST Application] Sent health check via queue: Score=%d, Issues=0x%08X\n\0".as_ptr(),
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
                syslog(LOG_INFO, b"[RUST Application] Received sensor data from C thread\n\0".as_ptr());
                SENSOR_DATA = msg.data.sensor;
            },
            4 => { // String processing request
                syslog(LOG_INFO, b"[RUST Application] Received string processing request from C thread\n\0".as_ptr());
            },
            5 => { // Array processing request
                syslog(LOG_INFO, b"[RUST Application] Received array processing request from C thread\n\0".as_ptr());
            },
            _ => {
                syslog(LOG_WARNING, b"[RUST Application] Unknown message type received: %d\n\0".as_ptr(), msg.msg_type);
            }
        }
    }
    1
}