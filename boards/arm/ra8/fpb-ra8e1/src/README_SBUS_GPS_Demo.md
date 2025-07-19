# RA8E1 SBUS and GPS Demo Implementation

## Overview

This implementation provides two demonstration programs for the FPB-RA8E1 board:

1. **SBUS Demo** - RC receiver SBUS protocol decoder using UART2 with DMA
2. **GPS Demo** - GPS NMEA sentence parser using UART3 with DMA

Both demos use RTT (Real-Time Transfer) for printf output and interactive commands.

## Pin Mapping

### SBUS (RC Receiver)
- **P802 (RXD2)** ← RC Receiver SBUS output (J18 #1, D0)
- Protocol: 100 kbps, 8E2 (8 data bits, even parity, 2 stop bits), inverted signal
- UART2 RX only (no TX needed)

### GPS Module
- **P310 (TXD3)** → GPS RX (J18 #2, D1) 
- **P309 (RXD3)** ← GPS TX (J18 #1, D0)
- Protocol: 38400 baud, 8N1 (8 data bits, no parity, 1 stop bit)
- UART3 bidirectional

## Files Created/Modified

### Header Files
- `ra8e1_sbus_demo.h` - SBUS demo interface and data structures
- `ra8e1_gps_demo.h` - GPS demo interface and data structures

### Source Files
- `ra8e1_sbus_demo.c` - SBUS demonstration implementation
- `ra8e1_gps_demo.c` - GPS demonstration implementation

### Configuration Files
- `configs/sbus/defconfig` - Configuration for SBUS demo
- `configs/gps/defconfig` - Configuration for GPS demo

## Features

### SBUS Demo Features
- Real-time SBUS frame decoding
- 16-channel RC data extraction (11-bit resolution)
- Digital channels 17 and 18 support
- Frame lost and failsafe detection
- RTT interactive commands:
  - `c` - Show channel values
  - `s` - Show reception status
  - `r` - Reset counters
  - `h` - Show help menu
  - `q` - Quit demo

### GPS Demo Features
- NMEA 0183 sentence parsing
- Support for GGA, RMC, and GSA sentences
- Position, time, and status extraction
- Checksum validation
- RTT interactive commands:
  - `p` - Show position data
  - `s` - Show reception status
  - `r` - Reset counters
  - `h` - Show help menu
  - `q` - Quit demo

## UART Driver Integration

Both demos use the RA8 UART driver with DMA for efficient data transfer:

### SBUS Configuration
```c
config.base = R_SCI2_BASE;
config.baud = 100000;
config.bits = 8;
config.parity = 2;  // Even parity
config.stop = 2;    // 2 stop bits
config.rx_pin = RA_GPIO_PIN(RA_GPIO_PORT8, 2); // P802
config.rx_dma.enabled = true;
config.rx_dma.channel = 0;
```

### GPS Configuration
```c
config.base = R_SCI3_BASE;
config.baud = 38400;
config.bits = 8;
config.parity = 0;  // No parity
config.stop = 1;    // 1 stop bit
config.tx_pin = RA_GPIO_PIN(RA_GPIO_PORT3, 10); // P310
config.rx_pin = RA_GPIO_PIN(RA_GPIO_PORT3, 9);  // P309
config.rx_dma.enabled = true;
config.tx_dma.enabled = true;
config.rx_dma.channel = 2;
config.tx_dma.channel = 3;
```

## RTT Integration

Both demos support RTT for debugging and user interaction:

```c
#ifdef CONFIG_SEGGER_RTT
#include "SEGGER_RTT.h"
#define RTT_PRINTF(...)     SEGGER_RTT_printf(0, __VA_ARGS__)
#define RTT_HASKEY()        SEGGER_RTT_HasKey()
#define RTT_GETKEY()        SEGGER_RTT_GetKey()
#else
#define RTT_PRINTF(...)     printf(__VA_ARGS__)
#define RTT_HASKEY()        (false)
#define RTT_GETKEY()        (0)
#endif
```

## Building and Running

### SBUS Demo
```bash
cd nuttx
make distclean
./tools/configure.sh fpb-ra8e1:sbus
make
```

### GPS Demo
```bash
cd nuttx
make distclean
./tools/configure.sh fpb-ra8e1:gps
make
```

## Usage

1. Flash the appropriate firmware to the FPB-RA8E1 board
2. Connect RTT debugger (J-Link, etc.)
3. Connect SBUS receiver or GPS module to appropriate pins
4. Monitor RTT output for real-time data
5. Use RTT input for interactive commands

## Data Structures

### SBUS Data
```c
struct sbus_data_s
{
  uint16_t channels[16];  // Channel values (172-1811)
  uint8_t flags;          // Status flags
  bool frame_lost;        // Frame lost indicator
  bool failsafe;          // Failsafe indicator
  bool ch17;              // Digital channel 17
  bool ch18;              // Digital channel 18
  uint32_t timestamp;     // Frame timestamp
};
```

### GPS Data
```c
struct gps_data_s
{
  double latitude;        // Latitude in degrees
  double longitude;       // Longitude in degrees
  float altitude;         // Altitude in meters
  uint8_t hour, minute, second;
  uint8_t day, month;
  uint16_t year;
  uint8_t fix_type;       // Fix type
  uint8_t satellites_used;
  float hdop, vdop, pdop; // Dilution of precision
  float speed_kmh;        // Speed in km/h
  float course;           // Course in degrees
  bool fix_valid;         // Fix validity
  uint32_t timestamp;     // System timestamp
};
```

## Error Handling

Both demos include comprehensive error handling:
- UART initialization errors
- DMA transfer errors
- Frame/sentence parsing errors
- Checksum validation errors
- Buffer overflow protection

## Performance Considerations

- DMA reduces CPU load for data transfer
- State machine parsing for efficient real-time processing
- Minimal memory footprint
- Non-blocking operation with callback-based architecture

## Future Enhancements

1. **SBUS**: Add support for SBUS2 protocol extensions
2. **GPS**: Add support for additional NMEA sentence types (GSV, GLL, etc.)
3. **Both**: Add data logging capabilities
4. **Both**: Add configuration via RTT commands
5. **Integration**: Combine both demos for complete RC/GPS system
