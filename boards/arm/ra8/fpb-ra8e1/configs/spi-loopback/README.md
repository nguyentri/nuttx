# SPI Loopback Demo for FPB-RA8E1

## Overview

This demo implements SPI communication between two SPI units on the RA8E1 FPB board:
- **SPI0** as Master
- **SPI1** as Slave

The demo is based on the Renesas FSP SPI examples and demonstrates both separate and simultaneous write/read operations between the two SPI units.

## Hardware Requirements

### Pin Connections Required

To run this demo, you need to connect the SPI0 and SPI1 pins externally:

| SPI0 (Master) | SPI1 (Slave) | Function |
|---------------|--------------|----------|
| P410 (MOSI0)  | P202 (MOSI1) | Master Out, Slave In |
| P411 (MISO0)  | P203 (MISO1) | Master In, Slave Out |
| P412 (RSPCK0) | P204 (RSPCK1)| Serial Clock |
| P413 (SSL0)   | P205 (SSL1)  | Slave Select |

**Important**: Make sure to connect these pins externally with jumper wires before running the demo.

## Software Configuration

### Configuration File

Use the `spi-loopback` configuration:

```bash
cd nuttx
make distclean
./tools/configure.sh fpb-ra8e1:spi-loopback
make
```

### Key Configuration Options

```makefile
CONFIG_RA8_SPI=y                 # Enable SPI support
CONFIG_RA_SPI0=y                 # Enable SPI0 (Master)
CONFIG_RA_SPI1=y                 # Enable SPI1 (Slave)
CONFIG_SPI_LOOPBACK_DEMO=y       # Enable the loopback demo
CONFIG_RA8_SPI_DMA=y             # Enable DMA support
CONFIG_RA_DTC=y                  # Enable Data Transfer Controller
CONFIG_SPI_EXCHANGE=y            # Enable simultaneous read/write
```

## Demo Features

### Test 1: Write-and-Read (Separate Operations)
1. Slave prepares to receive data from Master
2. Master sends data to Slave
3. Slave sends response data to Master
4. Master receives response from Slave
5. Data verification

### Test 2: Write-Read (Simultaneous Operations)
1. Both Master and Slave perform simultaneous write/read operations
2. Data is exchanged in both directions at the same time
3. Data verification

### Data Patterns
- Master transmits test patterns: 0x12345678, 0xABCDEF00, 0x55AA55AA, 0xFF00FF00
- Slave responds with inverted patterns for verification
- 32-bit transfers, 32 words buffer size
- 1 MHz transfer frequency

## Usage

### From NuttX Shell

```bash
nsh> spi_loopback
```

### Expected Output

```
RA8E1 SPI Loopback Demo
=======================
This demo tests SPI communication between two SPI units:
- SPI0 as Master
- SPI1 as Slave
Tests both separate and simultaneous write/read operations.

=== Starting SPI Loopback Demo Tests ===

--- Test 1: Write-and-Read ---
✓ SPI loopback test PASSED - all data verified successfully

--- Test 2: Write-Read (Simultaneous) ---
✓ SPI loopback test PASSED - all data verified successfully

=== SPI Loopback Demo Tests COMPLETED SUCCESSFULLY ===

SPI Loopback Demo completed successfully!
```

## Troubleshooting

### Common Issues

1. **No Hardware Connections**
   - Ensure SPI0 and SPI1 pins are properly connected
   - Check jumper wire connections

2. **Data Verification Failures**
   - Check signal integrity
   - Verify clock frequency settings
   - Ensure proper grounding

3. **Initialization Failures**
   - Verify FSP integration is enabled
   - Check clock configuration
   - Ensure both SPI0 and SPI1 are enabled

### Debug Configuration

Enable debug output:
```makefile
CONFIG_DEBUG_FEATURES=y
CONFIG_DEBUG_SPI=y
CONFIG_DEBUG_SPI_INFO=y
```

## Implementation Details

### Architecture

The demo follows NuttX SPI driver architecture:

```
Application Layer: ra8e1_spi_loopback_demo.c
    ↓
NuttX SPI API: spi.h interfaces
    ↓  
Board Layer: ra8e1_spi.c
    ↓
Arch Layer: RA8 SPI driver with FSP integration
    ↓
Hardware: RA8E1 SPI0/SPI1 controllers
```

### Key Functions

- `ra8e1_spi_loopback_demo_init()` - Initialize demo
- `ra8e1_spi_loopback_demo_test()` - Run tests
- `spi_test_write_and_read()` - Separate write/read test
- `spi_test_write_read()` - Simultaneous write/read test
- `spi_verify_data()` - Data verification

### FSP Integration

The demo leverages Renesas FSP features:
- SPI_B driver for enhanced functionality
- DTC (Data Transfer Controller) for efficient transfers
- Interrupt-driven operations
- Hardware flow control

## Performance

- Transfer Rate: 1 MHz (configurable)
- Buffer Size: 32 x 32-bit words
- DMA/DTC Support: Yes
- Interrupt Driven: Yes

## References

- [RA8E1 User Manual](https://www.renesas.com/ra8e1)
- [FSP SPI Examples](fpb_ra8e1/spi/spi_fpb_ra8e1_ep/)
- [NuttX SPI Documentation](https://nuttx.apache.org/docs/latest/components/drivers/special/spi.html)
