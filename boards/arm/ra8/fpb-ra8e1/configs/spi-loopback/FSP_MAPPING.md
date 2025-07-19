# SPI NuttX Implementation Mapping to FSP Examples

## Overview

This document shows how the NuttX SPI loopback demo maps to the original Renesas FSP SPI examples for the FPB-RA8E1 board.

## File Mapping

### FSP Example Files → NuttX Implementation

| FSP Example File | NuttX Equivalent | Purpose |
|------------------|------------------|---------|
| `spi_ep.c` | `ra8e1_spi_loopback_demo.c` | Main demo implementation |
| `spi_ep.h` | `ra8e1_spi_loopback_demo.h` | Demo header definitions |
| `hal_data.c` | NuttX SPI driver + board files | Hardware configuration |
| `hal_data.h` | Board header files | Hardware definitions |
| `vector_data.c` | NuttX interrupt system | Interrupt vectors |
| `vector_data.h` | NuttX interrupt headers | Interrupt definitions |
| `r_spi_b_cfg.h` | Kconfig + defconfig | Driver configuration |
| `r_dtc_cfg.h` | DTC configuration in NuttX | DMA configuration |

## Function Mapping

### FSP Functions → NuttX Equivalents

| FSP Function | NuttX Equivalent | Description |
|--------------|------------------|-------------|
| `R_SPI_B_Open()` | `ra8_spibus_initialize()` | Initialize SPI interface |
| `R_SPI_B_Write()` | `SPI_SNDBLOCK()` | Send data block |
| `R_SPI_B_Read()` | `SPI_RECVBLOCK()` | Receive data block |
| `R_SPI_B_WriteRead()` | `SPI_EXCHANGE()` | Simultaneous write/read |
| `R_SPI_B_Close()` | Automatic cleanup | Close SPI interface |
| `spi_master_callback()` | NuttX interrupt handling | Transfer completion callback |
| `spi_slave_callback()` | NuttX interrupt handling | Transfer completion callback |

## Key Implementation Differences

### 1. Initialization

**FSP Example:**
```c
#if defined (BOARD_RA8E1_FPB)
    err = R_SPI_B_Open(&g_spi_master_ctrl, &g_spi_master_cfg);
    err = R_SPI_B_Open(&g_spi_slave_ctrl, &g_spi_slave_cfg);
#endif
```

**NuttX Implementation:**
```c
g_spi_loopback.master = ra8_spibus_initialize(0);  // SPI0 as master
g_spi_loopback.slave = ra8_spibus_initialize(1);   // SPI1 as slave
```

### 2. Configuration

**FSP Example:**
```c
const spi_cfg_t g_spi_master_cfg = {
    .channel = 0,
    .operating_mode = SPI_MODE_MASTER,
    .clk_phase = SPI_CLK_PHASE_EDGE_ODD,
    .clk_polarity = SPI_CLK_POLARITY_LOW,
    .mode_fault = SPI_MODE_FAULT_ERROR_DISABLE,
    .bit_order = SPI_BIT_ORDER_MSB_FIRST,
    .bitrate = 1000000,
    // ... more config
};
```

**NuttX Implementation:**
```c
SPI_LOCK(g_spi_loopback.master, true);
SPI_SETMODE(g_spi_loopback.master, SPIDEV_MODE0);
SPI_SETBITS(g_spi_loopback.master, 32);
SPI_SETFREQUENCY(g_spi_loopback.master, SPI_FREQUENCY);
SPI_LOCK(g_spi_loopback.master, false);
```

### 3. Data Transfer

**FSP Example (Write-and-Read):**
```c
err = R_SPI_B_Read(&g_spi_slave_ctrl, g_slave_rx_buff, num_bytes, SPI_BIT_WIDTH_32_BITS);
err = R_SPI_B_Write(&g_spi_master_ctrl, g_master_tx_buff, num_bytes, SPI_BIT_WIDTH_32_BITS);
err = R_SPI_B_Write(&g_spi_slave_ctrl, g_slave_tx_buff, num_bytes, SPI_BIT_WIDTH_32_BITS);
err = R_SPI_B_Read(&g_spi_master_ctrl, g_master_rx_buff, num_bytes, SPI_BIT_WIDTH_32_BITS);
```

**NuttX Implementation (Write-and-Read):**
```c
ret = SPI_RECVBLOCK(g_spi_loopback.slave, g_spi_loopback.slave_rx_buff, 
                    SPI_BUFF_LEN * sizeof(uint32_t));
ret = SPI_SNDBLOCK(g_spi_loopback.master, g_spi_loopback.master_tx_buff,
                   SPI_BUFF_LEN * sizeof(uint32_t));
ret = SPI_SNDBLOCK(g_spi_loopback.slave, g_spi_loopback.slave_tx_buff,
                   SPI_BUFF_LEN * sizeof(uint32_t));
ret = SPI_RECVBLOCK(g_spi_loopback.master, g_spi_loopback.master_rx_buff,
                    SPI_BUFF_LEN * sizeof(uint32_t));
```

**FSP Example (Write-Read Simultaneous):**
```c
err = R_SPI_B_WriteRead(&g_spi_slave_ctrl, g_slave_tx_buff, g_slave_rx_buff, 
                        num_bytes_slave, SPI_BIT_WIDTH_32_BITS);
err = R_SPI_B_WriteRead(&g_spi_master_ctrl, g_master_tx_buff, g_master_rx_buff, 
                        num_bytes_master, SPI_BIT_WIDTH_32_BITS);
```

**NuttX Implementation (Write-Read Simultaneous):**
```c
ret = SPI_EXCHANGE(g_spi_loopback.slave, g_spi_loopback.slave_tx_buff,
                   g_spi_loopback.slave_rx_buff, SPI_BUFF_LEN * sizeof(uint32_t));
ret = SPI_EXCHANGE(g_spi_loopback.master, g_spi_loopback.master_tx_buff,
                   g_spi_loopback.master_rx_buff, SPI_BUFF_LEN * sizeof(uint32_t));
```

## Hardware Configuration Mapping

### FSP HAL Data Configuration

**FSP hal_data.c (Transfer Configuration):**
```c
const dtc_extended_cfg_t g_transfer0_cfg_extend = {
    .activation_source = VECTOR_NUMBER_SPI0_TXI,
};

const transfer_cfg_t g_transfer0_cfg = {
    .p_info = &g_transfer0_info,
    .p_extend = &g_transfer0_cfg_extend,
    .activation_source = VECTOR_NUMBER_SPI0_TXI,
    .auto_enable = false,
    .p_callback = NULL,
    .p_context = NULL,
    .irq = TRANSFER_IRQ_END,
};
```

**NuttX Configuration (defconfig):**
```makefile
CONFIG_RA8_SPI=y
CONFIG_RA_SPI0=y
CONFIG_RA_SPI1=y
CONFIG_RA8_SPI_DMA=y
CONFIG_RA_DTC=y
CONFIG_RA_DTC_CHANNELS=8
CONFIG_SPI_EXCHANGE=y
```

### Pin Configuration

**FSP Example Pin Assignment:**
- SPI0: P410(MOSI), P411(MISO), P412(RSPCK), P413(SSL)
- SPI1: P202(MOSI), P203(MISO), P204(RSPCK), P205(SSL)

**NuttX Board Configuration:**
- Handled in `board.h` and pin configuration files
- Same physical pins, configured through NuttX pin mapping

## Interrupt Handling

### FSP Callbacks

**FSP Example:**
```c
void spi_master_callback(spi_callback_args_t * p_args) {
    if (SPI_EVENT_TRANSFER_COMPLETE == p_args->event) {
        g_master_event_flag = SPI_EVENT_TRANSFER_COMPLETE;
    } else {
        g_master_event_flag = SPI_EVENT_TRANSFER_ABORTED;
    }
}
```

**NuttX Implementation:**
- Uses NuttX interrupt system
- Callbacks handled internally by SPI driver
- Application uses synchronous blocking calls

## Testing and Verification

### FSP Example Verification
```c
if(BUFF_EQUAL == memcmp(g_master_tx_buff, g_slave_rx_buff, num_bytes_master)) {
    // Success
} else {
    // Error
}
```

**NuttX Implementation:**
```c
for (i = 0; i < SPI_BUFF_LEN; i++) {
    if (g_spi_loopback.master_tx_buff[i] != g_spi_loopback.slave_rx_buff[i]) {
        spierr("Master TX / Slave RX mismatch at index %d: 0x%08lx != 0x%08lx\n",
               i, g_spi_loopback.master_tx_buff[i], g_spi_loopback.slave_rx_buff[i]);
        errors++;
    }
}
```

## Benefits of NuttX Implementation

1. **Simplified API**: Higher-level abstraction through NuttX SPI interface
2. **Portability**: Works across different NuttX-supported platforms
3. **Integration**: Better integration with NuttX filesystem and device model
4. **Debugging**: Enhanced debugging through NuttX debug infrastructure
5. **Memory Management**: Automatic memory management through NuttX
6. **Error Handling**: Standardized error codes and handling

## Hardware Requirements

Both implementations require the same hardware connections:

| Connection | Purpose |
|------------|---------|
| P410 ↔ P202 | MOSI0 ↔ MOSI1 |
| P411 ↔ P203 | MISO0 ↔ MISO1 |
| P412 ↔ P204 | RSPCK0 ↔ RSPCK1 |
| P413 ↔ P205 | SSL0 ↔ SSL1 |

## Build and Usage

### FSP Example
- Built using e2studio IDE
- Requires FSP configuration tools
- Platform-specific build

### NuttX Implementation
```bash
cd nuttx
make distclean
./tools/configure.sh fpb-ra8e1:spi-loopback
make
```

Usage:
```bash
nsh> spi_loopback
```

This mapping demonstrates how the FSP SPI example functionality has been successfully adapted to work within the NuttX framework while maintaining the same core functionality and test patterns.
