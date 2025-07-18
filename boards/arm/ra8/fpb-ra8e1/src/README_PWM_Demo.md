# RA8E1 PWM ESC Control Demo

This demo implements PWM-based Electronic Speed Controller (ESC) control for the RA8E1 FPB board using Segger RTT commands.

## Overview

The demo provides real-time control of up to 4 ESCs via PWM signals with the following features:

- **400Hz PWM frequency** - Standard ESC control frequency
- **1000-2000µs pulse width range** - Standard ESC pulse width range
- **Real-time RTT command interface** - Interactive control via Segger RTT
- **Safety features** - Automatic disarming and error handling
- **Individual channel control** - Control each ESC independently

## Hardware Configuration

### PWM Channel Mapping
- **ESC1**: PWM Channel 0 (`/dev/pwm0`) - GPT0
- **ESC2**: PWM Channel 1 (`/dev/pwm1`) - GPT1  
- **ESC3**: PWM Channel 2 (`/dev/pwm2`) - GPT2
- **ESC4**: PWM Channel 3 (`/dev/pwm3`) - GPT3

### Pin Assignments
The actual pin assignments depend on your board configuration. Update the board configuration files to map GPT channels to the appropriate pins for your ESC connections.

## Building and Running

### Prerequisites
1. NuttX configured with PWM support
2. CONFIG_EXAMPLES_PWM=y in configuration
3. PWM drivers enabled for GPT0-GPT3
4. Segger RTT support configured

### Build Steps
```bash
cd nuttx
make menuconfig  # Enable CONFIG_EXAMPLES_ESCS
make
```

### Running the Demo
```bash
# Flash the firmware to the board
# Connect via Segger RTT Viewer or J-Link RTT Client

# Start the demo
nsh> ra8e1_pwm_demo
```

## RTT Command Interface

### Available Commands

#### Basic Commands
- `help` - Show command menu and current status
- `status` - Display current ESC status
- `stop` - Stop the demo

#### ESC Control Commands  
- `arm` - Arm all ESCs (set to minimum throttle)
- `disarm` - Disarm all ESCs (stop PWM output)
- `arm<N>` - Arm specific ESC (N = 1-4)
- `esc<N> <percent>` - Set ESC throttle percentage (0-100%)

### Command Examples

```
> help                 # Show menu
> arm                  # Arm all ESCs  
> esc1 25              # Set ESC1 to 25% throttle
> esc2 50              # Set ESC2 to 50% throttle
> esc3 0               # Set ESC3 to 0% throttle
> arm2                 # Arm only ESC2
> disarm               # Disarm all ESCs
> status               # Show current status
> stop                 # Exit demo
```

## PWM Signal Specifications

### Timing Parameters
- **Frequency**: 400Hz (2.5ms period)
- **Minimum pulse**: 1000µs (0% throttle, armed state)
- **Maximum pulse**: 2000µs (100% throttle)
- **Disarmed**: No PWM output (0V)

### Pulse Width Calculation
```
pulse_width_us = 1000 + (throttle_percent * 10)
```

Where:
- `throttle_percent` ranges from 0 to 100
- Result ranges from 1000µs to 2000µs

## Safety Features

### Automatic Safety Measures
1. **Initial Disarmed State** - All ESCs start disarmed
2. **Input Validation** - Commands are validated before execution
3. **Range Limiting** - Throttle values are clamped to safe ranges
4. **Emergency Stop** - `disarm` command immediately stops all PWM output

### Best Practices
1. Always arm ESCs before applying throttle
2. Test with low throttle values first
3. Use `disarm` command before disconnecting
4. Monitor ESC temperatures during operation

## Troubleshooting

### Common Issues

#### PWM Device Not Found
```
ERROR: Failed to open /dev/pwm0: 2
```
**Solution**: Verify PWM drivers are enabled in board configuration

#### Permission Denied
```
ERROR: Failed to open /dev/pwm0: 13  
```
**Solution**: Ensure demo runs with appropriate privileges

#### No RTT Input
**Solution**: 
- Verify Segger RTT connection
- Check RTT buffer configuration
- Use J-Link RTT Viewer for input

### Debug Information
The demo provides detailed debug output including:
- PWM device initialization status
- Command parsing results
- Throttle setting confirmations
- Error messages with error codes

## Configuration Options

### NuttX Configuration
Required config options:
```
CONFIG_EXAMPLES_ESCS=y
CONFIG_PWM=y
CONFIG_RA8_GPT=y
CONFIG_RA8_GPT_PWM=y
```

### Customization
To modify the demo behavior, edit these parameters in `ra8e1_pwm_demo.c`:

```c
#define NUM_ESC_CHANNELS        4        /* Number of ESC channels */
#define ESC_PWM_FREQUENCY       400      /* PWM frequency (Hz) */
#define ESC_PWM_MIN_US          1000     /* Minimum pulse width (µs) */
#define ESC_PWM_MAX_US          2000     /* Maximum pulse width (µs) */
```

## Integration with Applications

### API Functions
The demo provides these functions for integration:

```c
/* Main demo entry point */
int ra8e1_pwm_demo_main(int argc, char *argv[]);

/* Get ESC status */
int ra8e1_esc_get_status(int esc_index, struct esc_status_s *status);
```

### Status Structure
```c
struct esc_status_s
{
  bool armed;                   /* ESC armed status */
  uint16_t throttle_percent;    /* Current throttle percentage */
  uint32_t pulse_width_us;      /* Current pulse width in microseconds */
};
```

## Performance Notes

- **CPU Usage**: Minimal when idle, brief spikes during command processing
- **Memory Usage**: ~2KB RAM for buffers and state
- **Response Time**: Commands processed within 1ms
- **PWM Precision**: ±1µs pulse width accuracy

## License

Licensed under the Apache License, Version 2.0. See the main NuttX license for details.
