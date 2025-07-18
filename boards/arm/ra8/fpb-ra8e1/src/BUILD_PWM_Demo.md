# PWM Demo Configuration and Build Instructions

## Configuration Requirements

### NuttX Configuration Options

Add these options to your NuttX configuration (via `make menuconfig`):

```
# PWM Support
CONFIG_PWM=y

# Examples
CONFIG_EXAMPLES_ESCS=y

# RA8 GPT Timer Support  
CONFIG_RA8_GPT=y
CONFIG_RA8_GPT_PWM=y

# Optional: Segger RTT Support
CONFIG_SEGGER_RTT=y
CONFIG_STREAM_RTT=y

# Optional: Debug Support
CONFIG_DEBUG_PWM=y
CONFIG_DEBUG_PWM_INFO=y
```

### Board Configuration

Edit `boards/arm/ra8/fpb-ra8e1/configs/nsh/defconfig` and add:

```
CONFIG_EXAMPLES_ESCS=y
CONFIG_RA8_GPT_PWM=y
CONFIG_PWM=y
```

## Build Instructions

1. **Configure NuttX:**
   ```bash
   cd nuttx
   make menuconfig
   # Enable the options listed above
   ```

2. **Clean and Build:**
   ```bash
   make clean
   make
   ```

3. **Flash to Board:**
   ```bash
   # Use your preferred flashing method
   # Example with J-Link:
   JLinkExe -device R7FA8E1AF -if SWD -speed 4000
   ```

## Pin Configuration

Update the board pin configuration in `boards/arm/ra8/fpb-ra8e1/include/board.h`:

```c
/* PWM Pin Definitions */
#define GPIO_GPT0_GTIOCA    /* Define pin for ESC1 */
#define GPIO_GPT1_GTIOCA    /* Define pin for ESC2 */
#define GPIO_GPT2_GTIOCA    /* Define pin for ESC3 */
#define GPIO_GPT3_GTIOCA    /* Define pin for ESC4 */
```

## Usage

1. **Start NuttX Shell:**
   ```
   nsh> 
   ```

2. **Run PWM Demo:**
   ```
   nsh> ra8e1_pwm_demo
   ```

3. **Use RTT Commands:**
   - Connect with J-Link RTT Viewer
   - Type commands like: `help`, `arm`, `esc1 25`, etc.

## Troubleshooting

### Build Errors
- Ensure all required CONFIG options are enabled
- Check that PWM drivers are compiled in

### Runtime Errors  
- Verify PWM device nodes exist: `/dev/pwm0`, `/dev/pwm1`, etc.
- Check pin configurations match your hardware
- Ensure proper power supply for ESCs

### RTT Connection Issues
- Use J-Link RTT Viewer or RTT Client
- Verify RTT configuration in NuttX
- Check debugger connection
