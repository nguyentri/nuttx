# 🧠 NuttX on Windows with MSYS2 – FPB-RA8E1 Board

This guide explains how to set up and build NuttX for the **FPB-RA8E1** board on **Windows** using **MSYS2**, the **GNU ARM Embedded Toolchain**, and optionally **CMake**.

---

## ✅ 1. Prerequisites

### 1.1 Install Required Tools

- [MSYS2](https://www.msys2.org/)
- [GNU Arm Embedded Toolchain 13.2.rel1](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads/13-2-rel1)

Extract the ARM toolchain to:
```
C:\UserSoftware\arm-toolchain\arm-gnu-toolchain-13.2.Rel1\
```

### 1.2 Install MSYS2 Packages

Open the MSYS2 shell and run:

```bash
pacman -Syu     # Restart shell if prompted
pacman -S --needed make gcc g++ git patch diffutils \
    sed grep findutils cmake autoconf automake libtool \
    pkgconf flex bison gperf texinfo python ncurses-devel
```

---



## ✅ 2. Clone Source Repos

```bash
mkdir -p ~/nuttxspace
cd ~/nuttxspace

git clone https://github.com/apache/nuttx nuttx
git clone https://github.com/apache/nuttx-apps apps
```

---

## ✅ 2. Environment Setup

Navigate to nuttx directory and run the environment setup script:

```bash
cd nuttx
run ./boards/arm/ra8/fpb-ra8e1/set_evn.sh
```

This script will:
- Configure ARM GCC toolchain paths
- Create and activate Python virtual environment
- Install kconfiglib for configuration management
- Set up environment variables for both Make and CMake

Verify toolchain installation:

```bash
which arm-none-eabi-gcc
arm-none-eabi-gcc --version
```

---



## ✅ 5. Configure for FPB-RA8E1

### List available configurations:

```bash
cd nuttx
./tools/configure.sh -L fpb-ra8e1
```

Available configurations:
- `fpb-ra8e1:nsh` - Basic NuttShell configuration
- `fpb-ra8e1:nsh-leds` - NuttShell with LED support

### Configure for basic NuttShell:

```bash
./tools/configure.sh fpb-ra8e1:nsh
```

### Configure with LED support:

```bash
./tools/configure.sh fpb-ra8e1:nsh-leds
```

---

## ✅ 6. Optional: Customize with menuconfig

```bash
# Set proper locale for menuconfig
export LC_ALL=C
export LANG=C

# If menuconfig fails, try alternative configuration
make menuconfig

# Alternative: Use direct configuration
make oldconfig
```

Enable/verify:
- `CONFIG_ARCH_LEDS`
- `CONFIG_USERLED`
- `CONFIG_DEBUG_*` if needed

---

## ✅ 7. Build NuttX (Make)

```bash
make -j
```

### Output:

- `nuttx` → ELF (for debugging)
- `nuttx.bin` → Binary (for flashing)

---

## ✅ 8. Build NuttX (CMake – Alternative)

### Configure using CMake

First, clean any previous make builds:
```bash
make distclean  # if you built with make before
```

Configure with CMake:
```bash
cd nuttx
cmake -B build -DBOARD_CONFIG=fpb-ra8e1:nsh -GNinja
```

For LED configuration:
```bash
cmake -B build -DBOARD_CONFIG=fpb-ra8e1:nsh-leds -GNinja
```

### Use `menuconfig` with CMake

```bash
cmake --build build -t menuconfig
```

### Build with CMake

```bash
cmake --build build
```

### Clean CMake build

```bash
# Clean build artifacts only
cmake --build build -t clean

# Complete clean (equivalent to make distclean)
rm -rf build/
```

> ELF and BIN will appear in the `build/` directory.

---

## ✅ 9. Flash to Board (Optional)

Convert ELF to `.hex` or `.bin`:

```bash
arm-none-eabi-objcopy -O ihex nuttx nuttx.hex
```

Use **J-Link**, **pyOCD**, or **OpenOCD** to flash.

---

## ✅ Troubleshooting

- `arm-none-eabi-gcc: command not found`
  → Add toolchain to `$PATH`
- `kconfig-tweak: command not found`
  → Build `kconfig-frontends`
- Line ending errors (`^M`)
  → Use `dos2unix` or set VS Code to LF

---

Enjoy using Apache NuttX on the FPB-RA8E1 🚀