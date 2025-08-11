# 🧠 NuttX on Windows with MSYS2

This guide explains how to set up, configure, and build NuttX for the **<Renesas RA8>** board on **Windows** using **MSYS2**, the **GNU ARM Embedded Toolchain**, and optionally **CMake**.

---

## ✅ 1. Prerequisites

### 1.1 Install Required Tools

- [MSYS2](https://www.msys2.org/)
- [GNU Arm Embedded Toolchain 13.2.rel1](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads/13-2-rel1)
- [CMake](https://cmake.org/download/) (for CMake build)
- [Ninja](https://ninja-build.org/) (recommended for CMake builds)

Extract the ARM toolchain to a directory, for example:

```
C:/UserSoftware/arm-toolchain/arm-gnu-toolchain-13.2.Rel1/
```

Add the toolchain's `bin` directory to your PATH in MSYS2:

```bash
export PATH=/c/UserSoftware/arm-toolchain/arm-gnu-toolchain-13.2.Rel1/bin:$PATH
```

### 1.2 Install MSYS2 Packages

Open the MSYS2 shell and run:

```bash
pacman -Syu     # Restart shell if prompted
pacman -S --needed make gcc g++ git patch diffutils     sed grep findutils cmake autoconf automake libtool     pkgconf flex bison gperf texinfo python ncurses-devel mingw-w64-x86_64-cmake mingw-w64-x86_64-ninja
```

---

## ✅ 2. Clone Source Repositories

```bash
mkdir -p ~/nuttxspace
cd ~/nuttxspace

git clone https://github.com/apache/nuttx nuttx
git clone https://github.com/apache/incubator-nuttx-apps apps
```

---

## ✅ 3. Environment Setup

Navigate to the `nuttx` directory and run the environment setup script:

```bash
cd nuttx
source ./tools/renesas/set_venv.sh
```

This script will:

- Configure ARM GCC toolchain paths
- Create and activate a Python virtual environment
- Install `kconfiglib` for configuration management
- Set up environment variables for both Make and CMake

Verify toolchain installation:

```bash
which arm-none-eabi-gcc
arm-none-eabi-gcc --version
```

---

## ✅ 4. Configure NuttX for <Renesas RA8 Board>

### Using Make (Legacy Build System)

List available configurations:

```bash
./tools/configure.sh -L | grep <Renesas RA8 Board>
```

Configure for the board:

```bash
./tools/configure.sh <Renesas RA8 Board>:<Configure Name>
```

Example:

```bash
./tools/configure.sh fpb-ra8e1:nsh
```

---

### Using CMake (Recommended for New Builds)

Create an out-of-tree build directory and configure:

```bash
cmake -B build -DBOARD_CONFIG=fpb-ra8e1:nsh -GNinja
```

You can optionally customize configuration with:

```bash
cmake --build . -t menuconfig
```

---

## ✅ 5. Build NuttX

### Using Make

```bash
make -j4
```

### Using CMake

```bash
cmake --build .
```

---

## ✅ 6. Clean Build Environment

### Using Make

- `make clean` — cleans build artifacts only.
- `make distclean` — cleans build artifacts *and* configuration files, resetting the source tree.

### Using CMake

- To clean build artifacts only:

  ```bash
  cmake --build . --target clean
  ```

- To fully clean (including configuration and generated files), delete the build directory:

  ```bash
  cd ../..
  rm -rf build/<Renesas RA8 Board>
  ```

---

## ✅ 7. Flashing Firmware (Optional)

Convert ELF to HEX:

```bash
arm-none-eabi-objcopy -O ihex nuttx nuttx.hex
```

Use your preferred flashing tool (J-Link, pyOCD, OpenOCD) to flash the firmware.

For example, using Renesas Programmer (RFP):

```bash
rfp-cli -d RA -tool jlink:1080979173 -if swd -file nuttx.hex -auto

---

Erase Chip

```bash
rfp-cli -d RA -tool jlink:1080979173 -if swd -erase-chip
```

## ✅ 8. Troubleshooting

- **`arm-none-eabi-gcc: command not found`**
  Ensure the ARM toolchain `bin` directory is in your PATH.

- **`kconfig-tweak: command not found`**
  Install `kconfiglib` in the Python virtual environment.

- **Line ending errors (`^M`)**
  Use `dos2unix` on scripts or set your editor to use LF line endings.

- **CMake errors like "build is not a directory" or "could not load cache"**
  Make sure to run the `cmake` configuration command with `-B build/<Renesas RA8 Board>` before building, and that the build directory exists.

---

## References

- [NuttX Official Documentation](https://nuttx.apache.org/docs/latest/)
- [NuttX CMake Build Guide](https://nuttx.apache.org/docs/latest/quickstart/compiling_cmake.html)
- [MSYS2 Official Website](https://www.msys2.org/)
- [GNU Arm Embedded Toolchain](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm)

---

*Happy building with NuttX!*
