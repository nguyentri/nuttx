#!/usr/bin/env bash

# === Arm Gcc Setup ===
export ARM_GCC_PATH="/c/UserSoftware/arm-toolchain/arm-gnu-toolchain-13.2.Rel1/bin"
export PATH="$ARM_GCC_PATH:$PATH"
export CROSSDEV=arm-none-eabi-

# === Jlink setup ===
export SEGGER_JLINK_PATH="/c/Users/a5094159/SEGGER/JLink_V850"
export PATH="$SEGGER_JLINK_PATH:$PATH"

# For GNU Make (legacy support)
export CC="${CROSSDEV}gcc"
export CXX="${CROSSDEV}g++"
export ARCH=arm

# Export full path to compilers for CMake
export CC="$ARM_GCC_PATH/arm-none-eabi-gcc"
export CXX="$ARM_GCC_PATH/arm-none-eabi-g++"
export ASM="$ARM_GCC_PATH/arm-none-eabi-gcc"

export CMAKE_C_COMPILER="$CC"
export CMAKE_CXX_COMPILER="$CXX"
export CMAKE_ASM_COMPILER="$ASM"

# === Verify ARM Toolchain ===
if ! command -v "${CROSSDEV}gcc" &> /dev/null; then
  echo "[!] ARM GCC toolchain not found in PATH!"
  echo "    Expected: ${CROSSDEV}gcc"
  echo "    PATH: $PATH"
  exit 1
fi

# === Verify JLink Tools ===
if ! command -v JLink &> /dev/null; then
  echo "[!] SEGGER J-Link tools not found in PATH!"
  echo "    Checked: $SEGGER_JLINK_PATH"
  exit 1
fi

echo "[+] ARM toolchain and SEGGER J-Link configured"
echo "    arm-none-eabi-gcc: $(which ${CROSSDEV}gcc)"
echo "    JLink: $(which JLink)"

# === Python Virtualenv Setup ===
# Create venv in parent directory (outside nuttx folder)
VENV_PATH="../venv"

if [ ! -d "$VENV_PATH" ]; then
  echo "[+] Creating Python virtual environment at $VENV_PATH..."
  python3 -m venv "$VENV_PATH"
fi

# Activate venv (correct path for Cygwin/MSYS2)
if [ -f "$VENV_PATH/bin/activate" ]; then
  source "$VENV_PATH/bin/activate"
elif [ -f "$VENV_PATH/Scripts/activate" ]; then
  source "$VENV_PATH/Scripts/activate"
else
  echo "[!] Could not find virtual environment activation script"
  echo "    Checked: $VENV_PATH/bin/activate and $VENV_PATH/Scripts/activate"
  exit 1
fi

echo "[+] Virtual environment activated"

# === Check if kconfiglib is installed in the virtual environment ===
if ! python -c "import kconfiglib" 2>/dev/null; then
  echo "[+] Installing kconfiglib in virtual environment..."
  pip install kconfiglib
  if [ $? -ne 0 ]; then
    echo "[!] Failed to install kconfiglib"
    exit 1
  fi
fi

# === Get kconfiglib path from the virtual environment ===
KCONFIGLIB_PATH=$(python -c "import kconfiglib; print(kconfiglib.__file__)" 2>/dev/null)

if [ -n "$KCONFIGLIB_PATH" ] && [ -f "$KCONFIGLIB_PATH" ]; then
  export KCONFIG_ENV="$KCONFIGLIB_PATH"
  echo "[+] KCONFIG_ENV set to: $KCONFIG_ENV"
else
  echo "[!] Failed to locate kconfiglib after installation."
  exit 1
fi

echo "[✓] Environment setup complete."
