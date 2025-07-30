#!/usr/bin/env bash

# === MSYS2 Tool Paths ===
export MSYS2_BIN="/usr/bin"
# Note: Don't use MINGW64 for PX4 builds as it lacks POSIX headers like sys/utsname.h

# === Custom Tool Paths ===
export ARM_GCC_PATH="/c/UserSoftware/arm-toolchain/arm-gnu-toolchain-13.2.Rel1/bin"

# === Build PATH ===
# Use MSYS2 POSIX tools (NOT MinGW64) for PX4 compatibility
# Prepend MSYS2 tools and ARM toolchain to existing PATH
export PATH="$MSYS2_BIN:$ARM_GCC_PATH:$PATH"

# === Clean Python Environment ===
unset PYTHONHOME
unset PYTHONPATH
unset GNUMAKE
unset GCC
unset cmake

# === Verify Core Tools ===
echo "[+] Core Tool Verification:"
echo "    make: $(which make) -> $(make --version | head -n1)"
echo "    find: $(which find) -> $(find --version | head -n1)"
echo "    ps: $(which ps) -> $(ps --version | head -n1)"
echo "    cmake: $(which cmake) -> $(cmake --version | head -n1)"
echo "    gcc: $(which gcc) -> $(gcc --version | head -n1)"

# === Python Setup ===
echo "[+] Python Verification:"
echo "    Python path: $(which python)"
echo "    Python version: $(python --version)"
echo "    pip version: $(python -m pip --version)"

# === Install remaining packages with --break-system-packages if needed ===
echo "[+] Installing additional Python packages..."
python -m pip install --break-system-packages \
    empy==3.3.4 \
    argcomplete \
    cerberus \
    kconfiglib==14.1.0 \
    pyros-genmsg \
    jsonschema==4.17.3 \
    toml==0.10.2 \
    pyserial \
    jinja2 \
    pyyaml \
    lxml

# === Verify Python Packages ===
echo "[+] Verifying Python packages:"
python -c "import em, jinja2, yaml, kconfiglib; print('✓ All packages found')" || {
    echo "[!] Python packages verification failed"
    exit 1
}

# === Verify XML support ===
echo "[+] Verifying XML support:"
python -c "from lxml import etree; print('✓ lxml.etree is working properly')" || {
    echo "[!] lxml XML support not working properly"
    echo "    This may cause XML validation warnings but build should continue"
}

# === Toolchain Setup ===
export CROSSDEV=arm-none-eabi-
export CC="${CROSSDEV}gcc"
export CXX="${CROSSDEV}g++"
export ARCH=arm

# For CMake
export CMAKE_C_COMPILER="$ARM_GCC_PATH/arm-none-eabi-gcc"
export CMAKE_CXX_COMPILER="$ARM_GCC_PATH/arm-none-eabi-g++"
export CMAKE_ASM_COMPILER="$ARM_GCC_PATH/arm-none-eabi-gcc"

# Verify ARM toolchain
echo "[+] ARM Tool Verification:"
echo "    ARM GCC: $(which ${CROSSDEV}gcc) -> $(${CROSSDEV}gcc --version | head -n1)"

echo "[✓] Environment fully set up"