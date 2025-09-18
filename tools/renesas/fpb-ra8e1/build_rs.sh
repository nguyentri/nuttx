#!/bin/bash
set -e

cd /home/a5094159/projects/nuttx_ra_dev/nuttx && make distclean && ./tools/configure.sh fpb-ra8e1:nsh-rust
cd /home/a5094159/projects/nuttx_ra_dev/nuttx/boards/arm/ra8/fpb-ra8e1/rust_lib && source "$HOME/.cargo/env" && cargo clean && cargo build --release --target thumbv8m.main-none-eabihf
cd /home/a5094159/projects/nuttx_ra_dev/nuttx && source "$HOME/.cargo/env" && make -j$(nproc)
