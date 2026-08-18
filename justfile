# oxifoc — FOC motor controller monorepo

# Validate the active fixed-point core and STM32F103 firmware only. The host,
# G474, F405, bridge, and remote crates remain source references in this fork.
check:
    @just check-core
    @just check-f103

# Shared fixed-point controller: formatting, linting, and behavior tests.
check-core:
    #!/usr/bin/env bash
    set -euo pipefail
    command cargo fmt --check -p oxifoc-core
    command cargo clippy -p oxifoc-core --all-targets -- -D warnings
    command cargo test -p oxifoc-core

# STM32F103 application: host tests plus the optimized Thumb image.
check-f103:
    #!/usr/bin/env bash
    set -euo pipefail
    cd oxifoc-f103
    command cargo fmt --check
    command cargo test --target aarch64-apple-darwin
    command cargo clippy --release --features firmware -- -D warnings
    command cargo build --release --features firmware

# Format the active core and F103 application.
fmt:
    command cargo fmt -p oxifoc-core
    cd oxifoc-f103 && command cargo fmt

# Run the active fixed-point behavior tests.
test:
    command cargo test -p oxifoc-core
    cd oxifoc-f103 && command cargo test --target aarch64-apple-darwin

# Build the STM32F103 firmware.
build-f103:
    cd oxifoc-f103 && command cargo build --release --features firmware

# Build the exact 26,200-byte image consumed by the resident CAN bootloader.
image-f103: build-f103
    #!/usr/bin/env bash
    set -euo pipefail
    host_triple=$(command rustc -vV | command sed -n 's/^host: //p')
    llvm_objcopy="$(command rustc --print sysroot)/lib/rustlib/$host_triple/bin/llvm-objcopy"
    elf=oxifoc-f103/target/thumbv7m-none-eabi/release/oxifoc-f103
    image="$elf.flash-region.bin"
    command "$llvm_objcopy" -O binary --gap-fill=0xff --pad-to=0x08009e58 "$elf" "$image"
    test "$(command wc -c < "$image" | command tr -d ' ')" -eq 26200
    echo "$image: 26,200 bytes"

# With no arguments this validates only. Pass `--yes` to transmit/install.
flash-f103 *ARGS: image-f103
    command env UV_CACHE_DIR=scratch/uv-cache uv run scripts/can_bootloader_flash.py \
        oxifoc-f103/target/thumbv7m-none-eabi/release/oxifoc-f103.flash-region.bin \
        {{ ARGS }}

# Build ESP32 bridge firmware.
build-bridge:
    cd oxifoc-bridge && cargo build --release

# Build and flash ESP32 bridge firmware.
flash-bridge:
    cd oxifoc-bridge && cargo run --release

# Build ESP32 remote firmware.
build-remote:
    cd oxifoc-remote && cargo build --release

# Build and flash ESP32 remote firmware.
flash-remote:
    cd oxifoc-remote && cargo run --release

# Run host CLI with arguments
cli *ARGS:
    cargo run -p oxifoc-host-cli -- {{ ARGS }}

# Run host GUI
gui:
    cargo run -p oxifoc-host-slint

# Run the virtual device as a TCP Router on :2025 (extra args after `virtual`)
virtual *ARGS:
    cargo run -p oxifoc-virtual -- --transport tcp --port 2025 {{ ARGS }}

# End-to-end test: spawns the virtual Router and drives it via host-lib over
# both TCP and UDP (HardwareInfo handshake, at_least_once Motor,
# effectively_once Detect).
e2e:
    cargo test -p oxifoc-virtual --test e2e

# Flash usage of the STM32F103 firmware.
size:
    #!/usr/bin/env bash
    set -euo pipefail
    host_triple=$(command rustc -vV | command sed -n 's/^host: //p')
    llvm_size="$(command rustc --print sysroot)/lib/rustlib/$host_triple/bin/llvm-size"
    measure() { # crate label limit_file target_dir extra_flags...
        local crate="$1" label="$2" memx="$3" target_dir="$4"; shift 4
        (cd "$crate" && cargo build --release --quiet --target-dir "$target_dir" "$@" 2>/dev/null) || { echo "$label: build failed"; exit 1; }
        local elf="$crate/$target_dir/thumbv7m-none-eabi/release/$crate"
        local limit=$(awk '/FLASH/ { for (i = 1; i <= NF; i++) if ($i == "LENGTH") { v = $(i + 2); if (v ~ /K$/) { sub(/K$/, "", v); print v * 1024 } else print v; exit } }' "$crate/$memx")
        local used=$(command "$llvm_size" "$elf" | tail -1 | awk '{print $1+$2}')
        printf "%-24s %7d / %7d bytes (%2d%%), headroom %d\n" \
            "$label" "$used" "$limit" "$((used * 100 / limit))" "$((limit - used))"
    }
    measure oxifoc-f103 oxifoc-f103 memory.x target --features firmware

# Clean active core and F103 build artifacts.
clean:
    command cargo clean -p oxifoc-core
    cd oxifoc-f103 && command cargo clean
