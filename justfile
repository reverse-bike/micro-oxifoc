# oxifoc — FOC motor controller monorepo

# Device firmware crates (excluded from workspace, different toolchain)
device_crates := "oxifoc-f103 oxifoc-bridge oxifoc-remote"

# Run all checks (fmt, clippy, tests — workspace + device crates)
check:
    @just check-host
    @just check-device

# Host workspace: fmt + clippy + tests
check-host:
    #!/usr/bin/env bash
    set -euo pipefail
    echo "git-rev sync across lock files..."
    python3 scripts/check-git-rev-sync.py
    echo "rustfmt (workspace)..."
    cargo fmt --check --all
    echo "clippy (workspace)..."
    cargo clippy --workspace --all-targets --quiet -- -D warnings
    echo "tests (workspace)..."
    output=$(cargo test --workspace --quiet 2>&1) || { echo "$output"; exit 1; }
    echo "oxifoc-core fixed-point slice..."
    cargo test -p oxifoc-core --quiet --no-default-features --features fixed-point
    echo "oxifoc-core without detection (gate must not rot)..."
    # clippy, not check: the embassy-gated modules are compiled ONLY in this
    # slice (no workspace member enables the feature), so this is their one
    # lint gate.
    cargo clippy -p oxifoc-core --quiet --no-default-features \
        --features algorithms,runtime,storage,delivery,defmt,embassy,virtual-motor,std -- -D warnings

# Device firmware: fmt + clippy + build (all targets)
check-device:
    #!/usr/bin/env bash
    set -euo pipefail
    filter() { grep -v 'unstable feature.*vfp2\|not stably supported\|unknown and unstable feature.*fp64\|still passed through to the codegen\|consider filing a feature request\|^  |\|^$' || true; }
    for crate in {{ device_crates }}; do
        echo "$crate: fmt + clippy + build..."
        (cd "$crate" && cargo fmt --check) || exit 1
        (cd "$crate" && cargo clippy --quiet -- -D warnings -W clippy::disallowed-methods 2>&1 | filter) || exit 1
        if [ "$crate" = "oxifoc-f103" ]; then
            (cd "$crate" && cargo build --release --quiet --features firmware 2>&1 | filter) || exit 1
        else
            (cd "$crate" && cargo build --release --quiet 2>&1 | filter) || exit 1
        fi
    done

# Format all code (workspace + device crates)
fmt:
    #!/usr/bin/env bash
    set -euo pipefail
    echo "rustfmt..."
    cargo fmt --all
    for crate in {{ device_crates }}; do
        (cd "$crate" && cargo fmt)
    done

# Run workspace tests
test:
    cargo test --workspace
    cargo test -p oxifoc-core --no-default-features --features fixed-point
    # HFI is behind features that are off by default; this pass runs the
    # `hfi`/`hfi-detect`-gated tests (g474/f405 config).
    cargo test -p oxifoc-core --features runtime,virtual-motor,storage,std,delivery,hfi,hfi-detect

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

# Clean all build artifacts
clean:
    cargo clean
    for crate in {{ device_crates }}; do (cd "$crate" && cargo clean); done
