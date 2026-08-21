# Validate the shared core and both STM32F103 applications.
check:
    @just check-core
    @just check-f103
    @just check-f103-calibration
    @just check-can

check-core:
    #!/usr/bin/env bash
    set -euo pipefail
    command cargo fmt --check -p oxifoc-core
    command cargo clippy -p oxifoc-core --all-targets -- -D warnings
    command cargo test -p oxifoc-core

check-f103:
    #!/usr/bin/env bash
    set -euo pipefail
    cd oxifoc-f103
    host_triple=$(command rustc -vV | command sed -n 's/^host: //p')
    command cargo fmt --check
    command cargo test --target "$host_triple"
    command cargo clippy --release --features firmware -- -D warnings
    command cargo build --release --features firmware
    llvm_objdump="$(command rustc --print sysroot)/lib/rustlib/$host_triple/bin/llvm-objdump"
    read -r retained_size retained_address retained_type <<< "$(command "$llvm_objdump" -h target/thumbv7m-none-eabi/release/oxifoc-f103 | command awk '$2 == ".retained" { print $3, $4, $5 }')"
    test "$retained_address" = 20004f00
    test "$retained_type" = BSS
    test "$((16#$retained_size))" -le 256

check-f103-calibration:
    #!/usr/bin/env bash
    set -euo pipefail
    cd oxifoc-f103-calibration
    host_triple=$(command rustc -vV | command sed -n 's/^host: //p')
    command cargo fmt --check
    command cargo test --target "$host_triple"
    command cargo clippy --release --features firmware -- -D warnings
    command cargo build --release --features firmware
    llvm_objdump="$(command rustc --print sysroot)/lib/rustlib/$host_triple/bin/llvm-objdump"
    read -r retained_size retained_address retained_type <<< "$(command "$llvm_objdump" -h target/thumbv7m-none-eabi/release/oxifoc-f103-calibration | command awk '$2 == ".retained" { print $3, $4, $5 }')"
    test "$retained_address" = 20004f00
    test "$retained_type" = BSS
    test "$((16#$retained_size))" -le 256

check-can:
    command env UV_CACHE_DIR=scratch/uv-cache uv run scripts/test_can_bootloader_flash.py
    command env UV_CACHE_DIR=scratch/uv-cache uv run scripts/test_can_f103_calibration.py

fmt:
    command cargo fmt -p oxifoc-core
    command cargo fmt --manifest-path oxifoc-f103/Cargo.toml
    command cargo fmt --manifest-path oxifoc-f103-calibration/Cargo.toml

test:
    #!/usr/bin/env bash
    set -euo pipefail
    host_triple=$(command rustc -vV | command sed -n 's/^host: //p')
    command cargo test -p oxifoc-core
    command cargo test --manifest-path oxifoc-f103/Cargo.toml --target "$host_triple"
    command cargo test --manifest-path oxifoc-f103-calibration/Cargo.toml --target "$host_triple"

build-f103:
    #!/usr/bin/env bash
    set -euo pipefail
    cd oxifoc-f103
    command cargo build --release --features firmware

build-f103-calibration:
    #!/usr/bin/env bash
    set -euo pipefail
    cd oxifoc-f103-calibration
    command cargo build --release --features firmware

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

# Build the exact 26,200-byte calibration image consumed by the CAN bootloader.
image-f103-calibration: build-f103-calibration
    #!/usr/bin/env bash
    set -euo pipefail
    host_triple=$(command rustc -vV | command sed -n 's/^host: //p')
    llvm_objcopy="$(command rustc --print sysroot)/lib/rustlib/$host_triple/bin/llvm-objcopy"
    elf=oxifoc-f103-calibration/target/thumbv7m-none-eabi/release/oxifoc-f103-calibration
    image="$elf.flash-region.bin"
    command "$llvm_objcopy" -O binary --gap-fill=0xff --pad-to=0x08009e58 "$elf" "$image"
    test "$(command wc -c < "$image" | command tr -d ' ')" -eq 26200
    echo "$image: 26,200 bytes"

# With no arguments this validates only. Pass --yes to transmit and install.
flash-f103 *ARGS: image-f103
    command env UV_CACHE_DIR=scratch/uv-cache uv run scripts/can_bootloader_flash.py oxifoc-f103/target/thumbv7m-none-eabi/release/oxifoc-f103.flash-region.bin {{ ARGS }}

# With no arguments this validates only. Pass --yes to transmit and install.
flash-f103-calibration *ARGS: image-f103-calibration
    command env UV_CACHE_DIR=scratch/uv-cache uv run scripts/can_bootloader_flash.py oxifoc-f103-calibration/target/thumbv7m-none-eabi/release/oxifoc-f103-calibration.flash-region.bin {{ ARGS }}

size:
    #!/usr/bin/env bash
    set -euo pipefail
    host_triple=$(command rustc -vV | command sed -n 's/^host: //p')
    llvm_size="$(command rustc --print sysroot)/lib/rustlib/$host_triple/bin/llvm-size"
    measure() {
        local crate="$1" label="$2" memx="$3" target_dir="$4"
        shift 4
        (
            cd "$crate"
            command cargo build --release --quiet --target-dir "$target_dir" "$@"
        ) || {
            echo "$label: build failed"
            exit 1
        }
        local elf="$crate/$target_dir/thumbv7m-none-eabi/release/$crate"
        local limit
        local used
        limit=$(command awk '/FLASH/ { for (i = 1; i <= NF; i++) if ($i == "LENGTH") { v = $(i + 2); if (v ~ /K$/) { sub(/K$/, "", v); print v * 1024 } else print v; exit } }' "$crate/$memx")
        used=$(command "$llvm_size" "$elf" | command tail -1 | command awk '{print $1+$2}')
        printf "%-24s %7d / %7d bytes (%2d%%), headroom %d\n" "$label" "$used" "$limit" "$((used * 100 / limit))" "$((limit - used))"
    }
    measure oxifoc-f103 oxifoc-f103 memory.x target --features firmware
    measure oxifoc-f103-calibration oxifoc-f103-calibration memory.x target --features firmware

clean:
    command cargo clean -p oxifoc-core
    command cargo clean --manifest-path oxifoc-f103/Cargo.toml
    command cargo clean --manifest-path oxifoc-f103-calibration/Cargo.toml
