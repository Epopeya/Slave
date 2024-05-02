#!/bin/sh

# Generate compile_commands.json
pio run -t compiledb

# Remove unavailable flags in clang
sed -i '' 's/-fno-tree-switch-conversion //g' compile_commands.json
sed -i '' 's/-fstrict-volatile-bitfields //g' compile_commands.json

# Add missing include dirs
sed -i '' "s@ src/main.cpp@ -I$HOME/.platformio/packages/toolchain-riscv32-esp/riscv32-esp-elf/include/c++/8.4.0 src/main.cpp@g" compile_commands.json
sed -i '' "s@ src/main.cpp@ -I$HOME/.platformio/packages/toolchain-riscv32-esp/riscv32-esp-elf/include/c++/8.4.0/riscv32-esp-elf src/main.cpp@g" compile_commands.json
