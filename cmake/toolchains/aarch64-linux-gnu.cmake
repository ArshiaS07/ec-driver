# Cross-compile for 64-bit ARM Linux with the GNU toolchain
# (Ubuntu: sudo apt install g++-aarch64-linux-gnu).
# Used by the linux-aarch64 preset, which also selects vcpkg's arm64-linux
# triplet. vcpkg chainloads this file for its own builds too; the C compiler is
# set for those, since this project itself is C++ only.
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)

# Run cross-built test binaries through QEMU user mode if it is installed
# (sudo apt install qemu-user), so ctest works on the build machine.
find_program(EC_QEMU qemu-aarch64)
if(EC_QEMU)
    set(CMAKE_CROSSCOMPILING_EMULATOR ${EC_QEMU} -L /usr/aarch64-linux-gnu)
endif()

# Libraries, headers and packages come from the target side only; build tools
# from the host.
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
