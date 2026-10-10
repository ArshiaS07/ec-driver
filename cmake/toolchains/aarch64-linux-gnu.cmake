# Cross-compile for 64-bit ARM Linux with the GNU toolchain
# (Ubuntu: sudo apt install g++-aarch64-linux-gnu).
# Used by the "aarch64-linux" preset; vcpkg builds dependencies for the
# matching arm64-linux triplet.
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

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
