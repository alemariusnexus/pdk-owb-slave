# Minimal CMake toolchain file to make SDCC for PDK work. Turns out you don't need anything SDCC or PDK specific, at
# least for simple projects.

set(CMAKE_CROSSCOMPILING ON)
set(CMAKE_SYSTEM_NAME "Generic")

# Without this, CMake fails during the configuration step when trying to build simple test programs. I'm sure there's
# another way to make this work, but I don't see any disadvantages from using this.
set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")
