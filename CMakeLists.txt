cmake_minimum_required(VERSION 3.23)
cmake_policy(SET CMP0135 NEW)

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_LIST_DIR}/scripts/cmake/CMakeToolchainDeluge.cmake)

project(Deluge
    LANGUAGES CXX C ASM
    VERSION 1.3.0
)

option(ENABLE_RTT "Enable SEGGER RTT debug output" ON)

# Colored output
set(CMAKE_COLOR_DIAGNOSTICS ON)
add_compile_options($<$<CXX_COMPILER_ID:Clang>:-fansi-escape-codes>)

# Global compilation options
add_compile_options(

    # Feature flags
    -fdiagnostics-parseable-fixits
    -fsigned-char

    # Debug symbols, also for release so we get a full ELF
    $<$<CONFIG:DEBUG,RELEASE,RELWITHDEBINFO>:-ggdb3> # Include

    # Optimization level
    $<$<CONFIG:DEBUG>:-Og>
    $<$<CONFIG:RELEASE,RELWITHDEBINFO>:-O2>

    # does not block inlining, just blocks considering all functions for inlining
    # https://developer.arm.com/documentation/100067/0600/Compiler-Command-line-Options/-finline-functions---fno-inline-functions?lang=en
    -fno-inline-functions

    # Link time optimizations
    $<$<CONFIG:RELEASE,RELWITHDEBINFO>:-flto=auto>
    $<$<CONFIG:RELEASE,RELWITHDEBINFO>:-fuse-linker-plugin>
    $<$<CONFIG:RELEASE,RELWITHDEBINFO>:-fdevirtualize-at-ltrans>
    -ffunction-sections
    -fdata-sections

    # C++ stuff
    $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-use-cxa-atexit>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-threadsafe-statics>

    # ASM stuff
    $<$<COMPILE_LANGUAGE:ASM>:-x>
    $<$<COMPILE_LANGUAGE:ASM>:assembler-with-cpp>
)

# Add third-party libraries
add_subdirectory(lib)

# Add source targets (startup, platform, drivers, controller, RZA1, RTT)
add_subdirectory(src)

# Add the main executable target
add_executable(deluge)

# Link against all source components
target_link_libraries(deluge PUBLIC
        startup
        platform
        controller
        -Wl,--whole-archive
        RTT
        -Wl,--no-whole-archive
)

# Project-only compile options (warnings, etc.)
set(DELUGE_C_STANDARD 23)
set(DELUGE_CXX_STANDARD 23)
list(APPEND DELUGE_COMMON_COMPILE_OPTIONS
    # Warnings
    -Warray-bounds=1
    -Wnull-dereference
    -Wparentheses
    -Wsequence-point
    -Werror=switch
    -Wextra
    -Wall
    -Wpedantic
    $<$<CONFIG:DEBUG>:-Wno-psabi>
    $<$<CONFIG:DEBUG>:-Wno-unused-parameter>

    $<$<CONFIG:DEBUG>:-Werror=write-strings>

    # Offsetof for non standard types is supported in GCC,
    # if another compiler does not support it they are obligated to error

    # Stack usage
    # tracked for diagnostic usage. We have a 32k stack so it's not that important
    # primarily concerned with unbounded stack use
    $<$<CONFIG:DEBUG>:-Wstack-usage=2000>
    $<$<CONFIG:RELEASE,RELWITHDEBINFO>:-Wstack-usage=2000>
)

target_compile_options(deluge PUBLIC ${DELUGE_COMMON_COMPILE_OPTIONS})

set_target_properties(deluge
    PROPERTIES
        C_STANDARD ${DELUGE_C_STANDARD}
        C_STANDARD_REQUIRED ON
        CXX_STANDARD ${DELUGE_CXX_STANDARD}
        CXX_STANDARD_REQUIRED ON
        CXX_EXTENSIONS ON
        SUFFIX ".elf"
)

set(BIN_FILE $<CONFIG>/deluge_controller.bin)

target_link_options(deluge PUBLIC
    -T ${PROJECT_SOURCE_DIR}/linker_script_rz_a1l.ld
    LINKER:--gc-sections
    LINKER:--entry=start
    -nostartfiles
    LINKER:--warn-common
    LINKER:--stats
    $<$<CONFIG:RELEASE,RELWITHDEBINFO>:-flto=auto>
    $<$<CONFIG:RELEASE,RELWITHDEBINFO>:-fuse-linker-plugin>
    $<$<CXX_COMPILER_ID:GNU>:-fdiagnostics-color=always>
)

add_custom_command(TARGET deluge POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary -S $<TARGET_FILE:deluge> ${BIN_FILE}
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Generating ${BIN_FILE}"
    BYPRODUCTS ${BIN_FILE}
)

add_custom_command(TARGET deluge POST_BUILD
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:deluge>
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
)

set_target_properties(deluge PROPERTIES
    ADDITIONAL_CLEAN_FILES "${BIN_FILE}"
)
