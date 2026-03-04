cmake_minimum_required(VERSION 3.20)
project(DelugeASIO VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- ASIO SDK (included in sdk/ under GPL-3.0) ---
set(ASIO_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/sdk")
if(NOT EXISTS "${ASIO_SDK_DIR}/common/asio.h")
    message(FATAL_ERROR
        "ASIO SDK not found. Extract the Steinberg ASIO SDK to:\n"
        "  ${ASIO_SDK_DIR}/\n"
        "Download from: https://www.steinberg.net/asiosdk"
    )
endif()

# --- libusbK ---
# Set LIBUSBK_DIR to your libusbK SDK installation, e.g.:
#   cmake -B build -DLIBUSBK_DIR="C:/libusbK-dev-kit"
if(NOT DEFINED LIBUSBK_DIR)
    find_path(LIBUSBK_DIR
        NAMES includes/libusbk.h
        PATHS
            "C:/libusbK-dev-kit"
            "$ENV{ProgramFiles}/libusbK"
            "$ENV{ProgramFiles\(x86\)}/libusbK"
        DOC "Path to libusbK SDK"
    )
endif()

if(NOT LIBUSBK_DIR OR NOT EXISTS "${LIBUSBK_DIR}/includes/libusbk.h")
    message(FATAL_ERROR
        "libusbK SDK not found. Set LIBUSBK_DIR to the SDK root, e.g.:\n"
        "  cmake -B build -DLIBUSBK_DIR=\"C:/libusbK-dev-kit\"\n"
        "Download from: https://github.com/mcuee/libusbk/releases"
    )
endif()

set(LIBUSBK_INCLUDE "${LIBUSBK_DIR}/includes")
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(LIBUSBK_LIB "${LIBUSBK_DIR}/bin/lib/amd64/libusbK.lib")
else()
    set(LIBUSBK_LIB "${LIBUSBK_DIR}/bin/lib/x86/libusbK.lib")
endif()

if(NOT EXISTS "${LIBUSBK_LIB}")
    message(FATAL_ERROR "libusbK import library not found at: ${LIBUSBK_LIB}")
endif()

# --- ASIO SDK source files (COM framework, registration) ---
set(ASIO_SDK_SOURCES
    ${ASIO_SDK_DIR}/common/combase.cpp
    ${ASIO_SDK_DIR}/common/dllentry.cpp
    ${ASIO_SDK_DIR}/common/register.cpp
)

# --- DelugeASIO DLL ---
add_library(DelugeASIO SHARED
    # Our driver code
    src/deluge_usb.h
    src/usb_transport.h
    src/usb_transport.cpp
    src/asio_driver.h
    src/asio_driver.cpp
    src/dll_main.cpp
    src/DelugeASIO.def

    # ASIO SDK support files
    ${ASIO_SDK_SOURCES}
)

target_include_directories(DelugeASIO PRIVATE
    src
    ${ASIO_SDK_DIR}/common    # asio.h, iasiodrv.h, combase.h, asiosys.h, wxdebug.h
    ${LIBUSBK_INCLUDE}
)

target_link_libraries(DelugeASIO PRIVATE
    ${LIBUSBK_LIB}
    ole32
    oleaut32
    advapi32
    winmm          # timeGetTime
)

target_compile_definitions(DelugeASIO PRIVATE
    NOMINMAX
)

# NOTE: Do NOT define WIN32_LEAN_AND_MEAN globally — the ASIO SDK's dllentry.cpp
# and register.cpp need full Windows headers (COM, OLE, etc.).
# The ASIO SDK also uses ANSI (char*) Win32 registry calls — do NOT define UNICODE/_UNICODE.

# Suppress warnings from the ASIO SDK's older C++ style
if(MSVC)
    target_compile_options(DelugeASIO PRIVATE
        /W3
        /wd4996    # Suppress deprecated warnings (sprintf etc in SDK)
        /wd4100    # Unreferenced formal parameter
    )
endif()

# COM DLL — use the .def file for exports
set_target_properties(DelugeASIO PROPERTIES
    OUTPUT_NAME "DelugeASIO"
    PREFIX ""
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

# --- Standalone Test App ---
# Links the driver code directly (no COM registration needed).
add_executable(asio_test
    test/asio_test.cpp

    # Directly compile the driver + transport (not via DLL)
    src/usb_transport.cpp
    src/asio_driver.cpp

    # ASIO SDK COM framework (needed for CUnknown base class)
    ${ASIO_SDK_DIR}/common/combase.cpp
)

target_include_directories(asio_test PRIVATE
    src
    ${ASIO_SDK_DIR}/common
    ${LIBUSBK_INCLUDE}
)

target_link_libraries(asio_test PRIVATE
    ${LIBUSBK_LIB}
    ole32
    oleaut32
    advapi32
    winmm
)

target_compile_definitions(asio_test PRIVATE
    NOMINMAX
)

if(MSVC)
    target_compile_options(asio_test PRIVATE
        /W3
        /wd4996
        /wd4100
    )
endif()

set_target_properties(asio_test PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)
