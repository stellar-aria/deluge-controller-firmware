# Controller application CMakeLists.txt

# Controller source files
set(CONTROLLER_SOURCES
    controller.cpp
    hardware_events.cpp
    usb_serial.cpp
    display_control.cpp
    usb_descriptors.c
    usb_audio.cpp
    usb_midi.c
    oled_init.c
    ${CMAKE_SOURCE_DIR}/src/util/cfunctions.c
)

# Add controller library
add_library(controller OBJECT ${CONTROLLER_SOURCES})

# Set C++23 standard for controller
set_target_properties(controller PROPERTIES
    CXX_STANDARD 23
    CXX_STANDARD_REQUIRED ON
)

# Include directories for controller
target_include_directories(controller PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/src/RZA1/system
)
target_include_directories(controller SYSTEM PUBLIC
    ${CMAKE_SOURCE_DIR}/lib/tinyusb/src
)

# Link against dependencies
target_link_libraries(controller PUBLIC
    RZA1
    RTT
    tinyusb
    drivers
)

# USB Audio mode: 0 = headset (async+implicit FB), 1 = speaker-only (async+explicit FB)
# Override with: cmake -DUSB_AUDIO_SPEAKER_ONLY=1
if(USB_AUDIO_SPEAKER_ONLY)
    target_compile_definitions(controller PUBLIC USB_AUDIO_SPEAKER_ONLY=1)
    target_compile_definitions(tinyusb PUBLIC USB_AUDIO_SPEAKER_ONLY=1)
endif()

# Verbose controller debug logging (boot trace, USB status, audio events).
# Off by default; enable with: cmake -DCONTROLLER_DEBUG=1
if(CONTROLLER_DEBUG)
    target_compile_definitions(controller PRIVATE CONTROLLER_DEBUG=1)
endif()
