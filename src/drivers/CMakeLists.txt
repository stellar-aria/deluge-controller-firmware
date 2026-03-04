add_library(drivers STATIC
    cache/invalidate.S
    dmac/dmac.c
    oled/oled.c
    oled/oled_low_level.c
    rspi/rspi.c
    ssi/ssi.c
    uart/uart.c
)

# Expose src/ as the include root so consumers use paths like "drivers/uart/uart.h",
# and driver sources can reach sibling directories (e.g. platform/timers_interrupts).
target_include_directories(drivers PUBLIC ${CMAKE_SOURCE_DIR}/src)
