add_library(startup OBJECT
    main.c
    resetprg.c
    c_lib_alternatives.S
    memmove.c
    sys_stubs.c
    terminate.cpp
)

target_include_directories(startup PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src
)
