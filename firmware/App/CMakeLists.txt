cmake_minimum_required(VERSION 3.22)

project(app)
add_library(app INTERFACE)

enable_language(C ASM)

add_subdirectory(Drivers/enc28j60)

target_compile_definitions(app INTERFACE
)

target_include_directories(app INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc
)

target_sources(app INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/app.c
)

target_link_directories(app INTERFACE
)

target_link_libraries(app INTERFACE
    stm32cubemx
    enc28j60
)
