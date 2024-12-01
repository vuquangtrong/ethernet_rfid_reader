cmake_minimum_required(VERSION 3.22)

project(enc28j60)
add_library(enc28j60 INTERFACE)

enable_language(C ASM)

target_compile_definitions(enc28j60 INTERFACE
)

target_include_directories(enc28j60 INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_sources(enc28j60 INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/enc28j60.c
)

target_link_directories(enc28j60 INTERFACE
)

target_link_libraries(enc28j60 INTERFACE
)
