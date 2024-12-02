cmake_minimum_required(VERSION 3.22)

project(app)
add_library(app INTERFACE)

enable_language(C ASM)

add_subdirectory(Drivers/enc28j60)


#
# LwIP
#
set (LWIP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/LwIP)

set (LWIP_INCLUDE_DIRS
    ${LWIP_DIR}/src/include
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc # for include "lwipopts.h", "arch/cc.h"
)

include(${LWIP_DIR}/src/Filelists.cmake)

#
# Main App
#
target_compile_definitions(app INTERFACE
)

target_include_directories(app INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc
    ${LWIP_INCLUDE_DIRS}
)

target_sources(app INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/app.c
)

target_link_directories(app INTERFACE
)

target_link_libraries(app INTERFACE
    stm32cubemx
    enc28j60
    lwipcore
)
