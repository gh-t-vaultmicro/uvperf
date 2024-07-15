cmake_minimum_required(VERSION 3.12)

project(
    uvperf
    VERSION 1.0.5
    LANGUAGES C
)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_COMPILER "g++")

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)
include_directories(${LIBUSB_INCLUDE_DIRS})

add_executable(uvperf uvperf.c log.c)

#target_link_libraries(uvperf "${CMAKE_SOURCE_DIR}/lib/libusbK.dll")
target_link_libraries(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/lib/libusbK.lib ${LIBUSB_LIBRARIES})


