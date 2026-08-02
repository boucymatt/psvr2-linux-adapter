# SPDX-License-Identifier: GPL-2.0
cmake_minimum_required(VERSION 3.16)
project(driver_psvr2 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_VISIBILITY_PRESET hidden)

set(REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../..")
set(OPENVR_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/openvr")
set(LIBPSVR2_DIR "${REPO_ROOT}/userspace/lib" CACHE PATH "libpsvr2 source dir")

add_library(driver_psvr2 MODULE
    src/hmd_driver_factory.cpp
    src/device_provider.cpp
    src/hmd_device_driver.cpp
    src/controller_device.cpp
    src/pose_source.cpp
    "${OPENVR_DIR}/driverlog.cpp"
)

target_include_directories(driver_psvr2 PRIVATE
    src
    "${OPENVR_DIR}"
    "${LIBPSVR2_DIR}"
)

find_library(LIBPSVR2_STATIC NAMES libpsvr2.a PATHS "${LIBPSVR2_DIR}" NO_DEFAULT_PATH)
if(LIBPSVR2_STATIC)
    target_link_libraries(driver_psvr2 PRIVATE "${LIBPSVR2_STATIC}")
else()
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(PSVR2 REQUIRED libpsvr2)
    target_include_directories(driver_psvr2 PRIVATE ${PSVR2_INCLUDE_DIRS})
    target_link_libraries(driver_psvr2 PRIVATE ${PSVR2_LIBRARIES})
endif()

find_package(Threads REQUIRED)
target_link_libraries(driver_psvr2 PRIVATE Threads::Threads)

set(PKG_DIR "${CMAKE_BINARY_DIR}/psvr2")
set_target_properties(driver_psvr2 PROPERTIES
    PREFIX ""
    LIBRARY_OUTPUT_DIRECTORY "${PKG_DIR}/bin/linux64"
)

add_custom_target(stage_resources ALL
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_CURRENT_SOURCE_DIR}/resources" "${PKG_DIR}"
    COMMENT "Staging SteamVR driver resources at ${PKG_DIR}"
    VERBATIM
)
add_dependencies(stage_resources driver_psvr2)
