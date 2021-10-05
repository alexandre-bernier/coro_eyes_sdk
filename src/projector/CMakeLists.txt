# Dependencies
## hidapi
find_path(LIBHIDAPI-LIBUSB_INCLUDE_DIR hidapi.h /usr/include/hidapi)
find_library(LIBHIDAPI-LIBUSB_LIBRARY hidapi-libusb)

set(HIDAPI_INCLUDE_DIRS ${LIBHIDAPI-LIBUSB_INCLUDE_DIR})
set(HIDAPI_LIBS ${LIBHIDAPI-LIBUSB_LIBRARY})

if(HIDAPI_INCLUDE_DIRS AND HIDAPI_LIBS)
    message(STATUS "HIDAPI found")
else()
    message(FATAL_ERROR "HIDAPI not found")
endif()

# Source files
set(PROJECTOR_INCLUDE_DIRS
#    ${PROJECT_SOURCE_DIR}/include/projector
#    ${PROJECT_SOURCE_DIR}/include/projector/dlp
    ${HIDAPI_INCLUDE_DIRS}
    CACHE INTERNAL "")

set(PROJECTOR_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/dlp/common/debug.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/dlp/common/module.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/dlp/common/other.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/dlp/common/parameters.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/dlp/common/returncode.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/dlp/common/image/image.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/dlp/common/pattern/pattern.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/dlp/common/pattern/pattern_sequence.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/dlp/dlp_platforms/dlp_platform.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/dlp/dlp_platforms/lightcrafter_4500/common.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/dlp/dlp_platforms/lightcrafter_4500/dlpc350_api.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/dlp/dlp_platforms/lightcrafter_4500/dlpc350_firmware.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/dlp/dlp_platforms/lightcrafter_4500/dlpc350_usb.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/dlp/dlp_platforms/lightcrafter_4500/lcr4500.cpp
    CACHE INTERNAL "")

# Libraries
set(PROJECTOR_LIBS
    ${HIDAPI_LIBS}
    CACHE INTERNAL "")
