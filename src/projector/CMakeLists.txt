# Source files
set(PROJECTOR_INCLUDE_DIRS
    ${PROJECT_SOURCE_DIR}/include/projector
    ${PROJECT_SOURCE_DIR}/include/projector/dlp
    CACHE INTERNAL "")

set(PROJECTOR_SRC
    dlp/common/debug.cpp
    dlp/common/module.cpp
    dlp/common/other.cpp
    dlp/common/parameters.cpp
    dlp/common/returncode.cpp
    dlp/common/image/image.cpp
    dlp/common/pattern/pattern.cpp
    dlp/common/pattern/pattern_sequence.cpp
    dlp/dlp_platforms/dlp_platform.cpp
    dlp/dlp_platforms/lightcrafter_4500/common.cpp
    dlp/dlp_platforms/lightcrafter_4500/dlpc350_api.cpp
    dlp/dlp_platforms/lightcrafter_4500/dlpc350_firmware.cpp
    dlp/dlp_platforms/lightcrafter_4500/dlpc350_usb.cpp
    dlp/dlp_platforms/lightcrafter_4500/lcr4500.cpp
    structured_light_patterns.cpp)

# Dependencies
## OpenCV
list(APPEND PROJECTOR_INCLUDE_DIRS ${OpenCV_INCLUDE_DIRS})

## hidapi
find_path(LIBHIDAPI-LIBUSB_INCLUDE_DIR hidapi.h /usr/include/hidapi)
find_library(LIBHIDAPI-LIBUSB_LIBRARY hidapi-libusb)

set(HIDAPI_INCLUDE_DIRS ${LIBHIDAPI-LIBUSB_INCLUDE_DIR})
set(HIDAPI_LIBS ${LIBHIDAPI-LIBUSB_LIBRARY})

if(NOT HIDAPI_INCLUDE_DIRS OR NOT HIDAPI_LIBS)
    message(FATAL_ERROR "HIDAPI not found.")
endif()

list(APPEND PROJECTOR_INCLUDE_DIRS ${HIDAPI_INCLUDE_DIRS})

# Build
add_library(projector ${PROJECTOR_SRC})
target_include_directories(projector PUBLIC ${PROJECTOR_INCLUDE_DIRS})
target_link_libraries(projector ${OpenCV_LIBS} ${HIDAPI_LIBS})
