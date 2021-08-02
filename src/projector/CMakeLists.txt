# Source files
set(PROJECTOR_INCLUDE_DIRS
    ${PROJECT_SOURCE_DIR}/include/projector
    CACHE INTERNAL "")

set(PROJECTOR_SRC
    common/debug.cpp
    common/module.cpp
    common/other.cpp
    common/parameters.cpp
    common/returncode.cpp
    common/image/image.cpp
    common/pattern/pattern.cpp
    common/pattern/pattern_sequence.cpp
    dlp_platforms/dlp_platform.cpp
    dlp_platforms/lightcrafter_4500/common.cpp
    dlp_platforms/lightcrafter_4500/dlpc350_api.cpp
    dlp_platforms/lightcrafter_4500/dlpc350_firmware.cpp
    dlp_platforms/lightcrafter_4500/dlpc350_usb.cpp
    dlp_platforms/lightcrafter_4500/lcr4500.cpp)

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
