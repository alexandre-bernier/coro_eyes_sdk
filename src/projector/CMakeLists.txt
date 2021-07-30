# Source files
set(PROJECTOR_INCLUDE_DIRS
    ${PROJECT_SOURCE_DIR}/include/projector
    ${PROJECT_SOURCE_DIR}/include/projector/dlp_platforms)

set(PROJECTOR_SRC
    dlp_platforms/dlp_platform.cpp
    dlp_platforms/lightcrafter_4500/common.cpp
    dlp_platforms/lightcrafter_4500/dlpc350_api.cpp
    dlp_platforms/lightcrafter_4500/dlpc350_firmware.cpp
    dlp_platforms/lightcrafter_4500/dlpc350_usb.cpp
    dlp_platforms/lightcrafter_4500/lcr4500.cpp)

# Dependencies
list(APPEND PROJECTOR_INCLUDE_DIRS ${OpenCV_INCLUDE_DIRS})

# Build
add_library(projector_api ${PROJECTOR_SRC})
target_include_directories(projector_api PUBLIC ${PROJECTOR_INCLUDE_DIRS})
target_link_libraries(projector_api ${OpenCV_LIBS})
