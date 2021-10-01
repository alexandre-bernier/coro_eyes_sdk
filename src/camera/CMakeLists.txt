# Source files
set(CAMERA_INCLUDE_DIRS
    ${PROJECT_SOURCE_DIR}/include/camera
    CACHE INTERNAL "")

set(CAMERA_SRC
    camera.cpp
    camera_calibration.cpp)

# Dependencies
## OpenCV
list(APPEND CAMERA_INCLUDE_DIRS ${OpenCV_INCLUDE_DIRS})

## FlyCapture
find_path(PG_FLYCAP_INCLUDE_DIR FlyCapture2.h "/usr/include/flycapture")
find_library(PG_FLYCAP_LIB flycapture "/usr/lib/")
find_library(PG_FLYCAP_LIB_C flycapture-c "/usr/lib/")
if(NOT PG_FLYCAP_LIB OR NOT PG_FLYCAP_LIB_C)
    message(FATAL_ERROR "Can't find FlyCap library.")
endif()
link_directories(${PG_FLYCAP_LIBRARY_DIRS})
list(APPEND CAMERA_INCLUDE_DIRS ${PG_FLYCAP_INCLUDE_DIRS})

# Build
add_library(camera ${CAMERA_SRC})
target_include_directories(camera PUBLIC ${CAMERA_INCLUDE_DIRS})
target_link_libraries(camera PRIVATE ${PG_FLYCAP_LIB} ${PG_FLYCAP_LIB_C} ${OpenCV_LIBS})
