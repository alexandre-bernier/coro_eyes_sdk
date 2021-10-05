# Dependencies
## FlyCapture
find_path(PG_FLYCAP_INCLUDE_DIR FlyCapture2.h "/usr/include/flycapture")
find_library(PG_FLYCAP_LIB flycapture "/usr/lib/")
find_library(PG_FLYCAP_LIB_C flycapture-c "/usr/lib/")
if(PG_FLYCAP_LIB AND PG_FLYCAP_LIB_C)
    message(STATUS "FlyCapture found")
else()
    message(FATAL_ERROR "Can't find FlyCap library.")
endif()
link_directories(${PG_FLYCAP_LIBRARY_DIRS})

# Source files
set(CAMERA_INCLUDE_DIRS
#    ${PROJECT_SOURCE_DIR}/include/camera
    ${PG_FLYCAP_INCLUDE_DIRS}
    CACHE INTERNAL "")

set(CAMERA_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/camera.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/camera_calibration.cpp
    CACHE INTERNAL "")

# Libraries
set(CAMERA_LIBS
    ${PG_FLYCAP_LIB}
    ${PG_FLYCAP_LIB_C}
    CACHE INTERNAL "")
