# Dependencies
find_package(OpenCV REQUIRED)
if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "Can't find OpenCV.")
endif()
link_directories(${OpenCV_LIBRARY_DIRS})

# Subdirectories
add_subdirectory(camera)
add_subdirectory(projector)
