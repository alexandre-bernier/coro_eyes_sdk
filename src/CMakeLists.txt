# Common dependencies
find_package(OpenCV REQUIRED)
if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "Can't find OpenCV.")
endif()
link_directories(${OpenCV_LIBRARY_DIRS})

# Subdirectories
add_subdirectory(camera)
add_subdirectory(projector)

# Libraries include dir and libs
set(CORO_EYES_SDK_INCLUDE_DIRS
    ${PROJECT_SOURCE_DIR}/include
    ${CAMERA_INCLUDE_DIRS}
    ${PROJECTOR_INCLUDE_DIRS}
    CACHE INTERNAL "")
set(CORO_EYES_SDK_LIBS
    camera
    projector
    CACHE INTERNAL "")

# Build SDK
add_library(CORO_EYES_SDK camera projector)
add_dependencies(CORO_EYES_SDK camera projector)
target_include_directories(CORO_EYES_SDK PUBLIC ${CORO_EYES_SDK_INCLUDE_DIRS})
target_link_libraries(CORO_EYES_SDK ${CORO_EYES_SDK_LIBS})
