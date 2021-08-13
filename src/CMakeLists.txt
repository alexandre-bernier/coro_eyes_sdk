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
    ${PROJECTOR_INCLUDE_DIRS}
    CACHE INTERNAL "")
set(CORO_EYES_SDK_LIBS
    projector
    CACHE INTERNAL "")

# Build SDK
add_library(CORO_EYES_SDK projector)
add_dependencies(CORO_EYES_SDK projector)
target_include_directories(CORO_EYES_SDK PUBLIC ${CORO_EYES_SDK_INCLUDE_DIRS})
target_link_libraries(CORO_EYES_SDK ${CORO_EYES_SDK_LIBS})
