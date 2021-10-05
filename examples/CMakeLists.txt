# Copy config files to binary location
file(COPY ../resources DESTINATION ${CMAKE_BINARY_DIR})

# Dependencies
## Threads
find_package(Threads REQUIRED)
if(Threads_FOUND)
    message(STATUS "Threads found")
endif()

# Build
## Projector example
add_executable(projector_example projector_example.cpp)
target_link_libraries(projector_example coro_eyes_sdk Threads::Threads)

## Camera example
add_executable(camera_example camera_example.cpp)
target_link_libraries(camera_example coro_eyes_sdk)

## Camera calibration example
add_executable(camera_calibration_example camera_calibration_example.cpp)
target_link_libraries(camera_calibration_example coro_eyes_sdk)

## Structured light example
add_executable(structured_light_example structured_light_example.cpp)
target_link_libraries(structured_light_example coro_eyes_sdk Threads::Threads)
