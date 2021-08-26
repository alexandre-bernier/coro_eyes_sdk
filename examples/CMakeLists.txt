# Copy config files to binary location
file(COPY ../resources DESTINATION ${CMAKE_BINARY_DIR})

# Dependencies
## Threads
find_package(Threads REQUIRED)

# Build
## Camera example
add_executable(camera_example camera_example.cpp)
add_dependencies(camera_example CORO_EYES_SDK)
target_include_directories(camera_example PUBLIC ${CORO_EYES_SDK_INCLUDE_DIRS})
target_link_libraries(camera_example ${CORO_EYES_SDK_LIBS} Threads::Threads)

## Camera calibration example
add_executable(camera_calibration_example camera_calibration_example.cpp)
add_dependencies(camera_calibration_example CORO_EYES_SDK)
target_include_directories(camera_calibration_example PUBLIC ${CORO_EYES_SDK_INCLUDE_DIRS})
target_link_libraries(camera_calibration_example ${CORO_EYES_SDK_LIBS})

## Projector example
add_executable(projector_example projector_example.cpp)
add_dependencies(projector_example CORO_EYES_SDK)
target_include_directories(projector_example PUBLIC ${CORO_EYES_SDK_INCLUDE_DIRS})
target_link_libraries(projector_example ${CORO_EYES_SDK_LIBS} Threads::Threads)
