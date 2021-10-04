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
add_dependencies(projector_example CORO_EYES_SDK)
target_include_directories(projector_example PUBLIC ${CORO_EYES_SDK_INCLUDE_DIRS})
target_link_libraries(projector_example ${CORO_EYES_SDK_LIBS} Threads::Threads)

## Camera example
add_executable(camera_example camera_example.cpp)
add_dependencies(camera_example CORO_EYES_SDK)
target_include_directories(camera_example PUBLIC ${CORO_EYES_SDK_INCLUDE_DIRS})
target_link_libraries(camera_example ${CORO_EYES_SDK_LIBS})

## Camera calibration example
add_executable(camera_calibration_example camera_calibration_example.cpp)
add_dependencies(camera_calibration_example CORO_EYES_SDK)
target_include_directories(camera_calibration_example PUBLIC ${CORO_EYES_SDK_INCLUDE_DIRS})
target_link_libraries(camera_calibration_example ${CORO_EYES_SDK_LIBS})

## Structured light example
add_executable(structured_light_example structured_light_example.cpp)
add_dependencies(structured_light_example CORO_EYES_SDK)
target_include_directories(structured_light_example PUBLIC ${CORO_EYES_SDK_INCLUDE_DIRS})
target_link_libraries(structured_light_example ${CORO_EYES_SDK_LIBS} Threads::Threads)
