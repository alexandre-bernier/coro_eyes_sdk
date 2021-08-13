file(COPY ../resources DESTINATION ${CMAKE_BINARY_DIR})

find_package(Threads REQUIRED)

add_executable(projector_example projector_example.cpp)
target_include_directories(projector_example PUBLIC ${CORO_EYES_SDK_INCLUDE_DIRS})
target_link_libraries(projector_example ${CORO_EYES_SDK_LIBS} Threads::Threads)
