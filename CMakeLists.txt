cmake_minimum_required(VERSION 3.18)

project(coro_eyes_sdk LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Subdirectories
add_subdirectory(src)

# Examples
option(BUILD_EXAMPLES "Build examples" ON)
if(BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# Doxygen documentation
option(BUILD_DOC "Build documentation" ON)
if(BUILD_DOC)
    find_package(Doxygen)
    if(Doxygen_FOUND)
        message(STATUS "Doxygen found")
        add_subdirectory(doc)
    else()
        message(STATUS "Doxygen not found, not building docs")
    endif()
endif()
