cmake_minimum_required(VERSION 3.5)

project(coro-eyes-sdk LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Subdirectories
add_subdirectory(src)

# Examples
option(BUILD_EXAMPLES "Build examples" ON)
if(BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# Doxygen documentation
find_package(Doxygen)
if(Doxygen_FOUND)
    add_subdirectory(doc)
else()
    message(STATUS "Doxygen not found, not building docs")
endif()
