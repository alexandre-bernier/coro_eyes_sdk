# Install directories
set(INSTALL_INCLUDE_DIR include/${CMAKE_PROJECT_NAME})
set(INSTALL_LIB_DIR lib/${CMAKE_PROJECT_NAME})

# Version config file
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/${CMAKE_PROJECT_NAME}Config.h.in ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}Config.h)

# Common dependencies
find_package(OpenCV 4 REQUIRED)
if(OpenCV_FOUND)
    message(STATUS "OpenCV found")
else()
    message(FATAL_ERROR "Can't find OpenCV 4")
endif()
set(EXTRA_INCLUDE_DIRS ${OpenCV_INCLUDE_DIRS})
set(EXTRA_LIBS ${OpenCV_LIBS})

# Subdirectories
add_subdirectory(camera)
add_subdirectory(projector)
add_subdirectory(structured_light)

# Libraries source, include dir and libs
set(_SRC
    ${CAMERA_SRC}
    ${PROJECTOR_SRC}
    ${STRUCTURED_LIGHT_SRC})
set(_INCLUDE_DIRS
    ${CAMERA_INCLUDE_DIRS}
    ${PROJECTOR_INCLUDE_DIRS}
    ${STRUCTURED_LIGHT_INCLUDE_DIRS})
set(_LIBS
    ${CAMERA_LIBS}
    ${PROJECTOR_LIBS}
    ${STRUCTURED_LIGHT_LIBS})

# Build SDK
add_library(${CMAKE_PROJECT_NAME} ${_SRC})
target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${INSTALL_INCLUDE_DIR}>
    PUBLIC ${EXTRA_INCLUDE_DIRS}
    PRIVATE ${_INCLUDE_DIRS})
target_link_libraries(${CMAKE_PROJECT_NAME}
    PUBLIC ${EXTRA_LIBS}
    PRIVATE ${_LIBS})

# Install target
install(TARGETS ${CMAKE_PROJECT_NAME}
    DESTINATION "${INSTALL_LIB_DIR}"
    EXPORT ${CMAKE_PROJECT_NAME}Targets)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include//
    DESTINATION ${INSTALL_INCLUDE_DIR})
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}Config.h
    DESTINATION ${INSTALL_INCLUDE_DIR})

install(EXPORT ${CMAKE_PROJECT_NAME}Targets
    FILE "${CMAKE_PROJECT_NAME}Targets.cmake"
    DESTINATION "lib/cmake/${CMAKE_PROJECT_NAME}")

include(CMakePackageConfigHelpers)
# generate the config file that includes the exports
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/${CMAKE_PROJECT_NAME}Config.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}-config.cmake"
    INSTALL_DESTINATION "lib/cmake/${CMAKE_PROJECT_NAME}"
    NO_SET_AND_CHECK_MACRO
    NO_CHECK_REQUIRED_COMPONENTS_MACRO)
# generate the version file for the config file
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}ConfigVersion.cmake"
    VERSION "${coro_eyes_sdk_VERSION_MAJOR}.${coro_eyes_sdk_VERSION_MINOR}"
    COMPATIBILITY AnyNewerVersion)

# install the configuration file
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}-config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}ConfigVersion.cmake
    DESTINATION "lib/cmake/${CMAKE_PROJECT_NAME}")

export(EXPORT ${CMAKE_PROJECT_NAME}Targets
  FILE "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}Targets.cmake")
