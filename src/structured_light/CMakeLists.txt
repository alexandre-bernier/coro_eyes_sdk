# Source files
set(STRUCTURED_LIGHT_INCLUDE_DIRS
    ${PROJECT_SOURCE_DIR}/include/structured_light
    CACHE INTERNAL "")

set(STRUCTURED_LIGHT_SRC
    structured_light.cpp)

# Dependencies
## OpenCV
list(APPEND STRUCTURED_LIGHT_INCLUDE_DIRS ${OpenCV_INCLUDE_DIRS})

# Build
add_library(structured_light ${STRUCTURED_LIGHT_SRC})
target_include_directories(structured_light PUBLIC ${STRUCTURED_LIGHT_INCLUDE_DIRS})
target_link_libraries(structured_light PRIVATE ${OpenCV_LIBS})
