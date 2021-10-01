# Source files
set(POINT_CLOUD_INCLUDE_DIRS
    ${PROJECT_SOURCE_DIR}/include/point_cloud
    CACHE INTERNAL "")

set(POINT_CLOUD_SRC
    point_cloud.cpp)

# Dependencies
## Open3D
find_package(Open3D REQUIRED)
if(NOT Open3D_FOUND)
    message(FATAL_ERROR "Can't find Open3D.")
endif()
list(APPEND POINT_CLOUD_INCLUDE_DIRS ${Open3D_INCLUDE_DIRS} ${Open3D_INCLUDE_DIRS}/open3d/3rdparty)

# Build
add_library(point_cloud ${POINT_CLOUD_SRC})
target_include_directories(point_cloud PUBLIC ${POINT_CLOUD_INCLUDE_DIRS})
target_link_libraries(point_cloud PRIVATE Open3D::Open3D)
