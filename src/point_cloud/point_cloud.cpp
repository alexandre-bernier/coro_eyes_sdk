/** @file point_cloud.cpp
 *  @brief Contains methods for the PointCloud class.
 *  @copyright BSD-3-Clause License
 */

#include <iostream>
#include <vector>
#include "point_cloud.h"

PointCloud::PointCloud()
{

}

PointCloud::~PointCloud()
{

}

/**
 * @brief Save a point cloud to file.
 * @param file_name: Name of the file
 * @return True if successful
 */
bool PointCloud::save(const std::string &file_name)
{
    return open3d::io::WritePointCloud(file_name, _point_cloud);
}

/**
 * @brief Load a point cloud from file.
 * @param file_name: Name of the file
 * @return True if successful
 */
bool PointCloud::load(const std::string &file_name)
{
    return open3d::io::ReadPointCloud(file_name, _point_cloud);
}

/**
 * @brief Launch a visualization window to display the loaded point cloud.
 * @return True if successful
 */
bool PointCloud::visualize()
{
    // Load the point cloud into a vector
    std::vector<std::shared_ptr<const open3d::geometry::Geometry> > point_clouds;
    point_clouds.push_back(std::make_shared<open3d::geometry::PointCloud>(_point_cloud));

    // Display the point cloud
    return open3d::visualization::DrawGeometries(point_clouds, "Point Cloud");
}
