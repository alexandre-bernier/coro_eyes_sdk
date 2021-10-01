/** @file point_cloud.h
 *  @brief Main header for the point cloud api.
 *  @copyright BSD-3-Clause License
 */
#ifndef _POINT_CLOUD_H
#define _POINT_CLOUD_H

#include <string>
#include <open3d/Open3D.h>

class PointCloud {

public:
    PointCloud();
    ~PointCloud();

    bool save(std::string const &file_name);
    bool load(std::string const &file_name);

    bool visualize();

private:
    open3d::geometry::PointCloud _point_cloud;

};

#endif // _POINT_CLOUD_H
