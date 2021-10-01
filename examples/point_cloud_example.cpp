/** @file point_cloud_example.cpp
 *  @brief Example showing how to manipulate a point cloud with Open3D.
 *  @copyright BSD-3-Clause License
 *  @example point_cloud_example.cpp
 */

#include <iostream>
#include "coro_eyes_sdk.h"

using namespace std;

int main(void)
{
    /**
     * @section var Variables declaration
     * @snippet point_cloud_example.cpp Variables
     */
    // [Variables]

    string point_cloud_file = "./point_cloud.ply";  // Path to the point cloud file that will be used

    PointCloud point_cloud;     // Instance of the PointCloud class from CoRo_Eyes_SDK

    // [Variables]


    /**
     * @section load_pt Load point cloud from file
     * @snippet point_cloud_example.cpp Load point cloud
     */
    // [Load point cloud]

    if(!point_cloud.load(point_cloud_file)) {

        cerr << "Couldn't load point cloud from file: " << point_cloud_file << "." << endl;

        cout << "Stopping application..." << endl;

        return -1;
    }

    // [Load point cloud]


    /**
     * @section visualize_pt Visualize point cloud
     * @snippet point_cloud_example.cpp Visualize point cloud
     */
    // [Visualize point cloud]

    if(!point_cloud.visualize()) {

        cerr << "Couldn't visualize loaded point cloud." << endl;

        cout << "Stopping application..." << endl;

        return -1;
    }

    // [Visualize point cloud]


    return 0;
}
