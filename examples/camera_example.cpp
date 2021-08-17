/** @file camera_example.cpp
 *  @brief Example showing how to use the camera api.
 *  @copyright BSD-3-Clause License
 *  @example camera_example.cpp
 */

#include <iostream>
#include <thread>
#include "coro_eyes_sdk.h"

using namespace std;

int main(void)
{
    // Get the number of available cameras
    unsigned int num_cameras = Camera::get_num_available_cameras();
    cout << "Number of available cameras: " << num_cameras << endl;

    // Get the GUIDs for every available camera
    cout << "Getting camera's GUIDs..." << endl;
    FlyCapture2::PGRGuid guid[num_cameras];
    for(unsigned int i=0; i<num_cameras; i++) {
        if(Camera::get_guid(i, &guid[i])) {
            cout << "Can't get GUID of camera " << i << endl;
            return -1;
        }
    }

    // Connect to all available cameras
    cout << "Connecting to all available cameras..." << endl;
    Camera camera[num_cameras];
    for(unsigned int i=0; i<num_cameras; i++) {
        if(camera[i].connect(&guid[i])) {
            cout << "Can't connect to camera " << i << endl;
        }
    }

    // Configure cameras
    cout << "Configuring all connected cameras..." << endl;
    for(unsigned int i=0; i<num_cameras; i++) {
        if(camera[i].configure()) {
            cout << "Can't configure camera " << i << endl;
        }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // Disconnect from all cameras
    cout << "Disconnecting from all cameras" << endl;
    for(unsigned int i=0; i<num_cameras; i++) {
        camera[i].disconnect();
    }
}
