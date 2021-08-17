/** @file camera.h
 *  @brief Main header for the camera api.
 *  @copyright BSD-3-Clause License
 */
#ifndef _CAMERA_H
#define _CAMERA_H

#include "flycapture/FlyCapture2.h"

class Camera : private FlyCapture2::Camera
{
public:
    enum TriggerSource : bool {
            Slave = false,
            Master = true
        };

    Camera();
    ~Camera();

    // Connection
    static unsigned int get_num_available_cameras();
    static int get_guid(unsigned int index, FlyCapture2::PGRGuid *guid);
    int connect(FlyCapture2::PGRGuid *guid);
    int disconnect();
    inline bool is_connected() {return IsConnected();}
    inline bool is_configured() {return _is_configured;}

    // Configuration
    int configure(FlyCapture2::Format7ImageSettings *f7_image_settings = nullptr, FlyCapture2::FC2Config *flycapture_config = nullptr);
    unsigned int get_camera_height();
    unsigned int get_camera_width();
    int set_property(FlyCapture2::Property *property);
    int get_property(FlyCapture2::PropertyType property_type, FlyCapture2::Property *property);

private:
    // Bus manager
    static FlyCapture2::BusManager _bus_manager;

    // Flags
    bool _is_configured = false;
    bool _is_started = false;
    bool _is_capturing = false;
};

#endif // _CAMERA_H
