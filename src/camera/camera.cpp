/** @file camera.cpp
 *  @brief Contains methods for Camera class.
 *  @copyright BSD-3-Clause License
 */

#include <iostream>
#include "camera.h"

// Declaration of static variables
FlyCapture2::BusManager Camera::_bus_manager;

Camera::Camera()
{

}

Camera::~Camera()
{

}

/**
 * @brief Returns the number of cameras physically connected to the computer.
 * @return Number of cameras available, 0 if an error occures
 */
unsigned int Camera::get_num_available_cameras()
{
    FlyCapture2::Error err;
    unsigned int num_cameras;

    err = _bus_manager.GetNumOfCameras(&num_cameras);
    if(err != FlyCapture2::PGRERROR_OK) {
        err.PrintErrorTrace();
        num_cameras = 0;
    }

    return num_cameras;
}

/**
 * @brief Recovers the FlyCapture2::PGRGuid related to the index. First camera starts at 0 and increment for each available camera.
 * @param [in] index: index of the camera (should not exceed the number of available cameras)
 * @param [out] guid: address of the FlyCapture2::PGRGuid of the camera
 * @return Error code FlyCapture2::Error (0 = No errors)
 */
int Camera::get_guid(unsigned int index, FlyCapture2::PGRGuid *guid)
{
    FlyCapture2::Error err;

    err = _bus_manager.GetCameraFromIndex(index, guid);

    if(err != FlyCapture2::PGRERROR_OK) {
        err.PrintErrorTrace();
    }

    return err.GetType();
}

/**
 * @brief Connect to a camera.
 * @param index of the camera
 * @bug PC (Ubuntu) sometimes freezes when disconnecting/connecting multiple times in quick succession.
 * So far, it hasn't happen after I unplugged/plugged back in the USB before reconnecting.
 * @return Error code FlyCapture2::Error (0 = No errors)
 */
int Camera::connect(FlyCapture2::PGRGuid *guid)
{
    FlyCapture2::Error err;

    // Connect camera
    err = this->Connect(guid);

    // Print error if any
    if(err != FlyCapture2::PGRERROR_OK) {
        err.PrintErrorTrace();
    }

    return err.GetType();
}

/**
 * @brief Disconnect from the camera.
 * @return Error code FlyCapture2::Error (0 = No errors)
 */
int Camera::disconnect()
{
    FlyCapture2::Error err;

    // Stop capture
    if(_is_started) {
        err = StopCapture();

        // Print error if any
        if(err != FlyCapture2::PGRERROR_OK) {
            err.PrintErrorTrace();
        }
    }

    // Reset flags
    _is_capturing = false;
    _is_started = false;

    // Disconnect
    err = Disconnect();

    // Print error if any
    if(err != FlyCapture2::PGRERROR_OK) {
        err.PrintErrorTrace();
    }

    return err.GetType();
}

/**
 * @brief Configure a camera
 * @param See FlyCapture2::Format7ImageSettings, or leave empty (nullptr) for default
 * @param See FlyCapture2::FC2Config, or leave empty (nullptr) for default
 * @return Error code FlyCapture2::Error (0 = No errors)
 */
int Camera::configure(FlyCapture2::Format7ImageSettings *f7_image_settings, FlyCapture2::FC2Config *flycapture_config)
{
    FlyCapture2::Error err;
    FlyCapture2::Format7ImageSettings _f7_image_settings;
    FlyCapture2::FC2Config _flycapture_config;

    // Look if Format7 Image Settings were provided, otherwise initialize with default values
    if(f7_image_settings == nullptr) {
        _f7_image_settings.mode = FlyCapture2::MODE_0;
        _f7_image_settings.height = get_camera_height();
        _f7_image_settings.width = get_camera_width();
        _f7_image_settings.offsetX = 0;
        _f7_image_settings.offsetY = 0;
        _f7_image_settings.pixelFormat = FlyCapture2::PIXEL_FORMAT_RAW8;    // Global shutter = RAW8; Rolling shutter = MONO8
    }
    else
        _f7_image_settings = *f7_image_settings;

    // Send Format7 Image Settings to the camera
    err = SetFormat7Configuration(&_f7_image_settings, (float)100.0);
    if(err != FlyCapture2::PGRERROR_OK) {
        std::cout << "Can't set Format7 Image Settings." << std::endl;
        err.PrintErrorTrace();
        return err.GetType();
    }

    // Look if FlyCapture Configurations were provided, otherwise initialize with default values
    if(flycapture_config == nullptr) {
        _flycapture_config.numBuffers = 1;
        _flycapture_config.numImageNotifications = 1;
        _flycapture_config.grabTimeout = FlyCapture2::TIMEOUT_NONE;
        _flycapture_config.grabMode = FlyCapture2::GrabMode::DROP_FRAMES;
        _flycapture_config.highPerformanceRetrieveBuffer = true;
        _flycapture_config.isochBusSpeed = FlyCapture2::BusSpeed::BUSSPEED_S5000;
        _flycapture_config.asyncBusSpeed = FlyCapture2::BusSpeed::BUSSPEED_ANY;
        _flycapture_config.bandwidthAllocation = FlyCapture2::BandwidthAllocation::BANDWIDTH_ALLOCATION_UNSPECIFIED;
        _flycapture_config.registerTimeoutRetries = 0;
        _flycapture_config.registerTimeout = 0;
    }
    else
        _flycapture_config = *flycapture_config;

    // Send FlyCapture Configurations to the camera
    err = SetConfiguration(&_flycapture_config);
    if(err != FlyCapture2::PGRERROR_OK) {
        std::cout << "Can't set FlyCapture Configurations." << std::endl;
        err.PrintErrorTrace();
    }

    return err.GetType();
}

/**
 * @brief Get the camera's image height in pixels.
 * @return Height of the image generated by the camera.
 */
unsigned int Camera::get_camera_height()
{
    FlyCapture2::Error err;
    FlyCapture2::Format7Info f7_info;
    bool f7_supported = false;

    // Retrieve Format7 info from camera
    err = GetFormat7Info(&f7_info, &f7_supported);
    if(err != FlyCapture2::PGRERROR_OK || !f7_supported) {
        std::cout << "Can't retrieve Format7 info." << std::endl;
        err.PrintErrorTrace();
        return 0;
    }

    // Return height
    return f7_info.maxHeight;
}

/**
 * @brief Get the camera's image width in pixels.
 * @return Width of the image generated by the camera.
 */
unsigned int Camera::get_camera_width()
{
    FlyCapture2::Error err;
    FlyCapture2::Format7Info f7_info;
    bool f7_supported = false;

    // Retrieve Format7 info from camera
    err = GetFormat7Info(&f7_info, &f7_supported);
    if(err != FlyCapture2::PGRERROR_OK || !f7_supported) {
        std::cout << "Can't retrieve Format7 info." << std::endl;
        err.PrintErrorTrace();
        return 0;
    }

    // Return height
    return f7_info.maxWidth;
}

/**
 * @brief Set a FlyCapture property in the camera.
 * @return Error code FlyCapture2::Error (0 = No errors)
 */
int Camera::set_property(FlyCapture2::Property *property)
{
    FlyCapture2::Error err;

    err = SetProperty(property);
    if(err != FlyCapture2::PGRERROR_OK) {
        std::cout << "Error setting property: " << property->type << std::endl;
        err.PrintErrorTrace();
    }

    return err.GetType();
}

/**
 * @brief Get a FlyCapture property from the camera.
 * @param FlyCapture2::PropertyType of the property to retrive
 * @param Pointer to an empty FlyCapture2::Property variable
 * @return Error code FlyCapture2::Error (0 = No errors)
 */
int Camera::get_property(FlyCapture2::PropertyType property_type, FlyCapture2::Property *property)
{
    FlyCapture2::Error err;
    property->type = property_type;

    err = GetProperty(property);
    if(err != FlyCapture2::PGRERROR_OK) {
        std::cout << "Error retrieving property: " << property_type << std::endl;
        err.PrintErrorTrace();
    }

    return err.GetType();
}
