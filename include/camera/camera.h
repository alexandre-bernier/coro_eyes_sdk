/** @file camera.h
 *  @brief Main header for the camera api.
 *  @copyright BSD-3-Clause License
 */
#ifndef _CAMERA_H
#define _CAMERA_H

#include <stdexcept>
#include <functional>
#include <opencv2/core.hpp>
#include <flycapture/FlyCapture2.h>

class Camera : private FlyCapture2::Camera
{
public:
    enum CameraPosition {
        Undefined = 0,
        Left,
        Right
    };

    enum ExceptionHandling : bool {
        NoExceptions = false,
        ThrowExceptions = true
    };

    Camera();
    ~Camera();

    // Connection
    static unsigned int get_num_available_cameras();
    static int get_guid(unsigned int index, FlyCapture2::PGRGuid *guid);
    int connect(FlyCapture2::PGRGuid *guid);
    int disconnect();
    inline bool is_connected() {return IsConnected();}

    // Info
    unsigned int get_serial_number();
    unsigned int get_camera_height();
    unsigned int get_camera_width();

    // Configuration
    int configure(FlyCapture2::Format7ImageSettings *f7_image_settings = nullptr, FlyCapture2::FC2Config *flycapture_config = nullptr);
    int set_property(FlyCapture2::Property *property, ExceptionHandling throw_exception = ExceptionHandling::NoExceptions);
    int get_property(FlyCapture2::PropertyType property_type, FlyCapture2::Property *property, ExceptionHandling throw_exception = ExceptionHandling::NoExceptions);
    int set_trigger_mode(FlyCapture2::TriggerMode *trigger_mode, ExceptionHandling throw_exception = ExceptionHandling::NoExceptions);
    int get_trigger_mode(FlyCapture2::TriggerMode *trigger_mode, ExceptionHandling throw_exception = ExceptionHandling::NoExceptions);
    int set_strobe_control(FlyCapture2::StrobeControl *strobe_control, ExceptionHandling throw_exception = ExceptionHandling::NoExceptions);
    int get_strobe_control(FlyCapture2::StrobeControl *strobe_control, ExceptionHandling throw_exception = ExceptionHandling::NoExceptions);

    // Image capture
    void set_new_frame_callback(std::function<void (cv::Mat*, void*)> callback, void *callback_data);
    int start_capture();
    int stop_capture();
    cv::Mat *get_last_frame() {return &_last_frame;};

    // Utility
    int set_properties_for_coro_eyes(CameraPosition camera_position);

private:
    // Bus manager
    static FlyCapture2::BusManager _bus_manager;

    // Flags
    bool _is_capturing = false;

    // Info
    unsigned int _camera_height = 0;
    unsigned int _camera_width = 0;
    int _get_format7_info(FlyCapture2::Format7Info *f7_info);
    int _get_camera_info(FlyCapture2::CameraInfo *camera_info);

    // Image capture
    cv::Mat _last_frame;
    static void _new_frame_internal_callback_wrapper(FlyCapture2::Image *frame, const void *context);
    void _new_frame_internal_callback(FlyCapture2::Image *frame);

    // External callback
    std::function<void(cv::Mat*, void*)> _new_frame_external_callback = nullptr;
    void *_external_callback_data = nullptr;
};

#endif // _CAMERA_H
