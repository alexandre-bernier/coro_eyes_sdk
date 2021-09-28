/** @file camera.h
 *  @brief Main header for the camera api.
 *  @copyright BSD-3-Clause License
 */
#ifndef _CAMERA_H
#define _CAMERA_H

#include <stdexcept>
#include <functional>
#include <mutex>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
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
    inline void set_image_buffer(unsigned int nb_images) {_image_buffer_size = nb_images;};

    // Image capture
    void set_new_frame_callback(std::function<void (cv::Mat, void *)> callback, void *callback_data);
    void unregister_new_frame_callback();
    int start_capture();
    int stop_capture();
    inline void start_buffering() {_image_buffer.clear(); _is_buffering = true;};
    inline void stop_buffering() {_is_buffering = false;};
    inline cv::Mat get_last_frame() {const std::lock_guard<std::mutex> lock(_last_frame_mutex); return _last_frame;};
    std::vector<cv::Mat> get_image_buffer_content();

    // Utility
    int set_properties_for_coro_eyes(CameraPosition camera_position);
    inline void set_camera_position(CameraPosition position) {_cam_position = position;}
    inline CameraPosition get_camera_position() {return _cam_position;}
    int set_camera_trigger(bool external);
    int set_shutter_speed(float shutter_speed);

private:
    // Bus manager
    static FlyCapture2::BusManager _bus_manager;

    // Flags
    bool _is_capturing = false;
    bool _is_buffering = false;
    std::mutex _last_frame_mutex;

    // Info
    unsigned int _camera_height = 0;
    unsigned int _camera_width = 0;
    int _get_format7_info(FlyCapture2::Format7Info *f7_info);
    int _get_camera_info(FlyCapture2::CameraInfo *camera_info);

    // Utility
    CameraPosition _cam_position = CameraPosition::Undefined;

    // Image capture
    cv::Mat _last_frame;
    std::vector<cv::Mat> _image_buffer;
    unsigned int _image_buffer_size = 0;
    static void _new_frame_internal_callback_wrapper(FlyCapture2::Image *frame, const void *context);
    void _new_frame_internal_callback(FlyCapture2::Image *frame);

    // External callback
    std::function<void(cv::Mat, void*)> _new_frame_external_callback = nullptr;
    void *_external_callback_data = nullptr;
};

#endif // _CAMERA_H
