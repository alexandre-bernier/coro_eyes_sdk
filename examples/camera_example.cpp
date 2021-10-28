/** @file camera_example.cpp
 *  @brief Example showing how to use the camera api.
 *  @copyright BSD-3-Clause License
 *  @example camera_example.cpp
 */

#include <iostream>
#include "coro_eyes_sdk.h"

/**
 * @brief Structure used as a callback argument.
 */
struct FrameInfo {
    std::string cv_window_name; ///< Name of the OpenCV window where the received frame should be displayed
    unsigned int image_height;  ///< Height of the image in pixels
    unsigned int image_width;   ///< Width of the image in pixels
};

/**
 * @brief Callback for every new frame received.
 * @param frame Pointer to the last frame received in OpenCV format
 * @param callback_data Pointer to a FrameInfo
 */
void camera_feed_callback(cv::Mat frame, void *callback_data)
{
    // Typecast callback data
    FrameInfo frame_info = *(FrameInfo*)callback_data;

    // Rescale image
    float scale_factor = 0.5;
    cv::Mat rescaled_frame;
    cv::resize(frame, rescaled_frame, cv::Size(frame_info.image_width*scale_factor, frame_info.image_height*scale_factor));

    // Show the frame in the appropriate window
    cv::imshow(frame_info.cv_window_name, rescaled_frame);
}

int main(void)
{
    /**
     * @section avail_cam Get the number of available cameras
     * @snippet camera_example.cpp Available cameras
     */
    // [Available cameras]

    unsigned int num_cameras = Camera::get_num_available_cameras();

    std::cout << "Number of available cameras: " << num_cameras << std::endl;

    if(num_cameras < 1) {

        std::cerr << "No camera connected. Stopping application..." << std::endl;

        return -1;

    }

    // [Available cameras]


    /**
     * @section guid Get the GUID for every available camera
     * @snippet camera_example.cpp GUID
     */
    // [GUID]

    std::cout << "Getting camera's GUIDs..." << std::endl;

    FlyCapture2::PGRGuid guid[num_cameras];

    for(unsigned int i=0; i<num_cameras; i++) {

        if(Camera::get_guid(i, &guid[i])) {

            std::cerr << "Can't get GUID of camera " << i << std::endl;

            return -1;
        }
    }

    // [GUID]


    /**
     * @section connect Connect to all available cameras
     * @snippet camera_example.cpp Connect
     */
    // [Connect]

    std::cout << "Connecting to all available cameras..." << std::endl;

    Camera camera[num_cameras];

    for(unsigned int i=0; i<num_cameras; i++) {

        if(camera[i].connect(&guid[i]) != FlyCapture2::PGRERROR_OK) {

            std::cerr << "Can't connect to camera " << i << std::endl;

        }

        std::cout << "Serial number" << ": " << camera[i].get_serial_number() << std::endl;

    }

    // [Connect]


    /**
     * @section configure Configure cameras
     * @snippet camera_example.cpp Configure
     */
    // [Configure]

    std::cout << "Configuring all connected cameras..." << std::endl;

    for(unsigned int i=0; i<num_cameras; i++) {

        if(camera[i].configure()) {

            std::cerr << "Can't configure camera " << i << std::endl;

            std::cout << "Disconnecting from all cameras" << std::endl;

            for(unsigned int i=0; i<num_cameras; i++) {

                camera[i].disconnect();

            }

            return -1;

        }

    }

    // [Configure]


    /**
     * @section coro_eyes_properties Set camera properties for CoRo Eyes
     * @snippet camera_example.cpp CoRo Eyes properties
     */
    // [CoRo Eyes properties]

    std::cout << "Setting up cameras for CoRo Eyes..." << std::endl;

    Camera::CameraPosition camera_position = Camera::CameraPosition::Undefined;

    for(unsigned int i=0; i<num_cameras; i++) {

        switch(camera[i].get_serial_number()) {

        case 19153384:  // Serial number of the left camera

            camera_position = Camera::CameraPosition::Left;

            break;

        case 19305617:  // Serial number of the right camera

            camera_position = Camera::CameraPosition::Right;

            break;

        default:

            std::cerr << "Unrecognized camera (" << camera[i].get_serial_number() << ")." << std::endl;

            break;

        }

        if(camera[i].set_properties_for_coro_eyes(camera_position)) {

            std::cerr << "Can't configure camera " << i << "." << std::endl;

            std::cout << "Disconnecting from all cameras" << std::endl;

            for(unsigned int i=0; i<num_cameras; i++) {

                camera[i].disconnect();

            }

            return -1;

        }

    }

    // [CoRo Eyes properties]


    /**
     * @section other_properties Overwriting some properties to make it work without the projector
     * @snippet camera_example.cpp Other properties
     */
    // [Other properties]

    for(unsigned int i_cam=0; i_cam<num_cameras; i_cam++) {

        camera[i_cam].set_camera_trigger(false);

        camera[i_cam].set_shutter_speed(8.0);

    }

    // [Other properties]


    /**
     * @section feeds Setup camera feeds
     * @snippet camera_example.cpp Camera feeds
     */
    // [Camera feeds]

    std::cout << "Starting up camera feeds..." << std::endl;

    FrameInfo frame_info[num_cameras];

    for(unsigned int i=0; i<num_cameras; i++) {

        frame_info[i].cv_window_name = "Camera " + std::to_string(i);

        frame_info[i].image_height = camera[i].get_camera_height();

        frame_info[i].image_width = camera[i].get_camera_width();

        cv::namedWindow(frame_info[i].cv_window_name, cv::WINDOW_AUTOSIZE);

        camera[i].set_new_frame_callback(camera_feed_callback, &frame_info[i]);

    }

    // [Camera feeds]


    /**
     * @section start_capture Start capturing with cameras
     * @snippet camera_example.cpp Start capture
     */
    // [Start capture]

    std::cout << "Starting captures..." << std::endl;

    for(unsigned int i=0; i<num_cameras; i++) {

        camera[i].start_capture();

    }

    // [Start capture]


    /**
     * @section wait_user Wait for the user to press a key
     * @snippet camera_example.cpp Wait for user
     */
    // [Wait for user]

    cv::waitKey();

    // [Wait for user]


    /**
     * @section close_windows Close all windows
     * @snippet camera_example.cpp Close windows
     */
    // [Close windows]

    cv::destroyAllWindows();

    // [Close windows]


    /**
     * @section stop_captures Stop all captures
     * @snippet camera_example.cpp Stop captures
     */
    // [Stop captures]

    std::cout << "Stopping captures..." << std::endl;

    for(unsigned int i=0; i<num_cameras; i++) {

        camera[i].stop_capture();

    }

    // [Stop captures]


    /**
     * @section disconnect Disconnect from all cameras
     * @snippet camera_example.cpp Disconnect
     */
    // [Disconnect]

    std::cout << "Disconnecting from all cameras..." << std::endl;

    for(unsigned int i=0; i<num_cameras; i++) {

        camera[i].disconnect();

    }

    // [Disconnect]
}
