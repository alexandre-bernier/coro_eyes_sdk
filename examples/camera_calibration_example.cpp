/** @file camera_calibration_example.cpp
 *  @brief Example showing how to calibrate a camera api.
 *  @copyright BSD-3-Clause License
 *  @example camera_calibration_example.cpp
 */

#include <iostream>
#include <vector>
#include <ctime>
#include "coro_eyes_sdk.h"

using namespace std;

/**
 * @brief Structure used as a callback argument.
 */
struct FrameInfo {
    std::string cv_window_name; ///< Name of the OpenCV window where the received frame should be displayed
    unsigned int image_height;  ///< Height of the image in pixels
    unsigned int image_width;   ///< Width of the image in pixels
};

/**
 * @brief Callback to display a new frame in a camera feed.
 * @param frame Pointer to the last frame received in OpenCV format
 * @param callback_data Pointer to a FrameInfo
 */
void camera_feed_callback(cv::Mat *frame, void *callback_data)
{
    // Typecast callback data
    FrameInfo frame_info = *(FrameInfo*)callback_data;

    // Rescale image
    float scale_factor = 0.5;
    cv::Mat rescaled_frame;
    cv::resize(*frame, rescaled_frame, cv::Size(frame_info.image_width*scale_factor, frame_info.image_height*scale_factor));

    // Show the frame in the appropriate window
    cv::imshow(frame_info.cv_window_name, rescaled_frame);
}

int main(void)
{
    /**
     * @section calib_settings Load the calibration settings from file
     * @snippet camera_calibration_example.cpp Calibration settings
     */
    // [Calibration settings]

    cout << "Loading the calibration configuration..." << endl;

    std::string calib_settings_file = "../resources/calibration/camera_calibration_settings.xml";

    cv::FileStorage fs(calib_settings_file, cv::FileStorage::READ);

    if(!fs.isOpened()) {
        std::cout << "Could not open the calibration configuration file: " << calib_settings_file << std::endl;
        return -1;
    }

    Calibration::Settings calib_settings;

    fs["Settings"] >> calib_settings;
    fs.release();

    if(!calib_settings.goodInput) {
        std::cout << "Invalid configuration detected. Stopping application... " << std::endl;
        return -1;
    }

    std::cout << "Configuration loaded." << std::endl;
    std::cout << std::endl;

    // [Calibration settings]


    /**
     * @section avail_cam Get the number of available cameras
     * @snippet camera_calibration_example.cpp Available cameras
     */
    // [Available cameras]

    unsigned int num_cameras = Camera::get_num_available_cameras();

    cout << "Number of available cameras: " << num_cameras << endl;

    if(num_cameras < 1) {

        cerr << "No camera connected. Stopping application..." << endl;

        return -1;

    }

    // [Available cameras]


    /**
     * @section guid Get the GUID for every available camera
     * @snippet camera_calibration_example.cpp GUID
     */
    // [GUID]

    cout << "Getting camera's GUIDs..." << endl;

    FlyCapture2::PGRGuid guid[num_cameras];

    for(unsigned int i=0; i<num_cameras; i++) {

        if(Camera::get_guid(i, &guid[i])) {

            cerr << "Can't get GUID of camera " << i << endl;

            return -1;
        }
    }

    // [GUID]


    /**
     * @section connect Connect to all available cameras
     * @snippet camera_calibration_example.cpp Connect
     */
    // [Connect]

    cout << "Connecting to all available cameras..." << endl;

    Camera camera[num_cameras];

    for(unsigned int i=0; i<num_cameras; i++) {

        if(camera[i].connect(&guid[i]) != FlyCapture2::PGRERROR_OK) {

            cerr << "Can't connect to camera " << i << endl;

        }

        cout << "Serial number" << ": " << camera[i].get_serial_number() << endl;

    }

    // [Connect]


    /**
     * @section configure Configure cameras
     * @snippet camera_calibration_example.cpp Configure
     */
    // [Configure]

    cout << "Configuring all connected cameras..." << endl;

    for(unsigned int i=0; i<num_cameras; i++) {

        if(camera[i].configure()) {

            cerr << "Can't configure camera " << i << endl;

            cout << "Disconnecting from all cameras" << endl;

            for(unsigned int i=0; i<num_cameras; i++) {

                camera[i].disconnect();

            }

            return -1;

        }

    }

    // [Configure]


    /**
     * @section coro_eyes_properties Set camera properties for CoRo Eyes
     * @snippet camera_calibration_example.cpp CoRo Eyes properties
     */
    // [CoRo Eyes properties]

    cout << "Setting up cameras for CoRo Eyes..." << endl;

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

            cerr << "Unrecognized camera (" << camera[i].get_serial_number() << ")." << endl;

            break;

        }

        if(camera[i].set_properties_for_coro_eyes(camera_position)) {

            cerr << "Can't configure camera " << i << "." << endl;

            cout << "Disconnecting from all cameras" << endl;

            for(unsigned int i=0; i<num_cameras; i++) {

                camera[i].disconnect();

            }

            return -1;

        }

    }

    // [CoRo Eyes properties]


    /**
     * @section feeds Setup camera feeds
     * @snippet camera_calibration_example.cpp Camera feeds
     */
    // [Camera feeds]

    cout << "Starting up camera feeds..." << endl;

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
     * @snippet camera_calibration_example.cpp Start capture
     */
    // [Start capture]

    cout << "Starting captures..." << endl;

    for(unsigned int i=0; i<num_cameras; i++) {

        camera[i].start_capture();

    }

    // [Start capture]


    /**
     * @section loop Capture images for camera calibration
     * @snippet camera_calibration_example.cpp Capture images
     */
    // [Capture images]

    bool good_image = false;

    cv::Mat captured_images[num_cameras];

    std::vector<cv::Point2f> temp_image_points[num_cameras];

    std::vector<std::vector<cv::Point2f>> image_points[num_cameras];

    // OpenCV window name for overlaid chessboard corners
    std::string cv_window_name_overlay[num_cameras];

    for(unsigned int i_cam=0; i_cam<num_cameras; i_cam++) {

        cv_window_name_overlay[i_cam] = "Chessboard corners for camera " + std::to_string(i_cam);

    }

    // For every required calibration images...
    for(int i_image=0; i_image<calib_settings.nrFrames; i_image++) {

        good_image = false;

        // Until the captured calibration image is good for all cameras
        while(!good_image) {

            // Wait for user to press 'Space'
            cout << endl;
            cout << "Press Space to capture the chessboard." << endl;

            while(cv::waitKey(1) != 32);

            // For every camera...
            cout << "Trying to find the chessboard..." << endl;

            for(unsigned int i_cam=0; i_cam<num_cameras; i_cam++) {

                // Save last frame
                captured_images[i_cam] = camera[i_cam].get_last_frame()->clone();

                // Try to find the chessboard corners
                good_image = Calibration::find_corners(calib_settings, captured_images[i_cam], temp_image_points[i_cam]);

                cout << "[Camera " << i_cam << "] Chessboard found: " << good_image << endl;

                // If the chessboard can't be found in one of the camera's capture image, drop all of them and retry
                if(!good_image) {

                    break;

                }

            }

        }

        // If images are good for all cameras
        if(good_image) {

            cout << "Drawing chessboard corners..." << endl;

            for(unsigned int i_cam=0; i_cam<num_cameras; i_cam++) {

                // Save the chessboard corners
                image_points[i_cam].push_back(temp_image_points[i_cam]);

                // Convert the captured image to RGB
                cv::cvtColor(captured_images[i_cam], captured_images[i_cam], cv::COLOR_GRAY2RGB);

                // Draw the chessboard corners
                cv::drawChessboardCorners(captured_images[i_cam], calib_settings.boardSize, cv::Mat(temp_image_points[i_cam]), good_image);

                // Rescale image
                float scale_factor = 0.5;
                cv::Mat rescaled_image;
                cv::resize(captured_images[i_cam], rescaled_image, cv::Size(camera[i_cam].get_camera_width()*scale_factor, camera[i_cam].get_camera_height()*scale_factor));

                // Display the overlaid image in a new window
                cv::imshow(cv_window_name_overlay[i_cam], rescaled_image);

            }

        }

    }

    cout << endl;

    // [Capture images]


    /**
     * @section close_corners_windows Close chessboard corners windows
     * @snippet camera_calibration_example.cpp Close chessboard corners windows
     */
    // [Close chessboard corners windows]

    for(unsigned int i_cam=0; i_cam<num_cameras; i_cam++) {

        cv::destroyWindow(cv_window_name_overlay[i_cam]);

    }

    // [Close chessboard corners windows]


    /**
     * @section calibration Camera calibration
     * @snippet camera_calibration_example.cpp Calibration
     */
    // [Calibration]

    cv::Size image_size(camera[0].get_camera_width(), camera[0].get_camera_height());

    Calibration::Data calib_data[num_cameras];

    bool calibration_successful[num_cameras];

    // Find the cameras' intrinsic matrix, distorsion coefficients and extrinsic matrices
    bool save_calibration = true;

    for(unsigned int i_cam=0; i_cam<num_cameras; i_cam++) {

        // Calibration
        calibration_successful[i_cam] = Calibration::run_calibration(calib_settings, image_size, calib_data[i_cam], image_points[i_cam]);

        // Display the calibration result
        cout << "Camera " << i_cam << "\n    " <<
                (calibration_successful[i_cam] ? "Calibration succeeded." : "Calibration failed.") <<
                " Avg. re-projection error = " << calib_data[i_cam].total_avg_error << endl;

        // Save calibration if the calibration succeeded for all cameras
        save_calibration &= calibration_successful[i_cam];

    }

    cout << endl;

    // [Calibration]


    /**
     * @section save_calib Save the calibration
     * @snippet camera_calibration_example.cpp Save calibration
     */
    // [Save calibration]

    if(save_calibration) {

        std::string calibration_data_file;

        // For all cameras...
        for(unsigned int i_cam=0; i_cam<num_cameras; i_cam++) {

            // Save calibration data
            calibration_data_file = "../resources/calibration/data/camera_" + std::to_string(camera[i_cam].get_serial_number()) + ".xml";

            Calibration::save_calibration(calibration_data_file, calib_settings, image_size, calib_data[i_cam], image_points[i_cam]);

        }

    }

    // [Save calibration]


    /**
     * @section wait_user Wait for the user to press a key
     * @snippet camera_calibration_example.cpp Wait for user
     */
    // [Wait for user]

    cout << "Press any key to exit." << endl;

    while(cv::waitKey(1) == -1);

    cout << endl;

    // [Wait for user]


    /**
     * @section stop_captures Stop all captures
     * @snippet camera_calibration_example.cpp Stop captures
     */
    // [Stop captures]

    cout << "Stopping captures..." << endl;

    for(unsigned int i=0; i<num_cameras; i++) {

        camera[i].stop_capture();

    }

    // [Stop captures]


    /**
     * @section close_all_windows Close all windows
     * @snippet camera_calibration_example.cpp Close all windows
     */
    // [Close all windows]

    cv::destroyAllWindows();

    // [Close all windows]


    /**
     * @section disconnect Disconnect from all cameras
     * @snippet camera_calibration_example.cpp Disconnect
     */
    // [Disconnect]

    cout << "Disconnecting from all cameras..." << endl;

    for(unsigned int i=0; i<num_cameras; i++) {

        camera[i].disconnect();

    }

    // [Disconnect]

	return 0;
}
