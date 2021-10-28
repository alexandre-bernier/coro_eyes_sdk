/** @file camera_calibration_example.cpp
 *  @brief Example showing how to calibrate a camera api.
 *  @copyright BSD-3-Clause License
 *  @example camera_calibration_example.cpp
 */

#include <iostream>
#include <vector>
#include <ctime>
#include "coro_eyes_sdk.h"

int main(void)
{
    /**
     * @section calib_settings Load the calibration settings from file
     * @snippet camera_calibration_example.cpp Calibration settings
     */
    // [Calibration settings]

    std::cout << "Loading the calibration configuration..." << std::endl;

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

    std::cout << "Number of available cameras: " << num_cameras << std::endl;

    if(num_cameras < 1) {

        std::cerr << "No camera connected. Stopping application..." << std::endl;

        std::cout << "Stopping application..." << std::endl;

        return -1;

    }

    // [Available cameras]


    /**
     * @section guid Get the GUID for every available camera
     * @snippet camera_calibration_example.cpp GUID
     */
    // [GUID]

    std::cout << "Getting camera's GUIDs..." << std::endl;

    FlyCapture2::PGRGuid guid[num_cameras];

    for(unsigned int i_cam=0; i_cam<num_cameras; i_cam++) {

        if(Camera::get_guid(i_cam, &guid[i_cam])) {

            std::cerr << "Can't get GUID of camera " << i_cam << std::endl;

            std::cout << "Stopping application..." << std::endl;

            return -1;
        }
    }

    // [GUID]


    /**
     * @section connect Connect to all available cameras
     * @snippet camera_calibration_example.cpp Connect
     */
    // [Connect]

    std::cout << "Connecting to all available cameras..." << std::endl;

    Camera camera[num_cameras];

    for(unsigned int i_cam=0; i_cam<num_cameras; i_cam++) {

        if(camera[i_cam].connect(&guid[i_cam]) != FlyCapture2::PGRERROR_OK) {

            std::cerr << "Can't connect to camera " << i_cam << std::endl;

            std::cout << "Stopping application..." << std::endl;

            return -1;

        }

        std::cout << "Serial number" << ": " << camera[i_cam].get_serial_number() << std::endl;

    }

    // [Connect]


    /**
     * @section configure Configure cameras
     * @snippet camera_calibration_example.cpp Configure
     */
    // [Configure]

    std::cout << "Configuring all connected cameras..." << std::endl;

    for(unsigned int i_cam=0; i_cam<num_cameras; i_cam++) {

        if(camera[i_cam].configure()) {

            std::cerr << "Can't configure camera " << i_cam << std::endl;

            std::cout << "Stopping application..." << std::endl;

            return -1;

        }

    }

    // [Configure]


    /**
     * @section coro_eyes_properties Set camera properties for CoRo Eyes
     * @snippet camera_calibration_example.cpp CoRo Eyes properties
     */
    // [CoRo Eyes properties]

    std::cout << "Setting up cameras for CoRo Eyes..." << std::endl;

    Camera::CameraPosition camera_position = Camera::CameraPosition::Undefined;

    int camL_index, camR_index;     // Index of the left and right camera

    for(unsigned int i_cam=0; i_cam<num_cameras; i_cam++) {

        switch(camera[i_cam].get_serial_number()) {

        case 19153384:  // Serial number of the left camera

            camera_position = Camera::CameraPosition::Left;

            camL_index = i_cam;

            break;

        case 19305617:  // Serial number of the right camera

            camera_position = Camera::CameraPosition::Right;

            camR_index = i_cam;

            break;

        default:

            std::cerr << "Unrecognized camera (" << camera[i_cam].get_serial_number() << ")." << std::endl;

            break;

        }

        if(camera[i_cam].set_properties_for_coro_eyes(camera_position)) {

            std::cerr << "Can't configure camera " << i_cam << "." << std::endl;

            std::cout << "Stopping application..." << std::endl;

            return -1;

        }

    }

    // [CoRo Eyes properties]


    /**
     * @section other_properties Overwriting some properties for calibration
     * @snippet camera_calibration_example.cpp Other properties
     */
    // [Other properties]

    for(unsigned int i_cam=0; i_cam<num_cameras; i_cam++) {

        camera[i_cam].set_camera_trigger(false);

        camera[i_cam].set_shutter_speed(8.0);

    }

    // [Other properties]


    /**
     * @section start_capture Start capturing with cameras
     * @snippet camera_calibration_example.cpp Start capture
     */
    // [Start capture]

    std::cout << "Starting captures..." << std::endl;

    for(unsigned int i_cam=0; i_cam<num_cameras; i_cam++) {

        camera[i_cam].start_capture();

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

            // Wait for user to press a key
            std::cout << std::endl;

            std::cout << "Press any key to capture the chessboard." << std::endl;

            do {

                cv::Mat image[num_cameras];

                for(unsigned int i_cam=0; i_cam<num_cameras; i_cam++) {

                    // Rescale image
                    float scale_factor = 0.5;
                    cv::Mat rescaled_frame;
                    cv::resize(camera[i_cam].get_last_frame(), image[i_cam], cv::Size(camera[i_cam].get_camera_width()*scale_factor, camera[i_cam].get_camera_height()*scale_factor));

                    imshow("Camera " + std::to_string(i_cam), image[i_cam]);

                }

            } while(cv::waitKey(1) == -1);

            // For every camera...
            std::cout << "Trying to find the chessboard..." << std::endl;

            for(unsigned int i_cam=0; i_cam<num_cameras; i_cam++) {

                // Save last frame
                captured_images[i_cam] = camera[i_cam].get_last_frame();

                // Try to find the chessboard corners
                good_image = Calibration::find_corners(calib_settings, captured_images[i_cam], temp_image_points[i_cam]);

                std::cout << "[Camera " << i_cam << "] Chessboard found: " << good_image << std::endl;

                // If the chessboard can't be found in one of the camera's capture image, drop all of them and retry
                if(!good_image) {

                    break;

                }

            }

        }

        // If images are good for all cameras
        if(good_image) {

            std::cout << "Drawing chessboard corners..." << std::endl;

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

            std::cout << std::endl;

            std::cout << "Calibration image " << i_image+1 << " of " << calib_settings.nrFrames << " acquired." << std::endl;

        }

    }

    std::cout << std::endl;

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
     * @section camera_calibration Camera calibration
     * @snippet camera_calibration_example.cpp Camera calibration
     */
    // [Camera calibration]

    std::cout << "Running camera calibration..." << std::endl;

    cv::Size image_size(camera[0].get_camera_width(), camera[0].get_camera_height());

    Calibration::Data camera_calib_data[num_cameras];

    bool camera_calib_successful[num_cameras];

    // Find the cameras' intrinsic matrix, distorsion coefficients and extrinsic matrices
    bool all_camera_calibrated = true;

    for(unsigned int i_cam=0; i_cam<num_cameras; i_cam++) {

        // Calibration
        camera_calib_successful[i_cam] = Calibration::run_camera_calibration(calib_settings, image_size, camera_calib_data[i_cam], image_points[i_cam]);

        // Display the calibration result
        std::cout << "\nCamera " << i_cam << " calibration\n    " <<
                (camera_calib_successful[i_cam] ? "Calibration succeeded." : "Calibration failed.") <<
                " Avg. re-projection error = " << camera_calib_data[i_cam].total_avg_error << std::endl;

        // Save calibration if the calibration succeeded for all cameras
        all_camera_calibrated &= camera_calib_successful[i_cam];

    }

    std::cout << std::endl;

    // [Camera calibration]


    /**
     * @section save_camera_calib Save the camera calibration
     * @snippet camera_calibration_example.cpp Save camera calibration
     */
    // [Save camera calibration]

    if(all_camera_calibrated) {

        std::cout << "Saving camera calibration to file..." << std::endl;

        std::string calibration_data_file;

        // For all cameras...
        for(unsigned int i_cam=0; i_cam<num_cameras; i_cam++) {

            // Save calibration data
            calibration_data_file = "../resources/calibration/data/camera_" + std::to_string(camera[i_cam].get_serial_number()) + ".xml";

            Calibration::save_camera_calibration(calibration_data_file, calib_settings, image_size, camera_calib_data[i_cam], image_points[i_cam]);

            std::cout << "\tCamera " << i_cam << ": " << calibration_data_file << std::endl;

        }

        std::cout << std::endl;

    }

    // [Save camera calibration]


    /**
     * @section stereo_calibration Stereo calibration
     * @snippet camera_calibration_example.cpp Stereo calibration
     */
    // [Stereo calibration]

    Calibration::StereoData stereo_calib_data;

    bool stereo_calib_successful = false;

    if(all_camera_calibrated) {

        if(num_cameras == 2) {

            std::cout << "Running stereo camera calibration..." << std::endl;

            // Find the cameras' intrinsic matrix, distorsion coefficients and extrinsic matrices
            // Calibration
            stereo_calib_successful = Calibration::run_stereo_calibration(calib_settings, image_size, stereo_calib_data,
                                                                          camera_calib_data[camL_index], image_points[camL_index], camera_calib_data[camR_index], image_points[camR_index]);

            // Display the calibration result
            std::cout << "\nStereo calibration\n    " <<
                    (stereo_calib_successful ? "Calibration succeeded." : "Calibration failed.") <<
                    " Avg. re-projection error = " << stereo_calib_data.total_avg_error << std::endl;

            std::cout << std::endl;

        }

    }

    // [Stereo calibration]


    /**
     * @section save_stereo_calib Save the stereo calibration
     * @snippet camera_calibration_example.cpp Save stereo calibration
     */
    // [Save stereo calibration]

    if(stereo_calib_successful) {

        std::cout << "Saving stereo camera calibration to file..." << std::endl;

        std::string calibration_data_file;

        // Save calibration data
        calibration_data_file = "../resources/calibration/data/stereo_" +
                std::to_string(camera[camL_index].get_serial_number()) + "_" +
                std::to_string(camera[camR_index].get_serial_number()) + ".xml";

        Calibration::save_stereo_calibration(calibration_data_file, calib_settings, image_size, stereo_calib_data, image_points[camL_index], image_points[camR_index]);

        std::cout << "\t" << calibration_data_file << std::endl;

        std::cout << std::endl;

    }

    // [Save stereo calibration]


    /**
     * @section stop_captures Stop all captures
     * @snippet camera_calibration_example.cpp Stop captures
     */
    // [Stop captures]

    std::cout << "Stopping captures..." << std::endl;

    for(unsigned int i_cam=0; i_cam<num_cameras; i_cam++) {

        camera[i_cam].stop_capture();

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

    std::cout << "Disconnecting from all cameras..." << std::endl;

    for(unsigned int i_cam=0; i_cam<num_cameras; i_cam++) {

        camera[i_cam].disconnect();

    }

    // [Disconnect]

	return 0;
}
