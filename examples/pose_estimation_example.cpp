/** @file pose_estimation_example.cpp
 *  @brief Example showing how to find the camera's pose.
 *  @copyright BSD-3-Clause License
 *  @example pose_estimation_example.cpp
 */

#include <iostream>
#include <vector>
#include <ctime>
#include "coro_eyes_sdk.h"

int main(void)
{
    /**
     * @section var Variables declaration
     * @snippet pose_estimation_example.cpp Variables
     */
    // [Variables]

    std::string cam_calib_data_file_path = "../resources/calibration/data/camera_";  // Path to the cameras calibration data files

    std::string stereo_calib_data_file_path = "../resources/calibration/data/stereo_";   // Path to stereo camera calibration data file

    std::string calib_data_file_extension = ".xml";

    // [Variables]


    /**
     * @section calib_settings Load the calibration settings from file
     * @snippet pose_estimation_example.cpp Calibration settings
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
     * @snippet pose_estimation_example.cpp Available cameras
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
     * @snippet pose_estimation_example.cpp GUID
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
     * @snippet pose_estimation_example.cpp Connect
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
     * @snippet pose_estimation_example.cpp Configure
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
     * @snippet pose_estimation_example.cpp CoRo Eyes properties
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
     * @snippet pose_estimation_example.cpp Other properties
     */
    // [Other properties]

    for(unsigned int i_cam=0; i_cam<num_cameras; i_cam++) {

        camera[i_cam].set_camera_trigger(false);

        camera[i_cam].set_shutter_speed(10.0);

    }

    // [Other properties]


    /**
     * @section load_calib_data Load the cameras' calibration data
     * @snippet pose_estimation_example.cpp Load calibration data
     */
    // [Load calibration data]

    Calibration::Data camera_calib_data[num_cameras];

    Calibration::StereoData stereo_calib_data;

    std::string file_name;

    // Load camera calibration data
    for(unsigned int i_cam=0; i_cam<num_cameras; i_cam++) {

        file_name = cam_calib_data_file_path + std::to_string(camera[i_cam].get_serial_number()) + calib_data_file_extension;

        if(!Calibration::load_camera_calibration(file_name, camera_calib_data[i_cam])) {

            std::cerr << "Error loading camera calibration data: " << file_name << "." << std::endl;

            std::cout << "Stopping application..." << std::endl;

            return -1;

        }

    }

    // Load stereo calibration data
    if(camL_index == 0) {

        file_name = stereo_calib_data_file_path + std::to_string(camera[0].get_serial_number()) + "_" +
                std::to_string(camera[1].get_serial_number()) + calib_data_file_extension;

    }

    else {

        file_name = stereo_calib_data_file_path + std::to_string(camera[1].get_serial_number()) + "_" +
                std::to_string(camera[0].get_serial_number()) + calib_data_file_extension;

    }

    if(!Calibration::load_stereo_calibration(file_name, stereo_calib_data)) {

        std::cerr << "Error loading stereo calibration data: " << file_name << "." << std::endl;

        std::cout << "Stopping application..." << std::endl;

        return -1;

    }

    // [Load calibration data]


    /**
     * @section start_capture Start capturing with cameras
     * @snippet pose_estimation_example.cpp Start capture
     */
    // [Start capture]

    std::cout << "Starting capture..." << std::endl;

    for(unsigned int i_cam=0; i_cam<num_cameras; i_cam++) {

        camera[i_cam].start_capture();

    }

    // [Start capture]


    /**
     * @section loop Capture images for camera calibration
     * @snippet pose_estimation_example.cpp Capture images
     */
    // [Capture images]

    int pose_cam_index = camL_index;

    bool good_image = false;

    cv::Mat captured_image;

    std::vector<cv::Point2f> temp_image_points;

    std::vector<cv::Point2f> image_points;

    // OpenCV window name for overlaid chessboard corners
    std::string cv_window_name = "Chessboard corners for camera " + std::to_string(pose_cam_index);

    // Until the captured calibration image is good
    while(!good_image) {

        // Wait for user to press a key
        std::cout << std::endl;

        std::cout << "Press any key to capture the chessboard." << std::endl;

        do {

            // cv::resize crashes if the image is 1x1
            if(camera[pose_cam_index].get_last_frame().rows > 1 && camera[pose_cam_index].get_last_frame().cols > 1) {

                // Rescale image
                float scale_factor = 0.5;
                cv::Mat rescaled_frame;
                cv::resize(camera[pose_cam_index].get_last_frame(), rescaled_frame, cv::Size(0,0), scale_factor, scale_factor);

                imshow(cv_window_name, rescaled_frame);

            }

        } while(cv::waitKey(1) == -1);

        std::cout << "Trying to find the chessboard..." << std::endl;

        // Save last frame
        captured_image = camera[pose_cam_index].get_last_frame();

        // Undistort the captured image
        cv::Mat undistored_captured_image;
        cv::undistort(captured_image, undistored_captured_image, camera_calib_data[pose_cam_index].intrinsic, camera_calib_data[pose_cam_index].distorsion);
        captured_image = undistored_captured_image;

        // Try to find the chessboard corners
        good_image = Calibration::find_corners(calib_settings, captured_image, temp_image_points);

        std::cout << "[Camera " << pose_cam_index << "] Chessboard found: " << good_image << std::endl;

    }

    // If images are good for all cameras
    if(good_image) {

        std::cout << "Drawing chessboard corners..." << std::endl;

        // Select the chessboard corners
        image_points.push_back(temp_image_points[0]);
        image_points.push_back(temp_image_points[calib_settings.boardSize.width-1]);
        image_points.push_back(temp_image_points[(calib_settings.boardSize.width*calib_settings.boardSize.height)-calib_settings.boardSize.width]);
        image_points.push_back(temp_image_points[(calib_settings.boardSize.width*calib_settings.boardSize.height)-1]);

        // Convert the captured image to RGB
        cv::cvtColor(captured_image, captured_image, cv::COLOR_GRAY2RGB);

        // Draw the selected chessboard corners
        cv::Scalar red(0, 0, 255);

        for(unsigned int i_point=0; i_point<image_points.size(); i_point++) {

            cv::circle(captured_image, image_points[i_point], 15, red, 3);

            cv::putText(captured_image, std::to_string(i_point+1), cv::Point(image_points[i_point].x+20, image_points[i_point].y-20), cv::FONT_HERSHEY_DUPLEX, 2, red, 2);

        }

        // Rescale image
        float scale_factor = 0.5;
        cv::Mat rescaled_image;
        cv::resize(captured_image, rescaled_image, cv::Size(0,0), scale_factor, scale_factor);

        // Display the overlaid image in a new window
        cv::imshow(cv_window_name, rescaled_image);

        cv::waitKey(100);

    }

    std::cout << std::endl;

    // [Capture images]


    /**
     * @section get_corner_coord Get corner coordinates
     * @snippet pose_estimation_example.cpp Get corner coordinates
     */
    // [Get corner coordinates]

    std::cout << "This script will find a transform (translation and rotation) from a user-defined reference frame to the CoRo Eyes left camera." << std::endl;
    std::cout << "\t1- Fix a reference frame of your choosing in the camera's environment (it can be anything)." << std::endl;
    std::cout << "\t2- Determine the orientation of that frame (i.e. in which direction does each axis point)." << std::endl;
    std::cout << "\t3- Using that new reference frame axes, measure the distance from that frame to all the chessboard corners identified in the image." << std::endl;

    unsigned int coord_scale = 1;
    std::string coord_scale_units = "m";
    unsigned int coord_scale_resp;
    bool resp_ok = false;

    while(!resp_ok) {
        std::cout << "In which units do you wish to enter the coordinates?" << std::endl;
        std::cout << "\t [1]: Millimeters (mm)" << std::endl;
        std::cout << "\t [2]: Centimeters (cm)" << std::endl;
        std::cout << "\t [3]: Decimeters (dm)" << std::endl;
        std::cout << "\t [4]: Meters (m)" << std::endl;
        std::cout << "--> ";
        std::cin >> coord_scale_resp;
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

        if(coord_scale_resp == 1 || coord_scale_resp == 2 || coord_scale_resp == 3 || coord_scale_resp == 4)
            resp_ok = true;
    }

    switch(coord_scale_resp){
    case 1:
        coord_scale = 1000;
        coord_scale_units = "mm";
        break;
    case 2:
        coord_scale = 100;
        coord_scale_units = "cm";
        break;
    case 3:
        coord_scale = 10;
        coord_scale_units = "dm";
        break;
    case 4:
        coord_scale = 1;
        coord_scale_units = "m";
        break;
    }

    std::vector<cv::Point3f> object_points;
    float corner_x, corner_y, corner_z;

    for(unsigned int i_corner=0; i_corner<image_points.size(); i_corner++) {
        std::cout << std::endl << "Using your user-defined reference frame, what is the distance (" << coord_scale_units << ") between that frame and the corner " << i_corner+1 << std::endl;

        std::cout << "X: ";
        std::cin >> corner_x;
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

        std::cout << "Y: ";
        std::cin >> corner_y;
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

        std::cout << "Z: ";
        std::cin >> corner_z;
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

        object_points.push_back(cv::Point3f(corner_x/coord_scale, corner_y/coord_scale, corner_z/coord_scale));
    }
    std::cout << std::endl;

    object_points.push_back(cv::Point3f(17.6/100, 23.0/100, -1.4/100));
    object_points.push_back(cv::Point3f(43.0/100, 23.0/100, -1.4/100));
    object_points.push_back(cv::Point3f(17.6/100, 4.9/100, -1.4/100));
    object_points.push_back(cv::Point3f(43.0/100, 4.9/100, -1.4/100));

    // [Get corner coordinates]


    /**
     * @section pose_estimation Pose estimation
     * @snippet pose_estimation_example.cpp Pose estimation
     */
    // [Pose estimation]

    std::cout << "Running pose estimation..." << std::endl;

    cv::Size image_size(camera[pose_cam_index].get_camera_width(), camera[pose_cam_index].get_camera_height());

    Calibration::Pose pose_data;

    bool pose_estimation_successful;

    // Pose estimation
    pose_estimation_successful = Calibration::run_pose_estimation(camera_calib_data[pose_cam_index], stereo_calib_data, object_points, image_points,
                                                                  pose_data);

    // Add frame id to pose data
    pose_data.frame = "coro_eyes";

    // [Pose estimation]


    /**
     * @section save_camera_calib Save the camera calibration
     * @snippet pose_estimation_example.cpp Save camera calibration
     */
    // [Save camera calibration]

    if(pose_estimation_successful) {

        std::cout << std::endl << "--- Results ---" << std::endl;

        std::cout << "Translation:" << std::endl;
        std::cout << "\tX = " << pose_data.translation[0] << std::endl;
        std::cout << "\tY = " << pose_data.translation[1] << std::endl;
        std::cout << "\tZ = " << pose_data.translation[2] << std::endl;

        std::cout << "Quaternions:" << std::endl;
        std::cout << "\tX = " << pose_data.quaternions[0] << std::endl;
        std::cout << "\tY = " << pose_data.quaternions[1] << std::endl;
        std::cout << "\tZ = " << pose_data.quaternions[2] << std::endl;
        std::cout << "\tW = " << pose_data.quaternions[3] << std::endl;

        std::cout << std::endl;

        std::cout << "Saving pose estimation result to file..." << std::endl;

        std::string pose_estimation_file;

        // Save calibration data
        pose_estimation_file = "../resources/calibration/data/pose_estimation_" + std::to_string(camera[pose_cam_index].get_serial_number()) + ".xml";

        Calibration::save_pose_estimation(pose_estimation_file, pose_data);

        std::cout << "\tPose estimation: " << pose_estimation_file << std::endl << std::endl;

    }

    // [Save camera calibration]


    /**
     * @section stop_captures Stop all captures
     * @snippet pose_estimation_example.cpp Stop captures
     */
    // [Stop captures]

    std::cout << "Stopping captures..." << std::endl;

    for(unsigned int i_cam=0; i_cam<num_cameras; i_cam++) {

        camera[i_cam].stop_capture();

    }

    // [Stop captures]


    /**
     * @section close_all_windows Close all windows
     * @snippet pose_estimation_example.cpp Close all windows
     */
    // [Close all windows]

    cv::destroyAllWindows();

    // [Close all windows]


    /**
     * @section disconnect Disconnect from all cameras
     * @snippet pose_estimation_example.cpp Disconnect
     */
    // [Disconnect]

    std::cout << "Disconnecting from all cameras..." << std::endl;

    for(unsigned int i_cam=0; i_cam<num_cameras; i_cam++) {

        camera[i_cam].disconnect();

    }

    // [Disconnect]

	return 0;
}
