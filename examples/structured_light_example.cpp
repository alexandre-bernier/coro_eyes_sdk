/** @file structured_light_example.cpp
 *  @brief Example showing how to generate a point cloud with the CoRo Eyes.
 *  @copyright BSD-3-Clause License
 *  @example structured_light_example.cpp
 */

#include <iostream>
#include <thread>
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
 * @brief Prints errors and warnings if there is any.
 * @param err The dlp::ReturnCode to print
 */
void print_dlp_errors(dlp::ReturnCode err)
{
    unsigned int i;
    if(err.hasErrors()) {
        for(i=0; i<err.GetErrorCount(); i++) {
            cerr << "Error: " << err.GetErrors().at(i) << endl;
        }
    }

    if(err.hasWarnings()) {
        for(i=0; i<err.GetWarningCount(); i++) {
            cout << "Warning: " << err.GetWarnings().at(i) << endl;
        }
    }
}

/**
 * @brief Prints the firmware upload progress.
 * @details Needs to be called in a separate thread before starting the firmware upload.
 * @param projector Pointer to a dlp::LCr4500 projector
 */
void print_firmware_upload_progress(dlp::LCr4500 *projector)
{
    // Write first message
    cout << "Uploading: 0%" << flush;

    // Give time for the firmware upload to start
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    int progress = 0;
    do {
        // Sleep
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        // Print progress
        cout << "\rUploading: " << projector->GetFirmwareUploadPercentComplete() << "% " << flush;
        switch(progress++) {
        case 0:
            cout << "|" << flush;
            break;
        case 1:
            cout << "/" << flush;
            break;
        case 2:
            cout << "—" << flush;
            break;
        case 3:
            cout << "\\" << flush;
            progress = 0;
            break;
        }
    } while(projector->FirmwareUploadInProgress());

    // Upload complete
    cout << "\rUpload done." << endl << flush;
}

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
     * @section var Variables declaration
     * @snippet structured_light_example.cpp Variables
     */
    // [Variables]

    dlp::ReturnCode ret;    // Return variable of all DLP's methods

    dlp::LCr4500 projector; // Instance of the projector (DLP)

    dlp::Parameters param;  // DLP class to hold the projector settings (DLP)

    string proj_param_file = "../resources/dlp_platforms/projector_settings.txt";   // Path to the projector settings file (DLP)

    string cam_calib_data_file_path = "../resources/calibration/data/camera_";  // Path to the cameras calibration data files

    string stereo_calib_data_file_path = "../resources/calibration/data/stereo_";   // Path to stereo camera calibration data file

    string calib_data_file_extension = ".xml";

    // [Variables]


    /**
     * @section proj_connect Connect to the projector
     * @snippet structured_light_example.cpp Connect to projector
     */
    // [Connect to projector]

    cout << "Connecting..." << endl;

    ret = projector.Connect("");

    print_dlp_errors(ret);

    if(ret.hasErrors()) {

        cout << "Stopping application..." << endl;

        return -1;

    }

    // [Connect to projector]


    /**
     * @section load_proj_settings Load the projector settings file
     * @snippet structured_light_example.cpp Load projector settings
     */
    // [Load projector settings]

    cout << "Loading parameters..." << endl;

    ret = param.Load(proj_param_file);

    print_dlp_errors(ret);

    if(ret.hasErrors()) {

        cout << "Stopping application..." << endl;

        return -1;

    }

    // [Load projector settings]


    /**
     * @section setup_proj Setup projector
     * @snippet structured_light_example.cpp Setup projector
     */
    // [Setup projector]

    cout << "Setting up projector..." << endl;

    ret = projector.Setup(param);

    print_dlp_errors(ret);

    if(ret.hasErrors()) {

        cout << "Stopping application..." << endl;

        return -1;

    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // [Setup projector]


    /**
     * @section avail_cam Get the number of available cameras
     * @snippet structured_light_example.cpp Available cameras
     */
    // [Available cameras]

    unsigned int num_cameras = Camera::get_num_available_cameras();

    cout << "Number of available cameras: " << num_cameras << endl;

    if(num_cameras < 1) {

        cerr << "No camera connected." << endl;

        cout << "Stopping application..." << endl;

        return -1;

    }

    else if(num_cameras != 2) {

        cerr << num_cameras << " cameras found. This application requires exactly 2." << endl;

        cout << "Stopping application..." << endl;

        return -1;

    }

    // [Available cameras]


    /**
     * @section guid Get the GUID for every available camera
     * @snippet structured_light_example.cpp GUID
     */
    // [GUID]

    cout << "Getting camera's GUIDs..." << endl;

    FlyCapture2::PGRGuid guid[num_cameras];

    for(unsigned int i_cam=0; i_cam<num_cameras; i_cam++) {

        if(Camera::get_guid(i_cam, &guid[i_cam])) {

            cerr << "Can't get GUID of camera " << i_cam << "." << endl;

            cout << "Stopping application..." << endl;

            return -1;
        }
    }

    // [GUID]


    /**
     * @section connect_cams Connect to all available cameras
     * @snippet structured_light_example.cpp Connect to cameras
     */
    // [Connect to cameras]

    cout << "Connecting to all available cameras..." << endl;

    Camera camera[num_cameras];

    for(unsigned int i=0; i<num_cameras; i++) {

        if(camera[i].connect(&guid[i]) != FlyCapture2::PGRERROR_OK) {

            cerr << "Can't connect to camera " << i << "." << endl;

            cout << "Stopping application..." << endl;

            return -1;

        }

        cout << "Serial number" << ": " << camera[i].get_serial_number() << endl;

    }

    // [Connect to cameras]


    /**
     * @section configure_cams Configure cameras
     * @snippet structured_light_example.cpp Configure cameras
     */
    // [Configure cameras]

    cout << "Configuring all connected cameras..." << endl;

    for(unsigned int i_cam=0; i_cam<num_cameras; i_cam++) {

        if(camera[i_cam].configure()) {

            cerr << "Can't configure camera " << i_cam << endl;

            cout << "Stopping application..." << endl;

            return -1;

        }

    }

    // [Configure cameras]


    /**
     * @section coro_eyes_cams_properties Set camera properties for CoRo Eyes
     * @snippet structured_light_example.cpp CoRo Eyes cameras properties
     */
    // [CoRo Eyes cameras properties]

    cout << "Setting up cameras for CoRo Eyes..." << endl;

    Camera::CameraPosition camera_position = Camera::CameraPosition::Undefined;

    unsigned int camL_index, camR_index;

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

            cerr << "Unrecognized camera (" << camera[i_cam].get_serial_number() << ")." << endl;

            break;

        }

        if(camera[i_cam].set_properties_for_coro_eyes(camera_position)) {

            cerr << "Can't configure camera " << i_cam << "." << endl;

            cout << "Stopping application..." << endl;

            return -1;

        }

    }

    // [CoRo Eyes cameras properties]


    /**
     * @section load_calib_data Load the cameras' calibration data
     * @snippet structured_light_example.cpp Load calibration data
     */
    // [Load calibration data]

    Calibration::Data camera_calib_data[num_cameras];

    Calibration::StereoData stereo_calib_data;

    string file_name;

    // Load camera calibration data
    for(unsigned int i_cam=0; i_cam<num_cameras; i_cam++) {

        file_name = cam_calib_data_file_path + to_string(camera[i_cam].get_serial_number()) + calib_data_file_extension;

        if(!Calibration::load_camera_calibration(file_name, camera_calib_data[i_cam])) {

            cerr << "Error loading camera calibration data: " << file_name << "." << endl;

            cout << "Stopping application..." << endl;

            return -1;

        }

    }

    // Load stereo calibration data
    if(camL_index == 0) {

        file_name = stereo_calib_data_file_path + to_string(camera[0].get_serial_number()) + "_" +
                to_string(camera[1].get_serial_number()) + calib_data_file_extension;

    }

    else {

        file_name = stereo_calib_data_file_path + to_string(camera[1].get_serial_number()) + "_" +
                to_string(camera[0].get_serial_number()) + calib_data_file_extension;

    }

    if(!Calibration::load_stereo_calibration(file_name, stereo_calib_data)) {

        cerr << "Error loading stereo calibration data: " << file_name << "." << endl;

        cout << "Stopping application..." << endl;

        return -1;

    }

    // [Load calibration data]


    /**
     * @section generate Generate structured light patterns
     * See StructuredLightPatterns.
     * @snippet structured_light_example.cpp Generate patterns
     */
    // [Generate patterns]

    cout << "Generating patterns..." << endl;

    unsigned int proj_height;

    projector.GetRows(&proj_height);

    unsigned int proj_width;

    projector.GetColumns(&proj_width);

    StructuredLight structured_light(proj_height, proj_width);

    structured_light.generate_gray_code_patterns();

    // [Generate patterns]


    /**
     * @section upload_proj_param Decide if we upload generated patterns to projector
     * Set parameter telling if we need to upload the patterns to the projector.
     * Only needs to be done once, or whenever the patterns are changed.
     * @snippet structured_light_example.cpp Upload projector parameter
     */
    // [Upload projector parameter]

    bool upload_patterns = false;

    dlp::Parameters upload_patterns_param;

    upload_patterns_param.Set(dlp::DLP_Platform::Parameters::SequencePrepared(!upload_patterns));

    projector.Setup(upload_patterns_param);

    if(upload_patterns)

        cout << "Patterns will be uploaded to the projector..." << endl;

    // [Upload projector parameter]


    /**
     * @section firmware_progress Print firmware upload progress
     * Print firmware upload completion in a seperate thread since the upload takes full control.
     * Can't put the upload in a seperate thread, because the images generated get corrupted if
     * 'PreparePatternSequence' isn't in the main thread.
     * @snippet structured_light_example.cpp Print upload progress
     */
    // [Print upload progress]

    if(upload_patterns) {

        std::thread print_progress_thread(&print_firmware_upload_progress, &projector);

        print_progress_thread.detach();

    }

    // [Print upload progress]


    /**
     * @section prepare Prepare patterns for the projector (DLP)
     * They will be sent to the projector if the dlp::DLP_Platform::Parameters::SequencePrepared parameter is set to false.
     * @snippet structured_light_example.cpp Prepare patterns
     */
    // [Prepare patterns]

    cout << "Preparing patterns..." << endl;

    dlp::Pattern::Sequence dlp_pattern_sequence;

    if(structured_light.get_pattern_type() == StructuredLight::PatternType::GrayCode)

        dlp_pattern_sequence = convert_gray_code_cv_patterns_to_dlp(structured_light.get_pattern_images());

    else if(structured_light.get_pattern_type() == StructuredLight::PatternType::Sinusoidal)

        dlp_pattern_sequence = convert_sinusoidal_cv_patterns_to_dlp(structured_light.get_pattern_images());

    projector.PreparePatternSequence(dlp_pattern_sequence);

    std::this_thread::sleep_for(std::chrono::milliseconds(1500));    // Wait to allow the print_progress_thread to finish properly

    // [Prepare patterns]


    /**
     * @section feeds Setup camera feeds
     * @snippet structured_light_example.cpp Camera feeds
     */
    // [Camera feeds]

    cout << "Starting up camera feeds..." << endl;

    FrameInfo frame_info[num_cameras];

    for(unsigned int i_cam=0; i_cam<num_cameras; i_cam++) {

        frame_info[i_cam].cv_window_name = "Camera " + std::to_string(i_cam);

        frame_info[i_cam].image_height = camera[i_cam].get_camera_height();

        frame_info[i_cam].image_width = camera[i_cam].get_camera_width();

        cv::namedWindow(frame_info[i_cam].cv_window_name, cv::WINDOW_AUTOSIZE);

        camera[i_cam].set_new_frame_callback(camera_feed_callback, &frame_info[i_cam]);

    }

    // [Camera feeds]


    /**
     * @section image_buffers Setup the image buffers
     * @snippet structured_light_example.cpp Image buffers
     */
    // [Image buffers]

    for(unsigned int i_cam=0; i_cam<num_cameras; i_cam++) {

        unsigned int nb_images = structured_light.get_nb_patterns() * 1.2;

        camera[i_cam].set_image_buffer(nb_images);

    }

    // [Image buffers]


    /**
     * @section start_capture Start capturing with cameras
     * @snippet structured_light_example.cpp Start capture
     */
    // [Start capture]

    cout << "Starting captures..." << endl;

    for(unsigned int i_cam=0; i_cam<num_cameras; i_cam++) {

        camera[i_cam].start_capture();

    }

    // [Start capture]


    /**
     * @section white Project white pattern
     * @snippet projector_example.cpp Project white
     */
    // [Project white]

    cout << "Projecting white..." << endl;

    print_dlp_errors(projector.ProjectSolidWhitePattern());

    // [Project white]


    /**
     * @section wait_user Wait for the user to press a key
     * @snippet structured_light_example.cpp Wait for user
     */
    // [Wait for user]

    cv::waitKey();

    // [Wait for user]


    /**
     * @section unregister_callback Unregister the new frame callback
     * @snippet structured_light_example.cpp Unregister callback
     */
    // [Unregister callback]

    for(unsigned int i_cam=0; i_cam<num_cameras; i_cam++) {

        camera[i_cam].unregister_new_frame_callback();

        cv::destroyWindow(frame_info[i_cam].cv_window_name);

    }

    // [Unregister callback]


    /**
     * @section proj_black Project black pattern
     * @details Projecting black to help detect the start of the sequence.
     * @snippet structured_light_example.cpp Project black
     */
    // [Project black]

    cout << "Projecting black..." << endl;

    print_dlp_errors(projector.ProjectSolidBlackPattern());

    // [Project black]


    /**
     * @section start_buffering Start buffering images
     * @snippet structured_light_example.cpp Start buffering
     */
    // [Start buffering]

    cout << "Starting to buffer images..." << endl;

    for(unsigned int i_cam=0; i_cam<num_cameras; i_cam++) {

        camera[i_cam].start_buffering();

    }

    // [Start buffering]


    /**
     * @section project_patterns Start patterns projection and wait for completion
     * @details Make sure the projection is dark before starting the pattern sequence,
     * otherwise we won't be able to detect the start of the sequence.
     * @snippet structured_light_example.cpp Project patterns
     */
    // [Project patterns]

    // Calculate how long it takes to project all patterns
    dlp::DLP_Platform::Parameters::SequencePeriod sequence_period;

    param.Get(&sequence_period);

    unsigned int sequence_duration = sequence_period.Get() * structured_light.get_nb_patterns() * 1.2;

    // Start pattern sequence projection
    cout << "Projecting patterns..." << endl;

    projector.StartPatternSequence(0, structured_light.get_nb_patterns(), false);

    // Wait for completion
    std::this_thread::sleep_for(std::chrono::microseconds(sequence_duration));

    // [Project patterns]


    /**
     * @section stop_buffering Stop buffering images
     * @snippet structured_light_example.cpp Stop buffering
     */
    // [Stop buffering]

    cout << "Stopping to buffer images..." << endl;

    for(unsigned int i_cam=0; i_cam<num_cameras; i_cam++) {

        camera[i_cam].stop_buffering();

    }

    // [Stop buffering]


    /**
     * @section stop Stop projection (turn off the lamp)
     * @snippet structured_light_example.cpp Stop projection
     */
    // [Stop projection]

    cout << "Stopping projection..." << endl;

    print_dlp_errors(projector.ProjectSolidBlackPattern());

    print_dlp_errors(projector.StopPatternSequence());

    // [Stop projection]


    /**
     * @section retrieve_images Retrieve buffered images from cameras and extract pattern images
     * @snippet structured_light_example.cpp Retrieve images
     */
    // [Retrieve images]

    cout << "Retrieving images..." << endl;

    std::vector<std::vector<cv::Mat> > captured_patterns;

    captured_patterns.resize(num_cameras);

    for(unsigned int i_cam=0; i_cam<num_cameras; i_cam++) {

        if(!structured_light.extract_pattern_images(camera[i_cam].get_image_buffer_content(), captured_patterns[i_cam])) {

            cerr << "Couldn't find all the patterns in the captured images." << endl;

            cout << "Stopping application..." << endl;

            return -1;

        }
    }

    // [Retrieve images]


    /**
     * @section remap_images Remap captured patterns using camera calibration
     * @snippet structured_light_example.cpp Remap images
     */
    // [Remap images]

    cout << "Remapping captured patterns..." << endl;

    // Calculate the reprojection maps
    Calibration::ReprojMaps reprojection_maps[num_cameras];

    cv::Size image_size(camera[0].get_camera_width(), camera[0].get_camera_height());

    Calibration::calculate_stereo_reproj_maps(camera_calib_data[camL_index], camera_calib_data[camR_index], stereo_calib_data,
                                              image_size, reprojection_maps[camL_index], reprojection_maps[camR_index]);

    // Remap images (remapped_images[0] => Right camera; remapped_images[1] => Left camera)
    std::vector<std::vector<cv::Mat> > remapped_images(num_cameras);

    if(!Calibration::remap_images(captured_patterns[camR_index], reprojection_maps[camR_index], remapped_images[0])) {

        cerr << "Error while remapping the captured patterns of the right camera." << endl;

        cout << "Stopping application..." << endl;

        return -1;

    }

    if(!Calibration::remap_images(captured_patterns[camL_index], reprojection_maps[camL_index], remapped_images[1])) {

        cerr << "Error while remapping the captured patterns of the left camera." << endl;

        cout << "Stopping application..." << endl;

        return -1;

    }

    // Show remapped images
//    for(unsigned int i_image=0; i_image<remapped_images[0].size(); i_image++) {

//        cv::Mat rescaled_image;

//        float scale_factor = 0.5;

//        cv::resize(remapped_images[1][i_image], rescaled_image,
//                   cv::Size(remapped_images[1][i_image].size().width*scale_factor, remapped_images[1][i_image].size().height*scale_factor));

//        cv::imshow("Remapped image (left)", rescaled_image);

//        cv::resize(remapped_images[0][i_image], rescaled_image,
//                   cv::Size(remapped_images[0][i_image].size().width*scale_factor, remapped_images[0][i_image].size().height*scale_factor));

//        cv::imshow("Remapped image (right)", rescaled_image);

//        cv::waitKey();

//    }

//        cv::destroyWindow("Remapped image (left)");

//        cv::destroyWindow("Remapped image (right)");

    // [Remap images]


    /**
     * @section compute_disparity Compute the disparity map
     * @details Make sure the first remapped_images' vector has the left camera's images.
     * @snippet structured_light_example.cpp Compute disparity map
     */
    // [Compute disparity map]

    cout << "Computing the disparity map..." << endl;

    cv::Mat disparity_map;

    if(!structured_light.compute_disparity_map(remapped_images, disparity_map)) {

        cerr << "Error while computing the disparity map." << endl;

        cout << "Stopping application..." << endl;

        return -1;
    }

    // [Compute disparity map]


    /**
     * @section color_disparity Color the disparity map
     * @snippet structured_light_example.cpp Color disparity map
     */
    // [Color disparity map]

    cout << "Applying colors to the disparity map..." << endl;

    cv::Mat colored_disparity_map = structured_light.apply_color_to_disparity_map(disparity_map);

    float scale_factor = 0.5;

    cv::Mat rescaled_colored_disparity_map;

    cv::resize(colored_disparity_map, rescaled_colored_disparity_map, cv::Size(colored_disparity_map.size().width*scale_factor, colored_disparity_map.size().height*scale_factor));

    cv::imshow("Disparity map (colored)", rescaled_colored_disparity_map);

    cv::waitKey();

    // [Color disparity map]


    /**
     * @section compute_point_cloud Compute the point cloud
     * @snippet structured_light_example.cpp Compute point cloud
     */
    // [Compute point cloud]

    cout << "Computing point cloud..." << endl;

    std::vector<cv::Point3f> point_cloud = structured_light.compute_point_cloud(disparity_map, stereo_calib_data.Q);

    // [Compute point cloud]


    /**
     * @section save_point_cloud Save the point cloud
     * @snippet structured_light_example.cpp Save point cloud
     */
    // [Save point cloud]

    cout << "Saving point cloud..." << endl;

    string point_cloud_file_name = "./point_cloud.ply";

    cv::viz::writeCloud(point_cloud_file_name, point_cloud);

    // [Save point cloud]


    /**
     * @section stop_captures Stop all captures
     * @snippet structured_light_example.cpp Stop captures
     */
    // [Stop captures]

    cout << "Stopping captures..." << endl;

    for(unsigned int i=0; i<num_cameras; i++) {

        camera[i].stop_capture();

    }

    // [Stop captures]


    /**
     * @section close_windows Close all windows
     * @snippet structured_light_example.cpp Close windows
     */
    // [Close windows]

    cv::destroyAllWindows();

    // [Close windows]


    /**
     * @section disconnect Disconnect from all devices
     * @snippet structured_light_example.cpp Disconnect
     */
    // [Disconnect]

    cout << "Disconnecting from all devices..." << endl;

    for(unsigned int i=0; i<num_cameras; i++) {

        camera[i].disconnect();

    }

    print_dlp_errors(projector.Disconnect());

    // [Disconnect]


    return 0;
}