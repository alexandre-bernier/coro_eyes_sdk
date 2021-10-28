/** @file projector_example.cpp
 *  @brief Example showing how to use the projector api.
 *  @copyright BSD-3-Clause License
 *  @example projector_example.cpp
 */

#include <iostream>
#include <thread>
#include <chrono>
#include "coro_eyes_sdk.h"

/**
 * @brief Prints errors and warnings if there is any.
 * @param err The dlp::ReturnCode to print
 */
void print_dlp_errors(dlp::ReturnCode err)
{
    unsigned int i;
    if(err.hasErrors()) {
        for(i=0; i<err.GetErrorCount(); i++) {
            std::cerr << "Error: " << err.GetErrors().at(i) << std::endl;
        }
    }

    if(err.hasWarnings()) {
        for(i=0; i<err.GetWarningCount(); i++) {
            std::cout << "Warning: " << err.GetWarnings().at(i) << std::endl;
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
    std::cout << "Uploading: 0%" << std::flush;

    // Give time for the firmware upload to start
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    int progress = 0;
    do {
        // Sleep
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        // Print progress
        std::cout << "\rUploading: " << projector->GetFirmwareUploadPercentComplete() << "% " << std::flush;
        switch(progress++) {
        case 0:
            std::cout << "|" << std::flush;
            break;
        case 1:
            std::cout << "/" << std::flush;
            break;
        case 2:
            std::cout << "—" << std::flush;
            break;
        case 3:
            std::cout << "\\" << std::flush;
            progress = 0;
            break;
        }
    } while(projector->FirmwareUploadInProgress());

    // Upload complete
    std::cout << "\rUpload done." << std::endl << std::flush;
}

int main(void)
{
    /**
     * @section var Variables declaration
     * @snippet projector_example.cpp Variables
     */
    // [Variables]

    dlp::ReturnCode ret;    // Return variable of all DLP's methods

    dlp::LCr4500 projector; // Instance of the projector (DLP)

    dlp::Parameters param;  // DLP class to hold the projector settings (DLP)

    std::string proj_param_file = "../resources/dlp_platforms/projector_settings.txt";   // Path to the projector settings file (DLP)

    // [Variables]


    /**
     * @section connection Connect to the projector
     * @snippet projector_example.cpp Connection
     */
    // [Connection]

    std::cout << "Connecting..." << std::endl;

    ret = projector.Connect("");

    print_dlp_errors(ret);

    if(ret.hasErrors()) {

        std::cout << "Stopping application..." << std::endl;

        return -1;

    }

    // [Connection]


    /**
     * @section load Load the projector settings file
     * @snippet projector_example.cpp Load settings
     */
    // [Load settings]

    std::cout << "Loading parameters..." << std::endl;

    ret = param.Load(proj_param_file);

    print_dlp_errors(ret);

    if(ret.hasErrors()) {

        std::cout << "Stopping application..." << std::endl;

        return -1;

    }

    // [Load settings]


    /**
     * @section setup Setup projector
     * @snippet projector_example.cpp Setup
     */
    // [Setup]

    std::cout << "Setting up projector..." << std::endl;

    ret = projector.Setup(param);

    print_dlp_errors(ret);

    if(ret.hasErrors()) {

        std::cout << "Stopping application..." << std::endl;

        return -1;

    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // [Setup]


    /**
     * @section white Project white pattern
     * @snippet projector_example.cpp Project white
     */
    // [Project white]

    std::cout << "Projecting white..." << std::endl;

    print_dlp_errors(projector.ProjectSolidWhitePattern());

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // [Project white]


    /**
     * @section black Project black pattern
     * @snippet projector_example.cpp Project black
     */
    // [Project black]

    std::cout << "Projecting black..." << std::endl;

    print_dlp_errors(projector.ProjectSolidBlackPattern());

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // [Project black]


    /**
     * @section stop Stop projection (turn off the lamp)
     * @snippet projector_example.cpp Stop projection
     */
    // [Stop projection]

    std::cout << "Stopping projection..." << std::endl;

    print_dlp_errors(projector.StopPatternSequence());

    // [Stop projection]


    /**
     * @section generate Generate structured light patterns
     * See StructuredLightPatterns.
     * @snippet projector_example.cpp Generate patterns
     */
    // [Generate patterns]

    std::cout << "Generating patterns..." << std::endl;

    unsigned int proj_height;

    projector.GetRows(&proj_height);

    unsigned int proj_width;

    projector.GetColumns(&proj_width);

    StructuredLight structured_light(proj_height, proj_width);

    structured_light.generate_gray_code_patterns();

    // [Generate patterns]


    /**
     * @section visualize Visualize generated patterns
     * Press any key to see the next image.
     * @snippet projector_example.cpp Visualize patterns
     */
    // [Visualize patterns]

    structured_light.visualize_patterns();

    // [Visualize patterns]


    /**
     * @section param Decide if we upload generated patterns to projector
     * Set parameter telling if we need to upload the patterns to the projector.
     * Only needs to be done once, or whenever the patterns are changed.
     * @snippet projector_example.cpp Upload parameter
     */
    // [Upload parameter]

    bool upload_patterns = false;

    dlp::Parameters upload_patterns_param;

    upload_patterns_param.Set(dlp::DLP_Platform::Parameters::SequencePrepared(!upload_patterns));

    projector.Setup(upload_patterns_param);

    if(upload_patterns)

        std::cout << "Patterns will be uploaded to the projector..." << std::endl;

    // [Upload parameter]


    /**
     * @section firmware_progress Print firmware upload progress
     * Print firmware upload completion in a seperate thread since the upload takes full control.
     * Can't put the upload in a seperate thread, because the images generated get corrupted if
     * 'PreparePatternSequence' isn't in the main thread.
     * @snippet projector_example.cpp Print upload progress
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
     * @snippet projector_example.cpp Prepare patterns
     */
    // [Prepare patterns]

    std::cout << "Preparing patterns..." << std::endl;

    dlp::Pattern::Sequence dlp_pattern_sequence;

    if(structured_light.get_pattern_type() == StructuredLight::PatternType::GrayCode)

        dlp_pattern_sequence = convert_gray_code_cv_patterns_to_dlp(structured_light.get_pattern_images());

    else if(structured_light.get_pattern_type() == StructuredLight::PatternType::Sinusoidal)

        dlp_pattern_sequence = convert_sinusoidal_cv_patterns_to_dlp(structured_light.get_pattern_images());

    projector.PreparePatternSequence(dlp_pattern_sequence);

    std::this_thread::sleep_for(std::chrono::milliseconds(1500));    // Wait to allow the print_progress_thread to finish properly

    // [Prepare patterns]


    /**
     * @section project_patterns Start patterns projection
     * @snippet projector_example.cpp Project patterns
     */
    // [Project patterns]

    std::cout << "Projecting patterns..." << std::endl;

    projector.StartPatternSequence(0, structured_light.get_nb_patterns(), false);

    // [Project patterns]


    /**
     * @section wait Wait for projection to complete
     * @snippet projector_example.cpp Wait for projection
     */
    // [Wait for projection]

    dlp::DLP_Platform::Parameters::SequencePeriod sequence_period;

    param.Get(&sequence_period);

    unsigned int sequence_duration = sequence_period.Get() * structured_light.get_nb_patterns();

    std::this_thread::sleep_for(std::chrono::microseconds((unsigned int)((float)sequence_duration*1.2)));

    // [Wait for projection]


    /**
     * @section stop2 Stop projection (turn off the lamp)
     * @snippet projector_example.cpp Stop projection 2
     */
    // [Stop projection 2]

    std::cout << "Stopping projection..." << std::endl;

    print_dlp_errors(projector.ProjectSolidBlackPattern());

    print_dlp_errors(projector.StopPatternSequence());

    // [Stop projection 2]


    /**
     * @section disconnect Disconnect from the projector
     * @snippet projector_example.cpp Disconnect
     */
    // [Disconnect]

    std::cout << "Disconnecting..." << std::endl;

    print_dlp_errors(projector.Disconnect());

    // [Disconnect]

    return 0;
}
