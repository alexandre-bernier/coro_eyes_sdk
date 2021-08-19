/** @file projector_example.cpp
 *  @brief Example showing how to use the projector api.
 *  @copyright BSD-3-Clause License
 *  @example projector_example.cpp
 */

#include <iostream>
#include <thread>
#include <chrono>
#include "coro_eyes_sdk.h"

using namespace std;

/**
 * @brief Prints errors and warnings if there is any.
 * @param err The dlp::ReturnCode to print
 */
void print_dlp_errors(dlp::ReturnCode err)
{
    unsigned int i;
    if(err.hasErrors()) {
        for(i=0; i<err.GetErrorCount(); i++) {
            cout << "Error: " << err.GetErrors().at(i) << endl;
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
    string proj_param_file = "../resources/dlp_platforms/projector_settings.txt";   // Path to the projector settings file (DLP)
    // [Variables]

    /**
     * @section connection Connect to the projector
     * @snippet projector_example.cpp Connection
     */
    // [Connection]
    cout << "Connecting..." << endl;
    ret = projector.Connect("");
    print_dlp_errors(ret);
    if(ret.hasErrors()) {
        cout << "Aborting..." << endl;
        return -1;
    }
    // [Connection]

    /**
     * @section load Load the projector settings file
     * @snippet projector_example.cpp Load settings
     */
    // [Load settings]
    cout << "Loading parameters..." << endl;
    ret = param.Load(proj_param_file);
    print_dlp_errors(ret);
    if(ret.hasErrors()) {
        cout << "Aborting..." << endl;
        return -1;
    }
    // [Load settings]

    /**
     * @section setup Setup projector
     * @snippet projector_example.cpp Setup
     */
    // [Setup]
    cout << "Setting up projector..." << endl;
    ret = projector.Setup(param);
    print_dlp_errors(ret);
    if(ret.hasErrors()) {
        cout << "Aborting..." << endl;
        return -1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // [Setup]

    /**
     * @section white Project white pattern
     * @snippet projector_example.cpp Project white
     */
    // [Project white]
    cout << "Projecting white..." << endl;
    print_dlp_errors(projector.ProjectSolidWhitePattern());
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    // [Project white]

    /**
     * @section black Project black pattern
     * @snippet projector_example.cpp Project black
     */
    // [Project black]
    cout << "Projecting black..." << endl;
    print_dlp_errors(projector.ProjectSolidBlackPattern());
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    // [Project black]

    /**
     * @section stop Stop projection (turn off the lamp)
     * @snippet projector_example.cpp Stop projection
     */
    // [Stop projection]
    cout << "Stopping projection..." << endl;
    print_dlp_errors(projector.StopPatternSequence());
    // [Stop projection]

    /**
     * @section generate Generate structured light patterns
     * See StructuredLightPatterns.
     * @snippet projector_example.cpp Generate patterns
     */
    // [Generate patterns]
    cout << "Generating patterns..." << endl;
    unsigned int proj_width;
    projector.GetColumns(&proj_width);
    unsigned int proj_height;
    projector.GetRows(&proj_height);
    StructuredLightPatterns patterns(proj_width, proj_height);
    patterns.generate_gray_code_patterns();
    // [Generate patterns]

    /**
     * @section visualize Visualize generated patterns
     * Press any key to see the next image.
     * @snippet projector_example.cpp Visualize patterns
     */
    // [Visualize patterns]
    patterns.visualize_patterns();
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
    cout << "Sending patterns to projector..." << endl;
    projector.PreparePatternSequence(*patterns.get_dlp_patterns());
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));    // Wait to allow the print_progress_thread to finish properly
    // [Prepare patterns]

    /**
     * @section project_patterns Start patterns projection
     * @snippet projector_example.cpp Project patterns
     */
    // [Project patterns]
    cout << "Projecting patterns..." << endl;
    projector.StartPatternSequence(0, patterns.get_nb_patterns(), false);
    // [Project patterns]

    /**
     * @section wait Wait for projection to complete
     * @snippet projector_example.cpp Wait for projection
     */
    // [Wait for projection]
    dlp::DLP_Platform::Parameters::SequencePeriod sequence_period;
    param.Get(&sequence_period);
    unsigned int sequence_duration = sequence_period.Get() * patterns.get_nb_patterns();
    std::this_thread::sleep_for(std::chrono::microseconds((unsigned int)((float)sequence_duration*1.2)));
    // [Wait for projection]

    /**
     * @section stop2 Stop projection (turn off the lamp)
     * @snippet projector_example.cpp Stop projection 2
     */
    // [Stop projection 2]
    cout << "Stopping projection..." << endl;
    print_dlp_errors(projector.ProjectSolidBlackPattern());
    print_dlp_errors(projector.StopPatternSequence());
    // [Stop projection 2]

    /**
     * @section disconnect Disconnect from the projector
     * @snippet projector_example.cpp Disconnect
     */
    // [Disconnect]
    cout << "Disconnecting..." << endl;
    print_dlp_errors(projector.Disconnect());
    // [Disconnect]

    return 0;
}
