#include <stdio.h>
#include <thread>
#include <chrono>
#include "projector.h"

using namespace std;

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

int main(void)
{
    dlp::ReturnCode ret;

    dlp::LCr4500 proj;

    dlp::Parameters param;
    string proj_param_file = "../resources/dlp_platforms/projector_settings.txt";

    // Connect projector
    cout << "Connecting..." << endl;
    ret = proj.Connect("");
    print_dlp_errors(ret);
    if(ret.hasErrors()) {
        cout << "Aborting..." << endl;
        return -1;
    }

    // Load parameter file
    cout << "Loading parameters..." << endl;
    ret = param.Load(proj_param_file);
    print_dlp_errors(ret);
    if(ret.hasErrors()) {
        cout << "Aborting..." << endl;
        return -1;
    }

    // Setup projector
    cout << "Setting up projector..." << endl;
    ret = proj.Setup(param);
    print_dlp_errors(ret);
    if(ret.hasErrors()) {
        cout << "Aborting..." << endl;
        return -1;
    }

    // Project white pattern
    cout << "Projecting white..." << endl;
    print_dlp_errors(proj.ProjectSolidWhitePattern());

    // Wait
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Project black pattern
    cout << "Stopping projection..." << endl;
    print_dlp_errors(proj.StopPatternSequence());

    // Disconnect projector
    cout << "Disconnecting..." << endl;
    print_dlp_errors(proj.Disconnect());

    return 0;
}
