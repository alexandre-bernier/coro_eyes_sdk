#include <stdio.h>
#include <thread>
#include <chrono>
#include "projector.h"

using namespace std;

void print_errors(dlp::ReturnCode err)
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
    dlp::LCr4500 proj;
    dlp::LCr4500::Parameters param;

    cout << "Connecting..." << endl;
    print_errors(proj.Connect(""));
    print_errors(proj.Setup(param));

    print_errors(proj.ProjectSolidWhitePattern());

    std::this_thread::sleep_for (std::chrono::seconds(1));

    print_errors(proj.ProjectSolidBlackPattern());

    cout << "Disconnecting..." << endl;
    print_errors(proj.Disconnect());

    return 0;
}
