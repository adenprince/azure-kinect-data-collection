/* Aden Prince
 * HiMER Lab at U. of Illinois, Chicago
 * Azure Kinect Data Collection
 * 
 * main.cpp
 * Calls functions to get input settings and start data collection.
 * 
 * Body tracking 3D viewer code obtained from: https://github.com/microsoft/Azure-Kinect-Samples/blob/master/body-tracking-samples/simple_3d_viewer/main.cpp
 */

#include "3DViewer.h"

using namespace std;

int main(int argc, char* argv[]) {
    InputSettings inputSettings;

    // Run startup GUI if there are no command line arguments
    if((argc > 1 && ParseInputSettingsFromArg(argc, argv, inputSettings)) ||
       (argc == 1 && runStartupGUI(inputSettings) == 0)) {
        // Either play the offline file or play from the device
        if(inputSettings.Offline == true) {
            PlayFile(inputSettings);
        }
        else {
            PlayFromDevice(inputSettings);
        }
    }
    else if(argc > 1) {
        // Print app usage if user entered incorrect arguments
        PrintUsage();
        return -1;
    }

    return 0;
}