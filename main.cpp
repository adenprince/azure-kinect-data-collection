// This file was made using sample code obtained from: https://github.com/microsoft/Azure-Kinect-Samples/blob/master/body-tracking-samples/simple_3d_viewer/main.cpp

#include "3DViewer.h"



#define VERIFY(result, error)                                                                            \
    if(result != K4A_RESULT_SUCCEEDED) {                                                                 \
        printf("%s \n - (File: %s, Function: %s, Line: %d)\n", error, __FILE__, __FUNCTION__, __LINE__); \
        exit(1);                                                                                         \
    }                                                                                                    \

using namespace std;

int main(int argc, char* argv[]) {
    InputSettings inputSettings;

    if (ParseInputSettingsFromArg(argc, argv, inputSettings)) {
        // Either play the offline file or play from the device
        if (inputSettings.Offline == true) {
            PlayFile(inputSettings);
        }
        else {
            PlayFromDevice(inputSettings);
        }
    }
    else {
        // Print app usage if user entered incorrect arguments.
        PrintUsage();
        return -1;
    }

    return 0;
}