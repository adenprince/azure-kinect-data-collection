/* Aden Prince
 * HiMER Lab at U. of Illinois, Chicago
 * Azure Kinect Data Collection
 * 
 * 3DViewer.h
 * Contains code used in multiple source files, such as libraries,
 * function declarations, and an InputSettings structure definition.
 * 
 * Body tracking 3D viewer code obtained from: https://github.com/microsoft/Azure-Kinect-Samples/blob/master/body-tracking-samples/simple_3d_viewer/main.cpp
 */

#define _USE_MATH_DEFINES
#include <cmath>

#include <array>
#include <iostream>
#include <fstream>
#include <string>
#include <chrono>

#include <k4arecord/playback.h>
#include <k4a/k4a.h>
#include <k4abt.h>

#include <BodyTrackingHelpers.h>
#include <Utilities.h>
#include <Window3dWrapper.h>

#include "imgui_dx11.h"
#include "imgui_internal.h"

#include "vec.h"

// Store option values for the program
struct InputSettings {
    k4a_depth_mode_t DepthCameraMode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    bool CpuOnlyMode = false;
    bool Offline = false;
    std::string InputFileName;
    std::string OutputFileName;
};

// Print command-line argument usage to the command line
void PrintUsage();
// Set input settings from a GUI
int runStartupGUI(InputSettings& is);
// Set input settings from command-line arguments
bool ParseInputSettingsFromArg(int argc, char** argv, InputSettings& inputSettings);
// Run body tracking data collection on a pre-recorded video file
void PlayFile(InputSettings inputSettings);
// Run body tracking data collection on a real-time capture from an Azure Kinect
void PlayFromDevice(InputSettings inputSettings);