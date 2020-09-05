/* Aden Prince
 * HiMER Lab at U. of Illinois, Chicago
 * Azure Kinect Data Collection
 * 
 * interface.cpp
 * Contains functions for getting program settings and displaying program usage.
 * 
 * Body tracking 3D viewer code obtained from: https://github.com/microsoft/Azure-Kinect-Samples/blob/master/body-tracking-samples/simple_3d_viewer/main.cpp
 */

#include <fstream>

#include <Window3dWrapper.h>

#include "imgui_dx11.h"
#include "imgui_internal.h"

#include "3DViewer.h"

// Print command-line argument usage to the command line
void PrintUsage() {
    printf("\nUSAGE: AzureKinectDataCollection.exe SensorMode[NFOV_UNBINNED, WFOV_BINNED](optional) RuntimeMode[CPU](optional)\n");
    printf("  - SensorMode: \n");
    printf("      NFOV_UNBINNED (default) - Narrow Field of View Unbinned Mode [Resolution: 640x576; FOI: 75 degree x 65 degree]\n");
    printf("      WFOV_BINNED             - Wide Field of View Binned Mode [Resolution: 512x512; FOI: 120 degree x 120 degree]\n");
    printf("  - RuntimeMode: \n");
    printf("      CPU - Use the CPU only mode. It runs on machines without a GPU but it will be much slower\n");
    printf("      OFFLINE - Play a specified file. Does not require Kinect device\n");
    printf("      OUTPUT - Write angle information to a specified file in CSV format\n");
    printf("e.g.   AzureKinectDataCollection.exe WFOV_BINNED CPU\n");
    printf("e.g.   AzureKinectDataCollection.exe CPU\n");
    printf("e.g.   AzureKinectDataCollection.exe WFOV_BINNED\n");
    printf("e.g.   AzureKinectDataCollection.exe OFFLINE MyFile.mkv\n");
    printf("e.g.   AzureKinectDataCollection.exe OUTPUT output.csv\n");
}

// Print 3D viewer window controls to the command line
void PrintAppUsage() {
    printf("\n");
    printf(" Basic Navigation:\n\n");
    printf(" Rotate: Rotate the camera by moving the mouse while holding mouse left button\n");
    printf(" Pan: Translate the scene by holding Ctrl key and drag the scene with mouse left button\n");
    printf(" Zoom in/out: Move closer/farther away from the scene center by scrolling the mouse scroll wheel\n");
    printf(" Select Center: Center the scene based on a detected joint by right clicking the joint with mouse\n");
    printf("\n");
    printf(" Key Shortcuts\n\n");
    printf(" ESC: quit\n");
    printf(" h: help\n");
    printf(" b: body visualization mode\n");
    printf(" k: 3d window layout\n");
    printf("\n");
}

// Check if a file exists with the passed filename
bool fileExists(std::string filename) {
    std::ifstream inputFile;
    inputFile.open(filename);
    bool isOpen = inputFile.is_open();
    inputFile.close();
    return isOpen;
}

// Get the first unused indexed output filename
std::string getIndexedFilename() {
    int fileIndex = 1;
    std::string curFilename = "output1.csv";

    // Run until an unused indexed output filename is found or the file index is too high
    while(fileExists(curFilename) && fileIndex < INT_MAX) {
        fileIndex++;
        curFilename = "output" + std::to_string(fileIndex) + ".csv";
    }

    // Check if the maximum number of numbered output files has been reached
    if(fileIndex == INT_MAX && fileExists(curFilename)) {
        std::string errorText = "Maximum number of indexed output files used.";
        printf("%s\n", errorText.c_str());
        MessageBoxA(0, errorText.c_str(), NULL, MB_OK | MB_ICONHAND);
        exit(1);
    }

    return curFilename;
}

// Create and handle startup GUI widgets
int startupGUIWidgets(InputSettings& inputSettings, std::string& errorText) {
    // 0: Continue running startup GUI, 1: Start data collection, -1: Quit program
    int startCollection = 0;

    const char* depth_modes[] = {"NFOV_2X2BINNED", "NFOV_UNBINNED", "WFOV_2X2BINNED", "WFOV_UNBINNED"};
    static int depth_mode_index = 1; // Default depth mode is NFOV_UNBINNED
    const char* frame_rates[] = {"30", "15", "5"};
    static int frame_rate_index = 0; // Default target frame rate is 30 FPS
    static bool cpu_mode = false;
    static bool offline_mode = false;
    static char input_filename[128] = "";
    static char output_filename[128] = "";

    // Disable depth mode and frame rate input if collecting data from file
    if(offline_mode) {
        ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
        ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);
    }
    ImGui::Combo("Depth camera mode", &depth_mode_index, depth_modes, IM_ARRAYSIZE(depth_modes));
    ImGui::Combo("Target frame rate", &frame_rate_index, frame_rates, IM_ARRAYSIZE(frame_rates));
    if(offline_mode) {
        ImGui::PopItemFlag();
        ImGui::PopStyleVar();
    }

    ImGui::Checkbox("CPU mode", &cpu_mode);
    ImGui::Checkbox("Collect data from file", &offline_mode);

    // Disable input filename text input if not collecting data from file
    if(!offline_mode) {
        ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
        ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);
    }
    ImGui::InputText("Input filename (.mkv)", input_filename, IM_ARRAYSIZE(input_filename));
    if(!offline_mode) {
        ImGui::PopItemFlag();
        ImGui::PopStyleVar();
    }

    // Check if the output filename in input settings and the text input do not match
    if(strcmp(output_filename, inputSettings.OutputFileName.c_str()) != 0) {
        // Copy the default output filename to the GUI once
        strcpy_s(output_filename, inputSettings.OutputFileName.c_str());
    }

    ImGui::InputText("Output filename", output_filename, IM_ARRAYSIZE(output_filename));

    // Update output filename in input settings
    inputSettings.OutputFileName = output_filename;

    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(ImColor::HSV(0.4f, 0.6f, 0.6f)));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(ImColor::HSV(0.4f, 0.7f, 0.7f)));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(ImColor::HSV(0.4f, 0.8f, 0.8f)));

    if(ImGui::Button("Start")) {
        // Reset error text
        errorText = "";

        inputSettings.CpuOnlyMode = cpu_mode;
        inputSettings.Offline = offline_mode;
        inputSettings.InputFileName = input_filename;

        if(depth_mode_index == 0) {
            inputSettings.DepthCameraMode = K4A_DEPTH_MODE_NFOV_2X2BINNED;
        }
        // No check for index 1 because depth mode is NFOV_UNBINNED by default
        else if(depth_mode_index == 2) {
            inputSettings.DepthCameraMode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
        }
        else if(depth_mode_index == 3) {
            inputSettings.DepthCameraMode = K4A_DEPTH_MODE_WFOV_UNBINNED;
        }

        // No check for index 0 because target frame rate is 30 FPS by default
        if(frame_rate_index == 1) {
            inputSettings.FrameRate = K4A_FRAMES_PER_SECOND_15;
        }
        else if(frame_rate_index == 2) {
            inputSettings.FrameRate = K4A_FRAMES_PER_SECOND_5;
        }

        // 1 is returned and data collection starts if there are no errors
        startCollection = 1;

        // Check for errors
        if(inputSettings.DepthCameraMode == K4A_DEPTH_MODE_WFOV_UNBINNED &&
           inputSettings.FrameRate == K4A_FRAMES_PER_SECOND_30) {
            errorText += "ERROR: WFOV_UNBINNED depth mode requires a lower frame rate\n";
            startCollection = 0;
        }

        if(offline_mode && !fileExists(inputSettings.InputFileName)) {
            errorText += "ERROR: Input file \"" + inputSettings.InputFileName + "\" does not exist\n";
            startCollection = 0;
        }

        // Check if there are no non-space characters in the output filename
        if(inputSettings.OutputFileName.find_first_not_of(' ') == std::string::npos) {
            errorText += "ERROR: Output filename is empty\n";
            startCollection = 0;
        }

        if(fileExists(inputSettings.OutputFileName)) {
            errorText += "ERROR: Output file \"" + inputSettings.OutputFileName + "\" already exists\n";
            startCollection = 0;
        }
    }

    ImGui::SameLine();

    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(ImColor::HSV(0.0f, 0.6f, 0.6f)));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(ImColor::HSV(0.0f, 0.7f, 0.7f)));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(ImColor::HSV(0.0f, 0.8f, 0.8f)));

    if(ImGui::Button("Quit")) {
        startCollection = -1; // Quit program
    }

    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.3f, 0.0f, 1.0f));
    ImGui::TextWrapped(errorText.c_str());

    // Remove style settings
    ImGui::PopStyleColor(7);

    // Return whether the program should continue running the GUI, start data collection, or quit
    return startCollection;
}

// Set input settings from a GUI
bool runStartupGUI(InputSettings& inputSettings) {
    // 0: Continue running startup GUI, 1: Start data collection, -1: Quit program
    int startCollection = 0;

    std::string errorText = "";

    inputSettings.OutputFileName = getIndexedFilename();

    // Correct font scaling
    if(!glfwInit()) {
        std::string errorText = "GLFW failed to initialize.";
        printf("%s\n", errorText.c_str());
        MessageBoxA(0, errorText.c_str(), NULL, MB_OK | MB_ICONHAND);
        exit(EXIT_FAILURE);
    }

    // Create application window
    WNDCLASSEX wc = {sizeof(WNDCLASSEX), CS_CLASSDC, WndProc, 0L, 0L, GetModuleHandle(NULL), NULL, NULL, NULL, NULL, _T("Program Settings"), NULL};
    ::RegisterClassEx(&wc);
    HWND hwnd = ::CreateWindow(wc.lpszClassName, _T("Program Settings"), WS_OVERLAPPEDWINDOW, 100, 100, 720, 520, NULL, NULL, wc.hInstance, NULL);

    initImGui(wc, hwnd);

    // Get main configuration and I/O between application and ImGui
    ImGuiIO& io = ImGui::GetIO(); (void) io;

    // Our state
    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

    // Main loop
    MSG msg;
    ZeroMemory(&msg, sizeof(msg));

    // Run until the window is closed or Quit is clicked
    while(msg.message != WM_QUIT && startCollection == 0) {
        // Poll and handle messages (inputs, window resize, etc.)
        if(::PeekMessage(&msg, NULL, 0U, 0U, PM_REMOVE)) {
            ::TranslateMessage(&msg);
            ::DispatchMessage(&msg);
            continue;
        }

        // Start the Dear ImGui frame
        ImGui_ImplDX11_NewFrame();
        ImGui_ImplWin32_NewFrame();
        ImGui::NewFrame();

        // Make next ImGui window fill OS window
        ImGui::SetNextWindowPos(ImVec2(0, 0));
        ImGui::SetNextWindowSize(io.DisplaySize);

        // Open startup GUI
        ImGui::Begin("Settings", (bool*) 0, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize);
        startCollection = startupGUIWidgets(inputSettings, errorText);
        ImGui::End();

        // Render
        ImGui::Render();
        g_pd3dDeviceContext->OMSetRenderTargets(1, &g_mainRenderTargetView, NULL);
        g_pd3dDeviceContext->ClearRenderTargetView(g_mainRenderTargetView, (float*) &clear_color);
        ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());

        g_pSwapChain->Present(1, 0); // Present with vsync
        //g_pSwapChain->Present(0, 0); // Present without vsync
    }

    // Cleanup
    ImGui_ImplDX11_Shutdown();
    ImGui_ImplWin32_Shutdown();
    ImGui::DestroyContext();

    CleanupDeviceD3D();
    ::DestroyWindow(hwnd);
    ::UnregisterClass(wc.lpszClassName, wc.hInstance);

    // Stop program if the window was closed or Quit was clicked
    if(msg.message == WM_QUIT || startCollection == -1) {
        return false;
    }

    // Empty message queue
    while(::PeekMessage(&msg, NULL, 0U, 0U, PM_REMOVE) != 0) {}

    return true;
}

// Set input settings from command-line arguments
bool ParseInputSettingsFromArg(int argc, char** argv, InputSettings& inputSettings) {
    for(int i = 1; i < argc; i++) {
        std::string inputArg(argv[i]);
        if(inputArg == std::string("NFOV_BINNED")) {
            inputSettings.DepthCameraMode = K4A_DEPTH_MODE_NFOV_2X2BINNED;
        }
        else if(inputArg == std::string("NFOV_UNBINNED")) {
            inputSettings.DepthCameraMode = K4A_DEPTH_MODE_NFOV_UNBINNED;
        }
        else if(inputArg == std::string("WFOV_BINNED")) {
            inputSettings.DepthCameraMode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
        }
        else if(inputArg == std::string("WFOV_UNBINNED")) {
            inputSettings.DepthCameraMode = K4A_DEPTH_MODE_WFOV_UNBINNED;
        }
        else if(inputArg == std::string("30_FPS")) {
            inputSettings.FrameRate = K4A_FRAMES_PER_SECOND_30;
        }
        else if(inputArg == std::string("15_FPS")) {
            inputSettings.FrameRate = K4A_FRAMES_PER_SECOND_15;
        }
        else if(inputArg == std::string("5_FPS")) {
            inputSettings.FrameRate = K4A_FRAMES_PER_SECOND_5;
        }
        else if(inputArg == std::string("CPU")) {
            inputSettings.CpuOnlyMode = true;
        }
        else if(inputArg == std::string("OFFLINE")) {
            inputSettings.Offline = true;
            if(i < argc - 1) {
                // Take the next argument after OFFLINE as input file name
                inputSettings.InputFileName = argv[i + 1];
                i++;
            }
            else {
                return false;
            }
        }
        else if(inputArg == std::string("OUTPUT")) {
            if(i < argc - 1) {
                // Take the next argument after OUTPUT as output file name
                inputSettings.OutputFileName = argv[i + 1];
                i++;
            }
            else {
                return false;
            }
        }
        else {
            printf("Error command not understood: %s\n", inputArg.c_str());
            return false;
        }
    }

    // Set output filename to default if not specified
    if(inputSettings.OutputFileName == "") {
        inputSettings.OutputFileName = getIndexedFilename();
    }
    // Check if output file already exists
    else if(fileExists(inputSettings.OutputFileName)) {
        printf("File %s already exists.\n", inputSettings.OutputFileName.c_str());
        return false;
    }

    return true;
}