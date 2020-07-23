/* Aden Prince
 * HiMER Lab at U. of Illinois, Chicago
 * Azure Kinect Data Collection
 * 
 * 3DViewer.cpp
 * Contains functions used for taking and processing user input,
 * displaying information, and collecting data.
 * 
 * ImGui sample code obtained from: https://github.com/ocornut/imgui/blob/master/examples/example_win32_directx11/main.cpp
 * ImGui font scaling code obtained from: https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/tools/k4aviewer/k4aviewer.cpp
 * Body tracking 3D viewer code obtained from: https://github.com/microsoft/Azure-Kinect-Samples/blob/master/body-tracking-samples/simple_3d_viewer/main.cpp
 */

#define _USE_MATH_DEFINES
#include <cmath>

#include <fstream>
#include <chrono>

#include <k4arecord/playback.h>
#include <k4a/k4a.h>

#include <BodyTrackingHelpers.h>
#include <Utilities.h>
#include <Window3dWrapper.h>

#include "imgui_dx11.h"
#include "imgui_internal.h"

#include "vec.h"
#include "3DViewer.h"

// Print command-line argument usage to the command line
void PrintUsage()
{
    printf("\nUSAGE: (k4abt_)simple_3d_viewer.exe SensorMode[NFOV_UNBINNED, WFOV_BINNED](optional) RuntimeMode[CPU](optional)\n");
    printf("  - SensorMode: \n");
    printf("      NFOV_UNBINNED (default) - Narrow Field of View Unbinned Mode [Resolution: 640x576; FOI: 75 degree x 65 degree]\n");
    printf("      WFOV_BINNED             - Wide Field of View Binned Mode [Resolution: 512x512; FOI: 120 degree x 120 degree]\n");
    printf("  - RuntimeMode: \n");
    printf("      CPU - Use the CPU only mode. It runs on machines without a GPU but it will be much slower\n");
    printf("      OFFLINE - Play a specified file. Does not require Kinect device\n");
    printf("      OUTPUT - Write angle information to a specified file in CSV format.\n");
    printf("e.g.   (k4abt_)simple_3d_viewer.exe WFOV_BINNED CPU\n");
    printf("e.g.   (k4abt_)simple_3d_viewer.exe CPU\n");
    printf("e.g.   (k4abt_)simple_3d_viewer.exe WFOV_BINNED\n");
    printf("e.g.   (k4abt_)simple_3d_viewer.exe OFFLINE MyFile.mkv\n");
    printf("e.g.   (k4abt_)simple_3d_viewer.exe OUTPUT output.csv\n");
}

// Print 3D viewer window controls to the command line
void PrintAppUsage()
{
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

// Global State and Key Process Function
bool s_isRunning = true;
Visualization::Layout3d s_layoutMode = Visualization::Layout3d::OnlyMainView;
bool s_visualizeJointFrame = false;

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
        printf("Maximum number of indexed output files used.\n");
        exit(1);
    }

    return curFilename;
}

// Convert three passed points into an angle between the vectors p2 to p1 and p2 to p3
float threePointsToAngle(k4a_float3_t& p1, k4a_float3_t& p2, k4a_float3_t& p3) {
    vec vec1(p2, p1);
    vec vec2(p2, p3);

    float dotProduct = vec1.x * vec2.x + vec1.y * vec2.y + vec1.z * vec2.z;
    float vec1Mag = sqrtf(vec1.x * vec1.x + vec1.y * vec1.y + vec1.z * vec1.z);
    float vec2Mag = sqrtf(vec2.x * vec2.x + vec2.y * vec2.y + vec2.z * vec2.z);

    // Use formula for getting angle between two vectors and convert result to degrees
    float res = acosf(dotProduct / (vec1Mag * vec2Mag)) * 180 / (float) M_PI;

    return res;
}

// Output joint angles from a passed skeleton 
void getJointAngles(uint32_t id, k4abt_skeleton_t& skeleton, std::ofstream& outputFile, long long durationCount) {
    // Calculate joint angles
    float leftElbowAngle = threePointsToAngle(skeleton.joints[K4ABT_JOINT_WRIST_LEFT].position,
                                              skeleton.joints[K4ABT_JOINT_ELBOW_LEFT].position,
                                              skeleton.joints[K4ABT_JOINT_SHOULDER_LEFT].position);
    float rightElbowAngle = threePointsToAngle(skeleton.joints[K4ABT_JOINT_WRIST_RIGHT].position,
                                               skeleton.joints[K4ABT_JOINT_ELBOW_RIGHT].position,
                                               skeleton.joints[K4ABT_JOINT_SHOULDER_RIGHT].position);
    float leftKneeAngle = threePointsToAngle(skeleton.joints[K4ABT_JOINT_HIP_LEFT].position,
                                             skeleton.joints[K4ABT_JOINT_KNEE_LEFT].position,
                                             skeleton.joints[K4ABT_JOINT_ANKLE_LEFT].position);
    float rightKneeAngle = threePointsToAngle(skeleton.joints[K4ABT_JOINT_HIP_RIGHT].position,
                                              skeleton.joints[K4ABT_JOINT_KNEE_RIGHT].position,
                                              skeleton.joints[K4ABT_JOINT_ANKLE_RIGHT].position);

    // Display joint angles and write them to a file
    ImGui::Text(u8"  Left elbow angle: %f°\n", leftElbowAngle);
    ImGui::Text(u8"  Right elbow angle: %f°\n", rightElbowAngle);
    ImGui::Text(u8"  Left knee angle: %f°\n", leftKneeAngle);
    ImGui::Text(u8"  Right knee angle: %f°\n", rightKneeAngle);

    outputFile << durationCount << "," << id << ","
               << leftElbowAngle << "," << rightElbowAngle << ","
               << leftKneeAngle << "," << rightKneeAngle << std::endl;
}

// Attempt to open output file and write the first line
void initOutputFile(std::ofstream& outputFile, std::string& outputFileName) {
    outputFile.open(outputFileName);

    if(outputFile.is_open()) {
        printf("Open file %s succeeded.\n", outputFileName.c_str());
    }
    else {
        printf("Open file %s failed.\n", outputFileName.c_str());
        s_isRunning = false; // Stop data collection from running
    }

    // Write column names to output file
    outputFile << "Time Since Last Frame,ID,Left Elbow Angle,Right Elbow Angle,Left Knee Angle,Right Knee Angle\n";
}

// Display body and angle information from frame
void processFrame(k4abt_frame_t& bodyFrame, std::ofstream& outputFile, int& processed_frames, std::chrono::high_resolution_clock::time_point& startTime) {
    size_t num_bodies = k4abt_frame_get_num_bodies(bodyFrame);
    processed_frames++;
    auto stopTime = std::chrono::high_resolution_clock::now();

    // The first frame should be 0 ms from the previous frame
    if(processed_frames == 1) {
        startTime = stopTime;
    }

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stopTime - startTime);
    startTime = std::chrono::high_resolution_clock::now();
    long long durationCount = duration.count();

    // Start ImGui window
    ImGui::Begin("Data", (bool*) 0, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize);
    ImGui::Text("Bodies detected: %zu", num_bodies);
    ImGui::Text("Frames processed: %d", processed_frames);
    ImGui::Text("Time since last frame: %lld ms", durationCount);

    // Add empty line to CSV file if no bodies are detected
    if(num_bodies == 0) {
        outputFile << durationCount << ",,,,," << std::endl;
    }

    // Process each detected body
    for(uint32_t i = 0; i < num_bodies; i++) {
        // Get and display data from current body
        uint32_t id = k4abt_frame_get_body_id(bodyFrame, i);
        k4abt_skeleton_t skeleton;
        k4abt_frame_get_body_skeleton(bodyFrame, i, &skeleton);

        ImGui::Separator();
        ImGui::Text("Body %d:", id);
        getJointAngles(id, skeleton, outputFile, durationCount);
    }

    ImGui::End();
}

// Create and handle startup GUI widgets
int startupGUIWidgets(InputSettings& inputSettings, std::string& errorText) {
    // 0: Continue running startup GUI, 1: Start data collection, -1: Quit program
    int startCollection = 0;

    const char* depth_modes[] = {"NFOV_UNBINNED", "WFOV_BINNED"};
    static int depth_mode_index = 0; // Default depth mode is NFOV_UNBINNED
    static bool cpu_mode = false;
    static bool offline_mode = false;
    static char input_filename[128] = "";
    static char output_filename[128] = "";

    ImGui::Combo("Depth camera mode", &depth_mode_index, depth_modes, IM_ARRAYSIZE(depth_modes));
    ImGui::Checkbox("CPU mode", &cpu_mode);
    ImGui::Checkbox("Collect data from file", &offline_mode);

    // Disable input filename text input if not collecting data from file
    if(offline_mode == false) {
        ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
        ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);
    }
    ImGui::InputText("Input filename", input_filename, IM_ARRAYSIZE(input_filename));
    if(offline_mode == false) {
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

        if(depth_mode_index == 1) {
            inputSettings.DepthCameraMode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
        }

        // 1 is returned and data collection starts if there are no errors
        startCollection = 1;

        // Check for errors
        if(offline_mode && fileExists(inputSettings.InputFileName) == false) {
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

// Initialize ImGui for DirectX 11
void initImGui(WNDCLASSEX& wc, HWND& hwnd) {
    const float GUIScalingFactor = 1.5f;

    // Initialize Direct3D
    if(!CreateDeviceD3D(hwnd)) {
        CleanupDeviceD3D();
        ::UnregisterClass(wc.lpszClassName, wc.hInstance);
        exit(1);
    }

    // Show the window
    ::ShowWindow(hwnd, SW_SHOWDEFAULT);
    ::UpdateWindow(hwnd);

    // Set up Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();

    // Set up Dear ImGui style
    ImGui::StyleColorsDark();

    // Set up Platform/Renderer bindings
    ImGui_ImplWin32_Init(hwnd);
    ImGui_ImplDX11_Init(g_pd3dDevice, g_pd3dDeviceContext);

    // Scale font
    ImGui::GetStyle().ScaleAllSizes(GUIScalingFactor);
    ImFontConfig fontConfig;
    constexpr float defaultFontSize = 13.0f;
    fontConfig.SizePixels = defaultFontSize * GUIScalingFactor;
    ImGui::GetIO().Fonts->AddFontDefault(&fontConfig);
}

// Set input settings from a GUI
bool runStartupGUI(InputSettings& inputSettings) {
    // 0: Continue running startup GUI, 1: Start data collection, -1: Quit program
    int startCollection = 0;
    
    std::string errorText = "";

    inputSettings.OutputFileName = getIndexedFilename();

    // Correct font scaling
    if(!glfwInit()) {
        exit(EXIT_FAILURE);
    }

    // Create application window
    WNDCLASSEX wc = {sizeof(WNDCLASSEX), CS_CLASSDC, WndProc, 0L, 0L, GetModuleHandle(NULL), NULL, NULL, NULL, NULL, _T("Program Settings"), NULL};
    ::RegisterClassEx(&wc);
    HWND hwnd = ::CreateWindow(wc.lpszClassName, _T("Program Settings"), WS_OVERLAPPEDWINDOW, 100, 100, 640, 480, NULL, NULL, wc.hInstance, NULL);
    
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

// Process 3D viewer window key inputs
int64_t ProcessKey(void* /*context*/, int key)
{
    // https://www.glfw.org/docs/latest/group__keys.html
    switch (key)
    {
        // Quit
    case GLFW_KEY_ESCAPE:
        s_isRunning = false;
        break;
    case GLFW_KEY_K:
        s_layoutMode = (Visualization::Layout3d)(((int)s_layoutMode + 1) % (int)Visualization::Layout3d::Count);
        break;
    case GLFW_KEY_B:
        s_visualizeJointFrame = !s_visualizeJointFrame;
        break;
    case GLFW_KEY_H:
        PrintAppUsage();
        break;
    }
    return 1;
}

// Close the program
int64_t CloseCallback(void* /*context*/)
{
    s_isRunning = false;
    return 1;
}

// Set input settings from command-line arguments
bool ParseInputSettingsFromArg(int argc, char** argv, InputSettings& inputSettings)
{
    for (int i = 1; i < argc; i++)
    {
        std::string inputArg(argv[i]);
        if (inputArg == std::string("NFOV_UNBINNED"))
        {
            inputSettings.DepthCameraMode = K4A_DEPTH_MODE_NFOV_UNBINNED;
        }
        else if (inputArg == std::string("WFOV_BINNED"))
        {
            inputSettings.DepthCameraMode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
        }
        else if (inputArg == std::string("CPU"))
        {
            inputSettings.CpuOnlyMode = true;
        }
        else if (inputArg == std::string("OFFLINE"))
        {
            inputSettings.Offline = true;
            if (i < argc - 1) {
                // Take the next argument after OFFLINE as file name
                inputSettings.InputFileName = argv[i + 1];
                i++;
            }
            else {
                return false;
            }
        }
        else if (inputArg == std::string("OUTPUT"))
        {
            if (i < argc - 1)
            {
                // Take the next argument after OUTPUT as file name
                inputSettings.OutputFileName = argv[i + 1];
                i++;
            }
            else
            {
                return false;
            }
        }
        else
        {
            printf("Error command not understood: %s\n", inputArg.c_str());
            return false;
        }
    }

    // Set output filename to default if not specified
    if (inputSettings.OutputFileName == "")
    {
        inputSettings.OutputFileName = getIndexedFilename();
    }
    // Check if output file already exists
    else if (fileExists(inputSettings.OutputFileName))
    {
        printf("File %s already exists.\n", inputSettings.OutputFileName.c_str());
        return false;
    }

    return true;
}

// Display graphics in the 3D viewer window
void VisualizeResult(k4abt_frame_t bodyFrame, Window3dWrapper& window3d, int depthWidth, int depthHeight)
{
    // Obtain original capture that generates the body tracking result
    k4a_capture_t originalCapture = k4abt_frame_get_capture(bodyFrame);
    k4a_image_t depthImage = k4a_capture_get_depth_image(originalCapture);

    std::vector<Color> pointCloudColors(depthWidth * depthHeight, { 1.f, 1.f, 1.f, 1.f });

    // Read body index map and assign colors
    k4a_image_t bodyIndexMap = k4abt_frame_get_body_index_map(bodyFrame);
    const uint8_t* bodyIndexMapBuffer = k4a_image_get_buffer(bodyIndexMap);
    for (int i = 0; i < depthWidth * depthHeight; i++)
    {
        uint8_t bodyIndex = bodyIndexMapBuffer[i];
        if (bodyIndex != K4ABT_BODY_INDEX_MAP_BACKGROUND)
        {
            uint32_t bodyId = k4abt_frame_get_body_id(bodyFrame, bodyIndex);
            pointCloudColors[i] = g_bodyColors[bodyId % g_bodyColors.size()];
        }
    }
    k4a_image_release(bodyIndexMap);

    // Visualize point cloud
    window3d.UpdatePointClouds(depthImage, pointCloudColors);

    // Visualize the skeleton data
    window3d.CleanJointsAndBones();
    uint32_t numBodies = k4abt_frame_get_num_bodies(bodyFrame);
    for (uint32_t i = 0; i < numBodies; i++)
    {
        k4abt_body_t body;
        VERIFY(k4abt_frame_get_body_skeleton(bodyFrame, i, &body.skeleton), "Get skeleton from body frame failed!");
        body.id = k4abt_frame_get_body_id(bodyFrame, i);

        // Assign the correct color based on the body id
        Color color = g_bodyColors[body.id % g_bodyColors.size()];
        color.a = 0.4f;
        Color lowConfidenceColor = color;
        lowConfidenceColor.a = 0.1f;

        // Visualize joints
        for (int joint = 0; joint < static_cast<int>(K4ABT_JOINT_COUNT); joint++)
        {
            if (body.skeleton.joints[joint].confidence_level >= K4ABT_JOINT_CONFIDENCE_LOW)
            {
                const k4a_float3_t& jointPosition = body.skeleton.joints[joint].position;
                const k4a_quaternion_t& jointOrientation = body.skeleton.joints[joint].orientation;

                window3d.AddJoint(
                    jointPosition,
                    jointOrientation,
                    body.skeleton.joints[joint].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM ? color : lowConfidenceColor);
            }
        }

        // Visualize bones
        for (size_t boneIdx = 0; boneIdx < g_boneList.size(); boneIdx++)
        {
            k4abt_joint_id_t joint1 = g_boneList[boneIdx].first;
            k4abt_joint_id_t joint2 = g_boneList[boneIdx].second;

            if (body.skeleton.joints[joint1].confidence_level >= K4ABT_JOINT_CONFIDENCE_LOW &&
                body.skeleton.joints[joint2].confidence_level >= K4ABT_JOINT_CONFIDENCE_LOW)
            {
                bool confidentBone = body.skeleton.joints[joint1].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM &&
                    body.skeleton.joints[joint2].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM;
                const k4a_float3_t& joint1Position = body.skeleton.joints[joint1].position;
                const k4a_float3_t& joint2Position = body.skeleton.joints[joint2].position;

                window3d.AddBone(joint1Position, joint2Position, confidentBone ? color : lowConfidenceColor);
            }
        }
    }

    k4a_capture_release(originalCapture);
    k4a_image_release(depthImage);
}

// Run body tracking data collection on a pre-recorded video file
void PlayFile(InputSettings inputSettings)
{
    const float GUIScalingFactor = 1.5f;

    // Initialize the 3d window controller
    Window3dWrapper window3d;

    // Create the tracker and playback handle
    k4a_calibration_t sensor_calibration;
    k4abt_tracker_t tracker = NULL;
    k4a_playback_t playback_handle = NULL;

    // Attempt to open pre-recorded video file
    const char* inputFileName = inputSettings.InputFileName.c_str();
    if (k4a_playback_open(inputFileName, &playback_handle) != K4A_RESULT_SUCCEEDED)
    {
        printf("Failed to open recording: %s\n", inputFileName);
        return;
    }

    if (k4a_playback_get_calibration(playback_handle, &sensor_calibration) != K4A_RESULT_SUCCEEDED)
    {
        printf("Failed to get calibration\n");
        return;
    }

    k4a_capture_t capture = NULL;
    k4a_stream_result_t result = K4A_STREAM_RESULT_SUCCEEDED;

    k4abt_tracker_configuration_t tracker_config = { K4ABT_SENSOR_ORIENTATION_DEFAULT };

    tracker_config.processing_mode = inputSettings.CpuOnlyMode ? K4ABT_TRACKER_PROCESSING_MODE_CPU : K4ABT_TRACKER_PROCESSING_MODE_GPU;

    VERIFY(k4abt_tracker_create(&sensor_calibration, tracker_config, &tracker), "Body tracker initialization failed!");

    k4abt_tracker_set_temporal_smoothing(tracker, 1);

    int depthWidth = sensor_calibration.depth_camera_calibration.resolution_width;
    int depthHeight = sensor_calibration.depth_camera_calibration.resolution_height;

    window3d.Create("3D Visualization", sensor_calibration);
    window3d.SetCloseCallback(CloseCallback);
    window3d.SetKeyCallback(ProcessKey);

    std::ofstream outputFile;
    initOutputFile(outputFile, inputSettings.OutputFileName);

    int processed_frames = 0;
    auto startTime = std::chrono::high_resolution_clock::now();

    // Create application window
    WNDCLASSEX wc = { sizeof(WNDCLASSEX), CS_CLASSDC, WndProc, 0L, 0L, GetModuleHandle(NULL), NULL, NULL, NULL, NULL, _T("Azure Kinect Data"), NULL };
    ::RegisterClassEx(&wc);
    HWND hwnd = ::CreateWindow(wc.lpszClassName, _T("Azure Kinect Data"), WS_OVERLAPPEDWINDOW, 100, 100, 480, 640, NULL, NULL, wc.hInstance, NULL);

    initImGui(wc, hwnd);

    // Get main configuration and I/O between application and ImGui
    ImGuiIO& io = ImGui::GetIO(); (void)io;

    // Our state
    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

    // Main loop
    MSG msg;
    ZeroMemory(&msg, sizeof(msg));

    // Run until getting capture data fails
    while (result == K4A_STREAM_RESULT_SUCCEEDED)
    {
        bool frameProcessed = false;

        if (::PeekMessage(&msg, NULL, 0U, 0U, PM_REMOVE))
        {
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

        result = k4a_playback_get_next_capture(playback_handle, &capture);
        // Check to make sure we have a depth image if we are not at the end of the file
        if (result != K4A_STREAM_RESULT_EOF)
        {
            k4a_image_t depth_image = k4a_capture_get_depth_image(capture);
            if (depth_image == NULL)
            {
                // If no depth image, print a warning and skip to next frame
                printf("Warning: No depth image, skipping frame\n");
                k4a_capture_release(capture);

                // Add empty line to output file
                outputFile << ",,,,," << std::endl;
                continue;
            }
            // Release the depth image
            k4a_image_release(depth_image);
        }
        if (result == K4A_STREAM_RESULT_SUCCEEDED)
        {
            // Enqueue capture and pop results - synchronous
            k4a_wait_result_t queue_capture_result = k4abt_tracker_enqueue_capture(tracker, capture, K4A_WAIT_INFINITE);

            // Release the sensor capture once it is no longer needed.
            k4a_capture_release(capture);

            k4abt_frame_t bodyFrame = NULL;
            k4a_wait_result_t pop_frame_result = k4abt_tracker_pop_result(tracker, &bodyFrame, K4A_WAIT_INFINITE);
            if (pop_frame_result == K4A_WAIT_RESULT_SUCCEEDED)
            {
                /************* Successfully got a body tracking result, process the result here ***************/
                processFrame(bodyFrame, outputFile, processed_frames, startTime);

                VisualizeResult(bodyFrame, window3d, depthWidth, depthHeight);
                // Release the bodyFrame
                k4abt_frame_release(bodyFrame);

                frameProcessed = true;
            }
            else
            {
                printf("Pop body frame result failed!\n");
                break;
            }

        }

        // Render GUI when the 3D viewer window has updated
        if (frameProcessed)
        {
            ImGui::Render();
            g_pd3dDeviceContext->OMSetRenderTargets(1, &g_mainRenderTargetView, NULL);
            g_pd3dDeviceContext->ClearRenderTargetView(g_mainRenderTargetView, (float*) &clear_color);
            ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());

            g_pSwapChain->Present(1, 0); // Present with vsync
            //g_pSwapChain->Present(0, 0); // Present without vsync
        }

        window3d.SetLayout3d(s_layoutMode);
        window3d.SetJointFrameVisualization(s_visualizeJointFrame);
        window3d.Render();
    }
    k4abt_tracker_shutdown(tracker);
    k4abt_tracker_destroy(tracker);
    window3d.Delete();
    printf("Finished body tracking processing!\n");
    k4a_playback_close(playback_handle);
    
    outputFile.close();

    // ImGui Cleanup
    ImGui_ImplDX11_Shutdown();
    ImGui_ImplWin32_Shutdown();
    ImGui::DestroyContext();

    CleanupDeviceD3D();
    ::DestroyWindow(hwnd);
    ::UnregisterClass(wc.lpszClassName, wc.hInstance);
}

// Run body tracking data collection on a real-time capture from an Azure Kinect
void PlayFromDevice(InputSettings inputSettings) {
    const float GUIScalingFactor = 1.5f;

    k4a_device_t device = nullptr;
    VERIFY(k4a_device_open(0, &device), "Open K4A Device failed!");

    // Start camera. Make sure depth camera is enabled.
    k4a_device_configuration_t deviceConfig = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    deviceConfig.depth_mode = inputSettings.DepthCameraMode;
    deviceConfig.color_resolution = K4A_COLOR_RESOLUTION_OFF;
    VERIFY(k4a_device_start_cameras(device, &deviceConfig), "Start K4A cameras failed!");

    // Get calibration information
    k4a_calibration_t sensorCalibration;
    VERIFY(k4a_device_get_calibration(device, deviceConfig.depth_mode, deviceConfig.color_resolution, &sensorCalibration),
        "Get depth camera calibration failed!");
    int depthWidth = sensorCalibration.depth_camera_calibration.resolution_width;
    int depthHeight = sensorCalibration.depth_camera_calibration.resolution_height;

    // Create Body Tracker
    k4abt_tracker_t tracker = nullptr;
    k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
    tracker_config.processing_mode = inputSettings.CpuOnlyMode ? K4ABT_TRACKER_PROCESSING_MODE_CPU : K4ABT_TRACKER_PROCESSING_MODE_GPU;
    VERIFY(k4abt_tracker_create(&sensorCalibration, tracker_config, &tracker), "Body tracker initialization failed!");
    // Initialize the 3d window controller
    Window3dWrapper window3d;
    window3d.Create("3D Visualization", sensorCalibration);
    window3d.SetCloseCallback(CloseCallback);
    window3d.SetKeyCallback(ProcessKey);

    std::ofstream outputFile;
    initOutputFile(outputFile, inputSettings.OutputFileName);

    int processed_frames = 0;
    auto startTime = std::chrono::high_resolution_clock::now();

    // Create application window
    WNDCLASSEX wc = { sizeof(WNDCLASSEX), CS_CLASSDC, WndProc, 0L, 0L, GetModuleHandle(NULL), NULL, NULL, NULL, NULL, _T("Azure Kinect Data"), NULL };
    ::RegisterClassEx(&wc);
    HWND hwnd = ::CreateWindow(wc.lpszClassName, _T("Azure Kinect Data"), WS_OVERLAPPEDWINDOW, 100, 100, 480, 640, NULL, NULL, wc.hInstance, NULL);

    initImGui(wc, hwnd);

    // Get main configuration and I/O between application and ImGui
    ImGuiIO& io = ImGui::GetIO(); (void)io;

    // Our state
    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

    // Main loop
    MSG msg;
    ZeroMemory(&msg, sizeof(msg));

    // Run until the program is closed
    while (s_isRunning)
    {
        bool frameProcessed = false;

        if (::PeekMessage(&msg, NULL, 0U, 0U, PM_REMOVE))
        {
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
        
        k4a_capture_t sensorCapture = nullptr;
        k4a_wait_result_t getCaptureResult = k4a_device_get_capture(device, &sensorCapture, 0); // timeout_in_ms is set to 0

        if (getCaptureResult == K4A_WAIT_RESULT_SUCCEEDED)
        {
            // timeout_in_ms is set to 0. Return immediately no matter whether the sensorCapture is successfully added
            // to the queue or not.
            k4a_wait_result_t queueCaptureResult = k4abt_tracker_enqueue_capture(tracker, sensorCapture, 0);

            // Release the sensor capture once it is no longer needed.
            k4a_capture_release(sensorCapture);

            if (queueCaptureResult == K4A_WAIT_RESULT_FAILED)
            {
                std::string errorText = "Error! Add capture to tracker process queue failed!";
                printf("%s\n", errorText.c_str());
                MessageBoxA(0, errorText.c_str(), NULL, MB_OK | MB_ICONHAND);
                break;
            }
        }
        else if (getCaptureResult != K4A_WAIT_RESULT_TIMEOUT)
        {
            std::string errorText = "Get depth capture returned error";
            printf("%s: %d\n", errorText.c_str(), getCaptureResult);
            MessageBoxA(0, errorText.c_str(), NULL, MB_OK | MB_ICONHAND);
            break;
        }

        // Pop Result from Body Tracker
        k4abt_frame_t bodyFrame = nullptr;
        k4a_wait_result_t popFrameResult = k4abt_tracker_pop_result(tracker, &bodyFrame, 0); // timeout_in_ms is set to 0
        if (popFrameResult == K4A_WAIT_RESULT_SUCCEEDED)
        {
            /************* Successfully got a body tracking result, process the result here ***************/
            processFrame(bodyFrame, outputFile, processed_frames, startTime);

            VisualizeResult(bodyFrame, window3d, depthWidth, depthHeight);
            // Release the bodyFrame
            k4abt_frame_release(bodyFrame);

            frameProcessed = true;
        }

        // Render GUI when the 3D viewer window has updated
        if (frameProcessed)
        {
            ImGui::Render();
            g_pd3dDeviceContext->OMSetRenderTargets(1, &g_mainRenderTargetView, NULL);
            g_pd3dDeviceContext->ClearRenderTargetView(g_mainRenderTargetView, (float*) &clear_color);
            ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());

            g_pSwapChain->Present(1, 0); // Present with vsync
            //g_pSwapChain->Present(0, 0); // Present without vsync
        }

        window3d.SetLayout3d(s_layoutMode);
        window3d.SetJointFrameVisualization(s_visualizeJointFrame);
        window3d.Render();
    }

    printf("Finished body tracking processing!\n");

    window3d.Delete();
    k4abt_tracker_shutdown(tracker);
    k4abt_tracker_destroy(tracker);

    k4a_device_stop_cameras(device);
    k4a_device_close(device);

    outputFile.close();
    
    // ImGui Cleanup
    ImGui_ImplDX11_Shutdown();
    ImGui_ImplWin32_Shutdown();
    ImGui::DestroyContext();

    CleanupDeviceD3D();
    ::DestroyWindow(hwnd);
    ::UnregisterClass(wc.lpszClassName, wc.hInstance);
}