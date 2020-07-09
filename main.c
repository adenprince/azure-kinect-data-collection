// This file was made using sample code obtained from: https://docs.microsoft.com/en-us/azure/kinect-dk/build-first-body-app

#include <stdio.h>
#include <stdlib.h>

#include <k4abt.h>

#include "vec.h"



#define VERIFY(result, error)                                                                            \
    if(result != K4A_RESULT_SUCCEEDED) {                                                                 \
        printf("%s \n - (File: %s, Function: %s, Line: %d)\n", error, __FILE__, __FUNCTION__, __LINE__); \
        exit(1);                                                                                         \
    }                                                                                                    \

int main() {
    const int MAX_FRAMES = 100;
    k4a_device_t device = NULL;
    VERIFY(k4a_device_open(0, &device), "Open K4A Device failed!");
    printf("Open K4A Device succeeded.\n");

    // Open a new file named output.csv in current directory
    FILE* output = NULL;
    fopen_s(&output, "output.csv", "w+"); // If this file exists, it will be overwritten

    // Write column names to the output file
    if(output != NULL) {
        fprintf(output, "ID,Left Elbow Angle,Right Elbow Angle,Left Knee Angle,Right Knee Angle\n");
    }

    // Start camera. Make sure depth camera is enabled.
    k4a_device_configuration_t deviceConfig = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    deviceConfig.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    deviceConfig.color_resolution = K4A_COLOR_RESOLUTION_OFF;
    VERIFY(k4a_device_start_cameras(device, &deviceConfig), "Start K4A cameras failed!");
    printf("Start K4A cameras succeeded.\n");

    k4a_calibration_t sensor_calibration;
    VERIFY(k4a_device_get_calibration(device, deviceConfig.depth_mode, deviceConfig.color_resolution, &sensor_calibration),
        "Get depth camera calibration failed!");
    printf("Get depth camera calibration succeeded.\n");

    k4abt_tracker_t tracker = NULL;
    k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
    VERIFY(k4abt_tracker_create(&sensor_calibration, tracker_config, &tracker), "Body tracker initialization failed!");
    printf("Body tracker initialization succeeded.\n");

    int frame_count = 0;

    // Runs for MAX_FRAMES iterations
    do {
        k4a_capture_t sensor_capture;
        k4a_wait_result_t get_capture_result = k4a_device_get_capture(device, &sensor_capture, K4A_WAIT_INFINITE);
        if(get_capture_result == K4A_WAIT_RESULT_SUCCEEDED) {
            frame_count++;
            k4a_wait_result_t queue_capture_result = k4abt_tracker_enqueue_capture(tracker, sensor_capture, K4A_WAIT_INFINITE);
            k4a_capture_release(sensor_capture); // Remember to release the sensor capture once you finish using it
            if(queue_capture_result == K4A_WAIT_RESULT_TIMEOUT) {
                // It should never hit timeout when K4A_WAIT_INFINITE is set.
                printf("Error! Add capture to tracker process queue timeout!\n");
                break;
            }
            else if(queue_capture_result == K4A_WAIT_RESULT_FAILED) {
                printf("Error! Add capture to tracker process queue failed!\n");
                break;
            }

            k4abt_frame_t body_frame = NULL;
            k4a_wait_result_t pop_frame_result = k4abt_tracker_pop_result(tracker, &body_frame, K4A_WAIT_INFINITE);
            if(pop_frame_result == K4A_WAIT_RESULT_SUCCEEDED) {
                // Successfully popped the body tracking result. Start your processing

                size_t num_bodies = k4abt_frame_get_num_bodies(body_frame);

                printf("%zu bodies are detected on frame %d\n", num_bodies, frame_count);

                // Add empty line to csv file if no bodies are detected
                if(num_bodies == 0 && output != NULL) {
                    fprintf(output, ",,,,\n");
                }

                for(uint32_t i = 0; i < num_bodies; i++) {
                    k4abt_skeleton_t skeleton;
                    k4abt_frame_get_body_skeleton(body_frame, i, &skeleton);
                    uint32_t id = k4abt_frame_get_body_id(body_frame, i);

                    // Store vectors representing lines between each joint
                    vec upperArmVecLeft = twoPointsToVec(skeleton.joints[K4ABT_JOINT_ELBOW_LEFT].position.xyz, skeleton.joints[K4ABT_JOINT_SHOULDER_LEFT].position.xyz);
                    vec forearmVecLeft = twoPointsToVec(skeleton.joints[K4ABT_JOINT_ELBOW_LEFT].position.xyz, skeleton.joints[K4ABT_JOINT_WRIST_LEFT].position.xyz);
                    
                    vec upperArmVecRight = twoPointsToVec(skeleton.joints[K4ABT_JOINT_ELBOW_RIGHT].position.xyz, skeleton.joints[K4ABT_JOINT_SHOULDER_RIGHT].position.xyz);
                    vec forearmVecRight = twoPointsToVec(skeleton.joints[K4ABT_JOINT_ELBOW_RIGHT].position.xyz, skeleton.joints[K4ABT_JOINT_WRIST_RIGHT].position.xyz);
                    
                    vec upperLegVecLeft = twoPointsToVec(skeleton.joints[K4ABT_JOINT_KNEE_LEFT].position.xyz, skeleton.joints[K4ABT_JOINT_HIP_LEFT].position.xyz);
                    vec lowerLegVecLeft = twoPointsToVec(skeleton.joints[K4ABT_JOINT_KNEE_LEFT].position.xyz, skeleton.joints[K4ABT_JOINT_ANKLE_LEFT].position.xyz);
                    
                    vec upperLegVecRight = twoPointsToVec(skeleton.joints[K4ABT_JOINT_KNEE_RIGHT].position.xyz, skeleton.joints[K4ABT_JOINT_HIP_RIGHT].position.xyz);
                    vec lowerLegVecRight = twoPointsToVec(skeleton.joints[K4ABT_JOINT_KNEE_RIGHT].position.xyz, skeleton.joints[K4ABT_JOINT_ANKLE_RIGHT].position.xyz);

                    // Calculate joint angles
                    float leftElbowAngle = twoVecsToAngle(upperArmVecLeft, forearmVecLeft);
                    float rightElbowAngle = twoVecsToAngle(upperArmVecRight, forearmVecRight);
                    float leftKneeAngle = twoVecsToAngle(upperLegVecLeft, lowerLegVecLeft);
                    float rightKneeAngle = twoVecsToAngle(upperLegVecRight, lowerLegVecRight);

                    // Print joint angles and write them to a file
                    printf("ID: %d\n", id);
                    printf("Left elbow angle: %f\n", leftElbowAngle);
                    printf("Right elbow angle: %f\n", rightElbowAngle);
                    printf("Left knee angle: %f\n", leftKneeAngle);
                    printf("Right knee angle: %f\n", rightKneeAngle);

                    if(output != NULL) {
                        fprintf(output, "%d,%f,%f,%f,%f\n", id, leftElbowAngle, rightElbowAngle, leftKneeAngle, rightKneeAngle);
                    }
                }

                k4abt_frame_release(body_frame); // Remember to release the body frame once you finish using it
            }
            else if(pop_frame_result == K4A_WAIT_RESULT_TIMEOUT) {
                //  It should never hit timeout when K4A_WAIT_INFINITE is set.
                printf("Error! Pop body frame result timeout!\n");
                break;
            }
            else {
                printf("Pop body frame result failed!\n");
                break;
            }
        }
        else if(get_capture_result == K4A_WAIT_RESULT_TIMEOUT) {
            // It should never hit time out when K4A_WAIT_INFINITE is set.
            printf("Error! Get depth frame time out!\n");
            break;
        }
        else {
            printf("Get depth capture returned error: %d\n", get_capture_result);
            break;
        }

    } while(frame_count < MAX_FRAMES);

    printf("Finished body tracking processing!\n");

    k4abt_tracker_shutdown(tracker);
    k4abt_tracker_destroy(tracker);
    k4a_device_stop_cameras(device);
    k4a_device_close(device);
    
    // Close file
    if(output != NULL) {
        fclose(output);
    }

    return 0;
}