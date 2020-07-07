// This file was made using sample code obtained from: https://docs.microsoft.com/en-us/azure/kinect-dk/build-first-app 

#pragma comment(lib, "k4a.lib")
#include <k4a/k4a.h>
#include <k4arecord/record.h>
#include <k4arecord/playback.h>

#include <stdio.h>
#include <stdlib.h>

int main() {
	uint32_t count = k4a_device_get_installed_count();
	if(count == 0) {
		printf("No k4a devices attached!\n");
		return 1;
	}

	// Open the first plugged in Kinect device
	k4a_device_t device = NULL;
	if(K4A_FAILED(k4a_device_open(K4A_DEVICE_DEFAULT, &device))) {
		printf("Failed to open k4a device!\n");
		return 1;
	}

	// Get the size of the serial number
	size_t serial_size = 0;
	k4a_device_get_serialnum(device, NULL, &serial_size);

	// Allocate memory for the serial, then acquire it
	char* serial = (char*) malloc(serial_size);
	k4a_device_get_serialnum(device, serial, &serial_size);
	printf("Opened device: %s\n", serial);
	free(serial);

	// Configure a stream of 4096x3072 BRGA color data at 15 frames per second
	k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	config.camera_fps = K4A_FRAMES_PER_SECOND_15;
	config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
	config.color_resolution = K4A_COLOR_RESOLUTION_3072P;

	// Start the camera with the given configuration
	if(K4A_FAILED(k4a_device_start_cameras(device, &config))) {
		printf("Failed to start cameras!\n");
		k4a_device_close(device);
		return 1;
	}

	// Camera capture and application specific code would go here

	// Shut down the camera when finished with application logic
	k4a_device_stop_cameras(device);
	k4a_device_close(device);

	return 0;
}