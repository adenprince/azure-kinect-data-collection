# Azure Kinect Data Collection

This tool collects body joint angle data from an Azure Kinect or an MKV file with depth from an Azure Kinect and stores it in a CSV (comma-separated values) file. A viewing window from the [Azure Kinect Body Tracking Simple3dViewer](https://github.com/microsoft/Azure-Kinect-Samples/tree/master/body-tracking-samples/simple_3d_viewer) is open while the program is running.

## Usage

Controls and command-line arguments are the same as the [Simple3dViewer](https://github.com/microsoft/Azure-Kinect-Samples/blob/master/body-tracking-samples/simple_3d_viewer/README.md#usage-info), with an added optional argument to set the output CSV file:

    AzureKinectDataCollection.exe OUTPUT outputNew.csv