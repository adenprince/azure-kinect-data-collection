# Azure Kinect Data Collection

This tool collects body joint angle data from an Azure Kinect or an MKV file with depth information from an Azure Kinect and stores it in a CSV (comma-separated values) file. A [3D viewer window](https://github.com/microsoft/Azure-Kinect-Samples/tree/master/body-tracking-samples/simple_3d_viewer) and a window showing angle data are open while data is being collected.

## Usage

A startup GUI with program options will open if there are no command-line arguments. 3D viewer window controls and command-line arguments are the same as the [Simple3dViewer](https://github.com/microsoft/Azure-Kinect-Samples/blob/master/body-tracking-samples/simple_3d_viewer/README.md#usage-info), with an added optional argument to set the output CSV file:

    AzureKinectDataCollection.exe OUTPUT outputNew.csv