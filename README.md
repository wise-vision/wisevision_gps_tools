# wisevision_gps_tools

This package is a part of tools useful for the WiseVision Dashboard, providing a GPS device manager - tool

Tool to create synthetic GPS coordinates for visualization purposes, works as a ROS2 node publishing GPS data.

Used in the WiseVision Dashboard for devices that do not have GPS sensor.

# Table of contents

For more information, please refer to the following sections:

### [Build and run (Start here)](docs/BUILD.md)
### [Minimal example](docs/MINIMAL_EXAMPLE.md)

If you want to contribute to this project, please read the following sections:
### [Code of conduct](docs/CODE_OF_CONDUCT.md)

## API:

### Services:
- /add_gps_device
- /delete_gps_device
- /modify_gps_device

### Publishers:
- /gps_devices_data
