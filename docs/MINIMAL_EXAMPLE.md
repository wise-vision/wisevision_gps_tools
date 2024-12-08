## Minimal Example

This minimal example demonstrates how to use the wisevision_gps_tools package to manage synthetic GPS devices for the WiseVision Dashboard. It guides you through adding, modifying, and deleting GPS devices, as well as viewing the published GPS data.

## Prerequisites

- [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html) or later
- [Build and run](docs/BUILD.md)
- [TODO: Add wisevision_data_black_box]

### Local run

Assuming that after building the package, you can source the workspace, and run the node:

``` bash
source install/setup.bash
ros2 run wisevision_gps_tools gps_device_manager_node
```

### Using the GPS Device Manager

The `gps_device_manager_node` provides services to add, delete, and modify GPS devices. It also publishes GPS data of the devices.

#### Services:

- `/add_gps_device` (wisevision_msgs/srv/AddGpsDevice): Add a new GPS device.
- `/delete_gps_device` (wisevision_msgs/srv/DeleteGpsDevice): Delete an existing GPS device.
- `/modify_gps_device` (wisevision_msgs/srv/ModifyGpsDevice): Modify an existing GPS device.

#### Publishers:
- `/gps_devices_data` (wisevision_msgs/msg/GpsDevicesPublisher): Publishes GPS data of all devices.


### Examples
#### Add a GPS Device

To add a new GPS device, call the `/add_gps_device` service:

```bash

ros2 service call /add_gps_device wisevision_msgs/srv/AddGpsDevice "{
  device_eui: { data: [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xAB, 0xCD] },
  device_name: 'Device_Initial',
  nav_value: { latitude: 40.7128, longitude: -74.0060, altitude: 50.0 }
}"
```
Response:

```bash
wisevision_msgs.srv.AddGpsDevice_Response(success=True, error='')
```

To see if the device was added, you can view the published GPS data:

```bash
ros2 topic echo /gps_devices_data
```


#### Delete a GPS Device

To delete an existing GPS device, call the `/delete_gps_device` service:

```bash
ros2 service call /delete_gps_device wisevision_msgs/srv/DeleteGpsDevice "{
  device_eui: { data: [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xAB, 0xCD] }
}"
```

Response:

```bash
wisevision_msgs.srv.DeleteGpsDevice_Response(success=True, error='')
```
#### Modify a GPS Device
To modify an existing GPS device:

```bash
ros2 service call /modify_gps_device wisevision_msgs/srv/ModifyGpsDevice "{
  device_eui: { data: [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xAB, 0xCD] },
  device_name: 'Device_Modified',
  nav_value: { latitude: 52.2297, longitude: 21.0122, altitude: 100.0 }
}"
```

Response:

```bash
wisevision_msgs.srv.ModifyGpsDevice_Response(success=True, error='')
```

#### Viewing Published GPS Data
The node publishes GPS data of all devices to the /gps_devices_data topic every 60 seconds. You can view the data using:

```bash
ros2 topic echo /gps_devices_data
```
Sample Output:

```yaml

devices_data:
- device_eui:
    data:
    - 0
    - 0
    - 0
    - 0
    - 0
    - 0
    - 171
    - 205
  device_name: Device_Modified
  nav_value:
    header:
      stamp:
        sec: 0
        nanosec: 0
      frame_id: ''
    status:
      status: 0
      service: 0
    latitude: 52.2297
    longitude: 21.0122
    altitude: 100.0
    position_covariance:
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    position_covariance_type: 0
  is_moving: false
Notes
The GPS devices are stored persistently in a YAML file (gps_devices.yaml) in the working directory.
Ensure that the device_eui is unique for each device and not all zeros.
The latitude, longitude, and altitude values must be within valid ranges:
Latitude: -90 to 90 degrees
Longitude: -180 to 180 degrees
Altitude: -1000 to 10000 meters
```