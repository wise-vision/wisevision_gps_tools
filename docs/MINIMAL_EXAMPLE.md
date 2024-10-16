
# Services
## add_gps_device

```
ros2 service call /add_gps_device wisevision_msgs/srv/AddGpsDevice "{device_eui: {data: [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]}, device_name: 'Device_Initial', nav_value: {latitude: 40.7128, longitude: -74.0060, altitude: 50.0}}"
```
Resposne:
```
wisevision_msgs.srv.AddGpsDevice_Response(success=True, error='')
```
## delete_gps_device

```
ros2 service call /delete_gps_device wisevision_msgs/srv/DeleteGpsDevice "{device_eui: {data: [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]}}"
```

Response:
```
wisevision_msgs.srv.DeleteGpsDevice_Response(success=True, error='')
```


## modify_gps_device

```
ros2 service call /modify_gps_device wisevision_msgs/srv/ModifyGpsDevice "{device_eui: {data: [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]}, device_name: 'Device_Zero', nav_value: {latitude: 52.2297, longitude: 21.0122, altitude: 100.0}}"
```

Resposne:
```
wisevision_msgs.srv.ModifyGpsDevice_Response(success=True, error='')
```

# Publishers
## Publisher (period: 60 s):

``` bash
ros2 topic echo /gps_devices_data
```
```
- device_eui:
    data:
    - 0
    - 0
    - 0
    - 0
    - 0
    - 0
    - 0
    - 0
  device_name: Device_Zero
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
```