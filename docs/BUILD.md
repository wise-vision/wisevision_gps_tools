# Build and run

## Local build
Prequistances:
``` bash
apt-get update && apt-get install -y libyaml-cpp-dev libjsoncpp-dev
```
Build:
``` bash
mkdir -p wisevision_gps_tools_ws/src && cd wisevision_gps_tools_ws/src
git clone git@github.com:wise-vision/wisevision_gps_tools.git
vcs import --recursive < wisevision_gps_tools/wisevision_gps_tools.repos
cd ..
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to wisevision_gps_tools
```
## Local run
``` bash
source install/setup.bash
ros2 run wisevision_gps_tools gps_device_manager_node
```

## Docker build and run
``` bash
mkdir -p wisevision_gps_tools_ws/src && cd wisevision_gps_tools_ws/src
git clone git@github.com:wise-vision/wisevision_gps_tools.git
cd wisevision_gps_tools
docker-compose up
```