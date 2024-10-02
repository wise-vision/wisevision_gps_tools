#include "rclcpp/rclcpp.hpp"
#include "wisevision_gps_tools/GpsDeviceManager.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<GpsDeviceManager>();

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}