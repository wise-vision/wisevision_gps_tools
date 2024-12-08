#include "rclcpp/rclcpp.hpp"
#include "wisevision_gps_tools/GpsDeviceManager.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<GpsDeviceManager>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}