#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "wisevision_msgs/srv/add_gps_device.hpp"
#include "wisevision_msgs/srv/delete_gps_device.hpp"
#include "wisevision_msgs/srv/modify_gps_device.hpp"
#include "yaml-cpp/yaml.h"
#include <string>
#include <unordered_map>

using AddGpsDevice = wisevision_msgs::srv::AddGpsDevice;
using DeleteGpsDevice = wisevision_msgs::srv::DeleteGpsDevice;
using ModifyGpsDevice = wisevision_msgs::srv::ModifyGpsDevice;

class GpsDeviceManager : public rclcpp::Node {
public:
  GpsDeviceManager();

private:
  void addGpsDevice(const std::shared_ptr<AddGpsDevice::Request> request,
                    std::shared_ptr<AddGpsDevice::Response> response);

  void deleteGpsDevice(const std::shared_ptr<DeleteGpsDevice::Request> request,
                       std::shared_ptr<DeleteGpsDevice::Response> response);

  void modifyGpsDevice(const std::shared_ptr<ModifyGpsDevice::Request> request,
                       std::shared_ptr<ModifyGpsDevice::Response> response);

  void saveToYAML();
  void loadFromYAML();
  rclcpp::Service<AddGpsDevice>::SharedPtr m_add_gps_device;
  rclcpp::Service<DeleteGpsDevice>::SharedPtr m_delete_gps_device;
  rclcpp::Service<ModifyGpsDevice>::SharedPtr m_modify_gps_device;

  std::unordered_map<std::string,
                     std::pair<std::string, sensor_msgs::msg::NavSatFix>>
      m_device_map;
  std::string m_yaml_file;
};