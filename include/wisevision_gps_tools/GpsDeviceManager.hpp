#pragma once

#include "lora_msgs/srv/get_messages.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "wisevision_msgs/msg/gps_devices_publisher.hpp"
#include "wisevision_msgs/srv/add_data_to_data_base.hpp"
#include "wisevision_msgs/srv/add_gps_device.hpp"
#include "wisevision_msgs/srv/delete_data_from_data_base.hpp"
#include "wisevision_msgs/srv/delete_gps_device.hpp"
#include "wisevision_msgs/srv/modify_gps_device.hpp"
#include "yaml-cpp/yaml.h"
#include <array>
#include <condition_variable>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_map>

using AddGpsDevice = wisevision_msgs::srv::AddGpsDevice;
using DeleteGpsDevice = wisevision_msgs::srv::DeleteGpsDevice;
using ModifyGpsDevice = wisevision_msgs::srv::ModifyGpsDevice;
using AddDataToDataBase = wisevision_msgs::srv::AddDataToDataBase;
using DeleteDataFromDataBase = wisevision_msgs::srv::DeleteDataFromDataBase;
using GpsDevicesPublisher = wisevision_msgs::msg::GpsDevicesPublisher;
using GetMessages = lora_msgs::srv::GetMessages;

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

  void initializeGpsPublisher();

  void saveToYAML();
  void loadFromYAML();
  rclcpp::Service<AddGpsDevice>::SharedPtr m_add_gps_device;
  rclcpp::Service<DeleteGpsDevice>::SharedPtr m_delete_gps_device;
  rclcpp::Service<ModifyGpsDevice>::SharedPtr m_modify_gps_device;
  rclcpp::Client<AddDataToDataBase>::SharedPtr m_add_data_to_data_base_client;
  rclcpp::Client<DeleteDataFromDataBase>::SharedPtr
      m_delete_data_from_data_base_client;
  rclcpp::Client<GetMessages>::SharedPtr m_get_messages_client;
  rclcpp::Publisher<GpsDevicesPublisher>::SharedPtr m_gps_publisher;
  rclcpp::TimerBase::SharedPtr m_timer;

  std::unordered_map<std::string,
                     std::pair<std::string, sensor_msgs::msg::NavSatFix>>
      m_device_map;
  std::string m_yaml_file;

  std::mutex m_mutex;
  std::condition_variable m_cv;
  rclcpp::CallbackGroup::SharedPtr m_callback_group;
};