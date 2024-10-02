#include "wisevision_gps_tools/GpsDeviceManager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "wisevision_msgs/srv/add_gps_device.hpp"
#include "wisevision_msgs/srv/delete_gps_device.hpp"
#include "wisevision_msgs/srv/modify_gps_device.hpp"
#include "yaml-cpp/yaml.h"
#include <fstream>

#include <array>
#include <iomanip>
#include <sstream>

std::string eui64ToString(const std::array<uint8_t, 8> &data) {
  std::stringstream ss;
  for (size_t i = 0; i < data.size(); i++) {
    ss << std::hex << std::setw(2) << std::setfill('0')
       << static_cast<int>(data[i]);
    if (i != data.size() - 1) {
      ss << ":";
    }
  }
  return ss.str();
}

GpsDeviceManager::GpsDeviceManager()
    : Node("gps_device_manager"), m_yaml_file("gps_devices.yaml") {
  this->declare_parameter<std::string>("yaml_file", m_yaml_file);
  this->get_parameter("yaml_file", m_yaml_file);

  loadFromYAML();

  m_add_gps_device = this->create_service<AddGpsDevice>(
      "add_gps_device",
      std::bind(&GpsDeviceManager::addGpsDevice, this, std::placeholders::_1,
                std::placeholders::_2));

  m_delete_gps_device = this->create_service<DeleteGpsDevice>(
      "delete_gps_device",
      std::bind(&GpsDeviceManager::deleteGpsDevice, this, std::placeholders::_1,
                std::placeholders::_2));

  m_modify_gps_device = this->create_service<ModifyGpsDevice>(
      "modify_gps_device",
      std::bind(&GpsDeviceManager::modifyGpsDevice, this, std::placeholders::_1,
                std::placeholders::_2));
}

void GpsDeviceManager::addGpsDevice(
    const std::shared_ptr<AddGpsDevice::Request> request,
    std::shared_ptr<AddGpsDevice::Response> response) {
  std::string device_eui = eui64ToString(request->device_eui.data);

  if (m_device_map.find(device_eui) != m_device_map.end()) {
    response->success = false;
    response->error = "Device already exists.";
    return;
  }

  m_device_map[device_eui] = {request->device_name, request->nav_value};
  saveToYAML();

  response->success = true;
  response->error = "";
}

void GpsDeviceManager::deleteGpsDevice(
    const std::shared_ptr<DeleteGpsDevice::Request> request,
    std::shared_ptr<DeleteGpsDevice::Response> response) {
  std::string device_eui = eui64ToString(request->device_eui.data);

  if (m_device_map.find(device_eui) == m_device_map.end()) {
    response->success = false;
    response->error = "Device not found.";
    return;
  }

  m_device_map.erase(device_eui);
  saveToYAML();

  response->success = true;
  response->error = "";
}

void GpsDeviceManager::modifyGpsDevice(
    const std::shared_ptr<ModifyGpsDevice::Request> request,
    std::shared_ptr<ModifyGpsDevice::Response> response) {
  std::string device_eui = eui64ToString(request->device_eui.data);

  if (m_device_map.find(device_eui) == m_device_map.end()) {
    response->success = false;
    response->error = "Device not found.";
    return;
  }

  m_device_map[device_eui] = {request->device_name, request->nav_value};
  saveToYAML();

  response->success = true;
  response->error = "";
}

void GpsDeviceManager::saveToYAML() {
  YAML::Emitter out;
  out << YAML::BeginMap;

  for (const auto &device : m_device_map) {
    out << YAML::Key << device.first;
    out << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "device_name" << YAML::Value << device.second.first;
    out << YAML::Key << "latitude" << YAML::Value
        << device.second.second.latitude;
    out << YAML::Key << "longitude" << YAML::Value
        << device.second.second.longitude;
    out << YAML::Key << "altitude" << YAML::Value
        << device.second.second.altitude;
    out << YAML::EndMap;
  }

  out << YAML::EndMap;

  std::ofstream fout(m_yaml_file);
  fout << out.c_str();
}

void GpsDeviceManager::loadFromYAML() {
  std::ifstream fin(m_yaml_file);
  if (!fin.good()) {
    RCLCPP_WARN(this->get_logger(),
                "YAML file not found, starting with an empty device list.");
    return;
  }

  YAML::Node data = YAML::LoadFile(m_yaml_file);
  for (const auto &node : data) {
    std::string device_eui = node.first.as<std::string>();
    auto device_info = node.second;

    std::string device_name = device_info["device_name"].as<std::string>();
    sensor_msgs::msg::NavSatFix nav_value;
    nav_value.latitude = device_info["latitude"].as<double>();
    nav_value.longitude = device_info["longitude"].as<double>();
    nav_value.altitude = device_info["altitude"].as<double>();

    m_device_map[device_eui] = {device_name, nav_value};
  }
}