#include "wisevision_gps_tools/GpsDeviceManager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "wisevision_msgs/srv/add_gps_device.hpp"
#include "wisevision_msgs/srv/delete_gps_device.hpp"
#include "wisevision_msgs/srv/modify_gps_device.hpp"
#include "yaml-cpp/yaml.h"
#include <array>
#include <fstream>
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

  m_callback_group =
      this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  loadFromYAML();

  m_add_gps_device = this->create_service<AddGpsDevice>(
      "add_gps_device",
      std::bind(&GpsDeviceManager::addGpsDevice, this, std::placeholders::_1,
                std::placeholders::_2),
      rmw_qos_profile_services_default, m_callback_group);

  m_delete_gps_device = this->create_service<DeleteGpsDevice>(
      "delete_gps_device",
      std::bind(&GpsDeviceManager::deleteGpsDevice, this, std::placeholders::_1,
                std::placeholders::_2),
      rmw_qos_profile_services_default, m_callback_group);

  m_modify_gps_device = this->create_service<ModifyGpsDevice>(
      "modify_gps_device",
      std::bind(&GpsDeviceManager::modifyGpsDevice, this, std::placeholders::_1,
                std::placeholders::_2),
      rmw_qos_profile_services_default, m_callback_group);

  m_add_data_to_data_base_client = this->create_client<AddDataToDataBase>(
      "add_data_to_database", rmw_qos_profile_services_default,
      m_callback_group);
  m_delete_data_from_data_base_client =
      this->create_client<DeleteDataFromDataBase>(
          "delete_data_from_database", rmw_qos_profile_services_default,
          m_callback_group);
  initializeGpsPublisher();
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

  auto add_data_to_data_base_request =
      std::make_shared<wisevision_msgs::srv::AddDataToDataBase::Request>();
  add_data_to_data_base_request->db_path = "devices_data/" + device_eui;
  add_data_to_data_base_request->query =
      "device_name=" + request->device_name +
      "&location={latitude:" + std::to_string(request->nav_value.latitude) +
      ",longitude:" + std::to_string(request->nav_value.longitude) +
      ",altitude:" + std::to_string(request->nav_value.altitude) + "}";

  std::unique_lock<std::mutex> lock(m_mutex);

  auto response_received_callback =
      [this, response](rclcpp::Client<AddDataToDataBase>::SharedFuture future) {
        auto result = future.get();
        {
          std::lock_guard<std::mutex> guard(m_mutex);
          if (result->success) {
            RCLCPP_INFO(this->get_logger(),
                        "Successfully added data to database");
            response->success = true;
            response->error = "";
          } else {
            RCLCPP_ERROR(this->get_logger(),
                         "Failed to add data to the database");
            response->success = false;
            response->error = "Failed to add data to the database.";
          }
        }
        m_cv.notify_one();
      };

  m_add_data_to_data_base_client->async_send_request(
      add_data_to_data_base_request, response_received_callback);

  m_cv.wait(lock);
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

  auto delete_data_base_request =
      std::make_shared<DeleteDataFromDataBase::Request>();
  delete_data_base_request->db_path = "devices_data/" + device_eui;

  std::unique_lock<std::mutex> lock(m_mutex);

  auto response_received_callback =
      [this,
       response](rclcpp::Client<DeleteDataFromDataBase>::SharedFuture future) {
        auto result = future.get();
        {
          std::lock_guard<std::mutex> guard(m_mutex);
          if (result->success) {
            RCLCPP_INFO(this->get_logger(),
                        "Successfully deleted data from database");
            response->success = true;
            response->error = "";
          } else {
            RCLCPP_ERROR(this->get_logger(),
                         "Failed to delete data from the database");
            response->success = false;
            response->error = "Failed to delete data from the database.";
          }
        }
        m_cv.notify_one();
      };

  m_delete_data_from_data_base_client->async_send_request(
      delete_data_base_request, response_received_callback);

  m_cv.wait(lock);
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

  auto add_data_to_data_base_request =
      std::make_shared<AddDataToDataBase::Request>();
  add_data_to_data_base_request->db_path = "devices_data/" + device_eui;
  add_data_to_data_base_request->query =
      "device_name=" + request->device_name +
      "&location={latitude:" + std::to_string(request->nav_value.latitude) +
      ",longitude:" + std::to_string(request->nav_value.longitude) +
      ",altitude:" + std::to_string(request->nav_value.altitude) + "}";

  std::unique_lock<std::mutex> lock(m_mutex);

  auto response_received_callback =
      [this, response](rclcpp::Client<AddDataToDataBase>::SharedFuture future) {
        auto result = future.get();
        {
          std::lock_guard<std::mutex> guard(m_mutex);
          if (result->success) {
            RCLCPP_INFO(this->get_logger(),
                        "Successfully updated data in the database");
            response->success = true;
            response->error = "";
          } else {
            RCLCPP_ERROR(this->get_logger(),
                         "Failed to update data in the database");
            response->success = false;
            response->error = "Failed to update data in the database.";
          }
        }
        m_cv.notify_one();
      };

  m_add_data_to_data_base_client->async_send_request(
      add_data_to_data_base_request, response_received_callback);

  m_cv.wait(lock);
}

void GpsDeviceManager::initializeGpsPublisher() {
  m_gps_publisher =
      this->create_publisher<wisevision_msgs::msg::GpsDevicesPublisher>(
          "gps_devices_data", 10);

  m_get_messages_client = this->create_client<GetMessages>(
      "/get_messages", rmw_qos_profile_services_default, m_callback_group);

  m_timer = this->create_wall_timer(
      std::chrono::seconds(60),
      [this]() {
        std::unique_lock<std::mutex> lock(m_mutex);

        auto request = std::make_shared<GetMessages::Request>();
        request->topic_name = "devices_data";
        request->message_type = "wisevision_msgs/GpsDevicesPublisher";

        auto response_received_callback =
            [this](rclcpp::Client<GetMessages>::SharedFuture future) {
              auto result = future.get();
              {
                std::lock_guard<std::mutex> guard(m_mutex);
                if (result) {
                  wisevision_msgs::msg::GpsDevicesPublisher msg;
                  msg.devices_data = result->gps_devices_data;
                  m_gps_publisher->publish(msg);
                  RCLCPP_INFO(this->get_logger(),
                              "Published GPS devices data.");
                } else {
                  RCLCPP_ERROR(this->get_logger(),
                               "Failed to retrieve data from service.");
                }
              }
              m_cv.notify_one();
            };

        std::thread([this, request, response_received_callback]() {
          m_get_messages_client->async_send_request(request,
                                                    response_received_callback);
        }).detach();

        m_cv.wait(lock);
      },
      m_callback_group);
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