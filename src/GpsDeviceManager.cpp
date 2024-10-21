#include "wisevision_gps_tools/GpsDeviceManager.hpp"

GpsDeviceManager::GpsDeviceManager() : Node("gps_device_manager"), m_yaml_file("gps_devices.yaml") {

  this->declare_parameter<std::string>("yaml_file", m_yaml_file);
  this->get_parameter("yaml_file", m_yaml_file);

  m_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  loadFromYAML();

  m_add_gps_device = this->create_service<AddGpsDevice>(
      "add_gps_device",
      std::bind(&GpsDeviceManager::addGpsDevice, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default,
      m_callback_group);

  m_delete_gps_device = this->create_service<DeleteGpsDevice>(
      "delete_gps_device",
      std::bind(&GpsDeviceManager::deleteGpsDevice, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default,
      m_callback_group);

  m_modify_gps_device = this->create_service<ModifyGpsDevice>(
      "modify_gps_device",
      std::bind(&GpsDeviceManager::modifyGpsDevice, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default,
      m_callback_group);

  m_add_data_to_data_base_client = this->create_client<AddDataToDataBase>("add_data_to_database",
                                                                          rmw_qos_profile_services_default,
                                                                          m_callback_group);

  m_delete_data_from_data_base_client = this->create_client<DeleteDataFromDataBase>("delete_data_from_database",
                                                                                    rmw_qos_profile_services_default,
                                                                                    m_callback_group);

  initializeGpsPublisher();
}

bool isDeviceEuiEmpty(const std::array<uint8_t, 8>& eui) {
  return std::all_of(eui.begin(), eui.end(), [](uint8_t byte) { return byte == 0; });
}

void GpsDeviceManager::addGpsDevice(const std::shared_ptr<AddGpsDevice::Request> request,
                                    std::shared_ptr<AddGpsDevice::Response> response) {

  std::string device_eui = eui64ToString(request->device_eui.data);

  if (isDeviceEuiEmpty(request->device_eui.data)) {
    response->success = false;
    response->error = "Device EUI cannot be empty or all zeros.";
    return;
  }

  if (request->device_eui.data.empty()) {
    response->success = false;
    response->error = "Device EUI cannot be empty.";
    return;
  }
  if (request->device_name.empty()) {
    response->success = false;
    response->error = "Device name cannot be empty.";
    return;
  }
  if (request->nav_value.altitude < -1000 || request->nav_value.altitude > 10000) {
    response->success = false;
    response->error = "Altitude must be between -1000 and 10000 meters.";
    return;
  }
  if (request->nav_value.latitude < -90 || request->nav_value.latitude > 90) {
    response->success = false;
    response->error = "Latitude must be between -90 and 90 degrees.";
    return;
  }
  if (request->nav_value.longitude < -180 || request->nav_value.longitude > 180) {
    response->success = false;
    response->error = "Longitude must be between -180 and 180 degrees.";
    return;
  }
  if (m_device_map.find(device_eui) != m_device_map.end()) {
    response->success = false;
    response->error = "Device already exists.";
    return;
  }

  m_device_map[device_eui] = {request->device_name, request->nav_value};
  saveToYAML();

  nlohmann::json location = {{"latitude", request->nav_value.latitude},
                             {"longitude", request->nav_value.longitude},
                             {"altitude", request->nav_value.altitude}};

  nlohmann::json query = {{"device_name", request->device_name}, {"location", location}};

  auto add_data_request = std::make_shared<AddDataToDataBase::Request>();
  add_data_request->db_path = "devices_data/" + device_eui;
  add_data_request->query = query.dump();

  if (!m_add_data_to_data_base_client->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(this->get_logger(), "Service add_data_to_database not available.");
    response->success = false;
    response->error = "Service not available.";
    return;
  }
  std::unique_lock<std::mutex> lock(m_mutex);

  auto response_received_callback = [this, response](rclcpp::Client<AddDataToDataBase>::SharedFuture future) {
    auto result = future.get();
    {
      std::lock_guard<std::mutex> guard(m_mutex);
      if (result->success) {
        RCLCPP_INFO(this->get_logger(), "Successfully added data to database");
        response->success = true;
        response->error = "";
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to add data to the database");
        response->success = false;
        response->error = "Failed to add data to the database.";
      }
    }
    m_cv.notify_one();
  };

  m_add_data_to_data_base_client->async_send_request(add_data_request, response_received_callback);

  m_cv.wait(lock);
}

void GpsDeviceManager::deleteGpsDevice(const std::shared_ptr<DeleteGpsDevice::Request> request,
                                       std::shared_ptr<DeleteGpsDevice::Response> response) {

  std::string device_eui = eui64ToString(request->device_eui.data);

  if (m_device_map.find(device_eui) == m_device_map.end()) {
    response->success = false;
    response->error = "Device not found.";
    return;
  }

  m_device_map.erase(device_eui);
  saveToYAML();

  auto delete_data_request = std::make_shared<DeleteDataFromDataBase::Request>();
  delete_data_request->db_path = "devices_data/" + device_eui;

  std::unique_lock<std::mutex> lock(m_mutex);

  auto response_received_callback = [this, response](rclcpp::Client<DeleteDataFromDataBase>::SharedFuture future) {
    auto result = future.get();
    {
      std::lock_guard<std::mutex> guard(m_mutex);
      if (result->success) {
        RCLCPP_INFO(this->get_logger(), "Successfully deleted data from database");
        response->success = true;
        response->error = "";
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to delete data from the database");
        response->success = false;
        response->error = "Failed to delete data from the database.";
      }
    }
    m_cv.notify_one();
  };

  m_delete_data_from_data_base_client->async_send_request(delete_data_request, response_received_callback);

  m_cv.wait(lock);
}

void GpsDeviceManager::modifyGpsDevice(const std::shared_ptr<ModifyGpsDevice::Request> request,
                                       std::shared_ptr<ModifyGpsDevice::Response> response) {

  std::string device_eui = eui64ToString(request->device_eui.data);

  if (m_device_map.find(device_eui) == m_device_map.end()) {
    response->success = false;
    response->error = "Device not found.";
    return;
  }
  if (request->nav_value.altitude < -1000 || request->nav_value.altitude > 10000) {
    response->success = false;
    response->error = "Altitude must be between -1000 and 10000 meters.";
    return;
  }
  if (request->nav_value.latitude < -90 || request->nav_value.latitude > 90) {
    response->success = false;
    response->error = "Latitude must be between -90 and 90 degrees.";
    return;
  }
  if (request->nav_value.longitude < -180 || request->nav_value.longitude > 180) {
    response->success = false;
    response->error = "Longitude must be between -180 and 180 degrees.";
    return;
  }

  m_device_map[device_eui] = {request->device_name, request->nav_value};
  saveToYAML();

  nlohmann::json location = {{"latitude", request->nav_value.latitude},
                             {"longitude", request->nav_value.longitude},
                             {"altitude", request->nav_value.altitude}};

  nlohmann::json query = {{"device_name", request->device_name}, {"location", location}};

  auto add_data_request = std::make_shared<AddDataToDataBase::Request>();
  add_data_request->db_path = "devices_data/" + device_eui;
  add_data_request->query = query.dump();

  std::unique_lock<std::mutex> lock(m_mutex);

  auto response_received_callback = [this, response](rclcpp::Client<AddDataToDataBase>::SharedFuture future) {
    auto result = future.get();
    {
      std::lock_guard<std::mutex> guard(m_mutex);
      if (result->success) {
        RCLCPP_INFO(this->get_logger(), "Successfully updated data in the database");
        response->success = true;
        response->error = "";
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to update data in the database");
        response->success = false;
        response->error = "Failed to update data in the database.";
      }
    }
    m_cv.notify_one();
  };

  m_add_data_to_data_base_client->async_send_request(add_data_request, response_received_callback);

  m_cv.wait(lock);
}

void GpsDeviceManager::initializeGpsPublisher() {
  m_gps_publisher = this->create_publisher<wisevision_msgs::msg::GpsDevicesPublisher>("gps_devices_data", 10);

  m_get_messages_client =
      this->create_client<GetMessages>("/get_messages", rmw_qos_profile_services_default, m_callback_group);

  m_timer = this->create_wall_timer(
      std::chrono::seconds(60),
      [this]() {
        auto request = std::make_shared<GetMessages::Request>();
        request->topic_name = "devices_data";
        request->message_type = "wisevision_msgs/GpsDevicesPublisher";

        auto response_received_callback = [this](rclcpp::Client<GetMessages>::SharedFuture future) {
          auto result = future.get();
          if (result) {
            wisevision_msgs::msg::GpsDevicesPublisher msg;
            msg.devices_data = result->gps_devices_data;
            m_gps_publisher->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Published GPS devices data.");
          } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to retrieve data from service.");
          }
        };

        m_get_messages_client->async_send_request(request, response_received_callback);
      },
      m_callback_group);
}

void GpsDeviceManager::saveToYAML() {
  YAML::Emitter out;
  out << YAML::BeginMap;

  for (const auto& device : m_device_map) {
    out << YAML::Key << device.first;
    out << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "device_name" << YAML::Value << device.second.first;
    out << YAML::Key << "latitude" << YAML::Value << device.second.second.latitude;
    out << YAML::Key << "longitude" << YAML::Value << device.second.second.longitude;
    out << YAML::Key << "altitude" << YAML::Value << device.second.second.altitude;
    out << YAML::EndMap;
  }

  out << YAML::EndMap;

  std::ofstream fout(m_yaml_file);
  if (!fout) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open YAML file for writing.");
    return;
  }
  fout << out.c_str();
}

void GpsDeviceManager::loadFromYAML() {
  std::ifstream fin(m_yaml_file);
  if (!fin.good()) {
    RCLCPP_WARN(this->get_logger(), "YAML file not found, starting with an empty device list.");
    return;
  }

  try {
    YAML::Node data = YAML::LoadFile(m_yaml_file);
    for (const auto& node : data) {
      std::string device_eui = node.first.as<std::string>();
      auto device_info = node.second;

      std::string device_name = device_info["device_name"].as<std::string>();
      sensor_msgs::msg::NavSatFix nav_value;
      nav_value.latitude = device_info["latitude"].as<double>();
      nav_value.longitude = device_info["longitude"].as<double>();
      nav_value.altitude = device_info["altitude"].as<double>();

      m_device_map[device_eui] = {device_name, nav_value};
    }
  } catch (const YAML::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to parse YAML file: %s", e.what());
  }
}