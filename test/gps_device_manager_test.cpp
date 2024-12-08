#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "wisevision_gps_tools/GpsDeviceManager.hpp"
#include "wisevision_msgs/srv/add_data_to_data_base.hpp"
#include "wisevision_msgs/srv/add_gps_device.hpp"
#include "wisevision_msgs/srv/delete_gps_device.hpp"
#include "wisevision_msgs/srv/modify_gps_device.hpp"

namespace fs = std::filesystem;

class GpsDeviceManagerTest : public ::testing::Test {
protected:
  std::string config_file_path = "gps_devices.yaml";
  void SetUp() override {
    rclcpp::init(0, nullptr);

    if (fs::exists(config_file_path)) {
      fs::remove(config_file_path);
    }

    manager_executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    manager_node = std::make_shared<GpsDeviceManager>();
    manager_executor->add_node(manager_node);

    manager_executor_thread = std::make_shared<std::thread>([this]() { manager_executor->spin(); });

    mock_service_node = rclcpp::Node::make_shared("mock_service_node");
    mock_service_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    mock_service_executor->add_node(mock_service_node);

    mock_add_data_to_data_base_service = mock_service_node->create_service<wisevision_msgs::srv::AddDataToDataBase>(
        "add_data_to_database",
        [this](const std::shared_ptr<wisevision_msgs::srv::AddDataToDataBase::Request> /*request*/,
               std::shared_ptr<wisevision_msgs::srv::AddDataToDataBase::Response> response) {
          response->success = true; // Simulate a successful response
        });

    mock_service_executor_thread = std::make_shared<std::thread>([this]() { mock_service_executor->spin(); });
  }

  void TearDown() override {
    manager_executor->cancel();
    manager_executor_thread->join();

    mock_service_executor->cancel();
    mock_service_executor_thread->join();

    rclcpp::shutdown();
  }

  rclcpp::executors::MultiThreadedExecutor::SharedPtr manager_executor;
  std::shared_ptr<GpsDeviceManager> manager_node;
  std::shared_ptr<std::thread> manager_executor_thread;

  rclcpp::Node::SharedPtr mock_service_node;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr mock_service_executor;
  std::shared_ptr<std::thread> mock_service_executor_thread;
  rclcpp::Service<wisevision_msgs::srv::AddDataToDataBase>::SharedPtr mock_add_data_to_data_base_service;
};

TEST_F(GpsDeviceManagerTest, AddGpsDeviceSuccess) {
  auto client = manager_node->create_client<wisevision_msgs::srv::AddGpsDevice>("add_gps_device");

  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(10))) << "Service not available after waiting.";

  auto request = std::make_shared<wisevision_msgs::srv::AddGpsDevice::Request>();
  request->device_name = "Test Device";
  request->device_eui.data = {0x01, 0x02, 0x03, 0x04, 0x05, 0x09, 0x07, 0x08};
  request->nav_value.latitude = 52.2296756;
  request->nav_value.longitude = 21.0122287;
  request->nav_value.altitude = 100;

  auto future = client->async_send_request(request);

  if (future.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
    auto response = future.get();
    EXPECT_TRUE(response->success) << "Failed to add GPS device.";
  } else {
    FAIL() << "Failed to add GPS device within the timeout.";
  }
}

TEST_F(GpsDeviceManagerTest, AddGpsDeviceEmpty) {
  auto client = manager_node->create_client<wisevision_msgs::srv::AddGpsDevice>("add_gps_device");

  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(10))) << "Service not available after waiting.";
  auto request = std::make_shared<wisevision_msgs::srv::AddGpsDevice::Request>();
  request->device_name = "Test Device";
  request->nav_value.latitude = 52.2296756;
  request->nav_value.longitude = 21.0122287;
  request->nav_value.altitude = 100;

  auto future = client->async_send_request(request);

  if (future.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
    auto response = future.get();
    EXPECT_FALSE(response->success) << "Successfully added GPS device with empty EUI.";
  } else {
    FAIL() << "Failed to add GPS device within the timeout.";
  }
  auto request_2 = std::make_shared<wisevision_msgs::srv::AddGpsDevice::Request>();

  request_2->device_eui.data = {0x01, 0x02, 0x03, 0x04, 0x05, 0x09, 0x07, 0x08};
  request_2->nav_value.latitude = 52.2296756;
  request_2->nav_value.longitude = 21.0122287;
  request_2->nav_value.altitude = 100;

  future = client->async_send_request(request_2);

  if (future.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
    auto response = future.get();
    EXPECT_FALSE(response->success) << "Successfully added GPS device with empty Name.";
  } else {
    FAIL() << "Failed to add GPS device within the timeout.";
  }
}

TEST_F(GpsDeviceManagerTest, AddGpsDeviceBadGpsValues) {
  auto client = manager_node->create_client<wisevision_msgs::srv::AddGpsDevice>("add_gps_device");

  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(10))) << "Service not available after waiting.";

  auto request = std::make_shared<wisevision_msgs::srv::AddGpsDevice::Request>();
  request->device_name = "Test Device";
  request->device_eui.data = {0x01, 0x02, 0x03, 0x04, 0x05, 0x09, 0x07, 0x08};
  request->nav_value.latitude = 91;
  request->nav_value.longitude = 181;
  request->nav_value.altitude = 100;

  auto future = client->async_send_request(request);

  if (future.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
    auto response = future.get();
    EXPECT_FALSE(response->success) << "Successfully added GPS device with bad values og GPS.";
  } else {
    FAIL() << "Failed to add GPS device within the timeout.";
  }
}

TEST_F(GpsDeviceManagerTest, AddGpsDeviceDuplicate) {
  auto client = manager_node->create_client<wisevision_msgs::srv::AddGpsDevice>("add_gps_device");

  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(10))) << "Service not available after waiting.";

  auto request = std::make_shared<wisevision_msgs::srv::AddGpsDevice::Request>();
  request->device_name = "Test Device";
  request->device_eui.data = {0x01, 0x02, 0x03, 0x04, 0x05, 0x09, 0x07, 0x08};
  request->nav_value.latitude = 52.2296756;
  request->nav_value.longitude = 21.0122287;
  request->nav_value.altitude = 100;

  auto future = client->async_send_request(request);

  if (future.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
    auto response = future.get();
    EXPECT_TRUE(response->success) << "Failed to add firts GPS device.";
  } else {
    FAIL() << "Failed to add GPS device within the timeout.";
  }

  auto request_2 = std::make_shared<wisevision_msgs::srv::AddGpsDevice::Request>();
  request_2->device_name = "Test Device";
  request_2->device_eui.data = {0x01, 0x02, 0x03, 0x04, 0x05, 0x09, 0x07, 0x08};
  request_2->nav_value.latitude = 52.2296756;
  request_2->nav_value.longitude = 21.0122287;
  request_2->nav_value.altitude = 100;

  future = client->async_send_request(request_2);

  if (future.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
    auto response_2 = future.get();
    EXPECT_FALSE(response_2->success) << "Failed to add second GPS device.";
  } else {
    FAIL() << "Failed to add GPS device within the timeout.";
  }
}

TEST_F(GpsDeviceManagerTest, DeleteGpsDeviceNotSuccess) {
  auto client = manager_node->create_client<wisevision_msgs::srv::DeleteGpsDevice>("delete_gps_device");

  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(10))) << "Service not available after waiting.";

  auto request = std::make_shared<wisevision_msgs::srv::DeleteGpsDevice::Request>();
  request->device_eui.data = {0x01, 0x02, 0x03, 0x04, 0x05, 0x09, 0x07, 0x08};

  auto future = client->async_send_request(request);

  if (future.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
    auto response = future.get();
    EXPECT_FALSE(response->success) << "Failed to get error for delete GPS device, which doesn't exist.";
  } else {
    FAIL() << "Failed to add GPS device within the timeout.";
  }
}

TEST_F(GpsDeviceManagerTest, ModifyGpsDeviceNotSuccess) {
  auto client = manager_node->create_client<wisevision_msgs::srv::ModifyGpsDevice>("modify_gps_device");

  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(10))) << "Service not available after waiting.";

  auto request = std::make_shared<wisevision_msgs::srv::ModifyGpsDevice::Request>();
  request->device_eui.data = {0x01, 0x02, 0x03, 0x04, 0x05, 0x09, 0x07, 0x08};

  auto future = client->async_send_request(request);

  if (future.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
    auto response = future.get();
    EXPECT_FALSE(response->success) << "Failed to get error for modify GPS device, which doesn't exist.";
  } else {
    FAIL() << "Failed to add GPS device within the timeout.";
  }
}
