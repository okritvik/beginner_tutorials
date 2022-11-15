// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file client_publisher.cpp
 * @author Kumara Ritvik Oruganti (okritvik@umd.edu)
 * @brief This file uses the server and publishes the response to a new topic
 * @version 0.1
 * @date 2022-11-14
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "ros2_cpp_pubsub/msg/data.hpp"
#include "std_msgs/msg/string.hpp"
#include "ros2_cpp_pubsub/srv/change_string.hpp"

using namespace std::chrono_literals;

// Typedefs declared by using to improve code readability

using my_datatype = ros2_cpp_pubsub::msg::Data;

using PUBLISHER   = rclcpp::Publisher<my_datatype>::SharedPtr;
using TIMER       = rclcpp::TimerBase::SharedPtr;
using CLIENT    = rclcpp::Client<ros2_cpp_pubsub::srv::ChangeString>::SharedPtr;
using SERVICE     = ros2_cpp_pubsub::srv::ChangeString;
using REQUEST     = ros2_cpp_pubsub::srv::ChangeString::Request;
using RESPONSE    = rclcpp::Client<SERVICE>::SharedFuture;
using std::placeholders::_1;

/**
 * @brief A template that uses the server and publishes the response to a new topic.
 * 
 */
class ServicePublisher : public rclcpp::Node {
 public:
  // Constructor initializing node and counter
  ServicePublisher() :
    Node("server_publisher"),
    count_(0) {
      // publisher
      publisher_ = this->create_publisher<my_datatype>("server_data", 10);

      // Pointer returned by std::bind for the call back function
      auto call_back_ptr = std::bind(&ServicePublisher::timer_callback, this);

      // Timer to publish frequency at a data rate
      timer_ = this->create_wall_timer(500ms, call_back_ptr);

      // Wait for service to connect to client
      auto service_name = "change_strings";

      // Create a client with the service name (same as in the server)
      client = create_client<SERVICE>(service_name);

      // Check if the operation is interrupted while waiting for server
      while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
          // Used one of the RCLCPP LOG Level
          RCLCPP_ERROR(this->get_logger(),
              "Interruped while waiting for the server.");

          return;
        }
        RCLCPP_INFO(this->get_logger(),
              "Server not available, waiting again...");
      }
  }

 private:
 /**
  * @brief Function that acts as a call back for making a request to the server
  * 
  */
  void timer_callback() {
    // Create a request
    auto request = std::make_shared<REQUEST>();
    request->input = "HELLO 808X! count: " + std::to_string(count_);
    count_++;

    // Call back pointer to generate a response
    auto call_back_ptr = std::bind(&ServicePublisher::response_callback,
                          this, _1);

    // Non blocking callback to receive the data
    client->async_send_request(request, call_back_ptr);
  }

/**
 * @brief Call back function to publish the response from the server
 * 
 * @param future 
 */
  void response_callback(RESPONSE future) {
    // Generate a response type
    auto response = my_datatype();

    // Get the data from the response
    response.my_data = future.get()->output.c_str();

    // RCLCPP_INFO(this->get_logger(), "Received response %s",
          // response.my_data.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing data to our new topic");

    // Publish the data
    publisher_->publish(response);
  }

  TIMER timer_;  // Timer
  PUBLISHER publisher_;  // Publisher
  size_t count_;  // Counter
  CLIENT client;  // Client
};

/**
 * @brief Main function to instantiate the nodes from the class
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServicePublisher>());
  rclcpp::shutdown();
  return 0;
}
