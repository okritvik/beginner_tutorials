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

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "ros2_cpp_pubsub/msg/data.hpp"
#include "std_msgs/msg/string.hpp"
#include "ros2_cpp_pubsub/srv/change_string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

using my_datatype = ros2_cpp_pubsub::msg::Data;

using PUBLISHER   = rclcpp::Publisher<my_datatype>::SharedPtr;
using TIMER       = rclcpp::TimerBase::SharedPtr;
using CLIENT    = rclcpp::Client<ros2_cpp_pubsub::srv::ChangeString>::SharedPtr;
using SERVICE     = ros2_cpp_pubsub::srv::ChangeString;
using REQUEST     = ros2_cpp_pubsub::srv::ChangeString::Request;
using RESPONSE    = rclcpp::Client<SERVICE>::SharedFuture;
using std::placeholders::_1;
class ServicePublisher : public rclcpp::Node {
 public:
  // Constructor initializing node and counter
  ServicePublisher() :
    Node("server_publisher"),
    count_(0) {
      publisher_ = this->create_publisher<my_datatype>("server_data", 10);
      auto call_back_ptr = std::bind(&ServicePublisher::timer_callback, this);
      timer_ = this->create_wall_timer(500ms, call_back_ptr);
      // Wait for service to connect to client
      auto service_name = "change_strings";
      client = create_client<SERVICE>(service_name);
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
  void timer_callback() {
    auto request = std::make_shared<REQUEST>();
    request->input = "HELLO 808X! count: " + std::to_string(count_);
    count_++;
    auto call_back_ptr = std::bind(&ServicePublisher::response_callback,
                          this, _1);

    client->async_send_request(request, call_back_ptr);
  }

  void response_callback(RESPONSE future) {
    auto response = my_datatype();
    response.my_data = future.get()->output.c_str();
    // RCLCPP_INFO(this->get_logger(), "Received response %s",
          // response.my_data.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing data to our new topic");

    publisher_->publish(response);
  }

  TIMER timer_;
  PUBLISHER publisher_;
  size_t count_;
  CLIENT client;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServicePublisher>());
  rclcpp::shutdown();
  return 0;
}
