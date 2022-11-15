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
 * @file publisher_member_function.cpp
 * @author Kumara Ritvik Oruganti (okritvik@umd.edu)
 * @brief Template File to publish data over a topic
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
 * @brief Class that publishes data to the topic /data
 * 
 */
class MinimalPublisher : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new Minimal Publisher object
   * 
   */
  MinimalPublisher() : Node("my_publisher"), count_(0) {
    publisher_ = this->create_publisher<my_datatype>("data", 10);
    auto call_back_ptr = std::bind(&MinimalPublisher::timer_callback, this);
    timer_ = this->create_wall_timer(500ms, call_back_ptr);
  }

 private:
 /**
  * @brief Call back function that gets executed after every time delay
  * 
  */
  void timer_callback() {
    auto message = my_datatype();
    message.my_data = "Hello, this server-pub-sub is developed by okritvik! "
        + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'",
                              message.my_data.c_str());

    publisher_->publish(message);
  }
  // Variables
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<my_datatype>::SharedPtr publisher_;
  size_t count_;
};

/**
 * @brief Main function that initializes the node
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
