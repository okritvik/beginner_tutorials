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
 * @file subscriber_member_function.cpp
 * @author Kumara Ritvik Oruganti (okritvik@umd.edu)
 * @brief Template File to subscribe data over a topic
 * @version 0.1
 * @date 2022-11-14
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ros2_cpp_pubsub/msg/data.hpp"

using my_datatype = ros2_cpp_pubsub::msg::Data;

using std::placeholders::_1;

/**
 * @brief Template class to subscribe over a topic
 * 
 */
class MinimalSubscriber : public rclcpp::Node {
 public:
 /**
  * @brief Construct a new Minimal Subscriber object
  * 
  */
  MinimalSubscriber() : Node("minimal_subscriber") {
    subscription_ = this->create_subscription<my_datatype>(
      "data", 1, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

 private:
 /**
  * @brief Call back function that gets called when there is a data published over the topic
  * 
  * @param msg 
  */
  void topic_callback(const my_datatype & msg) const {
    RCLCPP_INFO(this->get_logger(), "I received: '%s'", msg.my_data.c_str());
  }
  rclcpp::Subscription<my_datatype>::SharedPtr subscription_;
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
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
