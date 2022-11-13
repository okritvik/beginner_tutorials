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

using my_message = ros2_cpp_pubsub::msg::Data;

using PUBLISHER   = rclcpp::Publisher<my_message>::SharedPtr;
using TIMER       = rclcpp::TimerBase::SharedPtr;
using CLIENT    = rclcpp::Client<ros2_cpp_pubsub::srv::ChangeString>::SharedPtr;
using SERVICE     = ros2_cpp_pubsub::srv::ChangeString;
using REQUEST     = ros2_cpp_pubsub::srv::ChangeString::Request;
using RESPONSE    = rclcpp::Client<SERVICE>::SharedFuture;
using std::placeholders::_1;
using PARAMETER_EVENT  = std::shared_ptr<rclcpp::ParameterEventHandler>;
using PARAMETER_HANDLE = std::shared_ptr<rclcpp::ParameterCallbackHandle>;


class MinimalPublisher : public rclcpp::Node {
 public:
  // Constructor initializing node and counter
  MinimalPublisher() :
    Node("param_publisher"),
    count_(0) {
        publisher_ = this->create_publisher<my_message>("chatter", 10);
        auto call_back_ptr = std::bind(&MinimalPublisher::timer_callback, this);
        auto param = this->get_parameter("frequency");
        auto freq = param.get_parameter_value().get<std::float_t>();
        // Debug logger level
        RCLCPP_DEBUG(this->get_logger(),
                "Frequency: %f", freq);
        // Fatal logger level
        if (freq <= 0) {
            RCLCPP_FATAL(this->get_logger(),
                "Given frequency should be greater than zero!");
        }
        // Warning logger level
        if (freq > 500) {
            RCLCPP_WARN(this->get_logger(),
                "Your frequency of publishing is too high!");
        }
        m_param_subscriber_ =
                std::make_shared<rclcpp::ParameterEventHandler>(this);
        auto param_callback_ptr = std::bind(&MinimalPublisher::param_callback,
                this, _1);
        m_paramHandle_ = m_param_subscriber_->add_parameter_callback(
                    "frequency", param_callback_ptr);
        auto period = std::chrono::milliseconds(static_cast<int>(
                (1000 / freq)));
        m_timer_ = this->create_wall_timer(period, topicCallbackPtr);
  }

 private:
  void timer_callback() {
    auto message = my_message();
    message.my_data = "Hello, this param is developed by okritvik! "
        + std::to_string(count_++);
    // Debug logger level
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'",
                              message.my_data.c_str());

    publisher_->publish(message);
  }

  void param_callback(const rclcpp::Parameter & param) {
    RCLCPP_INFO(this->get_logger(),
                 "Received an update to parameter \"%s\" of type %s: %.2f",
                 param.get_name().c_str(),
                 param.get_type_name().c_str(),
                 param.as_double());

    auto period = std::chrono::milliseconds(std::static_cast<int>
        (1000 / param.as_double()));
    auto topic_callback_ptr = std::bind(&MinimalPublisher::timer_callback,
                            this);
    m_timer_ = this->create_wall_timer(period, topic_callback_ptr);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<my_message>::SharedPtr publisher_;
  size_t count_;
  PARAMETER_EVENT  m_param_subscriber_;
  PARAMETER_HANDLE m_paramHandle_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
