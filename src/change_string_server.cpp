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

#include "rclcpp/rclcpp.hpp"
#include "ros2_cpp_pubsub/srv/change_string.hpp"
#include "ros2_cpp_pubsub/msg/data.hpp"

#include <memory>

using my_data = ros2_cpp_pubsub::msg::Data;
using REQUEST = std::shared_ptr
                <ros2_cpp_pubsub::srv::ChangeString::Request>;
using RESPONSE = std::shared_ptr
                <ros2_cpp_pubsub::srv::ChangeString::Response>;

using NODE = rclcpp::Node;

using SERVICE = ros2_cpp_pubsub::srv::ChangeString;


void manipulate(const REQUEST request, RESPONSE response) {
    response->output = request->input;
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\n %s",
    //             request->input.c_str());
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response:
    // [%s, %s]", response->output.c_str(), "Modified");
}

int main(int argc, char **argv) {
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%d", argc);
    rclcpp::init(argc, argv);

    std::shared_ptr<NODE> node = NODE::make_shared("string_server");

    rclcpp::Service<SERVICE>::SharedPtr service =
                node->create_service<SERVICE>("change_strings",  &manipulate);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to manipulate string");

    rclcpp::spin(node);
    rclcpp::shutdown();
}
