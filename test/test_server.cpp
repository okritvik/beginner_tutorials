/**
 * @file test_server.cpp
 * @author Kumara Ritvik Oruganti (okritvik@umd.edu)
 * @brief Unit Testing for the Service
 * @version 0.1
 * @date 2022-11-29
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <memory>
#include <string>
#include <chrono>
#include <functional>

#include <gtest/gtest.h>
#include <stdlib.h>

#include "rclcpp/rclcpp.hpp"
#include "ros2_cpp_pubsub/srv/change_string.hpp"
#include "std_msgs/msg/string.hpp"
#include "ros2_cpp_pubsub/msg/data.hpp"

// Typedefs declared by using to improve code readability

using my_data = ros2_cpp_pubsub::msg::Data;
using REQUEST = std::shared_ptr
                <ros2_cpp_pubsub::srv::ChangeString::Request>;
using RESPONSE = std::shared_ptr
                <ros2_cpp_pubsub::srv::ChangeString::Response>;

using NODE = rclcpp::Node;

using SERVICE = ros2_cpp_pubsub::srv::ChangeString;

using namespace std::chrono_literals;
using my_datatype = ros2_cpp_pubsub::msg::Data;
using std::placeholders::_1;
using PUBLISHER   = rclcpp::Publisher<my_datatype>::SharedPtr;
using TIMER       = rclcpp::TimerBase::SharedPtr;
using CLIENT    = rclcpp::Client<ros2_cpp_pubsub::srv::ChangeString>::SharedPtr;

/**
 * @brief This function manipulates the input from client request
 * 
 * @param request Given by the client
 * @param response Given by the server to the client
 */
void manipulate(const REQUEST request, RESPONSE response) {
    auto input_str = static_cast<std::string>(request->input.c_str());
    auto add_str = " Manipulated by server";
    response->output = input_str+add_str;
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\n %s",
    //             request->input.c_str());
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response:
    // [%s, %s]", response->output.c_str(), "Modified");
}

TEST(TestServer, Manipulation1) {
    // rclcpp::init(0, nullptr);
    // SERVICE
    std::shared_ptr<NODE> node = NODE::make_shared("string_server");

    rclcpp::Service<SERVICE>::SharedPtr service =
                node->create_service<SERVICE>("change_strings",  &manipulate);

    // REQUEST
    // Wait for service to connect to client
    auto service_name = "change_strings";

    // Create a client with the service name (same as in the server)
    CLIENT client = node->create_client
                <ros2_cpp_pubsub::srv::ChangeString>(service_name);

    // Check if the operation is interrupted while waiting for server
    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            // Used one of the RCLCPP LOG Level
            // RCLCPP_ERROR(this->get_logger(),
                // "Interruped while waiting for the server.");

            return;
        }
        // RCLCPP_INFO(this->get_logger(),
        //         "Server not available, waiting again...");
    }

    auto request = std::make_shared
            <ros2_cpp_pubsub::srv::ChangeString::Request>();
    request->input = "TEST";
    client->async_send_request(request,
        [](rclcpp::Client<SERVICE>::SharedFuture future) {
            // Generate a response type
            auto response = my_datatype();

            // Get the data from the response
            response.my_data = future.get()->output.c_str();
            auto req_result = "TEST Manipulated by server";
            EXPECT_TRUE(response.my_data == req_result);
            std::cout << "DONE WITH TEST\n";
        });
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    std::cout << "DONE SHUTTING DOWN ROS\n";
    return result;
}
