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
#include "rclcpp/rclcpp.hpp"
#include "ros2_cpp_pubsub/srv/change_string.hpp"
#include "ros2_cpp_pubsub/msg/data.hpp"

#include <memory>
#include <string>
#include <gtest/gtest.h>
#include <stdlib.h>

// Typedefs declared by using to improve code readability

using my_data = ros2_cpp_pubsub::msg::Data;
using REQUEST = std::shared_ptr
                <ros2_cpp_pubsub::srv::ChangeString::Request>;
using RESPONSE = std::shared_ptr
                <ros2_cpp_pubsub::srv::ChangeString::Response>;

using NODE = rclcpp::Node;

using SERVICE = ros2_cpp_pubsub::srv::ChangeString;

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

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to manipulate string");

    rclcpp::spin(node);

    // REQUEST
    // Wait for service to connect to client
    auto service_name = "change_strings";

    // Create a client with the service name (same as in the server)
    auto client = create_client<SERVICE>(service_name);

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

    auto request = std::make_shared<REQUEST>();
    request->input = "TEST";
    client->async_send_request(request,
        [](rclcpp::Client<SERVICE>::SharedFuture future) {
            // Generate a response type
            auto response = my_datatype();

            // Get the data from the response
            response.my_data = future.get()->output.c_str();
            EXPECT_TRUE(true);
            std::cout << "DONE WITH TEST\n";
        }
    )
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    std::cout << "DONE SHUTTING DOWN ROS\n";
    return result;
}
