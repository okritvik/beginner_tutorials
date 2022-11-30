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
#include "rclcpp/exceptions.hpp"
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

// RMW_IMPLEMENTATION definition to write gtest using test fixtures
#ifdef RMW_IMPLEMENTATION
# define CLASSNAME_(NAME, SUFFIX) NAME ## __ ## SUFFIX
# define CLASSNAME(NAME, SUFFIX) CLASSNAME_(NAME, SUFFIX)
#else
# define CLASSNAME(NAME, SUFFIX) NAME
#endif

/**
 * @brief Construct a new CLASSNAME object
 * 
 */
class CLASSNAME(test_services_client, RMW_IMPLEMENTATION) :
                                        public ::testing::Test {
    public:
        /**
         * @brief Set the Up Test Case function that initializes ros
         * 
         */
        static void SetUpTestCase() {
            rclcpp::init(0, nullptr);
        }

        /**
         * @brief Shutdown ros
         * 
         */
        static void TearDownTestCase() {
            rclcpp::shutdown();
        }
};


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
}

/**
 * @brief Call back function for subscriber
 * 
 * @param msg 
 */
void topic_callback(const my_datatype & msg) {
    std::cout << msg.my_data.c_str() << "\n\n\n\n" << std::endl;
}

/**
 * @brief Construct a new test f object that tests service
 * 
 */
TEST_F(CLASSNAME(test_services_client, RMW_IMPLEMENTATION), Manipulation1) {
    // SERVICE
    std::shared_ptr<NODE> node = NODE::make_shared("string_server1");

    rclcpp::Service<SERVICE>::SharedPtr service =
                node->create_service<SERVICE>("change_strings",  &manipulate);

    auto service_name = "change_strings";

    // Create a client with the service name (same as in the server)
    CLIENT client = node->create_client
                <ros2_cpp_pubsub::srv::ChangeString>(service_name);

    // Check if the operation is interrupted while waiting for server
    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            return;
        }
    }
    // REQUEST
    auto request = std::make_shared
            <ros2_cpp_pubsub::srv::ChangeString::Request>();
    // Request Data
    request->input = "TEST1";
    auto result = client->async_send_request(request);
    // Spin the node till there is a response from the service
    auto ret = rclcpp::spin_until_future_complete(node, result, 5s);

    // Assert if the spin until future returns a success
    ASSERT_EQ(ret, rclcpp::FutureReturnCode::SUCCESS);
    // Test if the result from the service is as expected
    EXPECT_EQ("TEST1 Manipulated by server", result.get()->output);
    RCLCPP_INFO(node->get_logger(), "DONE WITH TEST1\n");
}

/**
 * @brief Construct a new test f object that tests service
 * 
 */
TEST_F(CLASSNAME(test_services_client, RMW_IMPLEMENTATION), Manipulation2) {
    std::shared_ptr<NODE> node = NODE::make_shared("string_server2");
    // SERVICE
    rclcpp::Service<SERVICE>::SharedPtr service =
                node->create_service<SERVICE>("change_strings",  &manipulate);

    auto service_name = "change_strings";

    // Create a client with the service name (same as in the server)
    CLIENT client = node->create_client
                <ros2_cpp_pubsub::srv::ChangeString>(service_name);

    // Check if the operation is interrupted while waiting for server
    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            return;
        }
    }

    // Check if the operation is interrupted while waiting for server
    auto request = std::make_shared
            <ros2_cpp_pubsub::srv::ChangeString::Request>();
    // REQUEST
    request->input = "TEST2";
    // Request Data
    auto result = client->async_send_request(request);

    // Spin the node till there is a response from the service
    auto ret = rclcpp::spin_until_future_complete(node, result, 5s);
    // Assert if the spin until future returns a success
    ASSERT_EQ(ret, rclcpp::FutureReturnCode::SUCCESS);
    // Test if the result from the service is as expected
    EXPECT_EQ("TEST2 Manipulated by server", result.get()->output);
    RCLCPP_INFO(node->get_logger(), "DONE WITH TEST2\n");
}

/**
 * @brief Construct a new test f object that tests subscriber
 * 
 */
TEST_F(CLASSNAME(test_services_client, RMW_IMPLEMENTATION), Subscriber1) {
    std::shared_ptr<NODE> node = NODE::make_shared("subscriber");
    // Create a subscriber for /data topic
    rclcpp::Subscription<my_datatype>::SharedPtr subscription_;
    subscription_ = node->create_subscription<my_datatype>(
      "data", 1, &topic_callback);
    // Spin the node to get the data
    rclcpp::spin_some(node);
    // Pass the test if there are no errors
    EXPECT_TRUE(true);
    RCLCPP_INFO(node->get_logger(), "DONE WITH TEST3\n");
}

/**
 * @brief Initialize the ros node and start the gtests
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    std::cout << "DONE SHUTTING DOWN ROS\n";
    return result;
}
