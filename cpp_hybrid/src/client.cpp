// defines MJ_ROS_VERSION
#include "cpp_pubsub/ros_version.h"

#if MJ_ROS_VERSION == ROS_2 // ROS 2 specific includes

#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/srv/add_two_ints.hpp"

#include <chrono>
#include <memory>

using namespace std::chrono_literals;
using namespace tutorial_interfaces::srv;

namespace ros_cpp = rclcpp; // alias namespace for common functions

// common macro names for logging
#define INFO(X, Y) RCLCPP_INFO(rclcpp::get_logger(X), Y)
#define ERROR(X, Y) RCLCPP_ERROR(rclcpp::get_logger(X), Y)
#define INFO_STREAM(x, msg) RCLCPP_INFO_STREAM(rclcpp::get_logger(x), msg)

#elif MJ_ROS_VERSION == ROS_1 // ROS 1 specific includes

#include "ros/ros.h"
#include "tutorial_interfaces/AddTwoInts.h"

namespace ros_cpp = ros; // alias namespace for common functions
using namespace tutorial_interfaces;

// common macro names for info logging
#define INFO(X, Y) ROS_INFO_NAMED(X, Y)
#define ERROR(X, Y) ROS_ERROR_NAMED(X, Y)
#define INFO_STREAM(x, msg) ROS_INFO_STREAM_NAMED(x, "" << msg)

#endif

// common include
#include <cstdlib>

int main(int argc, char **argv)
{
#if MJ_ROS_VERSION == ROS_2 // ROS 2 specific code
    rclcpp::init(argc, argv);
#elif MJ_ROS_VERSION == ROS_1 // ROS 1 specific code
    ros::init(argc, argv, "add_two_ints_client");
#endif

    if (argc != 3) {
        INFO("rclcpp", "usage: add_two_ints_client X Y");
        return 1;
    }

#if MJ_ROS_VERSION == ROS_2 // ROS 2 specific code
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_client");
    rclcpp::Client<tutorial_interfaces::srv::AddTwoInts>::SharedPtr client = node->create_client<tutorial_interfaces::srv::AddTwoInts>("add_two_ints");
#elif MJ_ROS_VERSION == ROS_1 // ROS 1 specific code
    ros::NodeHandle node;
    ros::ServiceClient client = node.serviceClient<tutorial_interfaces::AddTwoInts>("add_two_ints");
#endif


    auto request = std::make_shared<AddTwoInts::Request>();
    request->a = atoll(argv[1]);
    request->b = atoll(argv[2]);

    while (
#if MJ_ROS_VERSION == ROS_2
        !client->wait_for_service(1s)
#elif MJ_ROS_VERSION == ROS_1
        !ros::service::waitForService(client.getService(), 1.)
#endif
    ) {
        if (!ros_cpp::ok()) {
            ERROR("rclcpp", "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        INFO("rclcpp", "service not available, waiting again...");
    }

#if MJ_ROS_VERSION == ROS_2
    auto result = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
        INFO_STREAM("rclcpp", "Sum: " << result.get()->sum);
    } else {
        ERROR("rclcpp", "Failed to call service add_two_ints");
    }
#elif MJ_ROS_VERSION == ROS_1
    AddTwoInts srv;
    srv.request = *request;
    if (client.call(srv)) {
        INFO_STREAM("rclcpp", "Sum: " << srv.response.sum);
    } else {
        ERROR("rclcpp", "Failed to call service add_two_ints");
    }
#endif

    ros_cpp::shutdown();
    return 0;
}