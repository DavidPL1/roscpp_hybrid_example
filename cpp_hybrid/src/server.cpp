#include "cpp_hybrid/ros_version.h"

#if MJ_ROS_VERSION == ROS_2

#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/srv/add_two_ints.hpp"

#include <memory>

namespace ros_cpp = rclcpp;

#define INFO(X, Y) RCLCPP_INFO(rclcpp::get_logger(X), Y)

#elif MJ_ROS_VERSION == ROS_1

#include "ros/ros.h"
#include "tutorial_interfaces/AddTwoInts.h"

namespace ros_cpp = ros;

#define INFO(X, Y) ROS_INFO_NAMED(X, Y)

#endif

// common core library function
int core_add(int a, int b) {
    return a + b;
}

#if MJ_ROS_VERSION == ROS_2

void add(const std::shared_ptr<tutorial_interfaces::srv::AddTwoInts::Request> request,
         std::shared_ptr<tutorial_interfaces::srv::AddTwoInts::Response> response) {
    response->sum = core_add(request->a, request->b);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld", request->a, request->b);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
}

#elif MJ_ROS_VERSION == ROS_1

bool add(tutorial_interfaces::AddTwoInts::Request &req,
         tutorial_interfaces::AddTwoInts::Response &res) {
    res.sum = core_add(req.a, req.b);
    ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
    ROS_INFO("sending back response: [%ld]", (long int)res.sum);

    return true;
}

#endif

int main(int argc, char **argv)
{
#if MJ_ROS_VERSION == ROS_2
    ros_cpp::init(argc, argv);
    auto node = ros_cpp::Node::make_shared("add_two_ints_server");
    auto service = node->create_service<tutorial_interfaces::srv::AddTwoInts>("add_two_ints", &add);
    RCLCPP_INFO(ros_cpp::get_logger("rclcpp"), "Ready to add two ints.");
    ros_cpp::spin(node);
#elif MJ_ROS_VERSION == ROS_1
    ros::init(argc, argv, "add_two_ints_server");
    ros::NodeHandle node;
    auto service = node.advertiseService("add_two_ints", add);
    ROS_INFO("Ready to add two ints.");
    ros::spin();
#endif

    // common macro for logging
    INFO("rclcpp", "Shutting down server");

    ros_cpp::shutdown();
}