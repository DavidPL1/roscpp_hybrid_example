#include "cpp_hybrid/ros_version.h"

#if MJ_ROS_VERSION == ROS_2

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace ros_cpp = rclcpp;

#elif MJ_ROS_VERSION == ROS_1

#include "ros/ros.h"
#include "std_msgs/String.h"

namespace ros_cpp = ros;

#else
#error "Unsupported ROS version!"
#endif

using std::placeholders::_1;

#if MJ_ROS_VERSION == ROS_2

class MinimalSubscriber : public rclcpp::Node {
    public:
     MinimalSubscriber() : Node("minimal_subscriber") {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1)
        );
     }

    private:
     void topic_callback(const std_msgs::msg::String &msg) const {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
     }

     rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

#elif MJ_ROS_VERSION == ROS_1

class MinimalSubscriber {
    public:
     MinimalSubscriber() {
        nh = ros::NodeHandle();
        subscriber_ = nh.subscribe("topic", 10, &MinimalSubscriber::topic_callback, this);
     }

    private:
     void topic_callback(const std_msgs::String::ConstPtr &msg) const {
        ROS_INFO("I heard: '%s'", msg->data.c_str());
     }

     ros::NodeHandle nh;
     ros::Subscriber subscriber_;
};

#endif


int main(int argc, char *argv[]) {
#if MJ_ROS_VERSION == ROS_2
    ros_cpp::init(argc, argv);
#elif MJ_ROS_VERSION == ROS_1
    ros_cpp::init(argc, argv, "minimal_subscriber");
#endif
    auto node = std::make_shared<MinimalSubscriber>();
#if MJ_ROS_VERSION == ROS_2
    ros_cpp::spin(node);
#elif MJ_ROS_VERSION == ROS_1
    ros_cpp::spin();
#endif
    ros_cpp::shutdown();
    return 0;
}