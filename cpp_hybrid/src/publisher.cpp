#include "cpp_hybrid/ros_version.h"

#if MJ_ROS_VERSION == ROS_2

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace ros_cpp = rclcpp;

#elif MJ_ROS_VERSION == ROS_1

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

namespace ros_cpp = ros;

#endif

using namespace std::chrono_literals;

#if MJ_ROS_VERSION == ROS_2

class MinimalPublisher : public rclcpp::Node
{
    public:
     MinimalPublisher() : Node("minimal_publisher"), count_(0) {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        timer_ = this->create_wall_timer(
            500ms, std::bind(&MinimalPublisher::timer_callback, this)
        );
     }

    private:
     void timer_callback() {
        auto message = std_msgs::msg::String();
        message.data = "Hello world " + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing '%s'", message.data.c_str());
        publisher_->publish(message);
     }
    
     rclcpp::TimerBase::SharedPtr timer_;
     rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
     size_t count_;
};
#elif MJ_ROS_VERSION == ROS_1

class MinimalPublisher
{
    public:
     MinimalPublisher() : count_(0) {
        nh = ros::NodeHandle();
        publisher_ = nh.advertise<std_msgs::String>("topic", 10);
        timer_ = nh.createWallTimer(ros::WallDuration(.5), boost::bind(&MinimalPublisher::timer_callback, this));
     }

    private:
     void timer_callback() {
        auto message = std_msgs::String();
        message.data = "Hello world " + std::to_string(count_++);
        ROS_INFO("Publishing '%s'", message.data.c_str());
        publisher_.publish(message);
     }
    
     ros::NodeHandle nh;
     ros::WallTimer timer_;
     ros::Publisher publisher_;
     size_t count_;
     
};


#endif

int main(int argc, char *argv[])
{
#if MJ_ROS_VERSION == ROS_2
   ros_cpp::init(argc, argv);
#elif MJ_ROS_VERSION == ROS_1
   ros_cpp::init(argc, argv, "minimal_publisher");
#endif
   auto node = std::make_shared<MinimalPublisher>();
#if MJ_ROS_VERSION == ROS_2
   ros_cpp::spin(node);
#elif MJ_ROS_VERSION == ROS_1
   ros::spin();
#endif
   ros_cpp::shutdown();
   return 0;
}