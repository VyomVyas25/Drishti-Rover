#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <iostream>
using namespace std;

class xbox_node : public rclcpp::Node
{
public:
    xbox_node() : Node("xbox_controller")
    {
        // Publisher to /cmd_vel topic
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Subscription to /joy topic
        sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&xbox_node::joy_callback, this, std::placeholders::_1));
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
    {
        // Handle axes (left joystick: axes[1] and axes[2])
        auto cmd_msg = geometry_msgs::msg::Twist();
        cmd_msg.linear.x = joy_msg->axes[1]; // Forward/backward
        cmd_msg.linear.y = joy_msg->axes[3]; // Left/right

        // Handle button presses (A: buttons[0], B: buttons[1], X: buttons[3], Y: buttons[4])
        cmd_msg.linear.z = joy_msg->buttons[0];
        cmd_msg.angular.y = joy_msg->buttons[1];
        cmd_msg.angular.z = joy_msg->buttons[2];
        cmd_msg.angular.x = joy_msg->buttons[3];

        // Debug output for axes and buttons
        cout << "Axes: [" << joy_msg->axes[1] << ", " << joy_msg->axes[3] << "]" << endl;
        cout << "Buttons: A(" << joy_msg->buttons[0] << "), B(" << joy_msg->buttons[1]<< "), X(" << joy_msg->buttons[2] << "), Y(" << joy_msg->buttons[3] << ")" << endl;

        // Publish the Twist message
        pub_->publish(cmd_msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<xbox_node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}