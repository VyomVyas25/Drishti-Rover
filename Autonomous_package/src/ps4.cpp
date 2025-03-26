#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

class PS4BotControl : public rclcpp::Node
{
public:
    PS4BotControl()
    : Node("ps4_bot_control")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>("/xbox/joy", 10, std::bind(&PS4BotControl::joy_callback, this, std::placeholders::_1));
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        auto twist = geometry_msgs::msg::Twist();
        twist.linear.x = msg->axes[1];  // Right joystick vertical axis
        twist.angular.z = msg->axes[3]; // Right joystick horizontal axis
        publisher_->publish(twist);
    }
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PS4BotControl>());
    rclcpp::shutdown();
    return 0;
}
