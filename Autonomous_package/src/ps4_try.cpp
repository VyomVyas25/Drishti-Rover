#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

class RoboticArmControl : public rclcpp::Node
{
public:
    RoboticArmControl()
    : Node("robotic_arm_control")
    {
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_1", 10);
        joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>("/ps4/joy", 10, std::bind(&RoboticArmControl::joy_callback, this, std::placeholders::_1));
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
    {
        auto twist = geometry_msgs::msg::Twist();

        // Default all motor speeds to 0
        twist.linear.x = 0.0; // Motor 1
        twist.linear.y = 0.0; // Motor 2
        twist.linear.z = 0.0; // Motor 3
        twist.angular.x = 0.0; // Motor 4
        twist.angular.y = 0.0; // Motor 5
        twist.angular.z = 0.0; // Stepper Motor

        // Check buttons and set speed based on left joystick vertical axis (axes[1])
        if (joy_msg->buttons[0]) { // Button 0 for Motor 1
            twist.linear.x = joy_msg->axes[1] * (255.0/6); // Left joystick vertical controls speed
        }
        if (joy_msg->buttons[1]) { // Button 1 for Motor 2
            twist.linear.y = joy_msg->axes[1] * 255.0; // Left joystick vertical controls speed
        }
        if (joy_msg->buttons[2]) { // Button 2 for Motor 3
            twist.linear.z = joy_msg->axes[1] * (-255.0); // Left joystick vertical controls speed
        }
        if (joy_msg->buttons[3]) { // Button 3 for Motor 4
            twist.angular.x = joy_msg->axes[1] * 255.0; // Left joystick vertical controls speed
        }
        if (joy_msg->buttons[4]) { // Button 4 for Motor 5
            twist.angular.y = joy_msg->axes[1] * 255.0; // Left joystick vertical controls speed
        }
        if (joy_msg->buttons[5]) { // Button 5 for Stepper Motor
            twist.angular.z = joy_msg->axes[1] * 255.0; // Left joystick vertical controls speed
        }

        // Publish the Twist message
        cmd_vel_publisher_->publish(twist);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoboticArmControl>());
    rclcpp::shutdown();
    return 0;
}   