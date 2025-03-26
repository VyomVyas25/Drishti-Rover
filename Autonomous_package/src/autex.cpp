#include <chrono>
#include <functional>
#include <memory>
#include <iostream>
#include <vector>
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using namespace std;

class IrcNode : public rclcpp::Node
{
public:
    IrcNode() : Node("ircNode"), state_(State::MOVING_FORWARD), count_(0)
    {
        subscription_0 = this->create_subscription<geometry_msgs::msg::Twist>("/arrow_info", 10, std::bind(&IrcNode::arr_detNode, this, _1));
        subscription_1 = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&IrcNode::arrowNode, this, _1));
        subscription_2 = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&IrcNode::thetaNode, this, _1));
        publisher_1 = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        timer_ = create_wall_timer(50ms, std::bind(&IrcNode::timer_callback, this));
        start_time_ = this->get_clock()->now();
        last_arrow_time_ = this->get_clock()->now(); // Initialize last_arrow_time_
    }

private:
    enum class State
    {
        MOVING_FORWARD,
        FOLLOWING_ARROW,
        ORIENTING_TO_ARROW,
        STOPPING,
        TURNING
    };

    float direction = 0, deviation = 0, linear_velocity_ = 0.5, angular_velocity_ = 0.5, angular_velocity_2 = 0.0, obj_dis;
    float sum = 0.0, goal_ang = 0.0, theta = 0.0, theta_new = 0.0;
    std::chrono::duration<float> stop_duration_ = 10s;
    float rotation_angle_ = M_PI / 2;
    rclcpp::Time start_time_;
    rclcpp::Time last_arrow_time_; // Time of the last arrow detection
    State state_;
    const float arrow_timeout_ = 0.5; // 500ms timeout for arrow detection

    void arr_detNode(const geometry_msgs::msg::Twist &message)
    {
        direction = message.linear.z;  // Arrow direction
        deviation = message.linear.y; // Deviation from the center

        // Update last arrow detection time if direction is non-zero
        if (direction != 0)
        {
            last_arrow_time_ = this->get_clock()->now();
        }
    }

    void arrowNode(const sensor_msgs::msg::LaserScan &msg)
    {
        sum = 0.0;
        float i = 0;
        for (double x : msg.ranges)
        {
            x = x > 3 || isinf(x) ? 3 : x;
            if (i < 6 || i > msg.ranges.size() - 6)
            {
                obj_dis = 1 - (x / 3.0);
                sum -= (i < 6 ? (1 - obj_dis) * i : -(1 - obj_dis) * (msg.ranges.size() - i));
            }
            i++;
        }
        linear_velocity_ = 4.5 * (0.4 - exp(exp(obj_dis) - 3)) * 0.5;
    }

    void thetaNode(const nav_msgs::msg::Odometry &msg)
    {
        theta = quatToEuler(msg.pose.pose.orientation);
    }

    double quatToEuler(const geometry_msgs::msg::Quaternion &quaternion)
    {
        tf2::Quaternion tf_quat;
        tf2::fromMsg(quaternion, tf_quat);
        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
        return yaw;
    }

    void timer_callback()
    {
        auto vel = geometry_msgs::msg::Twist();
        auto now = this->get_clock()->now();
        float elapsed_seconds = (now - start_time_).seconds();
        float orientation_error = theta_new - theta;

        switch (state_)
        {
        case State::MOVING_FORWARD:
            // Default forward movement
            vel.linear.x = std::min(linear_velocity_, 0.4f);
            orientation_error = abs(orientation_error) > 3.14 ? orientation_error - (6.28 * (abs(orientation_error) / orientation_error)) : orientation_error;
            orientation_error = ((abs(orientation_error) > 0.025) ? 0.5 * orientation_error : 0.0);
            vel.angular.z = orientation_error; // Correct orientation using yaw data

            if (direction != 0) // Arrow detected
            {
                state_ = State::FOLLOWING_ARROW;
            }
            break;

        case State::FOLLOWING_ARROW:
            vel.linear.x = std::min(linear_velocity_, 0.5f);
            vel.angular.z = deviation / 500;

            // Check if the arrow detection timed out
            if ((now - last_arrow_time_).seconds() > arrow_timeout_)
            {
                theta_new = theta;
                state_ = State::MOVING_FORWARD; // Transition back to moving forward    
            }

            if (linear_velocity_ < 0.1) // Stop moving forward
            {
                state_ = State::ORIENTING_TO_ARROW;
            }
            break;

        case State::ORIENTING_TO_ARROW:
            sum = sum / 2.5;
            vel.linear.x = 0.0;
            vel.angular.z = abs(sum) > 0.3 ? 0.3 * sum / abs(sum) : sum; // Orient perpendicular

            if (abs(sum) < 0.0001) // Perpendicular to arrow
            {
                state_ = State::STOPPING;
                start_time_ = now;
            }
            break;

        case State::STOPPING:
            vel.linear.x = 0.0;
            vel.angular.z = 0.0;

            if (elapsed_seconds >= stop_duration_.count())
            {
                state_ = State::TURNING;
                start_time_ = now;
            }
            break;

        case State::TURNING:
            vel.linear.x = 0.0;
            vel.angular.z = direction * angular_velocity_;

            if (elapsed_seconds >= (rotation_angle_ / (angular_velocity_ * 0.95)))
            {
                state_ = State::MOVING_FORWARD;
                theta_new = theta; // Update theta
                start_time_ = now;
            }
            break;
        }

        vel.angular.z = abs(vel.angular.z) < 0.03 ? 0 : 0.5 * abs(vel.angular.z) / vel.angular.z;
        vel.angular.z = abs(vel.angular.z) > 0.5 ? 0.5 * abs(vel.angular.z) / vel.angular.z : vel.angular.z;

        vel.angular.x = vel.linear.x - vel.angular.z;
        vel.angular.y = vel.linear.x + vel.angular.z;

        // Publish velocity
        publisher_1->publish(vel);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_0;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_1;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_2;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_1;
    size_t count_;
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IrcNode>());
    rclcpp::shutdown();
    return 0;
}