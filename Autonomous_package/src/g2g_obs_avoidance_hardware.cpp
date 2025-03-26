#include <iostream>
#include <vector>
#include <bits/stdc++.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using namespace std;

class rover_pubsub : public rclcpp::Node
{
public:
  rover_pubsub() : Node("roverNode")
  {
    gps_subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("/fix", 10, std::bind(&rover_pubsub::gps_callback, this, std::placeholders::_1));
    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu", 10, std::bind(&rover_pubsub::imu_callback, this, std::placeholders::_1));
    LiDAR_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&rover_pubsub::avoid_obs, this, std::placeholders::_1));
    vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    timer_ = this->create_wall_timer(40ms, std::bind(&rover_pubsub::timer_callback, this));
    set_goal();
  }
  vector<float> scan;

private:
  double x_self, y_self, theta_self, x_goal, y_goal, original_g_dis, g_dis, angle, g_angle, obs_lin, g_lin, lin_vel, goal_ang, obs_ang;
  long double x, obj_dis, max, i, sum;
  void gps_callback(const sensor_msgs::msg::NavSatFix msg)
  {
    x_self = msg.latitude;
    y_self = msg.longitude;
  }

  void imu_callback(const sensor_msgs::msg::Imu &msg)
  {
    theta_self = quatToEuler(msg.orientation);
  }

  double quatToEuler(const geometry_msgs::msg::Quaternion &quaternion)
  {
    tf2::Quaternion tf_quat;
    tf2::fromMsg(quaternion, tf_quat);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
    return yaw;
  }

  void set_goal()
  {
    cout << "enter latitude:" << endl;
    cin >> x_goal;
    cout << "enter longitude:" << endl;
    cin >> x_goal;
    original_g_dis = sqrt(pow(x_goal - x_self, 2) + pow(y_goal - y_self, 2));
  }

  void avoid_obs(const sensor_msgs::msg::LaserScan &msg)
  {
    for (long double x : msg.ranges)
    {
      x = (x > 2) ? 2 : x;
      obj_dis = 2 - (x / 2);
      max = (1 - obj_dis) * 2;
      scan.push_back(obj_dis);

      if (i < 46 || i > msg.ranges.size()-46)
      {
        sum -= 6 * obj_dis * ((i > 57) ? i : i - msg.ranges.size());
      }
      i++;
    }
    scan.clear();
  }

  void timer_callback()
  {
    double g_dis, angle, vel = 0, angle_diff = 0, k_lin, obs_lin, obs_ang, goal_ang;
    auto message = geometry_msgs::msg::Twist();
    g_dis = sqrt(pow(x_goal - x_self, 2) + pow(y_goal - y_self, 2));
    if (g_dis > 0.01)
    {
      angle = atan2(y_goal - y_self, x_goal - x_self) - theta_self;
      if (angle > 3.14)
      {
        angle = angle - 6.28;
      }
      else if (angle < -3.14)
      {
        angle = angle + 6.28; // angle for avoiding infinity
      }
      angle_diff = 0.45 * angle; // angle differnce of the current posn and goal posn
      if (angle_diff > 0.4)
      {
        angle_diff = 0.4 * angle_diff / abs(angle_diff); // limiting goal angular velocity
      }

      vel = (g_dis / original_g_dis > 0.5) ? 0.5 : (g_dis / original_g_dis);
      obs_lin = (exp(this->obj_dis) - 2);
      k_lin = 0.7 - exp(obs_lin); // calculator for linear velocity due to obstacle

      goal_ang = 4 / (1 + exp(-100 * (max - 2))); // calculator for goal angular velocity
      obs_ang = abs(7.389 - exp(goal_ang)) * sum;

      message.linear.x = 4.5 * k_lin * vel; // adjusting linear velocity
      message.angular.z = obs_ang + 0.225 * goal_ang * angle_diff;
      if (abs(message.angular.z) > 0.5)
      {
        message.angular.z = 0.5 * abs(message.angular.z) / message.angular.z; // limiting the resultant angular velocity
      }
      vel_publisher_->publish(message);
    }
    else
    {
      set_goal();
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr LiDAR_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rover_pubsub>());
  rclcpp::shutdown();
  return 0;
}