#include <memory>
#include <iostream>
#include <vector>
#include <string>
#include <bits/stdc++.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using namespace std;

class CamNode : public rclcpp::Node
{
public:
    CamNode() : Node("CamNode") , count_(0){
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&CamNode::dataNode, this, _1));
        timer_ = this->create_wall_timer(100ms, std::bind(&CamNode::timer_callback, this));

     }
vector<float> scan;

private:
float depth, obj_dis, max, sum, k_lin,linear_velocity_,range, first_obj_dis;

void dataNode(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    first_obj_dis = -1;
    sum = 0.0;
    bool first_obj_dis_computed = false; 

    for (size_t i = 0; i < msg->ranges.size(); i+=1.4)
    {
        float x = msg->ranges[i];
        // cout << msg->ranges.size() << endl;
        x = (x >= 3 || isnan(x)) ? 3 : x; 
        obj_dis = 1 - (x / 3.0);
        
        if (!first_obj_dis_computed)
        {
            first_obj_dis = obj_dis;
            first_obj_dis_computed = true;  
        }

        if ((i < 15.4 || i > msg->ranges.size() - 15.4))
        {
            sum -= (i < 15.4 ? (1-obj_dis) * i : -(1-obj_dis) * (msg->ranges.size() - i)); // resultant 
            // cout << sum << " " << first_obj_dis << endl;
        }
        // cout << i << endl;
    }
}

void timer_callback(){
    sum = sum/5;
    linear_velocity_ = 4.5 * (0.4 - exp(exp(first_obj_dis) - 3)) * 0.5;
    cout << sum << " " << linear_velocity_ << endl;
    // cout << first_obj_dis << " " << sum << endl;
}

rclcpp::TimerBase::SharedPtr timer_;
rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
size_t count_;

};

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CamNode>());
    rclcpp::shutdown();
    return 0;
}