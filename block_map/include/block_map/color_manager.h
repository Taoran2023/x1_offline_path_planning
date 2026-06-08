#ifndef COLOR_MANAGER_H_
#define COLOR_MANAGER_H_

#include <iostream>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>

using namespace std;

class ColorManager{
public:
    ColorManager(){};
    ~ColorManager(){};
    void init(rclcpp::Node::SharedPtr node);
    inline std_msgs::msg::ColorRGBA Id2Color(int idx, double a);

    vector<std_msgs::msg::ColorRGBA> c_l_;
};


inline std_msgs::msg::ColorRGBA ColorManager::Id2Color(int idx, double a){
    std_msgs::msg::ColorRGBA color;
    idx = idx % int(c_l_.size());
    color = c_l_[idx];
    color.a = a;
    return color;
}

#endif