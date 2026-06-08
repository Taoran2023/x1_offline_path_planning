#include <block_map/color_manager.h>

void ColorManager::init(rclcpp::Node::SharedPtr node){
    auto gp = [&](const std::string &name, auto def) {
        if (!node->has_parameter(name)) node->declare_parameter(name, def);
        return node->get_parameter(name).get_value<decltype(def)>();
    };

    vector<double> CR = gp("block_map.colorR", std::vector<double>{});
    vector<double> CG = gp("block_map.colorG", std::vector<double>{});
    vector<double> CB = gp("block_map.colorB", std::vector<double>{});

    std_msgs::msg::ColorRGBA color;
    for(int i = 0; i < (int)CG.size(); i++){
        color.a = 1.0;
        cout<<CR[i]/255<<endl;
        color.r = CR[i]/255;
        color.g = CG[i]/255;
        color.b = CB[i]/255;
        c_l_.push_back(color);
    }
}
