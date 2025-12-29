#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/srv/get_map.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

class MapNode : public rclcpp::Node {
    public:
        MapNode() : Node("map_node") {
            occupancy_service = this->create_service<nav_msgs::srv::GetMap>("/occupancy_service", std::bind(&MapNode::GetGrid, this, _1, _2));
        }

    private:
        void GetGrid(const std::shared_ptr<nav_msgs::srv::GetMap::Request> request, std::shared_ptr<nav_msgs::srv::GetMap::Response> response) {

        }
        std::shared_ptr<rclcpp::Service<nav_msgs::srv::GetMap>> occupancy_service;
};