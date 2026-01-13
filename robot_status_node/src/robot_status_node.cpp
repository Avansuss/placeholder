#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "linorobot2_interfaces/msg/robot_status.hpp"

using std::placeholders::_1;

class RobotStatusNode : public rclcpp::Node
{
public:
    RobotStatusNode() : Node("robot_status_node")
    {
        this->declare_parameter("robot_id", "robot_1");
        robot_id_ = this->get_parameter("robot_id").as_string();

        // Publisher naar de server
        fleet_pub_ = this->create_publisher<linorobot2_interfaces::msg::RobotStatus>(
            "fleet_status", 10);

        // Subscriber naar Navigatie Tekst
        nav_sub_ = this->create_subscription<std_msgs::msg::String>(
            "robot_status", 10, std::bind(&RobotStatusNode::nav_callback, this, _1));

        // Standaard waarden
        current_status_msg_.robot_id = robot_id_;
        current_status_msg_.status = linorobot2_interfaces::msg::RobotStatus::IDLE;
        current_status_msg_.battery_level = -1.0;

        RCLCPP_INFO(this->get_logger(), "Simple Status Node Active for: %s", robot_id_.c_str());
    }

private:
    void nav_callback(const std_msgs::msg::String& msg)
    {
        std::string text = msg.data;

        current_status_msg_.description = text;

        if (text.find("Heading to") != std::string::npos) {
            current_status_msg_.status = linorobot2_interfaces::msg::RobotStatus::MOVING;
            std::string prefix = "Heading to ";
            if (text.size() > prefix.size()) {
                current_status_msg_.target_node = text.substr(prefix.size());
            }
        }
        else if (text.find("Arrived at") != std::string::npos) {
            current_status_msg_.status = linorobot2_interfaces::msg::RobotStatus::IDLE;
            std::string prefix = "Arrived at ";
            if (text.size() > prefix.size()) {
                current_status_msg_.last_visited_node = text.substr(prefix.size());
                current_status_msg_.target_node = "";
            }
        }
        else if (text.find("Couldn't") != std::string::npos) {
            current_status_msg_.status = linorobot2_interfaces::msg::RobotStatus::ERROR;
        }

        fleet_pub_->publish(current_status_msg_);
    }

    std::string robot_id_;
    linorobot2_interfaces::msg::RobotStatus current_status_msg_;

    rclcpp::Publisher<linorobot2_interfaces::msg::RobotStatus>::SharedPtr fleet_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr nav_sub_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotStatusNode>());
    rclcpp::shutdown();
    return 0;


