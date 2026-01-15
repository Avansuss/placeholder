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
    // We gebruiken geen "robot_id" parameter meer. 
    // De identiteit wordt bepaald door de namespace (bijv. /robot11).

    // 1. Publisher naar de server (Fleet Manager)
    fleet_pub_ = this->create_publisher<linorobot2_interfaces::msg::RobotStatus>(
      "fleet_status", 10);

    // 2. Subscriber naar Navigatie Tekst (komt van NavGoalNode)
    nav_sub_ = this->create_subscription<std_msgs::msg::String>(
      "robot_status", 10, std::bind(&RobotStatusNode::nav_callback, this, _1));

    // Standaard waarden instellen
    current_status_msg_.status = linorobot2_interfaces::msg::RobotStatus::IDLE;
    current_status_msg_.battery_level = -1.0; // Voor later, als je batterij info hebt

    // Print even in welke namespace we draaien voor debugging
    RCLCPP_INFO(this->get_logger(), "Status Node active. I am running in namespace: %s", this->get_namespace());
  }

private:
  void nav_callback(const std_msgs::msg::String & msg)
  {
    std::string text = msg.data;

    // Vul de omschrijving (bijv: "Heading to Kitchen")
    current_status_msg_.description = text;

    // Simpele logica om de status code (INT) te bepalen op basis van de tekst
    if (text.find("Heading to") != std::string::npos) {
        current_status_msg_.status = linorobot2_interfaces::msg::RobotStatus::MOVING;
        
        // Haal de doel-locatie uit de tekst
        std::string prefix = "Heading to ";
        if (text.size() > prefix.size()) {
            current_status_msg_.target_node = text.substr(prefix.size());
        }
    }
    else if (text.find("Arrived at") != std::string::npos) {
        current_status_msg_.status = linorobot2_interfaces::msg::RobotStatus::IDLE;
        
        // Zet het doel weer op leeg en onthoud waar we zijn
        std::string prefix = "Arrived at ";
        if (text.size() > prefix.size()) {
            current_status_msg_.last_visited_node = text.substr(prefix.size());
            current_status_msg_.target_node = "";
        }
    }
    else if (text.find("Couldn't") != std::string::npos) {
        current_status_msg_.status = linorobot2_interfaces::msg::RobotStatus::ERROR;
    }

    // Verstuur het bericht naar de Fleet Manager
    fleet_pub_->publish(current_status_msg_);
  }

  // De bericht-structuur om te bewaren
  linorobot2_interfaces::msg::RobotStatus current_status_msg_;

  rclcpp::Publisher<linorobot2_interfaces::msg::RobotStatus>::SharedPtr fleet_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr nav_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotStatusNode>());
  rclcpp::shutdown();
  return 0;
}
