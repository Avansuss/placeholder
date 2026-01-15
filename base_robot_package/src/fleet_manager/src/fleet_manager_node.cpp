#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <thread>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// --- CONFIGURATION: Define your fleet here ---
const std::vector<std::string> ROBOT_LIST = {"robot11", "robot12", "robot13"};
// ---------------------------------------------

class FleetManager : public rclcpp::Node
{
public:
    FleetManager() : Node("fleet_manager_node")
    {
        RCLCPP_INFO(this->get_logger(), "--- Fleet Manager Started ---");

        for (const auto& robot_name : ROBOT_LIST) {
            setup_robot_connection(robot_name);
        }
    }

    void send_command(std::string robot_name, std::string target_location, int floor = -1)
    {
        if (publishers_.find(robot_name) == publishers_.end()) {
            std::cout << "\n[ERROR] Robot '" << robot_name << "' is not connected!" << std::endl;
            return;
        }

        auto move_msg = std_msgs::msg::String();
        move_msg.data = target_location;
        publishers_[robot_name]->publish(move_msg);

        if (floor >= 0 && floor <= 4) {
            auto vision_msg = std_msgs::msg::String();
            vision_msg.data = std::to_string(floor);
            vision_publishers_[robot_name]->publish(vision_msg);
        }
    }

    void start_mission(int floor)
    {
        if (mission_state_ != MissionState::IDLE) {
            RCLCPP_WARN(get_logger(), "Mission already running");
            return;
        }

        mission_floor_ = floor;
        mission_state_ = MissionState::STARTED;

        send_command("robot11", "pickup11", floor);
        send_command("robot12", "pickup12", floor);
        send_command("robot13", "dock13", floor);

        RCLCPP_INFO(get_logger(), "Mission started");
    }

private:
    std::map<std::string, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> publishers_;
    std::map<std::string, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> vision_publishers_;
    std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> subscribers_;

    enum class MissionState {
        IDLE,
        STARTED,
        ROBOT11_TO_LIFT,
        TRANSFER,
        ROBOT12_TO_DROPOFF,
        RETURN_HOME,
        DONE
    };

    MissionState mission_state_ = MissionState::IDLE;
    int mission_floor_ = -1;

    void setup_robot_connection(const std::string& name)
    {
        publishers_[name] =
            this->create_publisher<std_msgs::msg::String>("/" + name + "/move_command", 10);

        vision_publishers_[name] =
            this->create_publisher<std_msgs::msg::String>("/" + name + "/vision_command", 10);

        auto sub = this->create_subscription<std_msgs::msg::String>(
            "/" + name + "/robot_status", 10,
            [this, name](const std_msgs::msg::String::SharedPtr msg) {
                status_callback(name, msg);
            });

        subscribers_.push_back(sub);
    }

    void status_callback(std::string robot_name,
                         const std_msgs::msg::String::SharedPtr msg)
    {
        std::string text = msg->data;
        std::cout << "\n[" << robot_name << "] " << text << std::endl;

        const std::string prefix = "Arrived at ";
        if (text.find(prefix) == std::string::npos) {
            std::cout << "Command > " << std::flush;
            return;
        }

        std::string location = text.substr(prefix.length());

        if (robot_name == "robot11") {
            if (location == "pickup11" &&
                mission_state_ == MissionState::STARTED)
            {
                send_command("robot11", "dropoff11", mission_floor_);
                mission_state_ = MissionState::ROBOT11_TO_LIFT;
            }
            else if (location == "dropoff11" &&
                     mission_state_ == MissionState::ROBOT11_TO_LIFT)
            {
                mission_state_ = MissionState::TRANSFER;

                std::thread([this]() {
                    std::this_thread::sleep_for(std::chrono::seconds(3));

                    send_command("robot12", "dropoff12", mission_floor_);
                    send_command("robot11", "dock11", mission_floor_);

                    mission_state_ = MissionState::ROBOT12_TO_DROPOFF;
                }).detach();
            }
        }

        if (robot_name == "robot12") {
            if (location == "dropoff12" &&
                mission_state_ == MissionState::ROBOT12_TO_DROPOFF)
            {
                send_command("robot12", "dock12", mission_floor_);
                mission_state_ = MissionState::RETURN_HOME;
            }
            else if (location == "dock12" &&
                     mission_state_ == MissionState::RETURN_HOME)
            {
                mission_state_ = MissionState::DONE;
                RCLCPP_INFO(get_logger(), "Mission completed");
            }
        }

        std::cout << "Command > " << std::flush;
    }
};

// --- Input Loop ---
void user_input_loop(std::shared_ptr<FleetManager> node)
{
    std::string line;
    std::this_thread::sleep_for(std::chrono::seconds(1));

    std::cout << "\nCommand > " << std::flush;
    while (std::getline(std::cin, line)) {

        if (line == "exit" || line == "quit")
            break;

        if (line.empty()) {
            std::cout << "Command > " << std::flush;
            continue;
        }

        std::stringstream ss(line);
        std::string name, target;
        int floor = -1;

        ss >> name >> target >> floor;

        if (name == "Start" || name == "start") {
            node->start_mission(floor);
        }
        else if (!name.empty() && !target.empty()) {
            node->send_command(name, target, floor);
        }
        else {
            std::cout << "[ERROR] Invalid format!" << std::endl;
        }

        std::cout << "Command > " << std::flush;
    }

    rclcpp::shutdown();
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FleetManager>();

    std::thread input_thread(user_input_loop, node);
    rclcpp::spin(node);

    if (input_thread.joinable())
        input_thread.join();

    rclcpp::shutdown();
    return 0;
}
