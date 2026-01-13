#include <memory>
#include <string>
#include <map>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
using std::placeholders::_1;

class NavGoalNode : public rclcpp::Node
{
public:
    NavGoalNode() : Node("nav_goal_node")
    {
        // 1. Action Client (Nav2)
        this->client_ptr_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        // 2. Status Publisher
        this->status_pub_ = this->create_publisher<std_msgs::msg::String>("robot_status", 10);

        // 3. Command Subscriber
        this->command_sub_ = this->create_subscription<std_msgs::msg::String>(
            "move_command", 10, std::bind(&NavGoalNode::command_callback, this, _1));

        // 4. LOAD PARAMETERS
        load_parameters();

        RCLCPP_INFO(this->get_logger(), "Nav Node Ready. Loaded %zu locations from YAML.", locations_.size());
    }

private:
    std::map<std::string, std::vector<double>> locations_;
    std::string current_target_ = "";

    // --- Parameters Inladen ---
    void load_parameters() {
        // Stap A: Haal de lijst met namen op (uit de YAML: location_names)
        this->declare_parameter("location_names", std::vector<std::string>());
        std::vector<std::string> names = this->get_parameter("location_names").as_string_array();

        if (names.empty()) {
            RCLCPP_WARN(this->get_logger(), "No locations found in YAML config!");
            return;
        }

        // Stap B: Loop door de namen en haal de coördinaten op
        for (const auto& name : names) {
            // Declareer de parameter zodat ROS hem kan lezen
            this->declare_parameter(name, std::vector<double>());

            // Haal de [x, y, w] op
            std::vector<double> coords = this->get_parameter(name).as_double_array();

            // Check of het geldig is (moet 3 getallen hebben)
            if (coords.size() == 3) {
                locations_[name] = coords;
                RCLCPP_INFO(this->get_logger(), "Loaded: %s -> [X:%.1f, Y:%.1f]", name.c_str(), coords[0], coords[1]);
            }
            else {
                RCLCPP_ERROR(this->get_logger(), "Invalid coords for '%s'. Expected [x,y,w]", name.c_str());
            }
        }
    }

    void command_callback(const std_msgs::msg::String& msg)
    {
        std::string target = msg.data;

        if (target == current_target_) {
            RCLCPP_INFO(this->get_logger(), "Ignoring duplicate command: '%s'", target.c_str());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Received command: Go to '%s'", target.c_str());

        if (locations_.find(target) == locations_.end()) {
            std::string err = "Unknown location: " + target;
            RCLCPP_ERROR(this->get_logger(), "%s", err.c_str());
            publish_status(err);
            return;
        }

        current_target_ = target;

        std::vector<double> coords = locations_[target];
        send_goal(coords[0], coords[1], coords[2], target);
    }

    void publish_status(std::string text) {
        auto msg = std_msgs::msg::String();
        msg.data = text;
        this->status_pub_->publish(msg);
    }

    void send_goal(double x, double y, double w, std::string location_name)
    {
        if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_ERROR(this->get_logger(), "Action server unavailable.");
            return;
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.pose.position.x = x;
        goal_msg.pose.pose.position.y = y;
        goal_msg.pose.pose.orientation.w = w;

        std::string msg_start = "Heading to " + location_name;
        RCLCPP_INFO(this->get_logger(), "%s", msg_start.c_str());
        publish_status(msg_start);

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback =
            [this, location_name](const GoalHandleNavigateToPose::WrappedResult& result) {

            this->current_target_ = "";

            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                publish_status("Arrived at " + location_name);
            }
            else {
                publish_status("Couldn't reach " + location_name);
            }
            };
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

    rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavGoalNode>());
    rclcpp::shutdown();
    return 0;
}
