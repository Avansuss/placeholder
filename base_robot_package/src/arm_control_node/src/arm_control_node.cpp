//#include <vector>
//#include <iostream>
//#include <sstream>
//#include <string>
//#include <thread>
//#include <chrono>

//#include "rclcpp/rclcpp.hpp"
//#include "std_msgs/msg/string.hpp"

// // Port name: /dev/ttyACM1

//using namespace cv;

//int openSerialPort(const char* portname)
//{
    //int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    //if (fd < 0) {
        //cerr << "Error opening " << portname << ": "
             //<< strerror(errno) << endl;
        //return -1;
    //}
    //return fd;
//}

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sstream>
#include <string>
#include <iomanip>

class LiftButtonListener : public rclcpp::Node {
public:
    LiftButtonListener() : Node("liftbutton_listener") {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/position_command", 10,
            std::bind(&LiftButtonListener::callback, this, std::placeholders::_1)
        );
    }

private:
    void callback(const std_msgs::msg::String::SharedPtr msg) {
        std::string data = msg->data;
        int marker_id = -1;
        double x = 0, y = 0, z = 0;
        std::string knop;

        std::istringstream ss(data);
        std::string token;

        while (std::getline(ss, token, '|')) {
            std::string key, value;
            std::istringstream pair(token);
            if (std::getline(pair, key, ':') && std::getline(pair, value)) {
                key = trim(key);
                value = trim(value);
                if (key.find("Marker ID") != std::string::npos)
                    marker_id = std::stoi(value);
                else if (key == "x")
                    x = std::stod(value.substr(0, value.find(" ")));
                else if (key == "y")
                    y = std::stod(value.substr(0, value.find(" ")));
                else if (key == "z")
                    z = std::stod(value.substr(0, value.find(" ")));
                else if (key == "knop")
                    knop = value;
            }
        }

        std::cout << std::fixed << std::setprecision(2);
        std::cout << "=== Liftbutton: " << marker_id << " === ";

        // X = rechts/links
        if (x > 0)
            std::cout << x << " cm rechts, ";
        else if (x < 0)
            std::cout << -x << " cm links, ";

        // Y = omhoog/omlaag
        if (y > 0)
            std::cout << y << " cm omhoog, ";
        else if (y < 0)
            std::cout << -y << " cm omlaag, ";

        // Z = vooruit/achteruit
        if (z > 0)
            std::cout << z << " cm vooruit";
        else if (z < 0)
            std::cout << -z << " cm achteruit";

        std::cout << ", knop: " << knop << std::endl;
    }

    std::string trim(const std::string &s) {
        size_t start = s.find_first_not_of(" ");
        size_t end = s.find_last_not_of(" ");
        return (start == std::string::npos) ? "" : s.substr(start, end - start + 1);
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LiftButtonListener>());
    rclcpp::shutdown();
    return 0;
}
