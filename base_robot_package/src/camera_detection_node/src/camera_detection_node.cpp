#include <vector>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

using namespace cv;

class CameraArucoNode : public rclcpp::Node
{
public:
    CameraArucoNode() : Node("camera_detection_node")
    {
        RCLCPP_INFO(this->get_logger(), "Camera ArUco node gestart!");

        // Publisher en subscriber
        publisher_ = this->create_publisher<std_msgs::msg::String>(
            "/position_command", 10);

        vision_cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/vision_command",
            10,
            std::bind(&CameraArucoNode::visionCommandCallback, this, std::placeholders::_1)
        );

        // Camera openen
        cap_.open(0, CAP_V4L2);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Kan camera niet openen");
            rclcpp::shutdown();
            return;
        }

        // Camera instellingen
        cap_.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M','J','P','G'));
        cap_.set(CAP_PROP_FRAME_WIDTH, 1280);
        cap_.set(CAP_PROP_FRAME_HEIGHT, 720);

        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // ArUco dictionary en parameters
        dictionary_ = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
        parameters_ = aruco::DetectorParameters::create();

	cameraMatrix_ = (Mat_<double>(3,3) <<
    		900.0, 0.0, 640.0,
    		0.0, 900.0, 360.0,
    		0.0,   0.0,   1.0);


        distCoeffs_ = Mat::zeros(5,1,CV_64F);
        markerLength_ = 0.044f;

        SCALE_FIX = 1.0;

        knopIngedrukt_ = false;
        gewenste_liftknop_ = -1;

        // GUI setup
        namedWindow("Camera", WINDOW_NORMAL);
        resizeWindow("Camera", 640, 480);
        startWindowThread();
    }

    void run()
    {
        while (rclcpp::ok()) {
            process_frame();
            rclcpp::spin_some(this->shared_from_this());
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

private:
    void visionCommandCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        try {
            gewenste_liftknop_ = std::stoi(msg->data);
            RCLCPP_INFO(this->get_logger(),
                        "Vision command ontvangen: liftknop %d",
                        gewenste_liftknop_);
        } catch (...) {
            RCLCPP_WARN(this->get_logger(),
                        "Ongeldig vision_command bericht: '%s'",
                        msg->data.c_str());
        }
    }

    void process_frame()
    {
        if (gewenste_liftknop_ < 0) return;

        Mat frame;
        cap_ >> frame;
        if (frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "Leeg frame ontvangen!");
            return;
        }

        std::vector<int> markerIds;
        std::vector<std::vector<Point2f>> markerCorners, rejected;
        std::vector<Vec3d> rvecs, tvecs;

        aruco::detectMarkers(frame, dictionary_, markerCorners, markerIds, parameters_, rejected);

        if (!markerIds.empty()) {
            aruco::estimatePoseSingleMarkers(
                markerCorners,
                markerLength_,
                cameraMatrix_,
                distCoeffs_,
                rvecs,
                tvecs
            );

            std::stringstream ss;
            bool publish = false;

            for (size_t i = 0; i < markerIds.size(); ++i) {
                if (markerIds[i] != gewenste_liftknop_) continue;
                publish = true;

		//double markerPixels =
    			//cv::norm(markerCorners[i][0] - markerCorners[i][1]);

		//std::cout << "[DEBUG] Marker pixels: "
          		//<< markerPixels << std::endl;


                double x = tvecs[i][0] * 100.0 * SCALE_FIX;
                double y = tvecs[i][1] * 100.0 * SCALE_FIX;
                double z = tvecs[i][2] * 100.0 * SCALE_FIX;
                double xyAfstand = std::sqrt(x*x + y*y);

                if (z < 7.0) {
                    knopIngedrukt_ = true;
                } else {
                    knopIngedrukt_ = false;
                }

                ss << " === Marker ID: " << markerIds[i]
                   << " | x: " << x << " cm" // x en ze waren qua waardes omgewisseld
                   << " | y: " << y << " cm"
                   << " | z: " << z << " cm"
                   << " | xyDist: " << xyAfstand << " cm"
                   << " | knop: "
                   << (knopIngedrukt_ ? "INGEDRUKT" : "NIET ingedrukt");

                // Teken marker bounding box en assen
                aruco::drawDetectedMarkers(frame, markerCorners, markerIds);
                drawFrameAxes(frame,
                              cameraMatrix_,
                              distCoeffs_,
                              rvecs[i],
                              tvecs[i],
                              markerLength_ * 0.5f);
            }

            if (publish) {
                auto message = std_msgs::msg::String();
                message.data = ss.str();
                publisher_->publish(message);
            }
        }

        imshow("Camera", frame);
        if (waitKey(1) == 'q') {
            cap_.release();
            destroyAllWindows();
            rclcpp::shutdown();
        }
    }

    // ROS2 members
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr vision_cmd_sub_;

    // Camera en ArUco
    VideoCapture cap_;
    Ptr<aruco::Dictionary> dictionary_;
    Ptr<aruco::DetectorParameters> parameters_;
    Mat cameraMatrix_, distCoeffs_;
    float markerLength_;

    double SCALE_FIX;
    bool knopIngedrukt_;
    int gewenste_liftknop_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraArucoNode>();
    node->run();
    rclcpp::shutdown();
    return 0;
}
