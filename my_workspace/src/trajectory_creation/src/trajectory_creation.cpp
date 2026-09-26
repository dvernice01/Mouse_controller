#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/int8.hpp>

#include <control_msgs/srv/set_odometry.hpp> 
// #include <gazebo_msgs/srv/set_entity_state.hpp> // Header for Gazebo

#include <chrono> 
#include <string>
#include <thread>
#include <csignal>

using namespace std::chrono_literals;

enum RobotState : int8_t {
    NORMAL = 0,
    STOP = 1,
    FORWARD = 2,
    REVERSE_NORMAL = 3,
    REVERSE_FORWARD = 4
};

class TrajectoryCreationNode : public rclcpp::Node {
public:
    TrajectoryCreationNode() : Node("trajectory_creation_node") {
        std::cout << "  -> [Costruttore] Inizio creazione nodo ROS" << std::endl;
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("PathPlanner/path", 10);
        state_publisher_ = this->create_publisher<std_msgs::msg::Int8>("PathPlanner/command", 10);
        odom_reset_client_ = this->create_client<control_msgs::srv::SetOdometry>("/bicycle_steering_controller/set_odometry");
        
        canvas_ = cv::Mat(600, 800, CV_8UC3, cv::Scalar(255, 255, 255));
    
        cv::namedWindow("Mouse Trajectory");
        cv::imshow("Mouse Trajectory", canvas_); 
        cv::waitKey(1);
        cv::setMouseCallback("Mouse Trajectory", onMouseCallback, this);
    
        current_path_.header.frame_id = "odom";
    }
    
    // Used to "canc" button
    void ClearFunction() {
        current_path_.poses.clear();
        canvas_.setTo(cv::Scalar(255, 255, 255));
        cv::imshow("Mouse Trajectory", canvas_);
    }
    // Used to "space" button
    void SpawnRobot() {
        // RViz (Odometry Reset)
        if (!odom_reset_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Odometry reset service not available!");
            return;
        }
        auto odom_request = std::make_shared<control_msgs::srv::SetOdometry::Request>();
        odom_request->x = 0.0;
        odom_request->y = 0.0;
        odom_request->yaw = 0.0;
        odom_reset_client_->async_send_request(odom_request);
        RCLCPP_INFO(this->get_logger(), "Odometry reset request sent.");

        /* 
        // 3b.Gazebo (Teletransport)
        if (gazebo_client_->wait_for_service(std::chrono::seconds(1))) {
            auto gz_request = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
            gz_request->state.name = "robot_name"; // Replace with your robot's name in Gazebo
            gz_request->state.pose.position.x = 0.0;
            gz_request->state.pose.position.y = 0.0;
            gz_request->state.pose.position.z = 0.0;
            gazebo_client_->async_send_request(gz_request);
        }
        */
    }

private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr state_publisher_;

    rclcpp::Client<control_msgs::srv::SetOdometry>::SharedPtr odom_reset_client_;
    // rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr gazebo_client_;

    nav_msgs::msg::Path current_path_;

    double conversion_factor_ = 100.0; 
    double center_x_ = 400.0;
    double center_y_ = 300.0;
    double last_x_ = 0.0; 
    double last_y_ = 0.0; 
    
    double start_x_ = 0.0;
    double start_y_ = 0.0;
    double initial_yaw_ = 0.0;
    bool direction_set_ = false;

    int last_x_pixel_ = -1;
    int last_y_pixel_ = -1;
    int current_track_id_ = 1;
    cv::Mat canvas_;

    static void onMouseCallback(int event, int x, int y, int flags, void* userdata) {
        TrajectoryCreationNode* node = static_cast<TrajectoryCreationNode*>(userdata);
        if (node != nullptr) {
            node->processMouse(event, x, y, flags);
        }
    }

    void processMouse(int event, int x, int y, int flags) {
        // Every time the mouse is clicked, clear the path and reset the direction
        if (event == cv::EVENT_LBUTTONDOWN) {
            current_path_.poses.clear(); 
            direction_set_ = false;
            last_x_pixel_ = x;
            last_y_pixel_ = y;
        }
        // Use to stop the robot when the middle mouse button is clicked.
        else if (event == cv::EVENT_MBUTTONDOWN) {
            std_msgs::msg::Int8 state_msg;
            state_msg.data = STOP;
            state_publisher_->publish(state_msg);
        }
        // Double click to set the robot in forward or reverse forward mode, depending on whether the Ctrl key is pressed.
        else if (event == cv::EVENT_LBUTTONDBLCLK) {
            std_msgs::msg::Int8 state_msg;
            state_msg.data = (flags & cv::EVENT_FLAG_CTRLKEY) ? REVERSE_FORWARD : FORWARD;
            int8_t command_flag_ = state_msg.data;
            state_publisher_->publish(state_msg);
        }
        // Trajectory Creation
        else if (event == cv::EVENT_MOUSEMOVE && (flags & cv::EVENT_FLAG_LBUTTON)) { 
            std_msgs::msg::Int8 state_msg;
            state_msg.data = (flags & cv::EVENT_FLAG_CTRLKEY) ? REVERSE_NORMAL : NORMAL;
            int8_t command_flag_ = state_msg.data;
            state_publisher_->publish(state_msg);
            
            if (last_x_pixel_ == -1 && last_y_pixel_ == -1) {
                last_x_pixel_ = x;
                last_y_pixel_ = y;
            }
            
            // Convert pixel coordinates to meters
            double x_meters = (x - center_x_) / conversion_factor_;
            double y_meters = -(y - center_y_) / conversion_factor_;

            geometry_msgs::msg::PoseStamped new_point;
            // Set the header and frame_id for the new point. Trajectory are considered for once
            new_point.header.frame_id = std::to_string(current_track_id_);

            // Moving trajectory in local coordinates based on the initial yaw angle
            if (!direction_set_) {
                if (current_path_.poses.empty()) {
                    start_x_ = x_meters;
                    start_y_ = y_meters;
                    
                    new_point.pose.position.x = 0.0;
                    new_point.pose.position.y = 0.0;
                    current_path_.poses.push_back(new_point);
                } else {
                    double dx = x_meters - start_x_;
                    double dy = y_meters - start_y_;
                    
                    if (std::hypot(dx, dy) > 0.01) { 
                        initial_yaw_ = std::atan2(dy, dx);
                        direction_set_ = true;
                    }
                }
            } 

            // Computation of path's poses

            if (direction_set_) {
                double dx = x_meters - start_x_;
                double dy = y_meters - start_y_;

                double local_x = dx * std::cos(-initial_yaw_) - dy * std::sin(-initial_yaw_);
                double local_y = dx * std::sin(-initial_yaw_) + dy * std::cos(-initial_yaw_);

                new_point.pose.position.x = local_x;
                new_point.pose.position.y = local_y;
                current_path_.poses.push_back(new_point);
            }
            
            // Draw the trajectory on the canvas
            cv::line(canvas_, cv::Point(last_x_pixel_, last_y_pixel_), cv::Point(x, y), cv::Scalar(255, 0, 0), 2);
            last_x_pixel_ = x;
            last_y_pixel_ = y;
            
            cv::imshow("Mouse Trajectory", canvas_);
            cv::waitKey(1);
        } 
        // Publish the path when the left mouse button is released
        else if (event == cv::EVENT_LBUTTONUP) {
            if (!current_path_.poses.empty()) {
                path_publisher_->publish(current_path_);
            }
            current_track_id_ += 1;
            last_x_pixel_ = -1;
            last_y_pixel_ = -1;
        }
    }
};

// Automatically called when it's clicked Ctrl+C
void signalHandler(int signum) {
    (void)signum; 
    rclcpp::shutdown(); // Stopping rclcpp::ok()
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    std::signal(SIGINT, signalHandler);

    auto node = std::make_shared<TrajectoryCreationNode>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread ros_thread([&executor]() {
        executor.spin(); 
    });

    while (rclcpp::ok()) {
        int key = cv::waitKey(10); 
        
        if (key == 27) {
            rclcpp::shutdown();
        }
        else if (key == -1) { 
            node->ClearFunction();
        }
        else if (key == 32) { 
            node->SpawnRobot();
        }
    }

    cv::destroyAllWindows();
    
    if (ros_thread.joinable()) {
        ros_thread.join(); 
    }

    return 0;
}
    
