#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <std_msgs/msg/int8.hpp>
#include <cmath>

enum RobotState : int8_t {
    NORMAL = 0,
    STOP = 1,
    FORWARD = 2,
    REVERSE_NORMAL = 3,
    REVERSE_FORWARD = 4
};

class PurePursuitNode : public rclcpp::Node {
public:
    PurePursuitNode() : Node("pure_pursuit_node") {
        lookahead_distance_ = this->declare_parameter<double>("lookahead_distance", 1.0);
        linear_velocity_ = this->declare_parameter<double>("linear_velocity", 0.5);
        path_subscriber_ = this->create_subscription<nav_msgs::msg::Path>("PathPlanner/path", 10, std::bind(&PurePursuitNode::pathCallback, this, std::placeholders::_1));
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("bicycle_steering_controller/odometry", 10, std::bind(&PurePursuitNode::odomCallback, this, std::placeholders::_1));
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/bicycle_steering_controller/reference", 10);        
        stop_listener_ = this->create_subscription<std_msgs::msg::Int8>("PathPlanner/command", 10, std::bind(&PurePursuitNode::CommandCallback, this, std::placeholders::_1));
    }
private:
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr                path_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr            odom_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr      cmd_vel_publisher_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr                stop_listener_;
    nav_msgs::msg::Path current_path_;
    geometry_msgs::msg::Pose current_pose_;

    double lookahead_distance_;
    double linear_velocity_;
    size_t current_target_index_ = 0;  
    double L_d = 0.6; // Basing on the robot's wheelbase           
    double v = 0.2;
    int8_t command_flag_ = NORMAL;
    int contatore = 0;
    
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
        current_path_ = *msg;

        // Transform the path points from the robot's local frame to the global frame. 
        tf2::Quaternion q(
            current_pose_.orientation.x,
            current_pose_.orientation.y,
            current_pose_.orientation.z,
            current_pose_.orientation.w
        );
        tf2::Vector3 v(current_pose_.position.x, current_pose_.position.y, current_pose_.position.z);
        tf2::Transform robot_transform(q, v);

        for (size_t i = 0; i < current_path_.poses.size(); i++) {
            tf2::Vector3 p(
                current_path_.poses[i].pose.position.x,
                current_path_.poses[i].pose.position.y,
                0.0
            );
            // Frame transformation based on the command_flag_ to handle reverse movement
            if (command_flag_ == REVERSE_NORMAL){
                p.setX(-p.x());
                p.setY(-p.y());
            } 

            tf2::Vector3 transformed_p = robot_transform * p;

            current_path_.poses[i].pose.position.x = transformed_p.x();
            current_path_.poses[i].pose.position.y = transformed_p.y();
        }
        
        current_target_index_ = 0;
    }

    // Setting the command_flag_ based on the received message to control the robot's movement
    void CommandCallback(const std_msgs::msg::Int8::SharedPtr msg) {
        if (msg->data == STOP) {
            command_flag_ = STOP;
            std::cout << "Stop event received " << contatore++ << std::endl;
        }
        else if (msg->data == FORWARD) {
            command_flag_ = FORWARD;
            std::cout << "Forward event received " << contatore++ << std::endl;
        }
        else if (msg->data == NORMAL) {
            command_flag_ = NORMAL;
            // std::cout << "Normal event received " << contatore++ << std::endl;
        }
        else if (msg->data == REVERSE_NORMAL) {
            command_flag_ = REVERSE_NORMAL;
            std::cout << "Reverse Normal event received " << contatore++ << std::endl;
        }
        else if (msg->data == REVERSE_FORWARD) {
            command_flag_ = REVERSE_FORWARD;
            std::cout << "Reverse Forward event received " << contatore++ << std::endl;
        }
    }

    // Odometry Callback
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_pose_ = msg->pose.pose;
        computeControlCommand();
    }

    void computeControlCommand() {

        // Handle the STOP command by publishing a zero velocity command and clearing the path
        if (command_flag_ == STOP) {
            geometry_msgs::msg::TwistStamped stop_msg;
            stop_msg.header.stamp = this->now();
            stop_msg.header.frame_id = "base_link";
            stop_msg.twist.linear.x = 0.0;
            stop_msg.twist.angular.z = 0.0;
            cmd_vel_publisher_->publish(stop_msg);
            
            current_path_.poses.clear(); 
            return; 
        }
        

        if (command_flag_ == FORWARD || command_flag_ == REVERSE_FORWARD) {
            geometry_msgs::msg::TwistStamped straight_msg;
            straight_msg.header.stamp = this->now();
            straight_msg.header.frame_id = "base_link";
            
            straight_msg.twist.linear.x = (command_flag_ == FORWARD) ? v : -v; 
            straight_msg.twist.angular.z = 0.0;
            cmd_vel_publisher_->publish(straight_msg);
            
            return; 
        }

        if (current_path_.poses.empty()) {
            return; 
        }

        // Extract the robot's current orientation (yaw) from the quaternion
        tf2::Quaternion q(
            current_pose_.orientation.x,
            current_pose_.orientation.y,
            current_pose_.orientation.z,
            current_pose_.orientation.w
        );
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        double robot_x = current_pose_.position.x;
        double robot_y = current_pose_.position.y;
        
        double target_x = 0.0;
        double target_y = 0.0;
        bool target_found = false;

        for (size_t i = current_target_index_; i < current_path_.poses.size(); i++) {
            target_x = current_path_.poses[i].pose.position.x;
            target_y = current_path_.poses[i].pose.position.y;

            double distance = std::hypot(target_x - robot_x, target_y - robot_y);

            double dx = target_x - robot_x;
            double dy = target_y - robot_y;
            double local_x = dx * std::cos(yaw) + dy * std::sin(yaw);

            if (command_flag_ == REVERSE_NORMAL) {
                local_x = -local_x; 
            }
            bool is_valid_direction = (local_x > 0.0);

            if (distance >= L_d && is_valid_direction) {
                current_target_index_ = i;
                target_found = true;
                break;
            }
        }

        if (!target_found) {
            size_t last_index = current_path_.poses.size() - 1;
            target_x = current_path_.poses[last_index].pose.position.x;
            target_y = current_path_.poses[last_index].pose.position.y;
            current_target_index_ = last_index;

            double final_distance = std::hypot(target_x - robot_x, target_y - robot_y);
            
            // To stop the robot when it reaches the final point of the path
            if (final_distance < 0.15) {
                geometry_msgs::msg::TwistStamped stop_msg;
                stop_msg.header.stamp = this->now();
                stop_msg.header.frame_id = "base_link";
                stop_msg.twist.linear.x = 0.0;
                stop_msg.twist.angular.z = 0.0;
                cmd_vel_publisher_->publish(stop_msg);
                
                current_path_.poses.clear(); 
                command_flag_ = STOP; 
                return; 
            }
        }

        // Invert the velocity if the command_flag_ indicates reverse movement
        double current_v = v; 
        if (command_flag_ == REVERSE_NORMAL) {
            current_v = -v;   
        }

        // Compute the control command using the Pure Pursuit algorithm
        double angle_to_target = std::atan2(target_y - robot_y, target_x - robot_x);
        double alpha = angle_to_target - yaw;
        alpha = std::atan2(std::sin(alpha), std::cos(alpha));

        double distance_to_target = std::hypot(target_x - robot_x, target_y - robot_y);
        double omega = 0.0;
        
        if (distance_to_target > 0.001) {
            omega = (2.0 * current_v * std::sin(alpha)) / distance_to_target;
        }

        geometry_msgs::msg::TwistStamped cmd_msg;
        cmd_msg.header.stamp = this->now();
        cmd_msg.header.frame_id = "base_link";
        cmd_msg.twist.linear.x = current_v;
        cmd_msg.twist.angular.z = omega;
        
        cmd_vel_publisher_->publish(cmd_msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuitNode>());
    rclcpp::shutdown();
    return 0;
}