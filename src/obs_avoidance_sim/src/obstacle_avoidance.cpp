#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class ObstacleAvoidance : public rclcpp::Node {
public:
    ObstacleAvoidance() : Node("obstacle_avoidance") {
        // Initialize velocity publisher
        velocity_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("vel", 10);

        // Initialize subscriber for laser scan
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&ObstacleAvoidance::scan_callback, this, std::placeholders::_1));
        
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        bool obstacle_detected = false;
        float threshold = 0.5;
        for (const float &range : msg->ranges) {
            if (range < threshold) {
                obstacle_detected = true;
                break;
            }
        }

        auto vel_msg = geometry_msgs::msg::Twist();
        if (obstacle_detected) {
            // Stop and turn if an obstacle is detected
            vel_msg.linear.x = 0.0;
            vel_msg.angular.z = 0.5;
        } else {
            // Move forward if no obstacles are detected
            vel_msg.linear.x = 0.5;
            vel_msg.angular.z = 0.0;
        }

        // Publish the velcity command
        velocity_pub_->publish(vel_msg);
    }
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleAvoidance>());
    rclcpp::shutdown();
    return 0;
}