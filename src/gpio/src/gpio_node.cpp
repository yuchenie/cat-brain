#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include <cmath>
#include <vector>
#include <pigpiod_if2.h>

class GPIONode : public rclcpp::Node{
public: 
    GPIONode() : Node("gpio_node") {
        pi_ = pigpio_start(NULL, NULL);
        if (pi_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to pigpiod");
            rclcpp::shutdown();
            return;
        } else {
            RCLCPP_INFO(this->get_logger(), "pigpio initialized");
        } 

        set_mode(pi_, 17, PI_OUTPUT);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&GPIONode::timer_callback, this));
    }

    ~GPIONode() {
        if (pi_ >= 0) {
            pigpio_stop(pi_);
        }
    }

private:
    int pi_{-1};
    rclcpp::TimerBase::SharedPtr timer_;
    bool toggle_ = false;

    void timer_callback() {
        set_PWM_dutycycle(pi_, 17, 128);
        RCLCPP_INFO(this->get_logger(), "here");
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GPIONode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}