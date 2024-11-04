#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"


using namespace std::chrono_literals;

class JOY_WRAP : public rclcpp::Node
{
public:
    JOY_WRAP() : Node("joy_wrap"),count_(0)
    {
        _rescaleValueHead = 0.05;
        _vel_x = 0;
        _vel_y = 0;
        _vel_z = 0;
        _topicActive = false;

        _topic_sub = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&JOY_WRAP::cb, this, std::placeholders::_1));
        _topic_pub = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);
        
        _timer = this->create_wall_timer(100ms, std::bind(&JOY_WRAP::pub_timer_callback, this));
        //std::thread thread(&JOY_WRAP::publish, this);
        //thread.detach();


    }

private:
    float _rescaleValueHead;
    float _vel_x;
    float _vel_y;
    float _vel_z;
    bool _topicActive;
    
    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _topic_sub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _topic_pub;
    size_t count_;

    void cb(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
      
        _vel_x = msg->axes[0];
        _vel_y = msg->axes[1];
	    _vel_z = msg->axes[3];
        RCLCPP_INFO(this->get_logger(), "I heard: vel_x = %f, vel_y = %f, vel_z = %f\n", _vel_x, _vel_y, _vel_z);
    }

    void pub_timer_callback()
    {
        geometry_msgs::msg::Twist f;

        f.angular.x = 0;
        f.angular.y = 0;
	    f.angular.z = 0;

	    
        f.linear.x = _vel_x * _rescaleValueHead;
        f.linear.y = _vel_y * _rescaleValueHead;
        f.linear.z = _vel_z * _rescaleValueHead;
        
        _topic_pub->publish(f);

    
    }
};

int main(int argc, char** argv)
{
    std::cout<<"Start"<<std::endl;
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JOY_WRAP>());
    rclcpp::shutdown();
    return 0;
}