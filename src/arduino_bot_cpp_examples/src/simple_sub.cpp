#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>

using std::placeholders::_1;

class SimpleSubscriber : public rclcpp::Node{
public: 
    SimpleSubscriber() : Node("simple_subscriber"), counter_(0){
        sub_ = create_subscription<std_msgs::msg::String>("chatter", 10, std::bind(&SimpleSubscriber::timerCallback, this, _1));
    }

    void timerCallback(const std_msgs::msg::String &msg){
        RCLCPP_INFO(get_logger(), "Data Received:: %s", msg.data.c_str());
    }

private:
    unsigned int counter_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}