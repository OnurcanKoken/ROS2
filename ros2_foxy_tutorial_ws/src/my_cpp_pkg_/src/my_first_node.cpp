#include "rclcpp/rclcpp.hpp"

class MyNode : public rclcpp::Node
{
public:
    // create the node, give the name of the node
    MyNode() : Node("cpp_test"), counter_(0){
        RCLCPP_INFO(this->get_logger(), "Hello Cpp Node");

        timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                         std::bind(&MyNode::timerCallback, this));
    }

private:
    void timerCallback(){
        counter_++;
        RCLCPP_INFO(this->get_logger(), "Hello %d", counter_);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    int counter_;
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MyNode>();
    // pause the program here
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}