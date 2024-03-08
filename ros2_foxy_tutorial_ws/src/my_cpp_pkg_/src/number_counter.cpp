#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
 
class NumCountNode : public rclcpp::Node 
{
public:
    NumCountNode() : Node("number_counter"), counter_(0) 
    {
        subscriber_ = this->create_subscription<example_interfaces::msg::Int64>(
            "number", 10, 
            std::bind(&NumCountNode::callbackNumCount, this, std::placeholders::_1));
        
        publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number_count", 10);
        
        RCLCPP_INFO(this->get_logger(), "Number counter has been started.");
    }

private:
    void callbackNumCount(const example_interfaces::msg::Int64::SharedPtr msg){
        RCLCPP_INFO(this->get_logger(), "%ld", msg->data);
        
        this->counter_ += msg->data;
        auto msg_num = example_interfaces::msg::Int64();
        msg_num.data = this->counter_;
        publisher_->publish(msg_num);
    }

    std::int64_t counter_;
    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscriber_;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;

};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumCountNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}