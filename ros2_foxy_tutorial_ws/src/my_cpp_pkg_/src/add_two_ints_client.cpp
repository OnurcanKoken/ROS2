#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class AddTwoIntsClientNode : public rclcpp::Node
{
public:
    AddTwoIntsClientNode() : Node("add_two_ints_client")
    {
        // thread1_ = std::thread(std::bind(&AddTwoIntsClientNode::callAddTwoIntsService, this, 64, 78));
        threads_.push_back(std::thread(std::bind(&AddTwoIntsClientNode::callAddTwoIntsService, this, 64, 78)));
        threads_.push_back(std::thread(std::bind(&AddTwoIntsClientNode::callAddTwoIntsService, this, 4, 8)));
    }

    // able to call multiple times
    void callAddTwoIntsService(int a, int b){
        // create client, service name
        auto client = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
        // wait
        while(!client->wait_for_service(std::chrono::seconds(1))){
            RCLCPP_WARN(this->get_logger(), "Waiting for the server to be up...");
        }

         // create request
        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = a;
        request->b = b;

        auto future = client->async_send_request(request);
        // dont block the thread for waiting 
        // wait until the response 
        try{
            auto response = future.get();
            // process the result, what you want to do
            RCLCPP_INFO(this->get_logger(), "%d + %d = %d", a, b, response->sum);
        }
        catch (const std::exception &e){
            RCLCPP_ERROR(this->get_logger(), "Service call failed.");
        }
    }
 
private:
    // std::thread thread1_;
    std::vector<std::thread> threads_;
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddTwoIntsClientNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}