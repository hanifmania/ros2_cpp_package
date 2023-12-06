#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using namespace std::chrono_literals;

class DefaultCPPNode : public rclcpp::Node
{
public:
  DefaultCPPNode()
  : Node("default_cpp_node"), count1_(0), count2_(0)
  {
    pub1_ = this->create_publisher<std_msgs::msg::String>("topic1", 10);
    pub2_ = this->create_publisher<std_msgs::msg::String>("topic2", 10);

    sub1_ = this->create_subscription<std_msgs::msg::String>("topic1", 10, 
        std::bind(&DefaultCPPNode::topic1_callback, this, std::placeholders::_1));

    sub2_ = this->create_subscription<std_msgs::msg::String>("topic2", 10, 
        std::bind(&DefaultCPPNode::topic2_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(500ms, std::bind(&DefaultCPPNode::timer_callback, this));

    // Create a service server for add_two_ints service
    add_two_ints_server_ = this->create_service<example_interfaces::srv::AddTwoInts>(
        "add_two_ints",
        std::bind(&DefaultCPPNode::add_two_ints_callback, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  void topic1_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "I receive: '%s'", msg->data.c_str());
    RCLCPP_INFO(this->get_logger(), "Do callback 1...");
  }

  void topic2_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "I receive: '%s'", msg->data.c_str());
    RCLCPP_INFO(this->get_logger(), "Do callback 2...");
  }

  void timer_callback()
  {
    auto message1 = std_msgs::msg::String();
    message1.data = "Hello, topic1! " + std::to_string(count1_++);
    pub1_->publish(message1);

    auto message2 = std_msgs::msg::String();
    message2.data = "Hello, topic2! " + std::to_string(count2_++);
    pub2_->publish(message2);
  }

  void add_two_ints_callback(
    const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
    std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
  {
    response->sum = request->a + request->b;
    RCLCPP_INFO(this->get_logger(), "Adding: %ld + %ld = %ld", request->a, request->b, response->sum);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub1_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub2_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub1_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub2_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr add_two_ints_server_;
  size_t count1_;
  size_t count2_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DefaultCPPNode>());
  rclcpp::shutdown();
  return 0;
}
