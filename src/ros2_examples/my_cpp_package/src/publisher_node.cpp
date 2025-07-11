#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <rclcpp/init_options.hpp>
#include <signal.h>

/**
 * @author: Mehmet Kahraman
 * @date: 03.10.2023
 * @about: Publisher node
 **/

void nodesignal_handler(int signum)
{
  RCLCPP_WARN(rclcpp::get_logger("publisher_node"), "Caught signal %d. Shutting down...", signum);
  rclcpp::shutdown();
}

class MyROSClass : public rclcpp::Node
{

  private:
    
    double hz;
    std::shared_ptr<rclcpp::Rate> rate;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr normal_publisher;

    std_msgs::msg::String normal_msg;

  public:

    MyROSClass() : Node("publisher_node")
    {
      // Publisher
      hz = 50;
      rate = std::make_shared<rclcpp::Rate>(hz);
      normal_publisher = this->create_publisher<std_msgs::msg::String>("/cpp_nodes/normal_publisher_topic", 10);
      normal_msg.data = "Hello ROS 2 developer!";

      std::this_thread::sleep_for(std::chrono::seconds(1));

    }

    void execute_publisher()
    {

      RCLCPP_INFO(this->get_logger(), "Normal publisher will start.");

      while (rclcpp::ok())
      {
        normal_publisher->publish(normal_msg);
        RCLCPP_INFO(this->get_logger(), "Normal publisher is running...");
        rate->sleep();
      }
    }

};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  
  RCLCPP_INFO(rclcpp::get_logger("publisher_node"), "Node initialized.");
  auto node = std::make_shared<MyROSClass>();
  signal(SIGINT, nodesignal_handler);
  node->execute_publisher();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}