#include <chrono>
#include <functional>
#include <memory>
#include <string> 
#include <iostream>

#include "rclcpp/rclcpp.hpp"  // most ROS2 functions created by CPP
#include "std_msgs/msg/string.hpp"//built-in message type you will use to publish data.
#include "geometry_msgs/msg/point.hpp"

using namespace std::chrono_literals;

/* The next line creates the node class PointPublisher by inheriting from rclcpp::Node. */
class PointPublisher : public rclcpp::Node
{

public:
  PointPublisher()
  : Node("point_publisher"), count_(0) // publish constructor node name point_publisher, and initialize count_0 to 0
  {
   
    //rclcpp::Publisher::SharedPtr publisher_ = node->advertise<geometry_msgs::msg::Point>("position", 10);
    publisher_=this->create_publisher<geometry_msgs::msg::Point>("position", 10);
    /*Every 'this' in the code is referring to the node.*/
    timer_=this->create_wall_timer(500ms,std::bind(&PointPublisher::timer_callback,this)); 
  }
private:
  void timer_callback()
  {
    std::cout << "Please enter a the disired robot position: " <<std::endl;
    std::cout << "position x: ";
	  std::cin >> pos_x;
    std::cout << "position y: ";
	  std::cin >> pos_y;

    point_.x = pos_x;
    point_.y = pos_y;

    publisher_->publish(point_);
  }
  // declaration of the timer, publisher, and counter fields.
  double pos_x, pos_y;
  geometry_msgs::msg::Point point_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
  size_t count_;

};
int main(int argc, char *argv[])
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<PointPublisher>());
  rclcpp::shutdown();
  return 0;
}
