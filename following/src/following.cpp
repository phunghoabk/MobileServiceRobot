#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <memory>
#include <utility>
#include <iostream>

#include <chrono>
#include <math.h>
using namespace std;

static double clamp(double v, double v_min, double v_max)
{
    return std::min(std::max(v_min,v),v_max);
}

class Following : public rclcpp::Node
{
public:
  /// \brief Follow node, which subscribes to laser scan messages and publishes
  /// velocity commands.
  Following()
  : Node("following")
  {
    auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

    /*
    // Subscribe to sensor messages
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/dolly/laser/out", default_qos,
      std::bind(&Following::OnSensorMsg, this, std::placeholders::_1));
    */

    // Subscribe to point messages
    point_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
      "position", default_qos,
      std::bind(&Following::PositionControl, this, std::placeholders::_1));

    // Advertise velocity commands
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("dolly/cmd_vel", default_qos);
  }

private:
  /// \brief Callback for sensor message subscriber
  /// \param[in] _msg Laser scan message

  void PositionControl(geometry_msgs::msg::Point::SharedPtr _pos)
  {
    double dist_target =1;
    double ang_target=0;
    double dist_Kp=1;
    double ang_Kp=2.0;
    double vel_max=0.3; // max linear velocity
    double ang_max = 1.5;  // max angular velocity
    double dist_min = 0.4;
    double decay_factor_lin = 3;
    double decay_factor_ang = 15;
    double inject_factor_lin = 6;
    double inject_factor_ang = 20;
    double decay_ratio_lin=0;
    double inject_ratio_lin=0;

    auto x_receive = _pos -> x; // receive postion x, y from point publisher
    auto y_receive = _pos -> y;

    //Debug
    std::cout<< "x_receive "<< x_receive <<std::endl;
    std::cout<< "y_receive "<< y_receive <<std::endl;

    static std::chrono::time_point<std::chrono::high_resolution_clock> last_time;
    std::chrono::time_point<std::chrono::high_resolution_clock> now = std::chrono::high_resolution_clock::now();

    // calculate distance and angle from target point to robot
    double d = sqrt( pow(x_receive,2) + pow(y_receive,2) );
    double rad = atan2(x_receive, y_receive);

    double dist_err = d - dist_target;
    double ang_err = rad - ang_target;

    std::chrono::duration<double, std::milli> elapsed_ms = now - last_time; // convert duration to ms

    decay_ratio_lin = exp( -decay_factor_lin*elapsed_ms.count()/1000 ); //cmd = cmd * e^(-decayF*delta_t)
    inject_ratio_lin = exp( -inject_factor_lin*elapsed_ms.count()/1000 );

    double decay_ratio_ang = exp( -decay_factor_ang*elapsed_ms.count()/1000 );
    double inject_ratio_ang = exp( -inject_factor_ang*elapsed_ms.count()/1000 ); 

    // Populate command message, all weights have been calculated by trial and error
    auto cmd_msg = std::make_unique<geometry_msgs::msg::Twist>();
    
    if( d < dist_min){
      cmd_msg->linear.x *=  decay_ratio_lin;
      cmd_msg->angular.z  *= decay_ratio_ang;
    }
    else {
      cmd_msg->linear.x = clamp( dist_err*dist_Kp, -vel_max, vel_max)*(1-inject_ratio_lin) + cmd_msg->linear.x*inject_ratio_lin;
      cmd_msg->angular.z = clamp( ang_err*ang_Kp, -ang_max, ang_max)*(1-inject_ratio_ang) + cmd_msg->angular.z*inject_ratio_ang;
    }
    last_time = now;

    std::cout << "t " << elapsed_ms.count() << " ms" << std::endl;

    std::cout<< "distance(m) "<< d <<std::endl;
    std::cout<< "angle(rad) "<< rad <<std::endl;

    std::cout<< "dist_err "<< dist_err <<std::endl;
    std::cout<< "ang_err"<< ang_err <<std::endl;

    std::cout<< "cmd_msg->linear.x  "<< cmd_msg->linear.x  <<std::endl;
    std::cout<< "cmd_msg->angular.z "<< cmd_msg->angular.z <<std::endl;
    
    cmd_pub_->publish(std::move(cmd_msg));
  }

  
  /// \brief Point messages subscriber
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr point_sub_;

  /// \brief Laser messages subscriber
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;

  /// \brief Velocity command publisher
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

  /// \brief Minimum allowed distance from target
  double min_dist_ = 1.0;

  /// \brief Scale linear velocity, chosen by trial and error
  double linear_k_ = 0.02;

  /// \brief Scale angular velocity, chosen by trial and error
  double angular_k_ = 0.08;
};

int main(int argc, char * argv[])
{
  printf("Main_following \n");

  // Forward command line arguments to ROS
  rclcpp::init(argc, argv);

  // Create a node
  auto node = std::make_shared<Following>();

  // Run node until it's exited
  rclcpp::spin(node);

  // Clean up
  rclcpp::shutdown();
  return 0;
}