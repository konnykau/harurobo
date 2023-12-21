#include <sensor_msgs/msg/joy.hpp>
#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "undercarriage.hpp"

using std::placeholders::_1;

class Joy_subscriber : public rclcpp::Node
{
public:
  Joy_subscriber()
  : Node("joy_subscriber"),koinobori(undercarriage(0x100,0x200,0x300,0x640))
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&Joy_subscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const sensor_msgs::msg::Joy & msg)
  {  
    // RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg.buttons[0]);
    if(msg.buttons[9]){
        this->koinobori.update(msg.axes[0],msg.axes[1],left_turn);
    }
    else if(msg.buttons[10]){
        this->koinobori.update(msg.axes[0],msg.axes[1],right_turn);
    }
    else{
        this->koinobori.update(msg.axes[0],msg.axes[1],no_turn);
    }
    
    

  }
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;

  undercarriage koinobori;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Joy_subscriber>());
  rclcpp::shutdown();
  return 0;
}
