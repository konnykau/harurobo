#include <sensor_msgs/msg/joy.hpp>
#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "harurobo/undercarriage.hpp"
#include <can_utils.hpp>
#include <send_data.hpp>

using std::placeholders::_1;

class Undercarriage_Node: public rclcpp::Node
{
public:
  Undercarriage_Node()
  : Node("joy_subscriber"),koinobori(undercarriage(0x100,0x200,0x300,0x640))
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&Undercarriage_Node::topic_callback, this, _1));
    can_pub_ = this->create_publisher<can_plugins2::msg::Frame>("can_tx", 10);
  }
  

private:
  void topic_callback(const sensor_msgs::msg::Joy & msg)
  {  
    if(msg.buttons[6]){//startボタン
        auto RF_mode_frame = can_utils::generate_frame(koinobori.get_CAN_ID(motor_name::right_front_motor),0x5);
        auto LF_mode_frame = can_utils::generate_frame(koinobori.get_CAN_ID(motor_name::right_back_motor),0x5);
        auto RB_mode_frame = can_utils::generate_frame(koinobori.get_CAN_ID(motor_name::left_front_motor),0x5);
        auto LB_mode_frame = can_utils::generate_frame(koinobori.get_CAN_ID(motor_name::left_back_motor),0x5);
        can_pub_->publish(std::move(RF_mode_frame));
        can_pub_->publish(std::move(RB_mode_frame));
        can_pub_->publish(std::move(LF_mode_frame));
        can_pub_->publish(std::move(LB_mode_frame));
    }//mode velにする
    if(msg.buttons[4]){//backボタン
        auto RF_mode_frame = can_utils::generate_frame(koinobori.get_CAN_ID(motor_name::right_front_motor),0x0);
        auto LF_mode_frame = can_utils::generate_frame(koinobori.get_CAN_ID(motor_name::right_back_motor),0x0);
        auto RB_mode_frame = can_utils::generate_frame(koinobori.get_CAN_ID(motor_name::left_front_motor),0x0);
        auto LB_mode_frame = can_utils::generate_frame(koinobori.get_CAN_ID(motor_name::left_back_motor),0x0);
        can_pub_->publish(std::move(RF_mode_frame));
        can_pub_->publish(std::move(RB_mode_frame));
        can_pub_->publish(std::move(LF_mode_frame));
        can_pub_->publish(std::move(LB_mode_frame));
    }//mode disにする
    if(msg.buttons[9]){//ZL(left shouldderボタン)
        this->koinobori.update(msg.axes[0],msg.axes[1],turn_direction::left_turn);
    }//left turn
    else if(msg.buttons[10]){//ZR(right shouldderボタン)
        this->koinobori.update(msg.axes[0],msg.axes[1],turn_direction::right_turn);
    }//right turn
    else{
        this->koinobori.update(msg.axes[0],msg.axes[1],turn_direction::no_turn);
    }//平行移動
    can_pub_->publish(koinobori.make_CAN_Frame(motor_name::right_front_motor));
    can_pub_->publish(koinobori.make_CAN_Frame(motor_name::left_front_motor));
    can_pub_->publish(koinobori.make_CAN_Frame(motor_name::left_back_motor));
    can_pub_->publish(koinobori.make_CAN_Frame(motor_name::right_back_motor));

  }
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    rclcpp::Publisher<can_plugins2::msg::Frame>::SharedPtr can_pub_;
    undercarriage koinobori;
    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Undercarriage_Node>());
  rclcpp::shutdown();
  return 0;
}
