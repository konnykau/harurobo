#include <sensor_msgs/msg/joy.hpp>
#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "harurobo/undercarriage.hpp"
#include <can_utils.hpp>
#include "harurobo/send_data.hpp"

using std::placeholders::_1;

class Undercarriage_Node: public rclcpp::Node//rclcpp::Nodeというクラスを継承してUndercarriage_Nodeという新しいクラスを作る
{
public:
  Undercarriage_Node()
  : Node("harurobo_koinobori_undercarriage"),koinobori(undercarriage(0x100,0x200,0x300,0x640))//ノード名と足回り用のシラスのCAN idの設定（初期化）
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&Undercarriage_Node::topic_callback, this, _1));//joy == コントローラーの入力をsubscription
    can_pub_ = this->create_publisher<can_plugins2::msg::Frame>("can_tx", 10);//canに対してpublish
  }
  

private:
  void topic_callback(const sensor_msgs::msg::Joy & msg)//この関数が随時実行されるらしい
  {  
    if(msg.buttons[9]){//startボタン
        can_pub_->publish(std::move(koinobori.make_CAN_mode(motor_name::right_front_motor,1)));
        can_pub_->publish(std::move(koinobori.make_CAN_mode(motor_name::left_front_motor,1)));
        can_pub_->publish(std::move(koinobori.make_CAN_mode(motor_name::left_back_motor,1)));
        can_pub_->publish(std::move(koinobori.make_CAN_mode(motor_name::right_back_motor,1)));
    }//mode velにする
    if(msg.buttons[8]){//backボタン
        can_pub_->publish(std::move(koinobori.make_CAN_mode(motor_name::right_front_motor,0)));
        can_pub_->publish(std::move(koinobori.make_CAN_mode(motor_name::left_front_motor,0)));
        can_pub_->publish(std::move(koinobori.make_CAN_mode(motor_name::left_back_motor,0)));
        can_pub_->publish(std::move(koinobori.make_CAN_mode(motor_name::right_back_motor,0)));
    }//mode disにする
///////////////////////////////ここの上がstartボタン、backボタンによるmodeの調整
///////////////////////////////ここの下から平行移動、回転をするための個々のモーターのターゲットを決めるif文
    if(msg.buttons[4]){//ZL(left shouldderボタン)
        this->koinobori.update(-msg.axes[0],msg.axes[1],turn_direction::left_turn);
    }//left turn
    else if(msg.buttons[5]){//ZR(right shouldderボタン)
        this->koinobori.update(-msg.axes[0],msg.axes[1],turn_direction::right_turn);
    }//right turn
    else{
        this->koinobori.update(-msg.axes[0],msg.axes[1],turn_direction::no_turn);
    }//平行移動//x軸はjoyの入力時点で反転していた
/////////////////////////////ここの下からは、上で決めたターゲットをシラスに向かって送ってあげる関数
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
