#include <sensor_msgs/msg/joy.hpp>
#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class Joy_subscriber : public rclcpp::Node
{
public:
  Joy_subscriber()
  : Node("joy_subscriber")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "Joy", 10, std::bind(&Joy_subscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const sensor_msgs::msg::Joy & msg) const
  {  
    RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg.buttons[0]);
  }
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Joy_subscriber>());
  rclcpp::shutdown();
  return 0;
}















// //
// #include <math.h>
// #define cos45 1/sqrt(2)

// struct vec2d{
//     float x;
//     float y;
//     //ベクトル成分
//     friend auto operator*(vec2d a,vec2d b) -> float{
//         return a.x*b.x+b.y*a.y;
//     }//内積
//     static vec2d make(float x,float y);//ｘ,ｙを要素にもつベクトルを返す
// };

// vec2d vec2d::make(float x,float y){
//     return vec2d{.x = x,.y = y};
// }

// class motor{
//     private:
//     const vec2d direction;//モーターの向いている方向ベクトル
//     float TARGET;//TARGET
//     const int CAN_ID;//CAN ID

//     public:
//     motor(float x,float y,int CAN_ID)
//     :direction(vec2d(x,y)),CAN_ID(CAN_ID),TARGET(0)
//     {}//初期化
//     void set_target(float power){
//         this->TARGET = power;
//     }//TARGETを代入
//     friend auto operator*(vec2d a,motor b) -> float{
//         return a*b.direction;
//     }//内積


// };





// class undercarriage{
//     private:
//     vec2d direction;//進みたい方向
//     motor right_front_motor;
//     motor left_front_motor;
//     motor left_back_motor;
//     motor right_back_motor;
//     //四輪オムニ    

//     public:
//     undercarriage(int right_front_CAN_ID,int left_front_CAN_ID,int left_back_CAN_ID,int right_back_CAN_ID)
//     :right_front_motor(motor(-cos45,cos45,right_front_CAN_ID)),left_front_motor(motor(cos45,cos45,left_front_CAN_ID)),
//     left_back_motor(motor(-cos45,-cos45,left_back_CAN_ID)),right_back_motor(motor(cos45,-cos45,right_back_CAN_ID))
//     {}//初期化
//     void entry_data();
//     void set_vector(float x,float y);//行きたい方向
//     void update();//4タイヤがうまく回るようにする
//     void send_data();//データを送る

// };
// void undercarriage::entry_data(){
//     int x,y;
//     //データ入力
//     //書きかけ


//     this->set_vector(x,y);
    
// }

// void undercarriage::set_vector(float x,float y){
//     this->direction.x = x;
//     this->direction.y = y;
// }

// # define NUMBER 50
// //多分TARGETの最大値になるはず

// void undercarriage::update(){
//     right_front_motor.set_target(direction*this->right_front_motor*NUMBER);
//     left_front_motor.set_target(direction*this->left_front_motor*NUMBER);
//     left_back_motor.set_target(direction*this->left_back_motor*NUMBER);
//     right_back_motor.set_target(direction*this->right_back_motor*NUMBER);
// }

// void undercarriage::send_data(){
//     //書いて
// }

// class robot : undercarriage{
    

// };