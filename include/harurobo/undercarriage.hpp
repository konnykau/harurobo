#include <math.h>
#include <can_utils.hpp>
#include "harurobo/send_data.hpp"
#include "harurobo/fry_lib/math.hpp"
#include "harurobo/fry_lib/vector.hpp"
constexpr float cos45 = 1/FRY::sqrt(2);

class motor{
    private:
    const FRY::vec2d direction;//モーターの向いている方向ベクトル
    const int CAN_ID;//CAN ID
    float TARGET;//TARGET 

    public:
    motor(float x,float y,int CAN_ID)
    :direction(FRY::vec2d(x,y)),CAN_ID(CAN_ID),TARGET(0)
    {}//初期化
    void set_target(float power){
        this->TARGET = power;
    }//TARGETを代入
    std::unique_ptr<can_plugins2::msg::Frame> make_frame()
    {
        return can_utils::shirasu_target(this->CAN_ID + 1,this->TARGET);
    }
    std::unique_ptr<can_plugins2::msg::Frame> mode_vel(){
        return can_utils::generate_frame(this->CAN_ID,0x5);
    }
    std::unique_ptr<can_plugins2::msg::Frame> mode_dis(){
        return can_utils::generate_frame(this->CAN_ID,0x0);
    }
    FRY::vec2d get_vec2d(){
        return direction;
    }


};


enum class turn_direction{left_turn, no_turn, right_turn};
enum class motor_name{right_front_motor, left_front_motor, left_back_motor, right_back_motor};


class undercarriage{
    private:
    FRY::vec2d direction;//進みたい方向
    public:
    motor right_front_motor;
    motor left_front_motor;
    motor left_back_motor;
    motor right_back_motor;
    //四輪オムニ    

    
    undercarriage(int right_front_CAN_ID,int left_front_CAN_ID,int left_back_CAN_ID,int right_back_CAN_ID)
    :right_front_motor(motor(-cos45,cos45,right_front_CAN_ID)),left_front_motor(motor(-cos45,-cos45,left_front_CAN_ID)),
    left_back_motor(motor(cos45,-cos45,left_back_CAN_ID)),right_back_motor(motor(cos45,cos45,right_back_CAN_ID))
    {}//初期化
    void set_motor_power(turn_direction turn_dir);//4タイヤがうまく回るようにする
    void set_direction(float x,float y);//行きたい方向
    std::unique_ptr<can_plugins2::msg::Frame> make_CAN_Frame(motor_name motor);//CANパッケージを詰め込む
    std::unique_ptr<can_plugins2::msg::Frame> make_CAN_mode(motor_name motor,bool motor_state);//modeを設定
    void update(float x,float y,turn_direction turn_dir);//他の関数を全部融合させた
};

inline void undercarriage::set_direction(float x,float y){
    this->direction.x = x;
    this->direction.y = y;
}



inline void undercarriage::set_motor_power(turn_direction turn_dir){
    constexpr float MAX_OF_TARGET = 20;
    //多分TARGETの最大値になるはず

    if(turn_dir == turn_direction::left_turn){
        right_front_motor.set_target(MAX_OF_TARGET/5);
        left_front_motor.set_target(MAX_OF_TARGET/5);
        left_back_motor.set_target(MAX_OF_TARGET/5);
        right_back_motor.set_target(MAX_OF_TARGET/5);
    }//ここの正負は不明
    else if(turn_dir == turn_direction::right_turn){
        right_front_motor.set_target(-MAX_OF_TARGET/5);
        left_front_motor.set_target(-MAX_OF_TARGET/5);
        left_back_motor.set_target(-MAX_OF_TARGET/5);
        right_back_motor.set_target(-MAX_OF_TARGET/5);
    }
    else{
        right_front_motor.set_target(direction*this->right_front_motor.get_vec2d()*MAX_OF_TARGET);
        left_front_motor.set_target(direction*this->left_front_motor.get_vec2d()*MAX_OF_TARGET);
        left_back_motor.set_target(direction*this->left_back_motor.get_vec2d()*MAX_OF_TARGET);
        right_back_motor.set_target(direction*this->right_back_motor.get_vec2d()*MAX_OF_TARGET);
    }
    


}

inline std::unique_ptr<can_plugins2::msg::Frame> undercarriage::make_CAN_Frame(motor_name motor){
    switch (motor)
    {
        case motor_name::left_back_motor:
        return this->left_back_motor.make_frame();

        case motor_name::right_back_motor:
        return this->right_back_motor.make_frame();

        case motor_name::right_front_motor:
        return this->right_front_motor.make_frame();

        case motor_name::left_front_motor:
        return this->left_front_motor.make_frame();

        default:
        return can_utils::shirasu_target(990,0.0);
    }
}
inline std::unique_ptr<can_plugins2::msg::Frame> undercarriage::make_CAN_mode(motor_name motor,bool motor_state){
    if(motor_state){
        switch (motor)
    {
        case motor_name::left_back_motor:
        return this->left_back_motor.mode_vel();

        case motor_name::right_back_motor:
        return this->right_back_motor.mode_vel();

        case motor_name::right_front_motor:
        return this->right_front_motor.mode_vel();

        case motor_name::left_front_motor:
        return this->left_front_motor.mode_vel();

        default:
        return can_utils::generate_frame(0x990,0x5);
    }
    }
    else{
        switch (motor)
    {
        case motor_name::left_back_motor:
        return this->left_back_motor.mode_dis();

        case motor_name::right_back_motor:
        return this->right_back_motor.mode_dis();

        case motor_name::right_front_motor:
        return this->right_front_motor.mode_dis();

        case motor_name::left_front_motor:
        return this->left_front_motor.mode_dis();

        default:
        return can_utils::generate_frame(0x990,0x0);
    }
    }
}

inline void undercarriage::update(float x,float y,turn_direction turn_dir)
{
    this->set_direction(x,y);
    this->set_motor_power(turn_dir);
}


