#include <math.h>
#include <can_utils.hpp>
#include "harurobo/send_data.hpp"
#include "harurobo/fry_lib/math.hpp"
#include "harurobo/fry_lib/vector.hpp"
constexpr float cos45 = 1/FRY::sqrt(2);

class motor{
    private:
    const FRY::vec2d direction;//モーターの向いている方向ベクトル
    const uint16_t CAN_ID;//CAN ID
    float TARGET;//TARGET 

    public:
    motor(float x,float y,uint16_t CAN_ID)
    :direction(FRY::vec2d(x,y)),CAN_ID(CAN_ID),TARGET(0)
    {}//初期化
    void set_target(float power){
        this->TARGET = power;
    }//TARGETを代入
    std::unique_ptr<can_plugins2::msg::Frame> make_frame()
    {
        return can_utils::shirasu_target(this->CAN_ID + 1,this->TARGET);
    }
    std::unique_ptr<can_plugins2::msg::Frame> mode_vel()
    {
        return can_utils::generate_frame(this->CAN_ID,static_cast<uint8_t>(0x5));
    }
    std::unique_ptr<can_plugins2::msg::Frame> mode_dis()
    {
        return can_utils::generate_frame(this->CAN_ID,static_cast<uint8_t>(0x0));
    }
    FRY::vec2d get_vec2d(){
        return this->direction;
    }


};


enum class turn_direction{left_turn, no_turn, right_turn};//回転するかしないか
enum class motor_name{right_front_motor, left_front_motor, left_back_motor, right_back_motor};//モーターの名前


class undercarriage{
    private:
    FRY::vec2d direction;//進みたい方向
    
    motor right_front_motor;
    motor left_front_motor;
    motor left_back_motor;
    motor right_back_motor;
    //四輪オムニ    
    public:
    
    undercarriage(uint16_t right_front_CAN_ID,uint16_t left_front_CAN_ID,uint16_t left_back_CAN_ID,uint16_t right_back_CAN_ID)
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
    constexpr float MAX_OF_TARGET = 20.0;
    constexpr float TURN_TARGET = 5.0;
    //多分TARGETの最大値になるはず
    float RF_TARGET = 0;
    float LF_TARGET = 0;
    float LB_TARGET = 0;
    float RB_TARGET = 0;

    if(turn_dir == turn_direction::left_turn){
        // right_front_motor.set_target(TURN_TARGET);
        // left_front_motor.set_target(TURN_TARGET);
        // left_back_motor.set_target(TURN_TARGET);
        // right_back_motor.set_target(TURN_TARGET);
        RF_TARGET = TURN_TARGET;
        LF_TARGET = TURN_TARGET;
        LB_TARGET = TURN_TARGET;
        RB_TARGET = TURN_TARGET;
    }
    else if(turn_dir == turn_direction::right_turn){
        // right_front_motor.set_target(-TURN_TARGET);
        // left_front_motor.set_target(-TURN_TARGET);
        // left_back_motor.set_target(-TURN_TARGET);
        // right_back_motor.set_target(-TURN_TARGET);
        RF_TARGET = -TURN_TARGET;
        LF_TARGET = -TURN_TARGET;
        LB_TARGET = -TURN_TARGET;
        RB_TARGET = -TURN_TARGET;
    }
    RF_TARGET += direction*this->right_front_motor.get_vec2d()*MAX_OF_TARGET;
    LF_TARGET += direction*this->left_front_motor.get_vec2d()*MAX_OF_TARGET;
    LB_TARGET += direction*this->left_back_motor.get_vec2d()*MAX_OF_TARGET;
    RB_TARGET += direction*this->right_back_motor.get_vec2d()*MAX_OF_TARGET;


    right_front_motor.set_target(RF_TARGET);
    left_front_motor.set_target(LF_TARGET);
    left_back_motor.set_target(LB_TARGET);
    right_back_motor.set_target(RB_TARGET);
    


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
        return can_utils::shirasu_target(0x990,static_cast<float>(0));
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
            return can_utils::generate_frame(0x990,static_cast<uint8_t>(0x5));
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
            return can_utils::generate_frame(0x990,static_cast<uint8_t>(0x0));
        }
    }
}

inline void undercarriage::update(float x,float y,turn_direction turn_dir)
{
    this->set_direction(x,y);
    this->set_motor_power(turn_dir);
}


