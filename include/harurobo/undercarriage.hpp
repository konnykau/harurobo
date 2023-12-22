#include <math.h>
#include <can_utils.hpp>
#include <send_data.hpp>
#define cos45 1/sqrt(2)
// constexpr float cos45 = 1/sqrt(2);

struct vec2d{
    float x;
    float y;
    //ベクトル成分
    friend auto operator*(const vec2d a,const vec2d b) -> float{
        return a.x*b.x+b.y*a.y;
    }//内積
    static vec2d make(float x,float y);//ｘ,ｙを要素にもつベクトルを返す
};

inline vec2d vec2d::make(float x,float y){
    return vec2d{.x = x,.y = y};
}

class motor{
    private:
    const vec2d direction;//モーターの向いている方向ベクトル
    const int CAN_ID;//CAN ID
    float TARGET;//TARGET
    std::unique_ptr<can_plugins2::msg::Frame> CAN_Frame;//can通信で送るもの    

    public:
    motor(float x,float y,int CAN_ID)
    :direction(vec2d(x,y)),CAN_ID(CAN_ID),TARGET(0)
    {}//初期化
    void set_target(float power){
        this->TARGET = power;
    }//TARGETを代入
    std::unique_ptr<can_plugins2::msg::Frame> make_frame()
    {
        this->CAN_Frame = can_utils::shirasu_target(this->CAN_ID + 1,this->TARGET);
        return can_utils::shirasu_target(this->CAN_ID + 1,this->TARGET);
    }
    int get_CAN_ID(){
        return this->CAN_ID;
    }
    // float get_TARGET(){
    //     return this->TARGET;
    // }
    vec2d get_vec2d(){
        return direction;
    }


};


enum class turn_direction{left_turn, no_turn, right_turn};
enum class motor_name{right_front_motor, left_front_motor, left_back_motor, right_back_motor};


class undercarriage{
    private:
    vec2d direction;//進みたい方向
    public:
    motor right_front_motor;
    motor left_front_motor;
    motor left_back_motor;
    motor right_back_motor;
    //四輪オムニ    

    
    undercarriage(int right_front_CAN_ID,int left_front_CAN_ID,int left_back_CAN_ID,int right_back_CAN_ID)
    :right_front_motor(motor(-cos45,cos45,right_front_CAN_ID)),left_front_motor(motor(cos45,cos45,left_front_CAN_ID)),
    left_back_motor(motor(-cos45,-cos45,left_back_CAN_ID)),right_back_motor(motor(cos45,-cos45,right_back_CAN_ID))
    {}//初期化
    void set_motor_power(turn_direction turn_dir);//4タイヤがうまく回るようにする
    void set_direction(float x,float y);//行きたい方向
    std::unique_ptr<can_plugins2::msg::Frame> make_CAN_Frame(motor_name motor);//CANパッケージを詰め込む
    void update(float x,float y,turn_direction turn_dir);//他の関数を全部融合させた
    int get_CAN_ID(motor_name motor);
    // float get_TARGET(motor_name motor);
};

inline void undercarriage::set_direction(float x,float y){
    this->direction.x = x;
    this->direction.y = y;
}

# define MAX_OF_TARGET 20
//多分TARGETの最大値になるはず

inline void undercarriage::set_motor_power(turn_direction turn_dir){

    if(turn_dir == turn_direction::left_turn){
        right_front_motor.set_target(MAX_OF_TARGET/5);
        left_front_motor.set_target(MAX_OF_TARGET/5);
        left_back_motor.set_target(MAX_OF_TARGET/5);
        right_back_motor.set_target(MAX_OF_TARGET/5);
    }
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
        return can_utils::shirasu_target(200,0.0);
    }
}

inline void undercarriage::update(float x,float y,turn_direction turn_dir)
{
    this->set_direction(x,y);
    this->set_motor_power(turn_dir);
}

inline int undercarriage::get_CAN_ID(motor_name motor){
    switch (motor)
    {
        case motor_name::left_back_motor:
        return this->left_back_motor.get_CAN_ID();

        case motor_name::right_back_motor:
        return this->right_back_motor.get_CAN_ID();

        case motor_name::right_front_motor:
        return this->right_front_motor.get_CAN_ID();

        case motor_name::left_front_motor:
        return this->left_front_motor.get_CAN_ID();

        default:
        return -1;
    }
}

// inline float undercarriage::get_TARGET(motor_name motor){
//     switch (motor)
//     {
//     case motor_name::left_back_motor:
//         return this->left_back_motor.get_TARGET();
    
//     case motor_name::right_back_motor:
//         return this->right_back_motor.get_TARGET();
    
//     case motor_name::right_front_motor:
//         return this->right_front_motor.get_TARGET();
    
//     case motor_name::left_front_motor:
//         return this->left_front_motor.get_TARGET();
    
//     default:
//     return 0;
//     }
// }
