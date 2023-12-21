#include <math.h>
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

vec2d vec2d::make(float x,float y){
    return vec2d{.x = x,.y = y};
}

class motor{
    private:
    const vec2d direction;//モーターの向いている方向ベクトル
    float TARGET;//TARGET
    const int CAN_ID;//CAN ID

    public:
    motor(float x,float y,int CAN_ID)
    :direction(vec2d(x,y)),CAN_ID(CAN_ID),TARGET(0)
    {}//初期化
    void set_target(float power){
        this->TARGET = power;
    }//TARGETを代入
    friend auto operator*(vec2d a,motor b) -> float{
        return a*b.direction;
    }//内積


};


enum turn_direction{left_turn = -1,no_turn = 0,right_turn = 1};


class undercarriage{
    private:
    vec2d direction;//進みたい方向
    motor right_front_motor;
    motor left_front_motor;
    motor left_back_motor;
    motor right_back_motor;
    //四輪オムニ    

    public:
    undercarriage(int right_front_CAN_ID,int left_front_CAN_ID,int left_back_CAN_ID,int right_back_CAN_ID)
    :right_front_motor(motor(-cos45,cos45,right_front_CAN_ID)),left_front_motor(motor(cos45,cos45,left_front_CAN_ID)),
    left_back_motor(motor(-cos45,-cos45,left_back_CAN_ID)),right_back_motor(motor(cos45,-cos45,right_back_CAN_ID))
    {}//初期化
    void set_motor_power(turn_direction turn_dir);//4タイヤがうまく回るようにする
    void set_vector(float x,float y);//行きたい方向
    void send_data();//データを送る
    void update(float x,float y,turn_direction turn_dir);//他の関数を全部融合させた
};

void undercarriage::set_vector(float x,float y){
    this->direction.x = x;
    this->direction.y = y;
}

# define NUMBER 20
//多分TARGETの最大値になるはず

void undercarriage::set_motor_power(turn_direction turn_dir){

    if(turn_dir == 1){
        right_front_motor.set_target(NUMBER/5);
        left_front_motor.set_target(NUMBER/5);
        left_back_motor.set_target(NUMBER/5);
        right_back_motor.set_target(NUMBER/5);
    }
    else if(turn_dir == -1){
        right_front_motor.set_target(-NUMBER/5);
        left_front_motor.set_target(-NUMBER/5);
        left_back_motor.set_target(-NUMBER/5);
        right_back_motor.set_target(-NUMBER/5);
    }
    else{
        right_front_motor.set_target(direction*this->right_front_motor*NUMBER);
        left_front_motor.set_target(direction*this->left_front_motor*NUMBER);
        left_back_motor.set_target(direction*this->left_back_motor*NUMBER);
        right_back_motor.set_target(direction*this->right_back_motor*NUMBER);
    }
    


}

void undercarriage::update(float x,float y,turn_direction turn_dir)
{
    this->set_vector(x,y);
    this->set_motor_power(turn_dir);
    this->send_data();
}

void undercarriage::send_data(){
    //書いて
}