# CRS_harurobo　こいのぼり用足回りのやつ

## 説明
CRSの春ロボ用の足回りのやつです。

四輪オムニをshirasu、DCで動かす想定。

can_plugins2がないと動かない

## 使い方
```C++
undercarriage Undercarriage(0x100,0x200,0x300,0x400);//右上から反時計回りにcan idで初期化

//mode velにする
{
can_pub_->publish(std::move(Undercarriage.make_CAN_mode(motor_name::right_front_motor,1)));
can_pub_->publish(std::move(Undercarriage.make_CAN_mode(motor_name::left_front_motor,1)));
can_pub_->publish(std::move(Undercarriage.make_CAN_mode(motor_name::left_back_motor,1)));
can_pub_->publish(std::move(Undercarriage.make_CAN_mode(motor_name::right_back_motor,1)));
}

//mode disにする
{
can_pub_->publish(std::move(Undercarriage.make_CAN_mode(motor_name::right_front_motor,0)));
can_pub_->publish(std::move(Undercarriage.make_CAN_mode(motor_name::left_front_motor,0)));
can_pub_->publish(std::move(Undercarriage.make_CAN_mode(motor_name::left_back_motor,0)));
can_pub_->publish(std::move(Undercarriage.make_CAN_mode(motor_name::right_back_motor,0)));
}

{//移動
if(/*左回転トリガー*/){
    this->Undercarriage.update(x,y,turn_direction::left_turn);
}//left turn
else if(/*右回転トリガー*/){
    this->Undercarriage.update(x,y,turn_direction::right_turn);
}
else{//無回転
    this->Undercarriage.update(x,y,turn_direction::no_turn);
}
//引数は平行移動x成分、y成分,回転方向
//回転成分の引数にとれるのはturn_direction::left_turn,turn_direction::right_turn,turn_direction::no_turnのみ
can_pub_->publish(Undercarriage.make_CAN_Frame(motor_name::right_front_motor));
can_pub_->publish(Undercarriage.make_CAN_Frame(motor_name::left_front_motor));
can_pub_->publish(Undercarriage.make_CAN_Frame(motor_name::left_back_motor));
can_pub_->publish(Undercarriage.make_CAN_Frame(motor_name::right_back_motor));
}
```

## 発射方法
`ros2 run joy joy_node`:joyのノードの起動

`ros2 run can_plugins2 <Tab二回>`:CAN用のノードの起動

`ros2 run harurobo undercarriage`:足回りのコードの実行

## くそな点
・undercarriage::updateの実装がくそ<-なぜこうしたのだ<-もしかしたら合理的なのかも？（改善方法が思いつかなかっただけ）