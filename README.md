# CRS_harurobo　こいのぼり用足回りのやつ

## 説明
CRSの春ロボ用の足回りのやつです。

四輪オムニをshirasu、DCで動かす想定。

can_plugins2がないと動かない

## 使い方
`ros2 run joy joy_node`:joyのノードの起動
`ros2 run can_plugins2 <わすれた>`:CAN用のノードの起動
`ros2 run harurobo undercarriage`:足回りのコードの実行
