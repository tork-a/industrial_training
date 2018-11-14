<!--
# Start in Simulation Mode
>In this exercise, we will start a ROS system that is ready to move the robot in simulation mode.
-->

# シミュレーションモードの開始
> シミュレーションモードで動作するロボットの ROS システムを起動します．

<!--
## Run setup launch file in simulation mode (simulated robot and sensor)

In a terminal
```
roslaunch collision_avoidance_pick_and_place ur5_setup.launch
```

Rviz will display all the workcell components including the robot in its default position; at this point
your system is ready.  However no motion will take place until we run the pick and place node.
-->

## セットアップ launch ファイルをシミュレーションモードで実行（ロボットとセンサともにシミュレーション）

ターミナルで下記のとおり実行します．
```
roslaunch collision_avoidance_pick_and_place ur5_setup.launch
```

RViz に初期位置にあるロボットを含む全ての作業セルコンポーネントが表示されてシステムが準備された状態になります．
ただしピック・アンド・プレース・ノードを実行するまでは何も動きません．

<!--
## Setup for real sensor and simulated robot
```
roslaunch collision_avoidance_pick_and_place ur5_setup.launch sim_sensor:=false
```
-->

## シミュレーションロボット＋実機センサのセットアップ
```
roslaunch collision_avoidance_pick_and_place ur5_setup.launch sim_sensor:=false
```

<!--
## Setup for real robot and simulated sensor data
```
roslaunch collision_avoidance_pick_and_place ur5_setup.launch sim_robot:=false robot_ip:= [robot ip]
```
-->

## 実機ロボット＋シミュレーションセンサのセットアップ
```
roslaunch collision_avoidance_pick_and_place ur5_setup.launch sim_robot:=false robot_ip:= [robot ip]
```

<!--
## Setup for real robot and real sensor
```
roslaunch collision_avoidance_pick_and_place ur5_setup.launch sim_robot:=false robot_ip:= [robot ip] sim_sensor:=false sim_gripper:=false
```
-->

## 実機ロボット＋実機センサのセットアップ
```
roslaunch collision_avoidance_pick_and_place ur5_setup.launch sim_robot:=false robot_ip:= [robot ip] sim_sensor:=false sim_gripper:=false
```
