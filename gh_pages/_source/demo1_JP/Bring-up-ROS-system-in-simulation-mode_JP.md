# シミュレーションモードの開始

シミュレーションモードで動作するロボットの ROS システムを起動します．

## セットアップ launch ファイルをシミュレーションモードで実行（ロボットとセンサともにシミュレーション）

ターミナルで下記のとおり実行します．

```
roslaunch collision_avoidance_pick_and_place ur5_setup.launch
```

RViz に初期位置にあるロボットを含む全ての作業セルコンポーネントが表示
されてシステムが準備された状態になります．ただしピック・アンド・プレー
ス・ノードを実行するまでは何も動きません．


## シミュレーションロボット＋実機センサのセットアップ

```
roslaunch collision_avoidance_pick_and_place ur5_setup.launch sim_sensor:=false
```

## 実機ロボット＋シミュレーションセンサのセットアップ

```
roslaunch collision_avoidance_pick_and_place ur5_setup.launch sim_robot:=false robot_ip:= [robot ip]
```

## 実機ロボット＋実機センサのセットアップ

```
roslaunch collision_avoidance_pick_and_place ur5_setup.launch sim_robot:=false robot_ip:= [robot ip] sim_sensor:=false sim_gripper:=false
```
