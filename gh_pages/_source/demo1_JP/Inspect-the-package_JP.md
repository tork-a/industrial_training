# Inspect the "pick_and_place_exercise" Package
>In this exercise, we will get familiar with all the files that you'll be interacting with throughout these exercises.

# ピック・アンド・プレース演習パッケージ

> 本演習ではこれからの演習で利用する全ファイルを導入してその構成を習熟します．

## Acquire and initialize the Workspace
## ワークスペースの作成と初期化
```
cp -r ~/industrial_training/exercises/Perception-Driven_Manipulation/template_ws ~/perception_driven_ws
cd ~/perception_driven_ws
source /opt/ros/kinetic/setup.bash
catkin init
```

## Download source dependencies
>Use the [wstool](http://wiki.ros.org/wstool) command to download the repositories listed in the **src/.rosinstall** file

## 依存するソースファイルのダウンロード
> [wstool](http://wiki.ros.org/wstool) コマンドを用いて src/.rosinstall でリストアップされているリポジトリをダウンロードします．
```
cd ~/perception_driven_ws/src/
wstool update
```

## Download debian dependencies
>Make sure you have installed and configured the [rosdep tool](http://wiki.ros.org/rosdep).
>Then, run the following command from the **src** directory of your workspace.

## 依存する Debian パッケージのダウンロード
> 前提として [rosdep tool](http://wiki.ros.org/rosdep) がインストールされていることを確認してください．
> 確認できましたらワークスペース内の **src** ディレクトリで下記コマンドを実行してください．
```
rosdep install --from-paths . --ignore-src -y
```

## Build your workspace
```
catkin build
```
>If the build fails then revisit the previous two steps to make sure all the dependencies were downloaded.

## ワークスペースのビルド
```
catkin build
```
> もしビルドが失敗するようでしたら前の2つのステップを再び行って依存関係にあるものがすべてダウンロードされていることを確認してください．


## Source the workspace
> Run the following command from your workspace parent directory

## ワークスペース設定の反映
> ワークスペースの大本のディレクトリにて下記コマンドを実行します．
```
source devel/setup.bash
```


## Locate and navigate into the package
## 演習用パッケージディレクトリへの移動
```
cd ~/perception_driven_ws/src/collision_avoidance_pick_and_place/
```

## Look into each file in the launch directory
```
ur5_setup.launch     : Brings up the entire ROS system (MoveIt!, rviz, perception, ROS-I drivers, robot I/O peripherals)
ur5_pick_and_place.launch   : Runs your pick and place node.
```

## launch ディレクトリの確認
```
ur5_setup.launch     : ROS システム全体の起動 (MoveIt!, RViz, 認識機能, ROS-I ドライバ, ロボット入出力周辺機器)
ur5_pick_and_place.launch   : ピック・アンド・プレース・ノードの実行
```


## Look into the config directory

```
ur5/
 - pick_and_place_parameters.yaml    : List of parameters read by the pick and place node.
 - rviz_config.rviz   : Rviz configuration file for display properties.
 - target_recognition_parameters.yaml    : Parameters used by the target recognition service for detecting the box from the sensor data.
 - test_cloud_obstacle_descriptions.yaml    : Parameters used to generate simulated sensor data (simulated sensor mode only)
 - collision_obstacles.txt   : Description of each obstacle blob added to the simulated sensor data (simulated sensor mode only)
```

## config ディレクトリの確認

```
ur5/
 - pick_and_place_parameters.yaml    : ピック・アンド・プレース・ノードで読まれるパラメータのリスト
 - rviz_config.rviz   : RViz の表示プロパティの設定
 - target_recognition_parameters.yaml    : センサ情報から箱を検出する目標認識サービスのパラメータ
 - test_cloud_obstacle_descriptions.yaml    : センサ情報をシミュレートするときのパラメータ（シミュレーションセンサモードのみ）
 - collision_obstacles.txt   : シミュレートされたセンサ情報に加える各障害物ブロブの記述（シミュレーションセンサモードのみ）
```

## Look into the src directory

```
nodes:
 - pick_and_place_node.cpp : Main application thread. Contains all necessary headers and function calls.

tasks: Source files with incomplete function definitions.  You will fill with code where needed in order to complete the exercise.
 - create_motion_plan.cpp
 - create_pick_moves.cpp
 - create_place_moves.cpp
 - detect_box_pick.cpp
 - pickup_box.cpp
 - place_box.cpp
 - move_to_wait_position.cpp
 - set_attached_object.cpp
 - set_gripper.cpp

utilities:  
 - pick_and_place_utilities.cpp : Contains support functions that will help you complete the exercise.
```

## src ディレクトリの確認

```
nodes :
 - pick_and_place_node.cpp : 全ての必要なヘッダや関数呼び出しを行うメインアプリケーションスレッド

tasks : 関数定義が不完全なソースファイルです．演習課題を完成させるために必要なコードを補完します．
 - create_motion_plan.cpp
 - create_pick_moves.cpp
 - create_place_moves.cpp
 - detect_box_pick.cpp
 - pickup_box.cpp
 - place_box.cpp
 - move_to_wait_position.cpp
 - set_attached_object.cpp
 - set_gripper.cpp

utilities:  
 - pick_and_place_utilities.cpp : 演習課題を完成するための補助関数
```
