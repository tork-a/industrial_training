# ピック・アンド・プレース演習パッケージ

> 本演習ではこれからの演習で利用する全ファイルを導入してその構成を習熟します．

## ワークスペースの作成と初期化

```
cp -r ~/industrial_training/exercises/Perception-Driven_Manipulation/template_ws ~/perception_driven_ws
cd ~/perception_driven_ws
source /opt/ros/kinetic/setup.bash
catkin init
```

## 依存するソースファイルのダウンロード

[wstool](http://wiki.ros.org/wstool) コマンドを用いて src/.rosinstall でリストアップされているリポジトリをダウンロードします．

```
cd ~/perception_driven_ws/src/
wstool update
```

## 依存する Debian パッケージのダウンロード

前提として [rosdep tool](http://wiki.ros.org/rosdep) がインストール
されていることを確認してください．確認できましたらワークスペース内の
**src** ディレクトリで下記コマンドを実行してください．

```
rosdep install --from-paths . --ignore-src -y
```

## ワークスペースのビルド

```
catkin build
```
もしビルドが失敗するようでしたら前の2つのステップを再び行って依存関係にあるものがすべてダウンロードされていることを確認してください．

## ワークスペース設定の反映

ワークスペースの大本のディレクトリにて下記コマンドを実行します．

```
source devel/setup.bash
```

## 演習用パッケージディレクトリ

```
cd ~/perception_driven_ws/src/collision_avoidance_pick_and_place/
```

## launch ディレクトリの確認

```
ur5_setup.launch     : ROS システム全体の起動（MoveIt!，RViz， 認識機能，ROS-Iドライバ，ロボット入出力周辺機器）
ur5_pick_and_place.launch   : ピック・アンド・プレース・ノードの実行
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
