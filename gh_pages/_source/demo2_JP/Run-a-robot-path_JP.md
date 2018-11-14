# ロボット軌道の実行
> 動作計画で作成されたロボットの軌道を使ってロボットを動かします．

## 演習用ソースファイル

* `plan_and_run/src/plan_and_run_node.cpp` にあるメインアプリケーションのソースファイルを開きます．
* メインプログラム内の `application.runPath()` という関数を見てください．
* `plan_and_run/src/tasks/run_path.cpp` ファイルを開いて該当する関数を見てください．
もしくは Qt/Eclipse で関数の任意の部分をクリックし "F3" を押すことによってその関数のソースファイルに移動します．
* `//ROS_ERROR_STREAM("Task '"<<__FUNCTION__ <<"' is incomplete. Exiting"); exit(-1);` を含む最初の行をコメントアウトしてプログラムが直ちに終了しないようにしてください．

## コードを完成させる

* `/*  Fill Code:` で始まるコメントブロックを見つけてそこにある記述どおりにコードを完成させてください．
* 指示に従って `[ COMPLETE HERE ]` とある各部分を書き換えてください

## コードのビルドと実行

* `cd` で本演習の catkin ワークスペースに移動して `catkin build` でビルドしてください．
* 次にアプリケーション launch フィアルを実行してください．

```
roslaunch plan_and_run demo_run.launch
```

## API リファレンス

* [MoveGroupInterface::move()](http://docs.ros.org/hydro/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroup.html#a4c63625e2e9eb5c342d1fc6732bd8cf7)
