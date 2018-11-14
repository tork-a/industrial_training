# ホームポジションへの移動

> アームを動かすために MoveIt! を使用します．

## 演習用ソースファイル

* `plan_and_run/src/plan_and_run_node.cpp` にあるメインアプリケーションのソースファイルを開きます．
* メインプログラム内の `application.moveHome()` という関数を見てください．
* `plan_and_run/src/tasks/move_home.cpp` ファイルを開いて該当する関数を見てください．
もしくは Qt/Eclipse で関数の任意の部分をクリックし "F3" を押すことによってその関数のソースファイルに移動します．
* `//ROS_ERROR_STREAM("Task '"<<__FUNCTION__ <<"' is incomplete. Exiting"); exit(-1);` を含む最初の行をコメントアウトしてプログラムが直ちに終了しないようにしてください．

## コードを完成させる

* ロボットをターゲットに向かって動かすために [move_group_interface::MoveGroup::move()](http://docs.ros.org/hydro/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroup.html#a4c63625e2e9eb5c342d1fc6732bd8cf7) メソッドを使用します．
* [moveit_msgs::MoveItErrorCodes](http://docs.ros.org/indigo/api/moveit_msgs/html/msg/MoveItErrorCodes.html) 構造体は `move()` 関数を呼び出し後の結果を確認に利用できる静的変数が含まれています．
* `/*  Fill Code:` で始まるコメントブロックを見つけてそこにある記述どおりにコードを完成させてください．
* 指示に従って `[ COMPLETE HERE ]` とある各部分を書き換えてください．

## コードのビルドと実行

* `cd` で本演習の catkin ワークスペースに移動して `catkin build` でビルドしてください．
* 次にアプリケーション launch フィアルを実行してください．

```
roslaunch plan_and_run demo_run.launch
```

## API リファレンス

* [setNamedTarget()](http://docs.ros.org/hydro/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroup.html#af6850334bb1b4f12e457257550d5f92c)
* [moveit::planning_interface::MoveGroup](http://docs.ros.org/hydro/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroup.html)
