<!--
# Plan a Robot Path
>In this exercise, we'll pass our trajectory to the Descartes planner in order to plan a robot path.
-->

# ロボット軌道の計画
> ロボット軌道計画を行うために前節の軌道を直交座標系プランナに送ります．

<!--
## Locate Exercise Source File

  * Go to the main application source file located in '''plan_and_run/src/plan_and_run_node.cpp'''.
  * In the main program , locate the function call to '''application.planPath()'''.
  * Go to the source file for that function located in the '''plan_and_run/src/tasks/plan_path.cpp'''. Alternatively, in Eclipse you can click in any part of the function and press "F3" to bring up that file.
  * Comment out the first line containing the ```//ROS_ERROR_STREAM("Task '"<<__FUNCTION__ <<"' is incomplete. Exiting"); exit(-1);``` entry so that the function doesn't quit immediately.
-->

## 演習用ソースファイル

  * `plan_and_run/src/plan_and_run_node.cpp` にあるメインアプリケーションのソースファイルを開きます．
  * メインプログラム内の `application.planPath()` という関数を見てください．
  * `plan_and_run/src/tasks/plan_path.cpp` ファイルを開いて該当する関数を見てください．
  もしくは Qt/Eclipse で関数の任意の部分をクリックし "F3" を押すことによってその関数のソースファイルに移動します．
  * `//ROS_ERROR_STREAM("Task '"<<__FUNCTION__ <<"' is incomplete. Exiting"); exit(-1);` を含む最初の行をコメントアウトしてプログラムが直ちに終了しないようにしてください．

<!--
## Complete Code

 * Observe the use of the [[AxialSymmetricPt::getClosesJointPose() | http://docs.ros.org/indigo/api/descartes_trajectory/html/classdescartes__trajectory_1_1CartTrajectoryPt.html#a1252c8f49a6e5a7d563b6d4a256b553b]] in order to get the joint values of the robot that is closest to an arbitrary joint pose.  Furthermore, this step allows us to select a single joint pose for the start and end rather than multiple valid joint configurations.
 * Call the [[DensePlanner::planPath() | http://docs.ros.org/indigo/api/descartes_planner/html/classdescartes__planner_1_1DensePlanner.html#a2181f674af57b92023deabb5e8323a2a]] method in order to compute a motion plan.
 * When planning succeeds, use the [[DensePlanner::getPath() | http://docs.ros.org/indigo/api/descartes_planner/html/classdescartes__planner_1_1DensePlanner.html#aafd40b5dc5ed39b4f10e9b47fda0419f]] method in order to retrieve the path from the planner and save it into the '''output_path''' variable.
 * Find comment block that starts with ```/*  Fill Code:``` and complete as described.

 * Replace every instance of ```[ COMPLETE HERE ]``` accordingly.
-->

## コードを完成させる

 * 任意の関節ポーズに最も近いロボット関節の値を取得するために [AxialSymmetricPt::getClosesJointPose()](http://docs.ros.org/indigo/api/descartes_trajectory/html/classdescartes__trajectory_1_1CartTrajectoryPt.html#a1252c8f49a6e5a7d563b6d4a256b553b) の利用のされかたを確認してください．また，ここでは複数の有効な関節構成ではなく，開始と終了などの1つの関節ポーズを選択することも可能です．
 * 動作計画を算出するために [DensePlanner::planPath()](http://docs.ros.org/indigo/api/descartes_planner/html/classdescartes__planner_1_1DensePlanner.html#a2181f674af57b92023deabb5e8323a2a) メソッドを呼び出します．
 * 動作計画作成に成功したら [DensePlanner::getPath()](http://docs.ros.org/indigo/api/descartes_planner/html/classdescartes__planner_1_1DensePlanner.html#aafd40b5dc5ed39b4f10e9b47fda0419f) メソッドを使ってプランナから軌道を取得して `output_path` 変数に格納します．
 * `/*  Fill Code:` で始まるコメントブロックを見つけてそこにある記述どおりにコードを完成させてください．
 * 指示に従って `[ COMPLETE HERE ]` とある各部分を書き換えてください

<!--
## Build Code and Run

 * `cd` into your catkin workspace and run `catkin build`
 * Then run the application launch file:
```
roslaunch plan_and_run demo_run.launch
```
-->

## コードのビルドと実行

 * `cd` で本演習の catkin ワークスペースに移動して `catkin build` でビルドしてください．
 * 次にアプリケーション launch フィアルを実行してください．
```
roslaunch plan_and_run demo_run.launch
```

<!-- ## API References -->
## API リファレンス

* [descartes_planner::DensePlanner](http://docs.ros.org/indigo/api/descartes_planner/html/classdescartes__planner_1_1DensePlanner.html)
