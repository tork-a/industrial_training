# ロボット軌道の計画

> ロボット軌道計画を行うために前節の軌道を直交座標系プランナに送ります．

## 演習用ソースファイル

* `plan_and_run/src/plan_and_run_node.cpp` にあるメインアプリケーションのソースファイルを開きます．
* メインプログラム内の `application.planPath()` という関数を見てください．
* `plan_and_run/src/tasks/plan_path.cpp` ファイルを開いて該当する関数を見てください．
もしくは Qt/Eclipse で関数の任意の部分をクリックし "F3" を押すことによってその関数のソースファイルに移動します．
* `//ROS_ERROR_STREAM("Task '"<<__FUNCTION__ <<"' is incomplete. Exiting"); exit(-1);` を含む最初の行をコメントアウトしてプログラムが直ちに終了しないようにしてください．

## コードを完成させる

* 任意の関節ポーズに最も近いロボット関節の値を取得するために [AxialSymmetricPt::getClosesJointPose()](http://docs.ros.org/indigo/api/descartes_trajectory/html/classdescartes__trajectory_1_1CartTrajectoryPt.html#a1252c8f49a6e5a7d563b6d4a256b553b) の利用のされかたを確認してください．また，ここでは複数の有効な関節構成ではなく，開始と終了などの1つの関節ポーズを選択することも可能です．
* 動作計画を算出するために [DensePlanner::planPath()](http://docs.ros.org/indigo/api/descartes_planner/html/classdescartes__planner_1_1DensePlanner.html#a2181f674af57b92023deabb5e8323a2a) メソッドを呼び出します．
* 動作計画作成に成功したら [DensePlanner::getPath()](http://docs.ros.org/indigo/api/descartes_planner/html/classdescartes__planner_1_1DensePlanner.html#aafd40b5dc5ed39b4f10e9b47fda0419f) メソッドを使ってプランナから軌道を取得して `output_path` 変数に格納します．
* `/*  Fill Code:` で始まるコメントブロックを見つけてそこにある記述どおりにコードを完成させてください．
* 指示に従って `[ COMPLETE HERE ]` とある各部分を書き換えてください

## コードのビルドと実行

* `cd` で本演習の catkin ワークスペースに移動して `catkin build` でビルドしてください．
* 次にアプリケーション launch フィアルを実行してください．

```
roslaunch plan_and_run demo_run.launch
```

## API リファレンス

* [descartes_planner::DensePlanner](http://docs.ros.org/indigo/api/descartes_planner/html/classdescartes__planner_1_1DensePlanner.html)
