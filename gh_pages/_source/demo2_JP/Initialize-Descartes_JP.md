# 直交座標系の初期化

> 直交座標系ロボットモデルとツールの準拘束軌道を計画するために利用する軌道プランナのセットアップを行います．

## 演習用ソースファイル

* `plan_and_run/src/plan_and_run_node.cpp` にあるメインアプリケーションのソースファイルを開きます．
* メインプログラム内の `application.initDescartes()` という関数を見てください．
* `plan_and_run/src/tasks/init_descartes.cpp` ファイルを開いて該当する関数を見てください．
もしくは Qt/Eclipse で関数の任意の部分をクリックし "F3" を押すことによってその関数のソースファイルに移動します．
* `//ROS_ERROR_STREAM("Task '"<<__FUNCTION__ <<"' is incomplete. Exiting"); exit(-1);` を含む最初の行をコメントアウトしてプログラムが直ちに終了しないようにしてください．

## コードを完成させる

* 適切にロボットが初期化されるように [descartes_core::RobotModel::initialize()](http://docs.ros.org/indigo/api/descartes_planner/html/classdescartes__planner_1_1DensePlanner.html#af6e9db3c1dec85046fc836136cf7b0fb) メソッドを呼び出します．
* また `robot_model_` 変数を `descartes_core::!DensePlanner::initialize()` にパスを通して直交座標系プランナを初期化します．
* `/*  Fill Code:` で始まるコメントブロックを見つけてそこにある記述どおりにコードを完成させてください．
* 指示に従って `[ COMPLETE HERE ]` とある各部分を書き換えてください．

## コードのビルドと実行

* `cd` で本演習の catkin ワークスペースに移動して `catkin build` でビルドしてください．
* 次にアプリケーション launch フィアルを実行してください．

```
roslaunch plan_and_run demo_run.launch
```

## API リファレンス

* [descartes_core::RobotModel](http://docs.ros.org/indigo/api/descartes_core/html/classdescartes__core_1_1RobotModel.html)
* [descartes_planner::DensePlanner](http://docs.ros.org/indigo/api/descartes_planner/html/classdescartes__planner_1_1DensePlanner.html)
