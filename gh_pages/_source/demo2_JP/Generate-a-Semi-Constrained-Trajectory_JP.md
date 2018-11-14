# 準拘束軌道の生成
> 直交座標上の姿勢の配列から直交座標系軌道を生成します．各点はツールの Z 軸については回転自由とします．

## 演習用ソースファイル

  * `plan_and_run/src/plan_and_run_node.cpp` にあるメインアプリケーションのソースファイルを開きます．
  * メインプログラム内の `application.generateTrajectory()` という関数を見てください．
  * `plan_and_run/src/tasks/generate_trajectory.cpp` ファイルを開いて該当する関数を見てください．
  もしくは Qt/Eclipse で関数の任意の部分をクリックし "F3" を押すことによってその関数のソースファイルに移動します．
  * `//ROS_ERROR_STREAM("Task '"<<__FUNCTION__ <<"' is incomplete. Exiting"); exit(-1);` を含む最初の行をコメントアウトしてプログラムが直ちに終了しないようにしてください．

## コードを完成させる

 * ツールの軌道上の全てのポーズを生成するために `createLemniscate()` がどのように発動されているかを確認してください．
 * Z 軸回りの回転自由な点を規定するために [AxialSymmetric](http://docs.ros.org/indigo/api/descartes_trajectory/html/classdescartes__trajectory_1_1AxialSymmetricPt.html#a552cfabcd4891ea01886fa1b258de7f1) コンストラクタを使用してください．
 * 列挙子 [AxialSymmetricPt::FreeAxis::Z_AXIS](http://docs.ros.org/indigo/api/descartes_trajectory/html/classdescartes__trajectory_1_1AxialSymmetricPt.html#a65bf672235bde219db6667892efebbc2) は Z 軸回りに自由度を設定することが可能です．
 * `/*  Fill Code:` で始まるコメントブロックを見つけてそこにある記述どおりにコードを完成させてください．
 * 指示に従って `[ COMPLETE HERE ]` とある各部分を書き換えてください．

## コードのビルドと実行

 * `cd` で本演習の catkin ワークスペースに移動して `catkin build` でビルドしてください．
 * 次にアプリケーション launch フィアルを実行してください．

```
roslaunch plan_and_run demo_run.launch
```

## API リファレンス

* [descartes_trajectory::AxialSymmetricPt](http://docs.ros.org/indigo/api/descartes_trajectory/html/classdescartes__trajectory_1_1AxialSymmetricPt.html)
