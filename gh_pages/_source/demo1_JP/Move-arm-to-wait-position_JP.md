# アームの待機ポジションへの移動

MoveIt! の `MoveGroup` クラスではロボットを動作させるための様々な方法
が用意されています．`MoveGroup` により目標関節角度や直交座標系上の目標
位置，セットアップ・アシスタントにより予め定義された位置にロボットを動
かすことができます．本演習では予め定義された関節角度姿勢にロボットを動
かします．

## 関数の位置

* メインプログラム内の `application.move_to_wait_position()` という
    関数を探してください．
* 関数の任意の部分をクリックし "F3" を押すことによってその関数のソー
    スファイルに移動します．
* もしくは `[Source directory]/src/tasks/move_to_wait_position.cpp'`
    ファイルを参照してください．
* `ROS_ERROR STREAM ...` を含む最初の行を削除してプログラムが実行さ
    れるようにしてください．

## コードを完成させる

* 各箇所にある `Fill Code:` コメントを見つけてそこの説明を読んでくださ
    い．そして `ENTER CODE HERE` と書いてある各部分を適切なコードで書
    き換えてください．

```
/* Fill Code:
     .
     .
     .
*/
/* ========  ENTER CODE HERE ======== */
```

<!--  * The name of the predefined "wait" pose was saved in the global variable '''cfg.WAIT_POSE_NAME''' during initialization. -->

* 初期化のときに予め定義された `wait` ポーズが `cfg.WAIT_POSE_NAME` に
  保存されています．

<!--##  Build Code and Run -->
##  コードのビルドと実行

<!--  * Compile the pick and place node  in Eclipse -->
* Qt でピック・アンド・プレース・ノードをコンパイルします．

```
Project -> Build Project
```

<!--  * Alternatively, in a terminal cd into the '''demo_manipulation''' directory and do the following -->

* もしくはターミナルで本演習のワークスペースディレクトリに移動して次のコマンドを実行します．

```
catkin build --cmake-args -G 'CodeBlocks - Unix Makefiles' --pkg collision_avoidance_pick_and_place
source ./devel/setup.bash
```

<!--  * Run your node with the launch file: -->
* launch ファイルでノードを実行します．

```
roslaunch collision_avoidance_pick_and_place ur5_pick_and_place.launch
```
<!--  * If the robot is not already in the wait position, it should move to the wait position. In the terminal, you will see something like the following message: -->

* ロボットが待機ポジションにない場合には待機ボジションに移動します．ターミナルには次のようなメッセージが表示されるはずです．

```
[ INFO] [1400553673.460328538]: Move wait Succeeded
[ERROR] [1400553673.460434627]: set_gripper is not implemented yet.  Aborting.
```

<!-- ## API References -->
## API リファレンス

* [setNamedTarget()](http://docs.ros.org/hydro/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroup.html#af6850334bb1b4f12e457257550d5f92c)
* [move()](http://docs.ros.org/hydro/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroup.html#a4c63625e2e9eb5c342d1fc6732bd8cf7)
