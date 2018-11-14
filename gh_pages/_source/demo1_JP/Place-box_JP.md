# 箱を置く
> 本演習では付加された荷物も含めて環境上の障害物を避けながら置くようにロボットを動かします．
> またタスクを完遂するためには吸引グリッパを適切な時に開閉する必要があります．

## 関数の位置

  * メインプログラム内の `application.place_box()` という関数を探してください．
  * 関数の任意の部分をクリックし "F3" を押すことによってその関数のソースファイルに移動します．
  * `ROS_ERROR STREAM ...` を含む最初の行を削除してプログラムが実行されるようにしてください．

## コードを完成させる

  * 各箇所にある `Fill Code:` コメントを見つけてその記述を読んでください．そして `ENTER CODE HERE` と書いてある各部分を適切なコードで書き換えてください．

```
/* Fill Code:
     .
     .
     .
*/
/* ========  ENTER CODE HERE ======== */
```

* [execute()](http://docs.ros.org/hydro/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroup.html#a82f1bb33058893e8a16fa49af24d689f) メソッドが動作計画をロボットに送っています．

##  コードのビルドと実行

* Qt でピック・アンド・プレース・ノードをコンパイルします．

```
Project -> Build Project
```

* もしくはターミナルで本演習のワークスペースディレクトリに移動して次のコマンドを実行します．

```
catkin build --cmake-args -G 'CodeBlocks - Unix Makefiles' --pkg collision_avoidance_pick_and_place
```

* launch ファイルでノードを実行します．

```
roslaunch collision_avoidance_pick_and_place ur5_pick_and_place.launch
```

* この時点で演習が完成して，ロボットがピック・アンド・プレース動作を行い待機姿勢に戻るはずです．おめでとう！

## API リファレンス

* [Useful " MoveGroup " Methods](http://ros.org/rosdoclite/groovy/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroup.html)
