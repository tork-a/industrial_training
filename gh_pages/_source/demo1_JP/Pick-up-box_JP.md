# 箱をつかみ上げる

> 本演習では環境上の障害物を避けながら目標物をつかむようにロボットを動かします．
> 各姿勢の計画と吸引グリッパを適切に開閉することにより達成されます．
> また，MoveIt! における動作計画の作成方法も紹介します．

## 関数の位置

  * メインプログラム内の `application.pickup_box()` という関数を探してください．
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

 * 動作計画を生成するのに用いる `robot_state` オブジェクトの取扱方法を理解するために `set_atteched_object` メソッドを調べてみてください．
 * 全体の動作計画リクエストの定義方法と送られる方法を見るために `create_motion_plan` メソッドを調べてみてください．
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

  * 前回の演習に加えてロボットがつかむ動作群（アプローチ・つかみ・後退）を実行し，ターミナルには次のように表示されるはずです．

```
[ INFO] [1400555978.404435764]: Execution completed: SUCCEEDED
[ INFO] [1400555978.404919764]: Pick Move 2 Succeeded
[ERROR] [1400555978.405061541]: create_place_moves is not implemented yet.  Aborting.
```

## API リファレンス

* [Useful '''MoveGroup'''](http://docs.ros.org/hydro/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroup.html)
