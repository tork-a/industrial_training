<!--
# Place Box
>In this exercise, we will move the robot through the place motions while avoiding obstacles with an attached payload.  In addition, the gripper must be opened or close at the appropriate time in order to complete the task.
-->

# 箱を置く
> 本演習では付加された荷物も含めて環境上の障害物を避けながら置くようにロボットを動かします．
> またタスクを完遂するためには吸引グリッパを適切な時に開閉する必要があります．

<!--
## Locate Function

  * In the main program , locate the function call to '''application.place_box()'''.
  * Go to the source file of that function by clicking in any part of the function and pressing "F3".
  * Remove the fist line containing the following '''ROS_ERROR_STREAM ...''' so that the program runs.
-->

## 関数の位置

  * メインプログラム内の `application.place_box()` という関数を探してください．
  * 関数の任意の部分をクリックし "F3" を押すことによってその関数のソースファイルに移動します．
  * `ROS_ERROR STREAM ...` を含む最初の行を削除してプログラムが実行されるようにしてください．

<!--
## Complete Code

  * Find every line that begins with the comment "''Fill Code: ''" and read the description.  Then, replace every instance of the comment  "''ENTER CODE HERE''"
 with the appropriate line of code
-->

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

<!--  * The [[execute()|http://docs.ros.org/hydro/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroup.html#a82f1bb33058893e8a16fa49af24d689f]] method sends a motion plan to the robot. -->
  * [execute()](http://docs.ros.org/hydro/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroup.html#a82f1bb33058893e8a16fa49af24d689f) メソッドが動作計画をロボットに送っています．

<!--
## Build Code and Run

  * Compile the pick and place node  in Eclipse
-->

##  コードのビルドと実行

  * Qt でピック・アンド・プレース・ノードをコンパイルします．
```
Project -> Build Project
```
<!--  * Alternatively, in a terminal cd into the '''demo_manipulation''' directory and do the following -->
  * もしくはターミナルで本演習のワークスペースディレクトリに移動して次のコマンドを実行します．
```
catkin build --cmake-args -G 'CodeBlocks - Unix Makefiles' --pkg collision_avoidance_pick_and_place
```

<!--  * Run your node with the launch file: -->
  * launch ファイルでノードを実行します．
```
roslaunch collision_avoidance_pick_and_place ur5_pick_and_place.launch
```
<!--  * At this point your exercise is complete and the robot should move through the pick and place motions and then back to the wait pose. Congratulations! -->
  * この時点で演習が完成して，ロボットがピック・アンド・プレース動作を行い待機姿勢に戻るはずです．おめでとう！

<!-- ## API References -->
## API リファレンス

* [Useful " MoveGroup " Methods](http://ros.org/rosdoclite/groovy/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroup.html)
