# Open Gripper
>In this exercise, the objective is to use a "grasp action client" to send a grasp goal that will open the gripper.

# グリッパを開く
> グリッパを開く目標を送るために "grasp action client" を利用することを目的としています．

## Locate Function

  * In the main program , locate the function call to '''application.set_gripper()'''.
  * Go to the source file of that function by clicking in any part of the function and pressing "F3".
  * Remove the fist line containing the following '''ROS_ERROR_STREAM ...''' so that the program runs.

## 関数の位置

  * メインプログラム内の `application.set_gripper()` という関数を探してください．
  * 関数の任意の部分をクリックし "F3" を押すことによってその関数のソースファイルに移動します．
  * `ROS_ERROR STREAM ...` を含む最初の行を削除してプログラムが実行されるようにしてください．


## Complete Code

  * Find every line that begins with the comment "''Fill Code: ''" and read the description.  Then, replace every instance of the comment  "''ENTER CODE HERE''"
 with the appropriate line of code.

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

 * The 'grasp_goal' goal property can take on three possible values:
 * `grasp_goal` のプロパティは次の3つの値をとることができます．
```
    grasp_goal.goal = object_manipulation_msgs::GraspHandPostureExecutionGoal::GRASP;
    grasp_goal.goal = object_manipulation_msgs::GraspHandPostureExecutionGoal::RELEASE;
    grasp_goal.goal = object_manipulation_msgs::GraspHandPostureExecutionGoal::PRE_GRASP;
```

  * Once the grasp flag has been set you can send the goal through the grasp action client
  * つかむフラグが設定できたら "grasp action client" に対して目標を送ることができます．

## Build Code and Run

  * Compile the pick and place node in QT

##  コードのビルドと実行

  * Qt でピック・アンド・プレース・ノードをコンパイルします．
```
Project -> Build Project
```

  * Alternatively, in a terminal cd into the '''demo_manipulation''' directory and do the following
  * もしくはターミナルで本演習のワークスペースディレクトリに移動して次のコマンドを実行します．
```
catkin build collision_avoidance_pick_and_place
```

  * Run your node with the launch file:
  * launch ファイルでノードを実行します．
```
roslaunch collision_avoidance_pick_and_place ur5_pick_and_place.launch
```

  * If the task succeeds you will see something like the following in the terminal (below). The robot will not move, only gripper I/O is triggered:
  * タスクが正常に実行された場合は次のようなメッセージが表示されるはずです．ロボットは動きませんグリッパの I/O だけ作動します．
```
[ INFO] [1400553290.464877904]: Move wait Succeeded
[ INFO] [1400553290.720864559]: Gripper opened
[ERROR] [1400553290.720985315]: detect_box_pick is not implemented yet.  Aborting.
```

## API References
## API リファレンス

[sendGoal()](http://ros.org/doc/hydro/api/actionlib/html/classactionlib_1_1SimpleActionClient.html)
