# グリッパを開く

グリッパを開く目標を送るために "grasp action client" を利用することを
目的としています．

## 関数の位置

  * メインプログラム内の `application.set_gripper()` という関数を探してください．
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

* `grasp_goal` のプロパティは次の3つの値をとることができます．

```
    grasp_goal.goal = object_manipulation_msgs::GraspHandPostureExecutionGoal::GRASP;
    grasp_goal.goal = object_manipulation_msgs::GraspHandPostureExecutionGoal::RELEASE;
    grasp_goal.goal = object_manipulation_msgs::GraspHandPostureExecutionGoal::PRE_GRASP;
```

* つかむフラグが設定できたら "grasp action client" に対して目標を送ることができます．


##  コードのビルドと実行

 * Qt でピック・アンド・プレース・ノードをコンパイルします．

```
Project -> Build Project
```

 * もしくはターミナルで本演習のワークスペースディレクトリに移動して次のコマンドを実行します．

```
catkin build collision_avoidance_pick_and_place
```

 * launch ファイルでノードを実行します．

```
roslaunch collision_avoidance_pick_and_place ur5_pick_and_place.launch
```


* タスクが正常に実行された場合は次のようなメッセージが表示されるはずで
  す．ロボットは動きません，グリッパの I/O だけ作動します．

```
[ INFO] [1400553290.464877904]: Move wait Succeeded
[ INFO] [1400553290.720864559]: Gripper opened
[ERROR] [1400553290.720985315]: detect_box_pick is not implemented yet.  Aborting.
```

## API リファレンス

* [sendGoal()](http://ros.org/doc/hydro/api/actionlib/html/classactionlib_1_1SimpleActionClient.html)
