<!--
# Create Place Moves
>The gripper moves through three poses in order to place the box: Approach, place and retreat. In this exercise, we will create these place poses for the '''tcp'''  coordinate frame and then transform them to the arm's wrist coordinate frame.
-->

# 置く動作の生成

> グリッパは箱を置くためのそれぞれの姿勢 : アプローチ・置く・後退 に動きます．
> 本演習ではツール中心位置座標系における置く動作を生成して，その後それらの姿勢を手首フレーム座標系に変換します．

<!--
## Locate Function

  * In the main program , locate the function call to '''application.create_place_moves()'''.
  * Go to the source file of that function by clicking in any part of the function and pressing "F3".
  * Remove the fist line containing the following '''ROS_ERROR_STREAM ...''' so that the program runs.
-->

## 関数の位置

  * メインプログラム内の `application.create_place_moves()` という関数を探してください．
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

<!-- * The box's position at the place location is saved in the global variable '''cfg.BOX_PLACE_TF'''. -->
 * 箱を置く位置はグローバル変数 `cfg.BOX_PLACE_TF` に保存されています．

<!-- * The '''create_manipulation_poses()''' uses the values of the approach and retreat distances in order to create the corresponding poses at the desired target. -->
 * `create_manipulation_poses()` は意図した目標にに対応するポーズを生成するためにアプローチと後退の距離を使用します．

<!--  * Since moveit plans the robot path for the arm's wrist, then it is necessary to convert all the place poses to the wrist coordinate frame. -->
 * MoveIt! がロボットの手首の軌道を計算するので，全ての置くための姿勢が手首フレームの座標系に変換される必要があります．

<!-- * The [[lookupTransform|http://mirror.umd.edu/roswiki/doc/hydro/api/tf/html/c++/classtf_1_1Transformer.html#ac01a9f8709a828c427f1a5faa0ced42b]] method can provide the pose of a target relative to another pose. -->
 * [lookupTransform](http://mirror.umd.edu/roswiki/doc/hydro/api/tf/html/c++/classtf_1_1Transformer.html#ac01a9f8709a828c427f1a5faa0ced42b) メソッドが目標姿勢から他の姿勢に対する相対変換を行います．

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
catkin build --cmake-args -G 'CodeBlocks - Unix Makefiles' --workspace collision_avoidance_pick_and_place
```

<!--  * Run your node with the launch file: -->
  * launch ファイルでノードを実行します．
```
roslaunch collision_avoidance_pick_and_place ur5_pick_and_place.launch
```
<!--  * The tcp and wrist position at the place location will be printed on the terminal. You should see something like: -->
  * 次のように置く場所に対するツール中心姿勢と手首位置がターミナルに表示されるはずです．
```
[ INFO] [1400556479.404133995]: Execution completed: SUCCEEDED
[ INFO] [1400556479.404574973]: Pick Move 2 Succeeded
[ INFO] [1400556479.404866351]: tcp position at place: 0x7fff1055d800
[ INFO] [1400556479.404934796]: wrist position at place: x: -0.422
y: 0.6
z: 0.3
[ERROR] [1400556479.404981729]: place_box is not implemented yet.  Aborting.
```

<!-- ## API References -->
## API リファレンス

* [lookupTransform](http://mirror.umd.edu/roswiki/doc/hydro/api/tf/html/c++/classtf_1_1Transformer.html#ac01a9f8709a828c427f1a5faa0ced42b)
* [TF Transforms and other useful data types](http://wiki.ros.org/tf/Overview/Data%20Types)
