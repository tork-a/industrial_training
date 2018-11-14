<!--
# Detect Box Pick Point
>The coordinate frame of the box's pick can be requested from a ros service that detects it by processing the sensor data. In this exercise, we will learn how to use a service client to call that ros service for the box pick pose.
-->

# 箱をつかむ位置の検出
> 箱をつかむ座標フレームはセンサ情報処理から検出する ROS サービスから要求できます．
> 箱をつかむ姿勢取得のための ROS サービスを呼び出すサービスクライアントの利用法について学習します．

<!--
## Locate Function

  * In the main program , locate the function call to '''application.detect_box_pick()'''.
  * Go to the source file of that function by clicking in any part of the function and pressing "F3".
  * Remove the fist line containing the following '''ROS_ERROR_STREAM ...''' so that the program runs.
-->

## 関数の位置

  * メインプログラム内の `application.detect_box_pick()` という関数を探してください．
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

<!-- * The '''target_recognition_client''' object in your programs can use the '''call()''' method to send a request to a ros service. -->
 * プログラム内の `target_recognition_client` オブジェクトは `call()` メソッドを用いて ROS サービスにリクエストを送ることができます．

<!-- * The ros service that receives the call will process the sensor data and return the pose for the box pick in the service structure member '''srv.response.target_pose'''. -->
 * その ROS サービスはセンサ情報を処理してサービス構造体メンバ `srv.response.target_pose` 内の箱をつかむ姿勢を返す要求を受け取ります．

<!--
## Build Code and Run

  * Compile the pick and place node in QT
-->

##  コードのビルドと実行

  * Qt でピック・アンド・プレース・ノードをコンパイルします．
```
Project -> Build Project
```

<!-- * Alternatively, in a terminal cd into the '''demo_manipulation''' directory and do the following -->
 * もしくはターミナルで本演習のワークスペースディレクトリに移動して次のコマンドを実行します．
```
catkin build --pkg collision_avoidance_pick_and_place
```

<!-- * Run your node with the launch file: -->
 * launch ファイルでノードを実行します．
```
roslaunch collision_avoidance_pick_and_place ur5_pick_and_place.launch
```

<!-- * A blue box and voxel grid obstacles will be displayed in rviz. In the terminal you should see a message like the following: -->
 * 1つの青い箱といくつかのボクセルグリッド障害物が RViz に表示され，ターミナルには次のようなメッセージが表示されるはずです．
```
[ INFO] [1400554224.057842127]: Move wait Succeeded
[ INFO] [1400554224.311158465]: Gripper opened
[ INFO] [1400554224.648747043]: target recognition succeeded
[ERROR] [1400554224.649055043]: create_pick_moves is not implemented yet.  Aborting.
```

<!-- ## API References -->
## API リファレンス

* [call()](http://docs.ros.org/hydro/api/roscpp/html/classros_1_1ServiceClient.html#a8a0c9be49046998a830df625babd396f)
