# ROS の初期化
> MoveIt! とその他のシステムパートが通信できるように ROS コンポーネントの初期化を行います．

## 演習用ソースファイル

* `plan_and_run/src/plan_and_run_node.cpp` にあるメインアプリケーションのソースファイルを開きます．
* メインプログラム内の `application.initRos()` という関数を見てください．
* `plan_and_run/src/tasks/init_ros.cpp` ファイルを開いて該当する関数を見てください．
もしくは Qt/Eclipse で関数の任意の部分をクリックし "F3" を押すことによってその関数のソースファイルに移動します．
* `//ROS_ERROR_STREAM("Task '"<<__FUNCTION__ <<"' is incomplete. Exiting"); exit(-1);` を含む最初の行をコメントアウトしてプログラムが直ちに終了しないようにしてください．

## コードを完成させる

* ROS のパブリッシャ `marker_publisher_` 変数がどのように初期化されるのかを確認してください．本プログラムのノードはそれを `visualization_msgs::!MarkerArray` メッセージを RViz 上に表示させるためにパブリッシュします．
* `moveit_run_path_client_` サービスクライアントは ROS ノードハンドラのテンプレートメソッド `ros::!NodeHandle::serviceClient()` を利用します．
* `/*  Fill Code:` で始まるコメントブロックを見つけてそこにある記述どおりにコードを完成させてください．
* 指示に従って `[ COMPLETE HERE ]` とある各部分を書き換えてください．

## コードのビルドと実行

* `cd` で本演習の catkin ワークスペースに移動して `catkin build` でビルドしてください．
* 次にアプリケーション launch フィアルを実行してください．

```
roslaunch plan_and_run demo_run.launch
```

## API リファレンス

* [visualization_msgs::MarkerArray](http://docs.ros.org/api/visualization_msgs/html/msg/MarkerArray.html)
* [NodeHandle::serviceClient()](http://docs.ros.org/indigo/api/roscpp/html/classros_1_1NodeHandle.html#aa3376eeca609c4985255cecfaadcbcc5)
