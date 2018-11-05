# Load Parameters
>In this exercise, we'll load some ROS parameters to initialize important variables within our program.

# パラメータの読込み
> プログラムで重要な変数を初期化するためにいくつかの ROS パラメータを読み込みます．

## Locate Exercise Source File

  * Go to the main application source file located in `plan_and_run/src/plan_and_run_node.cpp`.
  * In the main program, locate the function call to `application.loadParameters()`.
  * Go to the source file for that function located in the `plan_and_run/src/tasks/load_parameters.cpp`. Alternatively, in Eclipse you can click in any part of the function and press "F3" to bring up that file.
  * Comment out the first line containing the `//ROS_ERROR_STREAM("Task '"<<__FUNCTION__ <<"' is incomplete. Exiting"); exit(-1);` entry so that the function doesn't quit immediately.

## 演習用ソースファイル

  * `plan_and_run/src/plan_and_run_node.cpp` にあるメインアプリケーションのソースファイルを開きます．
  * メインプログラム内の `application.loadParameters()` という関数を見てください．
  * `plan_and_run/src/tasks/load_parameters.cpp` ファイルを開いて該当する関数を見てください．
  もしくは Qt/Eclipse で関数の任意の部分をクリックし "F3" を押すことによってその関数のソースファイルに移動します．
  * `//ROS_ERROR_STREAM("Task '"<<__FUNCTION__ <<"' is incomplete. Exiting"); exit(-1);` を含む最初の行をコメントアウトしてプログラムが直ちに終了しないようにしてください．

## Complete Code

 * Find comment block that starts with ```/*  Fill Code:``` and complete as described.

 * Replace every instance of ```[ COMPLETE HERE ]``` accordingly.


## コードを完成させる

 * `/*  Fill Code:` で始まるコメントブロックを見つけてそこにある記述どおりにコードを完成させてください．

 * 指示に従って `[ COMPLETE HERE ]` とある各部分を書き換えてください．


## Build Code and Run

 * `cd` into your catkin workspace and run:
```
catkin build --cmake-args -G 'CodeBlocks - Unix Makefiles'
source ./devel/setup.bash
```
 * Then run the application launch file:
```
roslaunch plan_and_run demo_setup.launch
roslaunch plan_and_run demo_run.launch
```

## コードのビルドと実行

 * `cd` で本演習の catkin ワークスペースに移動して次を実行してビルドを行ってください．
```
catkin build --cmake-args -G 'CodeBlocks - Unix Makefiles'
source ./devel/setup.bash
```

 * 次にアプリケーション launch ファイルを実行します．
```
roslaunch plan_and_run demo_setup.launch
roslaunch plan_and_run demo_run.launch
```


## API References
## API リファレンス

[ros::NodeHandle](http://docs.ros.org/indigo/api/roscpp/html/classros_1_1NodeHandle.html)

[NodeHandle::getParam()](http://docs.ros.org/indigo/api/roscpp/html/classros_1_1NodeHandle.html#afaaf745b7483da9a621b07db0700f866)
