# Motion Planning using C++
>In this exercise, we'll explore MoveIt's C++ interface to programatically move a robot.

# C++ を用いた動作計画

> ロボットをプログラムから動かすための MoveIt! の C++ インタフェースについて見ていきます．


## Motivation
Now that we’ve got a working MoveIt! configuration for your workcell and we’ve played a bit in RViz with the planning tools, let’s perform planning and motion in code. This exercise will introduce you to the basic C++ interface for interacting with the MoveIt! node in your own program. There are lots of ways to use MoveIt!, but for simple applications this is the most straight forward.


## モチベーション

前回の演習では MoveIt! の作業セル設定を行い，
動作計画ツールを用いて RViz でロボットを少し動かしました．
今度は動作計画とその実行をプログラムコードから行ってみましょう．

この演習では自身のプログラムから MoveIt! ノードとやり取りするための
基本的な C++ インターフェイスを紹介します．
MoveIt! を使用する方法はいろいろとありますが，
シンプルなアプリケーションの場合はこれが最も分かりやすい方法です．


## Reference Example
[Move Group Interface tutorial](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/move_group_interface/move_group_interface_tutorial.html#setup)


## リファレンス

* [Move Group Interface tutorial](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/move_group_interface/move_group_interface_tutorial.html#setup)


## 3. Further Information and Resources
 * [MoveIt! Tutorials](http://docs.ros.org/kinetic/api/moveit_tutorials/html/)
 * [MoveIt! home-page](http://moveit.ros.org/)


## 追加情報とリソース

 * [MoveIt! Tutorials](http://docs.ros.org/kinetic/api/moveit_tutorials/html/)
 * [MoveIt! home-page](http://moveit.ros.org/)


## Scan-N-Plan Application: Problem Statement
In this exercise, your goal is to modify the `myworkcell_core` node to:

 1. Move the robot’s tool frame to the center of the part location as reported by the service call to your vision node.


## Scan-N-Plan アプリケーション: 演習問題

本演習の目標は `myworkcell_core` ノードを次のように変更することです．

 1. 視覚ノードへのサービスコールによって通知された部品位置の中央に
    ロボットのツールフレームを移動します．


## Scan-N-Plan Application: Guidance

 1. Edit your `myworkcell_node.cpp` file.

    1. Add `#include <tf/tf.h>` to allow access to the tf library (for frame transforms/utilities).

       * Remember that we already added a dependency on the `tf` package in a previous exercise.

    1. In the `ScanNPlan` class's `start` method, use the response from the `LocalizePart` service to initialize a new `move_target` variable:

       ``` c++
       geometry_msgs::Pose move_target = srv.response.pose;
       ```

       * make sure to place this code _after_ the call to the vision_node's service.

 1. Use the `MoveGroupInterface` to plan/execute a move to the `move_target` position:

    1. In order to use the `MoveGroupInterface` class it is necessary to add the `moveit_ros_planning_interface` package as a dependency of your `myworkcell_core` package. Add the `moveit_ros_planning_interface` dependency by modifying your package's `CMakeLists.txt` (2 lines) and `package.xml` (1 line) as in previous exercises.

    1. Add the appropriate "include" reference to allow use of the `MoveGroupInterface`:

       ```c++
       #include <moveit/move_group_interface/move_group_interface.h>
       ```

    1. Create a `moveit::planning_interface::MoveGroupInterface` object in the `ScanNPlan` class's `start()` method. It has a single constructor that takes the name of the planning group you defined when creating the workcell moveit package (“manipulator”).

       ```c++
       moveit::planning_interface::MoveGroupInterface move_group("manipulator");
       ```

    1. Set the desired cartesian target position using the `move_group` object’s `setPoseTarget` function. Call the object's `move()` function to plan and execute a move to the target position.

       ```c++
       // Plan for robot to move to part
       move_group.setPoseReferenceFrame(base_frame);
       move_group.setPoseTarget(move_target);
       move_group.move();
       ```

    1. As described [here](http://docs.ros.org/jade/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroup.html#a4c63625e2e9eb5c342d1fc6732bd8cf7), the `move_group.move()` command requires use of an "asynchronous" spinner, to allow processing of ROS messages during the blocking `move()` command.  Initialize the spinner near the start of the `main()` routine after `ros::init(argc, argv, "myworkcell_node")`, and **replace** the existing `ros::spin()` command with `ros::waitForShutdown()`, as shown:

       ```c++
       ros::AsyncSpinner async_spinner(1);
       async_spinner.start();
       ...
       ros::waitForShutdown();
       ```

 1. Test the system!

    ``` bash
    catkin build
    roslaunch myworkcell_moveit_config myworkcell_planning_execution.launch
    roslaunch myworkcell_support workcell.launch
    ```

 1. More to explore...
    * In RViz, add a "Marker" display of topic "/ar_pose_visual" to confirm that the final robot position matches the position published by `fake_ar_publisher`
    * Try repeating the motion planning sequence:
      1. Use the MoveIt rviz interface to move the arm back to the "allZeros" position
      1. Ctrl+C the `workcell.launch` file, then rerun
    * Try updating the `workcell_node`'s `start` method to automatically move back to the `allZeros` position after moving to the AR_target position.  See [here](http://docs.ros.org/kinetic/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroup.html) for a list of `move_group`'s available methods.
    * Try moving to an "approach position" located a few inches away from the target position, prior to the final move-to-target.


## Scan-N-Plan アプリケーション: ガイダンス

 1. `myworkcell_node.cpp` ファイルを編集します．

    1. `#include <tf/tf.h>` を追加して，
       フレーム座標変換／ユーティリティのための
       tf ライブラリにアクセスできるようにします．

       * 既にこれまでの演習で
         `tf` パッケージの依存関係を追加していることを思い出してください．

    1. `ScanNPlan` クラスの `start` メソッドで
       `LocalizePart` サービスからの応答を用いて
       新しい `move_target` 変数を初期化します．

       ``` c++
       geometry_msgs::Pose move_target = srv.response.pose;
       ```

       * vision_node のサービスへの呼び出しの _後に_ このコードを配置してください．

 1. `MoveGroupInterface` を用いて
    `move_target` 位置への移動の動作計画・実行を行います．

    1. `MoveGroupInterface` クラスを使用するには
       `moveit_ros_planning_interface` パッケージを
       `myworkcell_core` パッケージの依存関係として追加する必要があります．

       これまでの演習同様にパッケージの `CMakeLists.txt`（2行）と
       `package.xml`（1行）を変更して
       `moveit_ros_planning_interface` 依存関係を追加してください．

    1. 適切な「インクルード」参照を追加して
       `MoveGroupInterface` を使用できるようにします．

       ```c++
       #include <moveit/move_group_interface/move_group_interface.h>
       ```

    1. `ScanNPlan` クラスの `start()` メソッドで
       `moveit::planning_interface::MoveGroupInterface` オブジェクトを作成します．

       作業セル MoveIt! パッケージ（ "manipulator" ）の作成時に定義した
       プランニング・グループの名前を持つ単一のコンストラクタがあります．

       ```c++
       moveit::planning_interface::MoveGroupInterface move_group("manipulator");
       ```

    1. `move_group` オブジェクトの `setPoseTarget` 関数を用いて
       直交座標系上の意図する目標位置を設定します．

       オブジェクトの `move()` 関数を呼び出して
       目標位置への移動の動作計画を行い，実行します．

       ```c++
       // Plan for robot to move to part
       move_group.setPoseReferenceFrame(base_frame);
       move_group.setPoseTarget(move_target);
       move_group.move();
       ```

    1. `move()` コマンドのブロック中に ROS メッセージを処理できるようにするためには，
       [ここ](http://docs.ros.org/jade/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroup.html#a4c63625e2e9eb5c342d1fc6732bd8cf7)
       に記述があるように，
       `move_group.move()` コマンドに「非同期の」スピナーを使用する必要があります．

       下記のように `main()` ルーチンのはじめのあたりで
       `ros::init(argc, argv, "myworkcell_node")` の後でスピナーを初期化し，
       既存の `ros::spin()` コマンドを
       `ros::waitForShutdown()` に **置き換え** ます．

       ```c++
       ros::AsyncSpinner async_spinner(1);
       async_spinner.start();
       ...
       ros::waitForShutdown();
       ```

 1. システムをテストします．

    ``` bash
    catkin build
    roslaunch myworkcell_moveit_config myworkcell_planning_execution.launch
    roslaunch myworkcell_support workcell.launch
    ```

 1. より詳しく見ていきます．

    * RViz でトピック "/ar_pose_visual" の "Marker" 表示を追加して，
      最終的なロボット位置が
      `fake_ar_publisher` によって発行された位置
      と一致することを確認します．
    * 動作計画シーケンスを繰り返してみてください．
      1. MoveIt! RViz インタフェースを用いてアームを
         "allZeros" の位置に戻します．
      1. Ctrl+C で `workcell.launch` を停止した後に
         再実行します．
    * `workcell_node` の `start` メソッドを更新して
      AR_target 位置に移動した後，自動的に
      `allZeros` 位置に戻るようにしてみてください．
        * `move_group` の利用可能なメソッドのリストについては
          [ここ](http://docs.ros.org/kinetic/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroup.html)
          を参照してください．
    * 最終的なターゲットへの移動の前に，
      目標位置から数インチ離れた「アプローチ位置」に移動してみてください．
