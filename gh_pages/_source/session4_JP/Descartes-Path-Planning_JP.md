# Introduction to Descartes Path Planning
>In this exercise, we will use what was learned in the previous exercises by creating a Descartes planner to create a robot path.


# 直交座標系における軌道計画入門

## Motivation
MoveIt! is a framework meant primarily for performing "free-space" motion where the objective is to move a robot from point A to point B and you don't particularly care about how that gets done. These types of problems are only a subset of frequently performed tasks. Imagine any manufacturing ''process'' like welding or painting. You very much care about where that tool is pointing the entire time the robot is at work.

This tutorial introduces you to Descartes, a ''Cartesian'' motion planner meant for moving a robot along some process path. It's only one of a number of ways to solve this kind of problem, but it's got some neat properties:
 * It's deterministic and globally optimum (to a certain search resolution).
 * It can search redundant degrees of freedom in your problem (say you have 7 robot joints or you have a process where the tool's Z-axis rotation doesn't matter).


# モチベーション

MoveIt! はロボットを A 地点から B 地点に移動させることを目的とした，
「フリースペース」モーションを主な目的としたフレームワークで，
それがどのように処理されているかについて
ユーザは特に気にしなくても大丈夫なようになっています．
このようなタイプの問題は実行されるタスクの多くあるケースの一部にすぎません．
溶接や塗装といった製造の「プロセス」を想像してみてください．
ロボット動作の最初から最後まで
ツールがどこを指しているのかについて非常に気を配っています．

このチュートリアルでは，
任意のプロセス経路に沿ってロボットを動かすための
Descrates: 直交座標系（Cartesian/カーテジアン）モーション・プランナを紹介します．
これはこの種の問題を解決する数多くの方法の1つではありますが，
それはいくつかの巧妙な特性を持っています．

  * 決定論的で，かつ全体としてみると（ある程度の検索解像度においては）最適です．
  * 課題の冗長自由度を探索できます．
    （ 7つのロボット関節がある，もしくはツールのZ軸回転が問題ではないプロセスのような場合 ）


## Reference Example
[Descartes Tutorial](http://wiki.ros.org/descartes/Tutorials/Getting%20Started%20with%20Descartes)


## リファレンス

* [Descartes Tutorial](http://wiki.ros.org/descartes/Tutorials/Getting%20Started%20with%20Descartes)


## Further Information and Resources
[Descartes Wiki](http://wiki.ros.org/descartes)

APIs:
 * [descartes_core::PathPlannerBase](http://docs.ros.org/indigo/api/descartes_core/html/classdescartes__core_1_1PathPlannerBase.html)
 * [descartes_planner::DensePlanner](http://docs.ros.org/indigo/api/descartes_planner/html/classdescartes__planner_1_1DensePlanner.html)
 * [descartes_planner::SparsePlanner](http://docs.ros.org/indigo/api/descartes_planner/html/classdescartes__planner_1_1SparsePlanner.html)

[Descartes Wiki](http://wiki.ros.org/descartes)


## 追加情報とリソース

APIs:
 * [descartes_core::PathPlannerBase](http://docs.ros.org/indigo/api/descartes_core/html/classdescartes__core_1_1PathPlannerBase.html)
 * [descartes_planner::DensePlanner](http://docs.ros.org/indigo/api/descartes_planner/html/classdescartes__planner_1_1DensePlanner.html)
 * [descartes_planner::SparsePlanner](http://docs.ros.org/indigo/api/descartes_planner/html/classdescartes__planner_1_1SparsePlanner.html)


## Scan-N-Plan Application: Problem Statement
In this exercise, you will add a new node to your Scan-N-Plan application, based on a reference template, that:
 1. Takes the nominal pose of the marker as input through a ROS service.
 1. Produces a joint trajectory that commands the robot to trace the perimeter of the marker (as if it is dispensing adhesive).


## Scan-N-Plan アプリケーション: 演習問題

本演習では次のリファレンス・テンプレートに基づいて
Scan-N-Plan アプリケーションに新しいノードを追加します．

 1. マーカーの公称姿勢を ROS サービスを介して入力として受け取ります．
 1. ロボットにマーカーの周囲をトレースするように命令する関節軌道を生成します．
    （ 接着剤を塗布するような場合を想定 ）


## Scan-N-Plan Application: Guidance
In the interest of time, we've included a file, `descartes_node.cpp`, that:
 1. Defines a new node & accompanying class for our Cartesian path planning.
 1. Defines the actual service and initializes the Descartes library.
 1. Provides the high level work flow (see planPath function).

Left to you are the details of:
 1. Defining a series of Cartesian poses that comprise a robot “path”.
 1. Translating those paths into something Descartes can understand.


## Scan-N-Plan アプリケーション: ガイダンス

時間節約のために，次の内容の `descartes_node.cpp` というファイルを導入します．

 1. 直交座標系における動作軌道計画のための新しいノードと関連クラスを定義
 1. 実際のサービスを定義し直交座標系ライブラリを初期化
 1. 高レベルの作業フローを提供（参照: planPath 関数）

演習課題として次のことを行います．

 1. ロボット「軌道」を構成する一連の直交座標系上の姿勢を定義します．
 1. これらの軌道を Descartes が処理できるものに変換します．


### Setup workspace

 1. Clone the Descartes repository into your workspace src/ directory.

    ```bash
    cd ~/catkin_ws/src
    git clone -b kinetic-devel https://github.com/ros-industrial-consortium/descartes.git
    ```

 1. Copy over the `ur5_demo_descartes` package into your workspace src/ directory.

    ```bash
    cp -r ~/industrial_training/exercises/4.1/src/ur5_demo_descartes .
    ```

 1. Copy over the `descartes_node_unfinished.cpp` into your core package's src/ folder and rename it `descartes_node.cpp`.

    ```bash
    cp ~/industrial_training/exercises/4.1/src/descartes_node_unfinished.cpp myworkcell_core/src/descartes_node.cpp
    ```

 1. Add dependencies for the following packages in the `CMakeLists.txt` & `package.xml` files, as in previous exercises.
    * `ur5_demo_descartes`
    * `descartes_trajectory`
    * `descartes_planner`
    * `descartes_utilities`

 1. Create rules in the `myworkcell_core` package's `CMakeLists.txt` to build a new node called `descartes_node`.  As in previous exercises, add these lines near similar lines in the template file (not as a block as shown below).

    ```cmake
    add_executable(descartes_node src/descartes_node.cpp)
    add_dependencies(descartes_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(descartes_node ${catkin_LIBRARIES})
    ```


### ワークスペースのセットアップ

 1. Descartes リポジトリをワークスペースの src/ ディレクトリにクローンします．

    ```bash
    cd ~/catkin_ws/src
    git clone -b kinetic-devel https://github.com/ros-industrial-consortium/descartes.git
    ```

 1. `ur5_demo_descartes` パッケージを
    ワークスペースの src/ ディレクトリにコピーします．

    ```bash
    cp -r ~/industrial_training/exercises/4.1/src/ur5_demo_descartes .
    ```

 1. `descartes_node_unfinished.cpp` を
    コアパッケージの src/ フォルダにコピーして，
    `descartes_node.cpp` に名前を変更します．

    ```bash
    cp ~/industrial_training/exercises/4.1/src/descartes_node_unfinished.cpp myworkcell_core/src/descartes_node.cpp
    ```

 1. これまでの演習と同じように下記のパッケージの依存関係を
    `CMakeLists.txt` と `package.xml` ファイルに記述します．

    * `ur5_demo_descartes`
    * `descartes_trajectory`
    * `descartes_planner`
    * `descartes_utilities`

 1. `descartes_node` という名前の新しいノードをビルドするために，
    `myworkcell_core` パッケージの `CMakeLists.txt` にルールを作成します．

    これまでの演習と同じように，これらの行をテンプレートファイルの類似した行の近くに追加します．
    （ 下は1つのブロックではありません ）

    ```cmake
    add_executable(descartes_node src/descartes_node.cpp)
    add_dependencies(descartes_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(descartes_node ${catkin_LIBRARIES})
    ```


### Complete Descartes Node

We will create a Service interface to execute the Descartes planning algorithm.

 1. Define a new service named `PlanCartesianPath.srv` in the `myworkcell_core` package's `srv/` directory.  This service takes the central target position and computes a joint trajectory to trace the target edges.

    ```
    # request
    geometry_msgs/Pose pose

    ---

    # response
    trajectory_msgs/JointTrajectory trajectory
    ```

 1. Add the newly-created service file to the `add_service_file()` rule in the package's `CMakeLists.txt`.

 1. Since our new service references a message type from another package, we'll need to add that other package (`trajectory_msgs`) as a dependency in the `myworkcell_core` `CMakeLists.txt` (3 lines) and `package.xml` (1 line) files.  

 1. Review `descartes_node.cpp` to understand the code structure.  In particular, the `planPath` method outlines the main sequence of steps.

 1. Search for the TODO commands in the Descartes node file and expand on those areas:
    1. In `makeToolPoses`, generate the remaining 3 sides of a path tracing the outside of our "AR Marker" target part.
    1. In `makeDescartesTrajectory`, convert the path you created into a Descartes Trajectory, one point at a time.
       * _Don't forget to transform each nominal point by the specified reference pose: `ref * point`_
    1. In `makeTolerancedCartesianPoint`, create a `new AxialSymmetricPt` from the given pose.
       * See [here](http://docs.ros.org/indigo/api/descartes_trajectory/html/classdescartes__trajectory_1_1AxialSymmetricPt.html) for more documentation on this point type
       * Allow the point to be symmetric about the Z-axis (`AxialSymmetricPt::Z_AXIS`), with an increment of 90 degrees (PI/2 radians)

 1. Build the project, to make sure there are no errors in the new `descartes_node`


### Descartes ノードを完成させる

Descartes 動作計画アルゴリズムを実行するためのサービス・インタフェースを作成します．

 1. `PlanCartesianPath.srv` という名前の新しいサービスを
    `myworkcell_core` パッケージの `srv/` ディレクトリに定義します．

    このサービスは目標物のセンタ位置を取得し，
    目標物のエッジをトレースするための関節軌道を計算します．

    ```
    # request
    geometry_msgs/Pose pose

    ---

    # response
    trajectory_msgs/JointTrajectory trajectory
    ```

 1. 新しく作成したサービスファイルを
    パッケージの `CMakeLists.txt` の
    ` add_service_file()` ルールに追加します．

 1. この新しいサービスは
    他のパッケージのメッセージタイプを参照しているので，
    `myworkcell_core` の `CMakeLists.txt`（3行）と
    `packagework.xml`（1行）に依存関係として
    他のパッケージ（ `trajectory_msgs` ）を追加する必要があります．

 1. `descartes_node.cpp` を見返してコード構造を理解してください．
    特に `planPath` メソッドは主要な手順の要点を記述しています．

 1. Descartes ノード・ファイルにて TODO 指示を検索し，
    それらの部分を次のように拡張します．

    1. `makeToolPoses` では
       目標物 "AR Marker" の外側をトレースする経路の
       残りの3辺を生成します．
    1. `makeDescartesTrajectory` で，
       作成した経路を直交座標系軌跡に1点1点変換します．
       * _公称点を指定された参照ポーズで変換することを忘れないでください．
         `ref * point`_
    1. `makeTolerancedCartesianPoint` では，
       与えられたポーズから `new AxialSymmetricPt` を作成します．
       * このポイント型の詳細については
         [ここ](http://docs.ros.org/indigo/api/descartes_trajectory/html/classdescartes__trajectory_1_1AxialSymmetricPt.html)
         を参照してください．
       * 90[deg]（ PI/2[rad] ）加算して，
         点を Z 軸対称（ `AxialSymmetricPt::Z_AXIS` ）にします．

 1. プロジェクトをビルドし，
    新しい `descartes_node` にエラーがないことを確認してください．


### Update Workcell Node

With the Descartes node completed, we now want to invoke its logic by adding a new `ServiceClient` to the primary workcell node. The result of this service is a joint trajectory that we must then execute on the robot. This can be accomplished in many ways; here we will call the `JointTrajectoryAction` directly.

 1. In `myworkcell_node.cpp`, add include statements for the following headers:

    ```c++
    #include <actionlib/client/simple_action_client.h>
    #include <control_msgs/FollowJointTrajectoryAction.h>
    #include <myworkcell_core/PlanCartesianPath.h>
    ```

    _You do not need to add new dependenies for these libraries/messages, because they are pulled in transitively from moveit._

 1. In your ScanNPlan class, add new private member variables: a ServiceClient for the `PlanCartesianPath` service and an action client for `FollowJointTrajectoryAction`:
    ```c++
    ros::ServiceClient cartesian_client_;
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_;
    ```

 1. Initialize these new objects in your constructor. Note that the action client has to be initialized in what is called the `initializer list`.
    ```c++
    ScanNPlan(ros::NodeHandle& nh) : ac_("joint_trajectory_action", true)
    {
      // ... code
      cartesian_client_ = nh.serviceClient<myworkcell_core::PlanCartesianPath>("plan_path");
    }
    ```
 1. At the end of the `start()` function, create a new Cartesian service and make the service request:

    ```c++
    // Plan cartesian path
    myworkcell_core::PlanCartesianPath cartesian_srv;
    cartesian_srv.request.pose = move_target;
    if (!cartesian_client_.call(cartesian_srv))
    {
      ROS_ERROR("Could not plan for path");
      return;
    }
    ```

 1. Continue adding the following lines, to execute that path by sending an action directly to the action server (bypassing MoveIt):

    ```c++
    // Execute descartes-planned path directly (bypassing MoveIt)
    ROS_INFO("Got cart path, executing");
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = cartesian_srv.response.trajectory;
    ac_.sendGoal(goal);
    ac_.waitForResult();
    ROS_INFO("Done");
    ```

 1. Build the project, to make sure there are no errors in the new `descartes_node`


### 作業セルノードの更新

Descartes ノードが完成した後，新しい `ServiceClient` を
メインの作業セルノードに追加して，そのロジックを呼び出す必要があります．
このサービスの出力はこの後にロボットで実行する関節軌道です．
ロボットでの実行は様々な方法で達成できますが，
ここでは `JointTrajectoryAction` を直接呼びます．

 1. `myworkcell_node.cpp` に次のヘッダの include 文を追加します．

    ```c++
    #include <actionlib/client/simple_action_client.h>
    #include <control_msgs/FollowJointTrajectoryAction.h>
    #include <myworkcell_core/PlanCartesianPath.h>
    ```

    _これらのライブラリとメッセージは
     MoveIt! から引かれているので
     新しい依存関係を追加する必要はありません．_

 1. ScanNPlan クラスで新しい `PlanCartesianPath` 変数の
    `ServiceClient` と
    `FollowJointTrajectoryAction` の
    アクションクライアントを追加します．

    ```c++
    ros::ServiceClient cartesian_client_;
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_;
    ```

 1. これらの新しいオブジェクトをコンストラクタで初期化します．
    アクションクライアントは `initializer list` と呼ばれるもので
    初期化されなければならないことに注意してください．

    ```c++
    ScanNPlan(ros::NodeHandle& nh) : ac_("joint_trajectory_action", true)
    {
      // ... code
      cartesian_client_ = nh.serviceClient<myworkcell_core::PlanCartesianPath>("plan_path");
    }
    ```
 1. `start()` 関数の最後で，
    新しい直交座標系サービスを作成し，
    サービス要求を行います．

    ```c++
    // Plan cartesian path
    myworkcell_core::PlanCartesianPath cartesian_srv;
    cartesian_srv.request.pose = move_target;
    if (!cartesian_client_.call(cartesian_srv))
    {
      ROS_ERROR("Could not plan for path");
      return;
    }
    ```

 1. 続けて次の行を追加して，
    （ MoveIt! をバイパスして）アクションサーバに
    直接アクションを送信することでその軌道を実行します．

    ```c++
    // Execute descartes-planned path directly (bypassing MoveIt)
    ROS_INFO("Got cart path, executing");
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = cartesian_srv.response.trajectory;
    ac_.sendGoal(goal);
    ac_.waitForResult();
    ROS_INFO("Done");
    ```

 1. プロジェクトをビルドし，
    新しい `descartes_node` にエラーがないことを確認してください．


### Test Full Application

 1. Create a new `setup.launch` file (in `workcell_support` package) that brings up everything except your workcell_node:

    ``` xml
    <include file="$(find myworkcell_moveit_config)/launch/myworkcell_planning_execution.launch"/>
    <node name="fake_ar_publisher" pkg="fake_ar_publisher" type="fake_ar_publisher_node" />
    <node name="vision_node" type="vision_node" pkg="myworkcell_core" output="screen"/>
    <node name="descartes_node" type="descartes_node" pkg="myworkcell_core" output="screen"/>
    ```

 1. Run the new setup file, then your main workcell node:

    ``` bash
    roslaunch myworkcell_support setup.launch
    rosrun myworkcell_core myworkcell_node
    ```

    It's difficult to see what's happening with the rviz planning-loop always running.  Disable this loop animation in rviz (Displays -> Planned Path -> Loop Animation), then rerun `myworkcell_node`.


### アプリケーション全体のテスト

 1. workcell_node を除くすべてのノードを起動する
    新しい `setup.launch` ファイルを
    `workcell_support` パッケージ内にを作成してください．

    ``` xml
    <include file="$(find myworkcell_moveit_config)/launch/myworkcell_planning_execution.launch"/>
    <node name="fake_ar_publisher" pkg="fake_ar_publisher" type="fake_ar_publisher_node" />
    <node name="vision_node" type="vision_node" pkg="myworkcell_core" output="screen"/>
    <node name="descartes_node" type="descartes_node" pkg="myworkcell_core" output="screen"/>
    ```

 1. 新しいセットアップファイルを実行してから
    メイン作業セルノードを実行します．

    ``` bash
    roslaunch myworkcell_support setup.launch
    rosrun myworkcell_core myworkcell_node
    ```

    RViz で動作計画のループアニメーションが常に実行されていると
    何が起きているのかを確認するのが難しくなります．
    RViz でこのループアニメーションを無効にし
    （ Displays -> Planned Path -> Loop Animation ）
    それから `myworkcell_node` を再実行します．


### Hints and Help

Hints:
 * The path we define in `makeToolPoses()` is relative to some known reference point on the part you are working with. So a tool pose of (0, 0, 0) would be exactly at the reference point, and not at the origin of the world coordinate system.
 * In `makeDescartesTrajectorty(...)` we need to convert the relative tool poses into world coordinates using the “ref” pose.
 * In `makeTolerancedCartesianPoint(...)` consider the following documentation for specific implementations of common joint trajectory points:
   * <http://docs.ros.org/indigo/api/descartes_trajectory/html/>
 * For additional help, review the completed reference code at `~/industrial_training/exercises/4.1/src`

### ヒントとヘルプ

ヒント:
 * `makeToolPoses()` で定義した軌道は
   作業部分の既知の参照点からの相対的なものです．
   したがって (0, 0, 0) のツール姿勢は
   ワールド座標系の原点ではなく参照点そのものに位置します．
 * `makeDescartesTrajectorty(...)` では "ref" 姿勢を使って
   相対的なツール姿勢をワールド座標に変換する必要があります．
 * `makeTolerancedCartesianPoint(...)` では，一般的な関節軌道の特定の実装に関する
   次のドキュメントを考慮してください．
   * <http://docs.ros.org/indigo/api/descartes_trajectory/html/>
 * さらに詳しいヘルプは
   `〜/industrial_training/exercises/4.1/src`
   の完成コードを参考にしてください．
