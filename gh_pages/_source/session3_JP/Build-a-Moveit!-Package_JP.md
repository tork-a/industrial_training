# Build a MoveIt! Package
>In this exercise, we will create a MoveIt! package for an industrial robot. This package creates the configuration and launch files required to use a robot with the MoveIt! Motion-Control nodes. In general, the MoveIt! package does not contain any C++ code.

# MoveIt! パッケージのビルド

> 産業用ロボットの MoveIt! パッケージを作成します．
  このパッケージは MoveIt! のモーションコントロールノードを用いてロボットを使用する
  ために必要な設定ファイルと launch ファイルを作成します．
  一般的には C++ コードをそれらの MoveIt! パッケージに含むことはしません．


## Motivation
MoveIt! is a free-space motion planning framework for ROS. It’s an incredibly useful and easy-to-use tool for planning motions between two points in space without colliding with anything. Under the hood MoveIt is quite complicated, but unlike most ROS libraries, it has a really nice GUI Wizard to get you going.


## モチベーション

MoveIt! は ROS の自由空間動作計画フレームワークです．
それは何かに干渉することなく空間内の2つのポイント間の
動作を計画するための非常に便利で使いやすいツールです．
MoveIt! のボンネットの中身は非常に複雑ですが，
多くの ROS ライブラリとは異なり，
本当に素晴らしい魅力的な GUI 操作画面を持っています．


## Reference Example
[Using MoveIt with ROS-I](http://wiki.ros.org/Industrial/Tutorials/Create_a_MoveIt_Pkg_for_an_Industrial_Robot)


## リファレンス

* [Create a MoveIt Package for an Industrial Robot](http://wiki.ros.org/Industrial/Tutorials/Create_a_MoveIt_Pkg_for_an_Industrial_Robot)


## Further Information and Resources
[MoveIt’s Standard Wizard Guide](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/setup_assistant/setup_assistant_tutorial.html)


## 追加情報とリソース

* [MoveIt’s Standard Wizard Guide](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/setup_assistant/setup_assistant_tutorial.html)


## Scan-N-Plan Application: Problem Statement
In this exercise, you will generate a MoveIt package for the UR5 workcell you built in a previous step. This process will mostly involve running the MoveIt! Setup Assistant. At the end of the exercise you should have the following:

 1. A new package called `myworkcell_moveit_config`

 1. A moveit configuration with one group ("manipulator"), that consists of the kinematic chain between the UR5’s `base_link` and `tool0`.


## Scan-N-Plan アプリケーション: 演習問題

前段階で作成した UR5 作業セルの MoveIt! パッケージを生成します．
このプロセスのほとんどは MoveIt! Setup Assistant の実行に関連するものです．
演習の最後には次のものができているはずです．

 1. 新しいパッケージ `myworkcell_moveit_config`

 1. UR5 の `base_link` と `tool0` 間の運動学的な連結で構成された
    1つのグループ（"manipulator"）を持つ MoveIt! コンフィグレーション


## Scan-N-Plan Application: Guidance

 1. Start the MoveIt! Setup Assistant (don't forget auto-complete with tab):

    ```
    roslaunch moveit_setup_assistant setup_assistant.launch
    ```

 1. Select "Create New MoveIt Configuration Package", select the `workcell.xacro` you created previously, then "Load File".

 1. Work your way through the tabs on the left from the top down.

    1. Generate a self-collision matrix.
    1. Add a fixed virtual base joint. e.g.

       ```
       name = 'FixedBase' (arbitrary)
       child = 'world' (should match the URDF root link)
       parent = 'world' (reference frame used for motion planning)
       type = 'fixed'
       ```

    1. Add a planning group called `manipulator` that names the kinematic chain between `base_link` and `tool0`. Note: Follow [ROS naming guidelines/requirements](http://wiki.ros.org/ROS/Patterns/Conventions) and don't use any whitespace, anywhere.

       a. Set the kinematics solver to `KDLKinematicsPlugin`

    1. Create a few named positions (e.g. "home", "allZeros", etc.) to test with motion-planning.

    1. Don't worry about adding end effectors/grippers or passive joints for this exercise.

    1. Enter author / maintainer info.

       _Yes, it's required, but doesn't have to be valid_

    1. Generate a new package and name it `myworkcell_moveit_config`.
       * make sure to create the package inside your `catkin_ws/src` directory

    1. The current MoveIt! Settup Assistant has a [bug](https://github.com/ros-planning/moveit/issues/955) that causes some minor errors and abnormal behaviors.  To fix these errors:
       1. Edit the `myworkcell_core_moveit_config/config/ompl_planning.yaml` file.
       1. Append the text string `kConfigDefault` to each planner name
          * e.g. `SBL:` -> `SBLkConfigDefault`, etc.

 The outcome of these steps will be a new package that contains a large number of launch and configuration files. At this point, it's possible to do motion planning, but not to execute the plan on any robot.  To try out your new configuration:

    catkin build
    source ~/catkin_ws/devel/setup.bash
    roslaunch myworkcell_moveit_config demo.launch

> Don't worry about learning how to use RViz to move the robot; that's what we'll cover in the next session!


## Scan-N-Plan アプリケーション: ガイダンス

 1. MoveIt! Setup Assistant を起動します．

   （ tab の補完機能を使ってください．）

    ```
    roslaunch moveit_setup_assistant setup_assistant.launch
    ```

 1. "Create New MoveIt Configuration Package" を選択してから
    以前の演習で作成した `workcell.xacro` を選択して "Load File" してください．

 1. 左上のタブを上から下に向かって作業していきます．

    1. 自己干渉マトリクス（ self-collision matrix ）を作成する．

    1. 次の例のように固定された仮想ベース関節を追加します．

       ```
       name = 'FixedBase' (arbitrary)
       child = 'world' (should match the URDF root link)
       parent = 'world' (reference frame used for motion planning)
       type = 'fixed'
       ```

    1. `base_link` と `tool0` 間に運動学的な連結（ kinematic chain ）を指定する
       `manipulator` というプランニング・グループを追加します．

       _注: [ROS命名ガイドライン・要件](http://wiki.ros.org/ROS/Patterns/Conventions)に従ってください．どこにも空白を使用しないでください．_

       a. キネマティック・ソルバを `KDLKinematicsPlugin` に設定します．

    1. 動作計画テストのために，いくつかの名前付き位置
       （ 例: "home"，"allZeros" など ）を作成します．

    1. 本演習ではエンドエフェクタ，グリッパ，またはパッシブジョイントの追加は考慮不要です．

    1. 著者・管理者情報を入力してください．

       _入力は必須ですが有効な内容である必要はありません．_

    1. 新しいパッケージを生成し `myworkcell_moveit_config` という名前を付けます．

        * `catkin_ws/src` ディレクトリ内にパッケージを作成してください．

    1. 現在の MoveIt! Settup Assistant にはいくつかの小さなエラーや異常な動作を引き起こす
       [バグ](https://github.com/ros-planning/moveit/issues/955) があります．
       これらのエラーを修正するには次のことを行ってください．

       1. `myworkcell_core_moveit_config/config/ompl_planning.yaml` ファイルを編集します．
       1. 各プランナ名にテキスト文字列 `kConfigDefault` を付加してください．
          * 例: `SBL:` -> `SBLkConfigDefault` など


 これらのステップの結果，
 多数の launch ファイルと設定ファイルを含む新しいパッケージになります．
 この時点でも動作計画（モーション・プランニング）は可能ですが，
 ロボットで動作計画を実行することはできません．
 新しい設定を試すには次のようにします．

   ```
   catkin build
   source ~/catkin_ws/devel/setup.bash
   roslaunch myworkcell_moveit_config demo.launch
   ```

> ロボットを動かすための RViz を使用方法をについては心配しないでください．
  それは次回の演習で説明します．


## Using MoveIt! with Physical Hardware

MoveIt!'s setup assistant generates a suite of files that, upon launch:

 * Loads your workspace description to the parameter server.
 * Starts a node `move_group` that offers a suite of ROS services & actions for doing kinematics, motion planning, and more.
 * An internal simulator that publishes the last planned path on a loop for other tools (like RViz) to visualize.

Essentially, MoveIt can publish a ROS message that defines a trajectory (joint positions over time), but it doesn't know how to pass that trajectory to your hardware.

To do this, we need to define a few extra files.

 1. Create a `controllers.yaml` file (`myworkcell_moveit_config/config/controllers.yaml`) with the following contents:

    ```
    controller_list:
      - name: ""
        action_ns: joint_trajectory_action
        type: FollowJointTrajectory
        joints: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint]
     ```

 1. Create the `joint_names.yaml` file (`myworkcell_moveit_config/config/joint_names.yaml`):

    ```
    controller_joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint]
    ```

 1. Fill in the existing, but blank, controller_manager launch file (`myworkcell_moveit_config/launch/myworkcell_moveit_controller_manager.launch.xml`):

    ``` xml
    <launch>
      <arg name="moveit_controller_manager"
           default="moveit_simple_controller_manager/MoveItSimpleControllerManager"/>
      <param name="moveit_controller_manager"
             value="$(arg moveit_controller_manager)"/>

      <rosparam file="$(find myworkcell_moveit_config)/config/controllers.yaml"/>
    </launch>
    ```

 1. Create a new `myworkcell_planning_execution.launch` (in `myworkcell_moveit_config/launch`):

    ``` xml
    <launch>
      <!-- The planning and execution components of MoveIt! configured to run -->
      <!-- using the ROS-Industrial interface. -->

      <!-- Non-standard joint names:
           - Create a file [robot_moveit_config]/config/joint_names.yaml
               controller_joint_names: [joint_1, joint_2, ... joint_N]
           - Update with joint names for your robot (in order expected by rbt controller)
           - and uncomment the following line: -->
      <rosparam command="load" file="$(find myworkcell_moveit_config)/config/joint_names.yaml"/>

      <!-- the "sim" argument controls whether we connect to a Simulated or Real robot -->
      <!--  - if sim=false, a robot_ip argument is required -->
      <arg name="sim" default="true" />
      <arg name="robot_ip" unless="$(arg sim)" />

      <!-- load the robot_description parameter before launching ROS-I nodes -->
      <include file="$(find myworkcell_moveit_config)/launch/planning_context.launch" >
       <arg name="load_robot_description" value="true" />
      </include>

      <!-- run the robot simulator and action interface nodes -->
      <group if="$(arg sim)">
        <include file="$(find industrial_robot_simulator)/launch/robot_interface_simulator.launch" />
      </group>

      <!-- run the "real robot" interface nodes -->
      <!--   - this typically includes: robot_state, motion_interface, and joint_trajectory_action nodes -->
      <!--   - replace these calls with appropriate robot-specific calls or launch files -->
      <group unless="$(arg sim)">
        <include file="$(find ur_bringup)/launch/ur5_bringup.launch" />
      </group>

      <!-- publish the robot state (tf transforms) -->
      <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />

      <include file="$(find myworkcell_moveit_config)/launch/move_group.launch">
        <arg name="publish_monitored_planning_scene" value="true" />
      </include>

      <include file="$(find myworkcell_moveit_config)/launch/moveit_rviz.launch">
        <arg name="config" value="true"/>
      </include>

    </launch>
    ```

 1. Now let's test the new launch files we created:

    ```
    roslaunch myworkcell_moveit_config myworkcell_planning_execution.launch
    ```


## 実機での MoveIt! の使用

MoveIt! Setup Assistant は起動時の一揃いのファイルを生成します．

 * ワークスペースの概要をパラメーター・サーバーにロードする．
 * 運動学，動作計画一連の ROS サービスとアクションを提供するノード `move_group` を開始する．
 * 視覚化ツール（ RViz など ）のループに直近に計画された軌道をパブリッシュする内部シミュレータ．

基本的には MoveIt! は
軌道（時間の経過による関節の位置）を定義する
ROS メッセージをパブリッシュできますが，
その軌道をハードウェアに渡す方法はわかりません．

これを行うにはいくつかの追加ファイルを定義する必要があります．

 1. 次の内容の `controllers.yaml` ファイルを作成する．
    (`myworkcell_moveit_config/config/controllers.yaml`)

    ```
    controller_list:
      - name: ""
        action_ns: joint_trajectory_action
        type: FollowJointTrajectory
        joints: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint]
     ```

 1. `joint_names.yaml` ファイルを作成する．
    (`myworkcell_moveit_config/config/joint_names.yaml`)

    ```
    controller_joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint]
    ```

 1. 既存の controller_manager launch ファイルを完成させる．
    (`myworkcell_moveit_config/launch/myworkcell_moveit_controller_manager.launch.xml`)

    ``` xml
    <launch>
      <arg name="moveit_controller_manager"
           default="moveit_simple_controller_manager/MoveItSimpleControllerManager"/>
      <param name="moveit_controller_manager"
             value="$(arg moveit_controller_manager)"/>

      <rosparam file="$(find myworkcell_moveit_config)/config/controllers.yaml"/>
    </launch>
    ```

 1. `myworkcell_planning_execution.launch` ファイルを
    `myworkcell_moveit_config/launch` 内に作成する．

    ``` xml
    <launch>
      <!-- The planning and execution components of MoveIt! configured to run -->
      <!-- using the ROS-Industrial interface. -->

      <!-- Non-standard joint names:
           - Create a file [robot_moveit_config]/config/joint_names.yaml
               controller_joint_names: [joint_1, joint_2, ... joint_N]
           - Update with joint names for your robot (in order expected by rbt controller)
           - and uncomment the following line: -->
      <rosparam command="load" file="$(find myworkcell_moveit_config)/config/joint_names.yaml"/>

      <!-- the "sim" argument controls whether we connect to a Simulated or Real robot -->
      <!--  - if sim=false, a robot_ip argument is required -->
      <arg name="sim" default="true" />
      <arg name="robot_ip" unless="$(arg sim)" />

      <!-- load the robot_description parameter before launching ROS-I nodes -->
      <include file="$(find myworkcell_moveit_config)/launch/planning_context.launch" >
       <arg name="load_robot_description" value="true" />
      </include>

      <!-- run the robot simulator and action interface nodes -->
      <group if="$(arg sim)">
        <include file="$(find industrial_robot_simulator)/launch/robot_interface_simulator.launch" />
      </group>

      <!-- run the "real robot" interface nodes -->
      <!--   - this typically includes: robot_state, motion_interface, and joint_trajectory_action nodes -->
      <!--   - replace these calls with appropriate robot-specific calls or launch files -->
      <group unless="$(arg sim)">
        <include file="$(find ur_bringup)/launch/ur5_bringup.launch" />
      </group>

      <!-- publish the robot state (tf transforms) -->
      <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />

      <include file="$(find myworkcell_moveit_config)/launch/move_group.launch">
        <arg name="publish_monitored_planning_scene" value="true" />
      </include>

      <include file="$(find myworkcell_moveit_config)/launch/moveit_rviz.launch">
        <arg name="config" value="true"/>
      </include>

    </launch>
    ```

 1. 新しく作成した launch ファイルをテストします．

    ```
    roslaunch myworkcell_moveit_config myworkcell_planning_execution.launch
    ```
