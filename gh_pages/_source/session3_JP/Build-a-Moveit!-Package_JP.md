# MoveIt! パッケージのビルド

> 産業用ロボットの MoveIt! パッケージを作成します．
  このパッケージは MoveIt! のモーションコントロールノードを用いて
  ロボットを使用するために必要な設定ファイルと
  launch ファイルを作成します．
  一般的には C++ コードを
  それらの MoveIt! パッケージに含むことはしません．


## モチベーション

MoveIt! は ROS の自由空間動作計画フレームワークです．
それは何かに干渉することなく空間内の2つのポイント間の
動作を計画するための非常に便利で使いやすいツールです．
MoveIt! のボンネットの中身は非常に複雑ですが，
多くの ROS ライブラリとは異なり，
本当に素晴らしい魅力的な GUI 操作画面を持っています．


## リファレンス

* [Create a MoveIt Package for an Industrial Robot](http://wiki.ros.org/Industrial/Tutorials/Create_a_MoveIt_Pkg_for_an_Industrial_Robot)


## 追加情報とリソース

* [MoveIt’s Standard Wizard Guide](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/setup_assistant/setup_assistant_tutorial.html)


## Scan-N-Plan アプリケーション: 演習問題

前段階で作成した UR5 作業セルの
MoveIt! パッケージを生成します．
このプロセスのほとんどは
MoveIt! Setup Assistant の実行に関連するものです．
演習の最後には次のものができているはずです．

 1. 新しいパッケージ `myworkcell_moveit_config`

 1. UR5 の `base_link` と `tool0` 間の運動学的な連結で構成された
    1つのグループ（"manipulator"）を持つ MoveIt! コンフィグレーション


## Scan-N-Plan アプリケーション: ガイダンス

 1. MoveIt! Setup Assistant を起動します．

   （ tab の補完機能を使ってください．）

    ```
    roslaunch moveit_setup_assistant setup_assistant.launch
    ```

 1. "Create New MoveIt Configuration Package" を選択してから
    以前の演習で作成した `workcell.xacro` を選択して
    "Load File" してください．

 1. 左上のタブを上から下に向かって作業していきます．

    1. 自己干渉マトリクス（ self-collision matrix ）を作成する．

    1. 次の例のように固定された仮想ベース関節を追加します．

       ```
       name = 'FixedBase' (arbitrary)
       child = 'world' (should match the URDF root link)
       parent = 'world' (reference frame used for motion planning)
       type = 'fixed'
       ```

    1. `base_link` と `tool0` 間に
       運動学的な連結（ kinematic chain ）を指定する
       `manipulator` というプランニング・グループを追加します．

       _注: [ROS命名ガイドライン・要件](http://wiki.ros.org/ROS/Patterns/Conventions)
        に従ってください．
        どこにも空白を使用しないでください．_

       a. キネマティック・ソルバを
          `KDLKinematicsPlugin` に設定します．

    1. 動作計画テストのために，いくつかの名前付き位置
       （ 例: "home"，"allZeros" など ）を作成します．

    1. 本演習ではエンドエフェクタ，グリッパ，またはパッシブジョイントの追加は考慮不要です．

    1. 著者・管理者情報を入力してください．

       _入力は必須ですが有効な内容である必要はありません．_

    1. 新しいパッケージを生成し
       `myworkcell_moveit_config` という名前を付けます．

        * `catkin_ws/src` ディレクトリ内にパッケージを作成してください．

    1. 現在の MoveIt! Settup Assistant には
       いくつかの小さなエラーや異常な動作を引き起こす
       [バグ](https://github.com/ros-planning/moveit/issues/955)
       があります．
       これらのエラーを修正するには次のことを行ってください．

       1. `myworkcell_core_moveit_config/config/ompl_planning.yaml`
          ファイルを編集します．
       1. 各プランナ名にテキスト文字列 `kConfigDefault` を付加してください．
          * 例: `SBL:` -> `SBLkConfigDefault` など


 これらのステップの結果，
 多数の launch ファイルと設定ファイルを含む
 新しいパッケージになります．
 この時点でも動作計画（モーション・プランニング）は可能ですが，
 ロボットで動作計画を実行することはできません．
 新しい設定を試すには次のようにします．

   ```
   catkin build
   source ~/catkin_ws/devel/setup.bash
   roslaunch myworkcell_moveit_config demo.launch
   ```

> ロボットを動かすための
  RViz の使用方法については
  次回の演習で説明しますので
  心配しないでください．


## 実機での MoveIt! の使用

MoveIt! Setup Assistant は
起動時の一揃いのファイルを生成します．

 * ワークスペースの概要をパラメーター・サーバーにロードする．
 * 運動学，動作計画一連の ROS サービスと
   アクションを提供するノード `move_group` を開始する．
 * 視覚化ツール（ RViz など ）のループに
   直近に計画された軌道をパブリッシュする内部シミュレータ．

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
