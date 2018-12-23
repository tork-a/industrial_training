# STOMP 入門

## モチベーション

 - STOMP
   ( Stochastic Trajectory Optimization for Motion Planning /
   運動計画のための確率的軌道最適化 )
   を利用した MoveIt! での動作計画方法の学習


## 情報とリソース

- [STOMP for MoveIt!]( http://rosindustrial.org/news/2015/9/25/stomp-for-indigo-presentation-from-the-moveit-community-meeting-3-sept-2015)
- [Plugins for MoveIt!]( http://moveit.ros.org/documentation/plugins/)


## 目標

* moveit_config パッケージにファイルを追加・変更して MoveIt! に STOMP を統合
* [RViz Motion Planning Plugin](http://docs.ros.org/hydro/api/moveit_ros_visualization/html/doc/tutorial.html)
  で STOMP を用いた動作計画の生成


## セットアップ

  * ワークスペースを作成します．
    ```
    mkdir --parent ~/catkin_ws/src
    cd ~/catkin_ws
    catkin init
    catkin build
    source devel/setup.bash
    ```

  * 既存の演習内容をコピーします．
    ```
    cd ~/catkin_ws/src
    cp -r ~/industrial_training/exercises/4.1 .
    ```

  * ワークスペースに industrial_moveit リポジトリをコピーします．
    ```
    cd ~/catkin_ws/src
    git clone https://github.com/ros-industrial/industrial_moveit.git
    cd ~/catkin_ws/src/industrial_moveit
    git checkout kinetic-devel
    ```

  * 足りない依存関係にあるパッケージをインストールします．
    ```
    cd ~/catkin_ws/src/4.1
    rosinstall . .rosinstall
    catkin build
    ```

  * **moveit_config** パッケージを
    [ MoveIt! Setup Assistant ](http://docs.ros.org/hydro/api/moveit_setup_assistant/html/doc/tutorial.html)
    を用いて作成します．


## STOMP の追加

1. "*stomp_planning_pipeline.launch.xml*" ファイルを
   **moveit_config** パッケージの **launch** ディレクトリに作成します．
   このファイルには次の内容を書き込んでください．

   ``` xml
   <launch>

     <!-- Stomp Plugin for MoveIt! -->
     <arg name="planning_plugin" value="stomp_moveit/StompPlannerManager" />

     <!-- The request adapters (plugins) ORDER MATTERS -->
     <arg name="planning_adapters" value="default_planner_request_adapters/FixWorkspaceBounds
                                          default_planner_request_adapters/FixStartStateBounds
                                          default_planner_request_adapters/FixStartStateCollision
                                          default_planner_request_adapters/FixStartStatePathConstraints" />

     <arg name="start_state_max_bounds_error" value="0.1" />

     <param name="planning_plugin" value="$(arg planning_plugin)" />
     <param name="request_adapters" value="$(arg planning_adapters)" />
     <param name="start_state_max_bounds_error" value="$(arg start_state_max_bounds_error)" />
     <rosparam command="load" file="$(find myworkcell_moveit_config)/config/stomp_planning.yaml"/>

   </launch>
   ```

   **!!!** launch ファイル内の設定ファイル **stomp_planning.yaml** の記述に注目してください．
   このファイルは moveit_config パッケージ内に置く必要があります．

1. 設定ファイル "*stomp_planning.yaml*" の作成

   このファイルには　STOMP で必要となるパラメータを記述します．
   パラメータは SRDF ファイルで定義されている各 "Planning Group" に固有なものとなります．
   したがって，3つのプランニング・グループ

     * manipulator
     * manipulator_tool
     * manipulator_rail

   が存在する場合，
   設定ファイルでは各プランニング・グループごとに特定のパラメータセットを定義します．

   ``` yaml
   stomp/manipulator_rail:
     group_name: manipulator_rail
     optimization:
       num_timesteps: 60
       num_iterations: 40
       num_iterations_after_valid: 0    
       num_rollouts: 30
       max_rollouts: 30
       initialization_method: 1 #[1 : LINEAR_INTERPOLATION, 2 : CUBIC_POLYNOMIAL, 3 : MININUM_CONTROL_COST
       control_cost_weight: 0.0
     task:
       noise_generator:
         - class: stomp_moveit/NormalDistributionSampling
           stddev: [0.05, 0.8, 1.0, 0.8, 0.4, 0.4, 0.4]
       cost_functions:
         - class: stomp_moveit/CollisionCheck
           collision_penalty: 1.0
           cost_weight: 1.0
           kernel_window_percentage: 0.2
           longest_valid_joint_move: 0.05
       noisy_filters:
         - class: stomp_moveit/JointLimits
           lock_start: True
           lock_goal: True
         - class: stomp_moveit/MultiTrajectoryVisualization
           line_width: 0.02
           rgb: [255, 255, 0]
           marker_array_topic: stomp_trajectories
           marker_namespace: noisy
       update_filters:
         - class: stomp_moveit/PolynomialSmoother
           poly_order: 6
         - class: stomp_moveit/TrajectoryVisualization
           line_width: 0.05
           rgb: [0, 191, 255]
           error_rgb: [255, 0, 0]
           publish_intermediate: True
           marker_topic: stomp_trajectory
           marker_namespace: optimized
   stomp/manipulator:
     group_name: manipulator
     optimization:
       num_timesteps: 40
       num_iterations: 40
       num_iterations_after_valid: 0    
       num_rollouts: 10
       max_rollouts: 10
       initialization_method: 1 #[1 : LINEAR_INTERPOLATION, 2 : CUBIC_POLYNOMIAL, 3 : MININUM_CONTROL_COST
       control_cost_weight: 0.0
     task:
       noise_generator:
         - class: stomp_moveit/NormalDistributionSampling
           stddev: [0.05, 0.4, 1.2, 0.4, 0.4, 0.1]
       cost_functions:
         - class: stomp_moveit/CollisionCheck
           kernel_window_percentage: 0.2
           collision_penalty: 1.0
           cost_weight: 1.0
           longest_valid_joint_move: 0.05
       noisy_filters:
         - class: stomp_moveit/JointLimits
           lock_start: True
           lock_goal: True
         - class: stomp_moveit/MultiTrajectoryVisualization
           line_width: 0.04
           rgb: [255, 255, 0]
           marker_array_topic: stomp_trajectories
           marker_namespace: noisy
       update_filters:
         - class: stomp_moveit/PolynomialSmoother
           poly_order: 5
         - class: stomp_moveit/TrajectoryVisualization
           line_width: 0.02
           rgb: [0, 191, 255]
           error_rgb: [255, 0, 0]
           publish_intermediate: True
           marker_topic: stomp_trajectory
           marker_namespace: optimized      
    ```

    **!!!** *このファイルを moveit_config パッケージの* **config** *ディレクトリに保存します．*

1. **move_group.launch** ファイルを修正します．

   launch ディレクトリ内の **move_group.launch** ファイルを開いて，
   次のように `pipeline` パラメータの値を `stomp` に変更します．

   ``` xml
       .
       .
       .
   <!-- move_group settings -->
   <arg name="allow_trajectory_execution" default="true"/>
   <arg name="fake_execution" default="false"/>
   <arg name="max_safe_path_cost" default="1"/>
   <arg name="jiggle_fraction" default="0.05" />
   <arg name="publish_monitored_planning_scene" default="true"/>

   <!-- Planning Functionality -->
   <include ns="move_group" file="$(find myworkcell_moveit_config)/launch/planning_pipeline.launch.xml">
     <arg name="pipeline" value="stomp" />
   </include>

       .
       .
       .
   ```


### STOMP とともに MoveIt! を実行

1. 2つ目のターミナルで **demo.lauch** ファイルを実行します．

   ```
   roslaunch myworkcell_moveit_config demo.launch
   ```

1. RViz にてロボットの "Start" と "Goal" の姿勢を選択して動作計画をします．

  * "Motion Planning" パネル内の "Planning" タブに移動します．
  * "Select Start State" ドロップダウンをクリックして，
    "allZeros" を選択してから "Update" をクリックします．
  * "Select Goal State" ドロップダウンをクリックして，
    "home" を選択してから "Update" をクリックします．
  * "Plan" ボタンをクリックして，
    目標位置に到達するために腕が障害物を越えていくのを観察してください．
    青い線はツールの軌道を示します．


### STOMP の探索

1. RViz で他の "Start" と "Goal" の位置を選択して，
   "Plan" を行いロボットの動きを見てください．
2. "Displays" パネルにある "Marker Array" チェックボックをクリックして
   *Noisy Trajectories* を表示してください．
   再び "Plan" ボタンをクリックすると，
   黄色い線の "Noisy Trajectory Markers" が表示されるはずです．

> STOMP は現在の軌道にノイズを適用した結果として
> 多数のノイジーな軌道を生成することによって，
> 作業空間を探索します．
> 適用されるノイズの度合いは
> "*stomp_config.yaml*" ファイルの
> "stddev" パラメータを調整することで変更できます．
> より大きい "stddev" 値は関節のより大きな動きに相当します．


### STOMP の設定

**stomp_config.yaml** のパラメータを変更し，
これらの変更が動作計画にどのような影響を与えるかを確認します．

1. 先程 **demo.launch** ファイルを実行したターミナルで
   Ctrl-C を入力してノードを停止します．
1. エディタで **stomp_config.yaml** ファイルを開きます．
1. "manipulator_rail" グループ内の
   "stddev" パラメータに割り当てられた値に注目してください．
   配列内の各値が関節に適用されるノイズの振幅です．
   例えば配列の左端の値は最初の関節 "rail_to_base" のノイズを設定するための値になります．
   レールは x 方向に沿って移動します．
   "rail_to_base" は直道関節ですので単位はメートル [m] です．
   回転関節の場合は，単位はラジアン [rad] です．
1. "stddev" の値を変更して（好ましくは一度に1つの変更），
   ファイルを保存してからターミナルで "demo.launch" ファイルを実行してください．
1. RViz ウィンドウに戻って，
   任意の "Start" と "Goal" の位置を選択して，
   変更が動作計画のパフォーマンスに与える影響を確認してください．

<!--
### More info on the STOMP parameters

 The [STOMP wiki]() explains these parameter in more detail.


### STOMP パラメータに関するより詳しい情報

 [STOMP Wiki]() は、これらのパラメータについて詳しく説明しています。
-->
