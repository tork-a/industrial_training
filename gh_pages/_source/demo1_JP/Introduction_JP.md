<!-- # Perception-Driven Manipulation Introduction -->

# イントロダクション - 知覚情報をもとに動作するマニピュレーション

<!--
## Goal
 * The purpose of these exercises is to implement a ROS node that drives a robot through a series of moves and actions in order to complete a pick and place task.  In addition, they will serve as an example of how to integrate a variety of software capabilities (perception, controller drivers, I/O, inverse kinematics, path planning, collision avoidance, etc) into a ROS-based industrial application.  
-->

## 目標
 * 本演習はロボットが物を持ち上げて置く「ピック・アンド・プレース・タスク」の一連の動作を行う ROS ノードを実行することを目標としています．加えて，これらの演習課題は ROS を基盤とした産業用アプリケーションに様々な機能（認識・制御ドライバ・入出力・逆運動学・軌道計画・衝突回避）を統合する用例となっています．

<!--
## Objectives
 * Understand the components and structure of a real or simulated robot application.
 * Learn how to command robot moves using Moveit!.
 * Learn how to move the arm to a joint or Cartesian position.
 * Leverage perception capabilities including AR tag recognition and PCL.
 * Plan collision-free paths for a pick and place task.
 * Control robot peripherals such as a gripper.
-->

## 目的
 * 実機またはシミュレーションのロボットアプリケーションの要素や構成の理解
 * MoveIt! を用いたロボット動作の司令方法の学習
 * 直交座標系上やリンク上の位置へのアーム移動の学習
 * AR タグ (Augmented Reality Tag) 認識や PCL (Point Cloud Library) などの認識機能の利用
 * ピック・アンド・プレースのための衝突回避軌道計画
 * グリッパのようなロボットの端部機器の制御
