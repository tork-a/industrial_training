# イントロダクション

<!--
## Goal
 * This application will demonstrate how to use the various components in the Descartes library for planning and executing a robot path from a semi-constrained trajectory of points.  
-->

## 目標
 * 本アプリケーションは準拘束軌道点群からなるロボット軌道の動作計画と実行を行うための直交座標系ライブラリの様々なコンポーネントを使う方法を実演します．

<!--
## Objectives
 * Become familiar with the Descartes workflow.
 * Learn how to load a custom Descartes RobotModel.
 * Learn how to create a semi-constrained trajectory from 6DOF tool poses.
 * Plan a robot path with a Descartes Planner.
 * Convert a Descartes Path into a MoveIt! message for execution.
 * Executing the path on the robot.
-->

## 目的
 * 直交座標系ライブラリのワークフローに慣れる
 * カスタム直交座標系ロボットモデルの読込み方法の習熟
 * 6自由度のツール姿勢の準拘束軌道の作成方法の習熟
 * 直交座標系のプランナでのロボット軌道を計画する
 * 直交座標系軌道から動作実行のための MoveIt! メッセージへ変換する
 * ロボットで動作軌道を実行する
