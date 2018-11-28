# C++ を用いた動作計画

> ロボットをプログラムから動かすための MoveIt! の C++ インタフェースについて見ていきます．


## モチベーション

前回の演習では MoveIt! の作業セル設定を行い，
動作計画ツールを用いて RViz でロボットを少し動かしました．
今度は動作計画とその実行をプログラムコードから行ってみましょう．

この演習では自身のプログラムから MoveIt! ノードとやり取りするための
基本的な C++ インターフェイスを紹介します．
MoveIt! を使用する方法はいろいろとありますが，
シンプルなアプリケーションの場合はこれが最も分かりやすい方法です．


## リファレンス

* [Move Group Interface tutorial](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/move_group_interface/move_group_interface_tutorial.html#setup)


## 追加情報とリソース

 * [MoveIt! Tutorials](http://docs.ros.org/kinetic/api/moveit_tutorials/html/)
 * [MoveIt! home-page](http://moveit.ros.org/)


## Scan-N-Plan アプリケーション: 演習問題

本演習の目標は
`myworkcell_core` ノードを次のように変更することです．

 1. 視覚ノードへのサービスコールによって通知された部品位置の中央に
    ロボットのツールフレームを移動します．


## Scan-N-Plan アプリケーション: ガイダンス

 1. `myworkcell_node.cpp` ファイルを編集します．

    1. `#include <tf/tf.h>` を追加して，
       フレーム座標変換／ユーティリティのための
       tf ライブラリにアクセスできるようにします．

       * 既にこれまでの演習で
         `tf` パッケージの依存関係を
         追加していることを思い出してください．

    1. `ScanNPlan` クラスの `start` メソッドで
       `LocalizePart` サービスからの応答を用いて
       新しい `move_target` 変数を初期化します．

       ``` c++
       geometry_msgs::Pose move_target = srv.response.pose;
       ```

       * vision_node のサービスへの呼び出しの _後に_
         このコードを配置してください．

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
       `moveit::planning_interface::MoveGroupInterface`
       オブジェクトを作成します．

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

    1. `move()` コマンドのブロック中に
       ROS メッセージを処理できるようにするためには，
       [ここ](http://docs.ros.org/jade/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroup.html#a4c63625e2e9eb5c342d1fc6732bd8cf7)
       に記述があるように，
       `move_group.move()` コマンドに
      「非同期の」スピナーを使用する必要があります．

       下記のように `main()` ルーチンのはじめのあたりで
       `ros::init(argc, argv, "myworkcell_node")`
       の後でスピナーを初期化し，
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
