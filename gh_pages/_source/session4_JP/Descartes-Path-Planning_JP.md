# 直交座標系における軌道計画入門

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
Descrates: 直交座標系（Cartesian/カーテジアン）
モーション・プランナを紹介します．
これはこの種の問題を解決する数多くの方法の1つではありますが，
それはいくつかの巧妙な特性を持っています．

  * 決定論的で，かつ全体としてみると（ある程度の検索解像度においては）最適です．
  * 課題の冗長自由度を探索できます．
    （ 7つのロボット関節がある，
      もしくはツールのZ軸回転が問題ではないプロセスのような場合 ）


## リファレンス

* [Descartes Tutorial](http://wiki.ros.org/descartes/Tutorials/Getting%20Started%20with%20Descartes)


## 追加情報とリソース

APIs:
 * [descartes_core::PathPlannerBase](http://docs.ros.org/indigo/api/descartes_core/html/classdescartes__core_1_1PathPlannerBase.html)
 * [descartes_planner::DensePlanner](http://docs.ros.org/indigo/api/descartes_planner/html/classdescartes__planner_1_1DensePlanner.html)
 * [descartes_planner::SparsePlanner](http://docs.ros.org/indigo/api/descartes_planner/html/classdescartes__planner_1_1SparsePlanner.html)


## Scan-N-Plan アプリケーション: 演習問題

本演習では次のリファレンス・テンプレートに基づいて
Scan-N-Plan アプリケーションに新しいノードを追加します．

 1. マーカーの公称姿勢を
    ROS サービスを介して入力として受け取ります．
 1. ロボットにマーカーの周囲をトレースするように命令する
    関節軌道を生成します．
    （ 接着剤を塗布するような場合を想定 ）


## Scan-N-Plan アプリケーション: ガイダンス

時間節約のために，次の内容の `descartes_node.cpp` というファイルを導入します．

 1. 直交座標系における動作軌道計画のための
    新しいノードと関連クラスを定義
 1. 実際のサービスを定義し
    直交座標系ライブラリを初期化
 1. 高レベルの作業フローを提供（参照: planPath 関数）

演習課題として次のことを行います．

 1. ロボット「軌道」を構成する
    一連の直交座標系上の姿勢を定義します．
 1. これらの軌道を
    Descartes が処理できるものに変換します．


### ワークスペースのセットアップ

 1. Descartes リポジトリを
    ワークスペースの src/ ディレクトリにクローンします．

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

 1. `descartes_node` という名前の
    新しいノードをビルドするために，
    `myworkcell_core` パッケージの
    `CMakeLists.txt` にルールを作成します．

    これまでの演習と同じように，
    これらの行をテンプレートファイルの類似した行の近くに追加します．
    （ 下は1つのブロックではありません ）

    ```cmake
    add_executable(descartes_node src/descartes_node.cpp)
    add_dependencies(descartes_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(descartes_node ${catkin_LIBRARIES})
    ```


### Descartes ノードを完成させる

Descartes 動作計画アルゴリズムを実行するための
サービス・インタフェースを作成します．

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


### 作業セルノードの更新

Descartes ノードが完成した後，
新しい `ServiceClient` をメインの作業セルノードに追加して，
そのロジックを呼び出す必要があります．
このサービスの出力はこの後にロボットで実行する関節軌道です．
ロボットでの実行は様々な方法で達成できますが，
ここでは `JointTrajectoryAction` を直接呼びます．

 1. `myworkcell_node.cpp` に
    次のヘッダの include 文を追加します．

    ```c++
    #include <actionlib/client/simple_action_client.h>
    #include <control_msgs/FollowJointTrajectoryAction.h>
    #include <myworkcell_core/PlanCartesianPath.h>
    ```

    _これらのライブラリとメッセージは
     MoveIt! から引かれているので
     新しい依存関係を追加する必要はありません．_

 1. ScanNPlan クラスで
    新しい `PlanCartesianPath` 変数の
    `ServiceClient` と
    `FollowJointTrajectoryAction` の
    アクションクライアントを追加します．

    ```c++
    ros::ServiceClient cartesian_client_;
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_;
    ```

 1. これらの新しいオブジェクトを
    コンストラクタで初期化します．
    アクションクライアントは
    `initializer list` と呼ばれるもので
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
    直接アクションを送信することで
    その軌道を実行します．

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


### ヒントとヘルプ

ヒント:
 * `makeToolPoses()` で定義した軌道は
   作業部分の既知の参照点からの相対的なものです．
   したがって (0, 0, 0) のツール姿勢は
   ワールド座標系の原点ではなく参照点そのものに位置します．
 * `makeDescartesTrajectorty(...)` では
   "ref" 姿勢を使って
   相対的なツール姿勢をワールド座標に変換する必要があります．
 * `makeTolerancedCartesianPoint(...)` では，
   一般的な関節軌道の特定の実装に関する
   次のドキュメントを考慮してください．
   * <http://docs.ros.org/indigo/api/descartes_trajectory/html/>
 * さらに詳しいヘルプは
   `〜/industrial_training/exercises/4.1/src`
   の完成コードを参考にしてください．
