# 高度な直交座標系動作軌道計画

> 直交座標系動作軌道プランナ Descrates を使って，
ロボットがつかんでいる部品を固定工具で加工する複雑な動作軌道を生成します．


## モチベーション

MoveIt! はロボットを A 点から B 点に移動させる
「フリースペース」モーションを主な目的としたフレームワークで，
それがどのように処理されているかについて
ユーザは特に気にしなくても大丈夫なようになっています．
しかし，このようなタイプの問題は実行されるタスクの多くあるケースの一部にすぎません．
溶接や塗装といった製造の「プロセス」を想像してみてください．
ロボット動作の最初から最後まで
ツールがどこを指しているのかについて非常に気を配っています．

このチュートリアルでは，
任意のプロセス経路に沿ってロボットを動かすための
Descrates: 直交座標系（Cartesian/カーテジアン）
モーション・プランナを紹介します．
これはこの種の問題を解決する数多くの方法の1つではありますが，
いくつかの巧妙な特性を有しています．

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

本演習では新たに 2つのノード と 2つの xacro ，設定ファイル を
Scan-N-Plan アプリケーションに付け加えます．

 1. 軌道設定ファイル `puzzle_bent.csv` を使って
    ロボットが部品を持ち，固定ツールを周るマニピュレーション軌道を作成します．
 1. ロボットにマーカーの周囲をトレースするように指示する関節軌道を生成します．
    （接着剤を塗布しているような例）


## Scan-N-Plan アプリケーション: ガイダンス

時間節約のためにいくつかのファイルを導入します．

 1. 1つ目は
    テンプレートノード `adv_descartes_node.cpp` で，
    演習の大部分は形状の複雑な部品のバリ取りのための
    複雑な軌道を作成するために費やされます．
 1. 2つ目のノードは
    `adv_myworkcell_node.cpp` で，
    `adv_descartes_node.cpp` によって提供される
    `adv_plan_path` サービスを呼び出すように
    `myworkcell_node.cpp` を更新したファイルです．
 1. 設定ファイル `puzzle_bent.csv` には
    部品座標系に対する相対的な軌道が書かれています．
 1. 2つの xacro ファイル
    `puzzle_mount.xacro`と` grinder.xacro`は
    urdf/xacro の `workcell.xacro` ファイルを更新するものです．

演習では次のことを行います．

  1. workcell.xacro ファイルを更新して，
     2つの新しい xacro ファイルを組み込みます．
  1. moveit_config パッケージを更新して，
     新しいエンド・エフェクタ・リンクを含めた
     本演習用の新しい Planning Group を定義します．
  1. ロボットの「軌道」を構成する一連の直交座標系上の姿勢を定義します．
  1. これらの軌道を Descrates ライブラリが理解できるものに変換します．


### ワークスペースのセットアップ

 1. 本演習では基本トレーニングコースと同じワークスペースを使用しますが，
    演習 4.1 で完成させたワークスペースがない場合は，
    完成したリファレンス・コードをコピーして，
    次に示す他の依存関係のある必須パッケージを持ってきます．
    既にワークスペースがある場合は次の手順に進んでください．

    ```bash
    mkdir ~/catkin_ws
    cd ~/catkin_ws
    cp -r ~/industrial_training/exercises/4.1/src .
    cd src
    git clone https://github.com/jmeyer1292/fake_ar_publisher.git
    git clone -b kinetic-devel https://github.com/ros-industrial-consortium/descartes.git
    sudo apt install ros-kinetic-ur-kinematics
    sudo apt install ros-kinetic-ur-description
    ```

 1. `adv_descartes_node_unfinished.cpp` をコアパッケージの
    src/ フォルダにコピーして，
    ファイル名を `adv_descartes_node.cpp` に変更してください．

    ```bash
    cp ~/industrial_training/exercises/5.0/src/adv_descartes_node_unfinished.cpp myworkcell_core/src/adv_descartes_node.cpp
    ```

 1. `myworkcell_core` パッケージの `CMakeLists.txt` にルールを作成して，
    新しいノード `adv_descartes_node` をビルドします．

    先の演習と同様に，次の各行をひとまとまりのブロックとしてではなく
    テンプレートファイルの同じような行に追加します．

    ```cmake
    add_executable(adv_descartes_node src/adv_descartes_node.cpp)
    add_dependencies(adv_descartes_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(adv_descartes_node ${catkin_LIBRARIES})
    ```

 1. `adv_myworkcell_node.cpp` をコアパッケージの src/ フォルダにコピーします．

    ```bash
    cp ~/industrial_training/exercises/5.0/src/myworkcell_core/src/adv_myworkcell_node.cpp myworkcell_core/src/
    ```

 1. `myworkcell_core` パッケージの `CMakeLists.txt` にルールを作成して，
    新しいノード `adv_myworkcell_node` をビルドします．

    先の演習と同様に，次の各行をひとまとまりのブロックとしてではなく
    テンプレートファイルの同じような行に追加します．

    ```cmake
    add_executable(adv_myworkcell_node src/adv_myworkcell_node.cpp)
    add_dependencies(adv_myworkcell_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(adv_myworkcell_node ${catkin_LIBRARIES})
    ```

 1. 必要な設定ファイルをコピーします．

    ``` bash
    mkdir ~/catkin_ws/src/myworkcell_core/config
    cp ~/industrial_training/exercises/5.0/src/myworkcell_core/config/puzzle_bent.csv myworkcell_core/config/
    cp ~/industrial_training/exercises/5.0/src/myworkcell_support/urdf/grinder.xacro myworkcell_support/urdf/
    cp ~/industrial_training/exercises/5.0/src/myworkcell_support/urdf/puzzle_mount.xacro myworkcell_support/urdf/
    mkdir ~/catkin_ws/src/myworkcell_support/meshes
    cp ~/industrial_training/exercises/5.0/src/myworkcell_support/meshes/* myworkcell_support/meshes/
    ```

 1. 新しいパッケージの依存関係を追加します．

    * `tf_conversions` を
      `CMakeLists.txt` に2ヶ所，
      `package.xml` に1ヶ所，追加します．


### workcell.xacro ファイルの更新

 1. 新しく追加する `grinder.xacro` と `puzzle_mount.xacro` ファイルのための
    2つの `<include>` タグを追加します．

 1. 工具のグラインダを **world** リンクに次のオフセット値で付加します．
    ``` xml
    <origin xyz="0.0 -0.4 0.6" rpy="0 3.14159 0"/>
    ```
    * `grinder.xacro` ファイルを見て
      適切な `<child_link>` の名前を探します．
    * 他の `<joint>` タグ定義の1つをコピーして，
      必要に応じて修正してください．

 1. パズルマウントをロボットの **tool0** フレームに次のオフセット値で付加します．
    ``` xml
    <origin xyz="0 0 0" rpy="1.5708 0 0"/>
    ```
    * `puzzle_mount.xacro` ファイルを見て
      適切な `<child_link>` の名前を探します．
      この部品の大本のリンクを見つけるには，
      様々な `<link>` と `<joint>` の定義を調べる必要があります．
    * `tool0` フレームはほとんどの ROS-I URDF で標準化された
      ロボットのエンド・エフェクタ取り付けフランジです．

 1. moveit_config パッケージの demo.launch を起動して
    作業セルを確認してください．
    グラインダがテーブルに立てられて，
    パズルの形状をした部品がロボットに付加されているはずです．
    ``` bash
    roslaunch myworkcell_moveit_config demo.launch
    ```


### 新しい Planning Group の moveit_config パッケージへの追加

 1. MoveIt! Setup Assistant を再び実行して，
    新しい Planning Group として **puzzle** を作成します．
    **base_link** から **part** リンクまでの運動学的な連結を定義してください．
    ``` bash
    roslaunch myworkcell_moveit_config setup_assistant.launch
    ```
    * _注: ジオメトリを追加したので
      許される干渉の行列も再生成する必要があります．_


### 高度な Descartes ノードを完成させる

 1. まず最初に，関数 `makePuzzleToolPoses()` を完成させます．
    ファイル **puzzle_bent.csv** へのパスが必要です．
    コードの移植性を考慮してフルパスでハードコードすることは避けます．
    そのためには ROS のツール `ros::package::getPath()` を利用して，
    関連パッケージへのルートパスを取得します．

    * 参考:  [getPath()](http://docs.ros.org/kinetic/api/roslib/html/c++/namespaceros_1_1package.html#ae9470dd201aa4e66abb833e710d812a4) API

 1. 次に，関数 `makeDescartesTrajectory()` を完成させます．
    フレーム **world** と **grinder_frame** との間の変換が必要です．
    また各点は Z 軸から +/-PI に設定された方向許容誤差を持つ必要があります．

    * 参考  
      * [lookupTransform()](http://docs.ros.org/kinetic/api/tf/html/c++/classtf_1_1Transformer.html#ac01a9f8709a828c427f1a5faa0ced42b) API
      * [tf::conversions](http://docs.ros.org/kinetic/api/tf_conversions/html/c++/tf__eigen_8h.html) 名前空間（Namespace）
      * [TolerancedFrame](https://github.com/ros-industrial-consortium/descartes/blob/kinetic-devel/descartes_trajectory/include/descartes_trajectory/cart_trajectory_pt.h#L156) の定義
      * [OrientationTolerance](https://github.com/ros-industrial-consortium/descartes/blob/kinetic-devel/descartes_trajectory/include/descartes_trajectory/cart_trajectory_pt.h#L139) の定義


### setup.launch ファイルの更新

 1. ファイルを更新します．
    **adv** というブール値の引数を取って，
    基本的なノードか拡張された Descartes ノードの
    いずれかを起動できるようにします．
    どのノードを起動するかを制御するには
    `<if>` と `<unless>` 修飾子を使います．

    * 参考
      * [roslaunch XML](http://wiki.ros.org/roslaunch/XML) wiki
      * [roslaunch XML](http://wiki.ros.org/ja/roslaunch/XML) wiki（日本語）


### アプリケーション全体のテスト

 1. セットアップファイルを実行してから，
    拡張した作業セルノードを実行します．

    ``` bash
    roslaunch myworkcell_support setup.launch adv:=true
    rosrun myworkcell_core adv_myworkcell_node
    ```

    * Descartes ノードは複雑な起動を計画するのに
      **何分かかかります** ので，終了まで待ってください．
    * RViz の Planning Path の Loop Animation が
      常に実行されいていると状況を確認しづらくなります．
      `adv_myworkcell_node` を実行する前に
      アニメーション・ループを RViz でオフにしてください．
        * Displays -> Planned Path -> Loop Animation
