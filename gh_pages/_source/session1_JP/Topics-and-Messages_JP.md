# トピックとメッセージ

> ROS のトピックとメッセージの概念について見てゆきます．


## モチベーション

私たちが見る最初のタイプの ROS 通信は
「トピック」と呼ばれるチャンネルを介して送信される
「メッセージ」という一方向通信です．
通常，1つのノードはトピックにメッセージを発行
（ publish／パブリッシュ ）し，
別のノードは同じトピックのメッセージを購読
（ subscribe／サブスクライブ ）します．
本演習では，既存のパブリッシャ（トピック／メッセージ）を購読する
「サブスクライバ・ノード」を作成します．


## リファレンス

* [Writing a Simple Publisher and Subscriber (C++)](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29)

* [シンプルな配信者(Publisher)と購読者(Subscriber)を書く(C++)](http://wiki.ros.org/ja/ROS/Tutorials/WritingPublisherSubscriber(c%2B%2B))


## 追加情報とリソース

* [Understanding ROS Topics](http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics)
* [Examining the Simple Publisher and Subscriber](http://wiki.ros.org/ROS/Tutorials/ExaminingPublisherSubscriber)
* [Creating a ROS msg and srv](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv)


* [ROSトピックの理解](http://wiki.ros.org/ja/ROS/Tutorials/UnderstandingTopics)
* [シンプルなPublisherとSubscriberを実行してみる](http://wiki.ros.org/ja/ROS/Tutorials/ExaminingPublisherSubscriber)
* [ROSのmsgとsrvを作る](http://wiki.ros.org/ja/ROS/Tutorials/CreatingMsgAndSrv)


## Scan-N-Plan アプリケーション: 演習問題

基本となる ROS ノードが作成できたので，
このノードで構築していきます．
ノード内にサブスクライバを作成します．

最初の ROS サブスクライバを作成すること
を目標としています．

 1. まず最初にメッセージ構造について学びます．
 1. また，トピック名も決定する必要があります．
 1. 最後にサブスクライバとして機能する C++ コードを書きます．


## Scan-N-Plan アプリケーション: ガイダンス

### 依存パッケージへの fake_ar_publisher の追加

1. 先ほどダウンロードした
   `fake_ar_publisher` パッケージを探します．

   ```
   rospack find fake_ar_publisher
   ```

1. 演習パッケージの
   `CMakeLists.txt` ファイル
   （ `~/catkin_ws/src/myworkcell_core/CMakeLists.txt` ）
   を編集します．

   既存のルールのコメントアウト
   を解除したり編集したりして，
   テンプレートファイルの
   一致する箇所で次の変更を行います．

   1. fake_ar_publisher パッケージを見つけるために
      cmake に指示します．

      ``` cmake
      ## Find catkin macros and libraries
      ## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
      ## is used, also find other catkin packages
      find_package(catkin REQUIRED COMPONENTS
        roscpp
        fake_ar_publisher
      )
      ```

   1. パブリッシャのための
      catkin のランタイム依存関係
      を追加します．

      ``` cmake
      ## The catkin_package macro generates cmake config files for your package
      ## Declare things to be passed to dependent projects
      ## LIBRARIES: libraries you create in this project that dependent projects also need
      ## CATKIN_DEPENDS: catkin_packages dependent projects also need
      ## DEPENDS: system dependencies of this project that dependent projects also need
      catkin_package(
       #  INCLUDE_DIRS include
       #  LIBRARIES myworkcell_core
         CATKIN_DEPENDS
           roscpp
           fake_ar_publisher
      #  DEPENDS system_lib
      )
      ```

   1. `add_executable` ルールの **下に** ある
      `add_dependencies` 行のコメント
      の解除と編集をしてください．

      ``` cmake
      add_dependencies(vision_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
      ```

1. 本演習のパッケージの
   `package.xml` に依存関係を追加します．

   ```xml
   <depend>fake_ar_publisher</depend>
   ```

1. 演習パッケージの
   catkin ワークスペースに
   `cd` で移動します．

   ```
   cd ~/catkin_ws
   ```

1. パッケージをビルドしてから，
   変更を現在のターミナルに反映させるために
   セットアップファイルを実行します．

   ```
   catkin build
   source ~/catkin_ws/devel/setup.bash
   ```

1. ターミナルで
   `rosmsg list` と入力してください．

   表示されたリストに
   `fake_ar_publisher/ARMarker`
   を見つけることができるはずです．

   あるパッケージにある
   メッセージだけを表示させたい場合には
   `rosmsg package <package_name>`
   とします．

1. `rosmsg show fake_ar_publisher/ARMarker`
   と入力してください．

   ターミナルはメッセージ内の
   フィールドのタイプと名前を返します．

   * `header` フィールドの下にある
     3つのフィールドはインデントされており，
     これらが `std_msgs/Header` メッセージ型のメンバ
     であることを示しています．


### パブリッシャ・ノードの実行

1. ターミナルで
   `rosrun fake_ar_publisher fake_ar_publisher_node`
   と入力してください．

   プログラムの起動とメッセージの配信を開始する
   様子が見られるはずです．

1. もう1つのターミナルで
   `rostopic list`
   と入力してください．

   トピックのリストに
   `/ar_pose_marker` があるはずです．

   `rostopic type /ar_pose_marker`
   を入力するとメッセージのタイプが返されます．

1. `rostopic echo /ar_pose_marker`
   と入力してください．

   ターミナルは各メッセージのフィールドを
   `---` 行で区切って表示します．

   Ctrl+C を押して終了します．

1. `rqt_plot` と入力してください．

   1. ウィンドウが開いたら
      "Topic:" 欄に
      `/ar_pose_marker/pose/pose/position/x` と入力して
      "+" ボタンをクリックしてください．
      X の値がプロットされるはずです．

   1. トピック欄に
      `/ar_pose_marker/pose/pose/position/y` と入力して
      追加ボタンをクリックしてください．
      X と Y の値が両方グラフになるはずです．

   1. ウィンドウを閉じて下さい．

1. パブリッシャ・ノードは
   次のタスクでも用いますので
   そのまま実行しておいてください．


## サブスクライバ・ノードの作成

1. `vision_node.cpp` ファイルを編集します．

1. メッセージ型のヘッダファイルをインクルードします．

   ``` c++
   #include <fake_ar_publisher/ARMarker.h>
   ```

1. トピックからメッセージを受け取ったときに実行
   （コールバック）されるコードを追加します．

   ``` c++
   class Localizer
   {
   public:
     Localizer(ros::NodeHandle& nh)
     {
         ar_sub_ = nh.subscribe<fake_ar_publisher::ARMarker>("ar_pose_marker", 1,
         &Localizer::visionCallback, this);
     }

     void visionCallback(const fake_ar_publisher::ARMarkerConstPtr& msg)
     {
         last_msg_ = msg;
         ROS_INFO_STREAM(last_msg_->pose.pose);
     }

     ros::Subscriber ar_sub_;
     fake_ar_publisher::ARMarkerConstPtr last_msg_;
   };
   ```

1. コールバックをトピックに接続するコード
   を `main()` の中に追加します．

   ``` c++
   int main(int argc, char** argv)
   {
     ...
     // The Localizer class provides this node's ROS interfaces
     Localizer localizer(nh);

     ROS_INFO("Vision node starting");
     ...
   }
   ```

   * "Hello World" と出力する機能は
     残しても残さなくてもどちらでも構いません．
   * これらの新しい行は `NodeHandle` の宣言よりも
     下でなければなりませんので
     `nh` は実際に定義されています．
   * `ros::spin()` コールは必ず保持してください．
     通常 `main` ルーチンの最後の行にします．
     `ros::spin()` の後のコードは
     ノードがシャットダウンされるまで実行されません．

1. `catkin build` を実行した後に
   `rosrun myworkcell_core vision_node`
   を実行してください．

1. パブリッシャーから得た座標が表示されます．

1. Ctrl+C でパブリッシャ・ノードを停止します．
   サブスクライバは情報表示を停止するはずです．

1. パブリッシャ・ノードを再び起動します．

   サブスクライバは
   新しくプログラムを実行したときと同じように
   メッセージの表示を継続するはずです．

   * これはシステム全体に影響を与えずに
     個々のノードを再起動できるようにする
     ROS の重要な機能です．


1. 新しいターミナルで
   `rqt_graph` と入力してください．

   次のようなウィンドウが表示されるはずです．

<p align="center"><img src=http://aeswiki.datasys.swri.edu/rositraining/Exercises/1.6?action=AttachFile&do=get&target=1.png /></p>

   * ウィンドウ内の四角形は
     システム上で現在利用可能なトピックを示しています．
   * 楕円形は ROS ノードです．
   * ノードから出ている矢印は
     ノードがパブリッシュするトピックを示し，
     ノードに入っている矢印は
     ノードがサブスクライブするトピックを示します．
