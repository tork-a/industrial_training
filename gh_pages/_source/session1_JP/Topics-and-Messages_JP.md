# Topics and Messages
>In this exercise, we will explore the concept of ROS messages and topics.

# トピックとメッセージ

> ROS のトピックとメッセージの概念について見てゆきます．

## Motivation
The first type of ROS communication that we will explore is a one-way communication called messages which are sent over channels called topics. Typically one node publishes messages on a topic and another node subscribes to messages on that same topic. In this module we will create a subscriber node which subscribes to an existing publisher (topic/message).


## モチベーション

私たちが見る最初のタイプの ROS 通信は
「トピック」と呼ばれるチャンネルを介して送信される
「メッセージ」という一方向通信です．
通常，1つのノードはトピックにメッセージを発行（ publish／パブリッシュ ）し，
別のノードは同じトピックのメッセージを購読（ subscribe／サブスクライブ ）します．
本演習では，既存のパブリッシャ（トピック／メッセージ）を購読する
「サブスクライバ・ノード」を作成します．


## Reference Example

[Create a Subscriber](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29)


## リファレンス

* [Writing a Simple Publisher and Subscriber (C++)](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29)

* [シンプルな配信者(Publisher)と購読者(Subscriber)を書く(C++)](http://wiki.ros.org/ja/ROS/Tutorials/WritingPublisherSubscriber(c%2B%2B))


## Further Information and Resources

[Understanding Topics](http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics)

[Examining Publisher & Subscriber](http://wiki.ros.org/ROS/Tutorials/ExaminingPublisherSubscriber)

[Creating Messages and Services](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv)


# 追加情報とリソース

* [Understanding ROS Topics](http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics)
* [Examining the Simple Publisher and Subscriber](http://wiki.ros.org/ROS/Tutorials/ExaminingPublisherSubscriber)
* [Creating a ROS msg and srv](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv)


* [ROSトピックの理解](http://wiki.ros.org/ja/ROS/Tutorials/UnderstandingTopics)
* [シンプルなPublisherとSubscriberを実行してみる](http://wiki.ros.org/ja/ROS/Tutorials/ExaminingPublisherSubscriber)
* [ROSのmsgとsrvを作る](http://wiki.ros.org/ja/ROS/Tutorials/CreatingMsgAndSrv)


## Scan-N-Plan Application: Problem Statement
We now have a base ROS node and we want to build on this node. Now we want to create a subscriber within our node.

Your goal is to create your first ROS subscriber:
 1. First you will want to find out the message structure.
 2. You also want to determine the topic name.
 3. Last you can write the c++ code which serves as the subscriber.


## Scan-N-Plan アプリケーション: 演習問題

基本となる ROS ノードが作成できたので，このノードで構築していきます．
ノード内にサブスクライバを作成します．

最初の ROS サブスクライバを作成することを目標としています．

 1. まず最初にメッセージ構造について学びます．
 1. また，トピック名も決定する必要があります．
 1. 最後にサブスクライバとして機能する C++ コードを書きます．


## Scan-N-Plan Application: Guidance
### Add the fake_ar_publisher Package as a Dependency

1. Locate the `fake_ar_publisher` package you downloaded earlier.

   ```
   rospack find fake_ar_publisher
   ```

2. Edit your package's `CMakeLists.txt` file (`~/catkin_ws/src/myworkcell_core/CMakeLists.txt`).  Make the following changes in the matching sections of the existing template file, by uncommenting and/or editing existing rules.

   1. Tell cmake to find the fake_ar_publisher package:

      ``` cmake
      ## Find catkin macros and libraries
      ## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
      ## is used, also find other catkin packages
      find_package(catkin REQUIRED COMPONENTS
        roscpp
        fake_ar_publisher
      )
      ```

   2. Add The catkin runtime dependency for publisher.

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

   3. Uncomment/edit the `add_dependencies` line __below__ your `add_executable` rule:

      ``` cmake
      add_dependencies(vision_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
      ```

3. add dependencies into your package's `package.xml`:

   ```xml
   <depend>fake_ar_publisher</depend>
   ```

4. `cd` into your catkin workspace

   ```
   cd ~/catkin_ws
   ```

5. Build your package and source the setup file to activate the changes in the current terminal.

   ```
   catkin build
   source ~/catkin_ws/devel/setup.bash
   ```

7. In a terminal, enter `rosmsg list`.  You will notice that, included in the list, is `fake_ar_publisher/ARMarker`.  If you want to see only the messages in a package, type `rosmsg package <package_name>`

8. Type `rosmsg show fake_ar_publisher/ARMarker`.  The terminal will return the types and names of the fields in the message.

   *Note that three fields under the `header` field are indented, indicating that these are members of the `std_msgs/Header` message type*


## Scan-N-Plan アプリケーション: ガイダンス

### 依存パッケージへの fake_ar_publisher の追加

1. 先ほどダウンロードした `fake_ar_publisher` パッケージを探します．

   ```
   rospack find fake_ar_publisher
   ```

1. 演習パッケージの
   `CMakeLists.txt` ファイル（ `~/catkin_ws/src/myworkcell_core/CMakeLists.txt` ）
   を編集します．

   既存のルールのコメントアウトを解除したり編集したりして，
   テンプレートファイルの一致する箇所で次の変更を行います．

   1. fake_ar_publisher パッケージを見つけるために cmake に指示します．

      ``` cmake
      ## Find catkin macros and libraries
      ## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
      ## is used, also find other catkin packages
      find_package(catkin REQUIRED COMPONENTS
        roscpp
        fake_ar_publisher
      )
      ```

   1. パブリッシャのための catkin のランタイム依存関係を追加します．

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
      `add_dependencies` 行のコメントを解除と編集をしてください．

      ``` cmake
      add_dependencies(vision_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
      ```

1. 本演習のパッケージの `package.xml` に依存関係を追加します．

   ```xml
   <depend>fake_ar_publisher</depend>
   ```

1. 演習パッケージの catkin ワークスペースに `cd` で移動します．

   ```
   cd ~/catkin_ws
   ```

1. パッケージをビルドして変更を現在のターミナルに反映させるために
   セットアップファイルを実行します．

   ```
   catkin build
   source ~/catkin_ws/devel/setup.bash
   ```

1. ターミナルで `rosmsg list` と入力してください．

   表示されたリストに `fake_ar_publisher/ARMarker` を見つけることができるはずです．

   あるパッケージにあるメッセージだけを表示させたい場合には
   `rosmsg package <package_name>` とします．

1. `rosmsg show fake_ar_publisher/ARMarker` と入力してください．

   ターミナルはメッセージ内のフィールドのタイプと名前を返します．

   * `header` フィールドの下にある3つのフィールドはインデントされており，
     これらが `std_msgs/Header` メッセージ型のメンバであることを示しています．


### Run a Publisher Node

1. In a terminal, type `rosrun fake_ar_publisher fake_ar_publisher_node`. You should see the program start up and begin publishing messages.

2. In another terminal, enter `rostopic list`.  You should see `/ar_pose_marker` among the topics listed. Entering `rostopic type /ar_pose_marker` will return the type of the message.

3. Enter `rostopic echo /ar_pose_marker`. The terminal will show the fields for each message as they come in, separated by a `---` line.  Press Ctrl+C to exit.

4. Enter `rqt_plot`.

   1. Once the window opens, type `/ar_pose_marker/pose/pose/position/x` in the "Topic:" field and click the "+" button. You should see the X value be plotted.

   2. Type `/ar_pose_marker/pose/pose/position/y` in the topic field, and click on the add button.  You will now see both the x and y values being graphed.

   3. Close the window

5. Leave the publisher node running for the next task.


### パブリッシャ・ノードの実行

1. ターミナルで `rosrun fake_ar_publisher fake_ar_publisher_node` と入力してください．

   プログラムの起動とメッセージの配信を開始する様子が見られるはずです．

1. もう1つのターミナルで `rostopic list` と入力してください．

   トピックのリストに `/ar_pose_marker` があるはずです．

   `rostopic type /ar_pose_marker` を入力するとメッセージのタイプが返されます．

1. `rostopic echo /ar_pose_marker` と入力してください．

   ターミナルは各メッセージのフィールドを `---` 行で区切って表示します．

   Ctrl+C を押して終了します．

1. `rqt_plot` と入力してください．

   1. ウィンドウが開いたら "Topic:" 欄に
      `/ar_pose_marker/pose/pose/position/x` と入力して
      "+" ボタンをクリックしてください．X の値がプロットされるはずです．

   1. トピック欄に `/ar_pose_marker/pose/pose/position/y` と入力して
      追加ボタンをクリックしてください．X と Y の値が両方グラフになるはずです．

   1. ウィンドウを閉じて下さい．

1. パブリッシャ・ノードは次のタスクでも用いますのでそのまま実行しておいてください．


## Create a Subscriber Node
1. Edit the `vision_node.cpp` file.

1. Include the message type as a header

   ``` c++
   #include <fake_ar_publisher/ARMarker.h>
   ```

1. Add the code that will be run when a message is received from the topic (the callback).

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

1. Add the code that will connect the callback to the topic (within `main()`)

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

   * You can replace or leave the "Hello World" print... your choice!
   * These new lines must go below the `NodeHandle` declaration, so `nh` is actually defined.
   * Make sure to retain the `ros::spin()` call.  It will typically be the last line in your `main` routine.  Code after `ros::spin()` won't run until the node is shutting down.

1. Run `catkin build`, then `rosrun myworkcell_core vision_node`.

1. You should see the positions display from the publisher.

1. Press Ctrl+C on the publisher node.  The subscriber will stop displaying information.

1. Start the publisher node again. The subscriber will continue to print messages as the new program runs.

   * This is a key capability of ROS, to be able to restart individual nodes without affecting the overall system.

1. In a new terminal, type `rqt_graph`. You should see a window similar to the one below:

<p align="center"><img src=http://aeswiki.datasys.swri.edu/rositraining/Exercises/1.6?action=AttachFile&do=get&target=1.png /></p>

   * The rectangles in the the window show the topics currently available on the system.
   * The ovals are ROS nodes.
   * Arrows leaving the node indicate the topics the node publishes, and arrows entering the node indicate the topics the node subscribes to.


## サブスクライバ・ノードの作成

1. `vision_node.cpp` ファイルを編集します．

1. メッセージ型のヘッダファイルをインクルードします．

   ``` c++
   #include <fake_ar_publisher/ARMarker.h>
   ```

1. トピックからメッセージを受け取ったときに実行（コールバック）されるコードを追加します．

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

1. コールバックをトピックに接続するコードを `main()` の中に追加します．

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

   * "Hello World" と出力する機能は残しても残さなくてもどちらでも構いません．
   * これらの新しい行は `NodeHandle` の宣言よりも下でなければなりませんので
     `nh` は実際に定義されています．
   * `ros::spin()` コールは必ず保持してください．
     通常 `main` ルーチンの最後の行にします．
     `ros::spin()` の後のコードはノードがシャットダウンされるまで実行されません．

1. `catkin build` を実行した後に
   `rosrun myworkcell_core vision_node` を実行してください．

1. パブリッシャーから得た座標が表示されます．

1. Ctrl+C でパブリッシャ・ノードを停止します．
   サブスクライバは情報表示を停止するはずです．

1. パブリッシャ・ノードを再び起動します．

   サブスクライバは新しくプログラムを実行したときと同じように
   メッセージの表示を継続するはずです．

   * これはシステム全体に影響を与えずに
     個々のノードを再起動できるようにする ROS の重要な機能です．


1. 新しいターミナルで `rqt_graph` と入力してください．

   次のようなウィンドウが表示されるはずです．

<p align="center"><img src=http://aeswiki.datasys.swri.edu/rositraining/Exercises/1.6?action=AttachFile&do=get&target=1.png /></p>

   * ウィンドウ内の四角形はシステム上で現在利用可能なトピックを示しています．
   * 楕円形は ROS ノードです．
   * ノードから出ている矢印はノードがパブリッシュするトピックを示し，
     ノードに入っている矢印はノードがサブスクライブするトピックを示します．
