# Creating Packages and Nodes
>In this exercise, we will create our own ROS package and node.

# パッケージとノードの作成

> ROS のパッケージとノードを作成します．


## Motivation
The basis of ROS communication is that multiple executables called nodes are running in an environment and communicating with each other in various ways. These nodes exist within a structure called a package. In this module we will create a node inside a newly created package.


## モチベーション

ROS の通信は，
ノードと呼ばれる複数の実行可能ファイルが環境内で実行され，
それらがさまざまな方法で相互に通信することが基本となっています．
これらのノードはパッケージと呼ばれる構造内に存在します．
本演習では新たに作成したパッケージ内にノードを作成します．


## Reference Example
[Create a Package](http://wiki.ros.org/ROS/Tutorials/CreatingPackage)


## リファレンス

* [Creating a ROS Package](http://wiki.ros.org/ROS/Tutorials/CreatingPackage)
* [ROSパッケージを作る](http://wiki.ros.org/ja/ROS/Tutorials/CreatingPackage)


## Further Information and Resources
[Building Packages](http://wiki.ros.org/ROS/Tutorials/BuildingPackages)

[Understanding Nodes](http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes)


## 追加情報とリソース

* [Building a ROS Package](http://wiki.ros.org/ROS/Tutorials/BuildingPackages)
* [Understanding ROS Nodes](http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes)


* [ROSのパッケージをビルドする](http://wiki.ros.org/ja/ROS/Tutorials/BuildingPackages)
* [ROSのノードを理解する](http://wiki.ros.org/ja/ROS/Tutorials/UnderstandingNodes)


## Scan-N-Plan Application: Problem Statement
We've installed ROS, created a workspace, and even built a few times. Now we want to create our own package and our own node to do what we want to do.

Your goal is to create your first ROS node:

 1. First you need to create a package inside your catkin workspace.

 2. Then you can write your own node


## Scan-N-Plan アプリケーション: 演習問題

これまでに ROS をインストールし，ワークスペースを作成し，さらにビルドを何回か行いました．
ここでは自分たちが行いたいことをするために独自のパッケージと独自のノードを作成します．

目標は ROS ノードを作成することです．

 1. まず catkin ワークスペース内にパッケージを作成する必要があります．

 2. 次にそこに独自のノードを記述します．


## Scan-N-Plan Application: Guidance

### Create a Package
1. cd into the catkin workspace src directory
   _Note: Remember that all packages should be created inside a workspace src directory._

   ```
   cd ~/catkin_ws/src
   ```

1. Use the ROS command to create a package called _myworkcell_core_ with a dependency on _roscpp_

   ```
   catkin create pkg myworkcell_core --catkin-deps roscpp
   ```

   See the [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_create.html) documentation for more details on this command.

   * _This command creates a directory and required files for a new ROS package._
   * _The first argument is the name of the new ROS package._
   * _Use `--catkin-deps` to specify packages which the newly created package depends on._

1. There will now be a folder named _myworkcell_core_. Change into that folder and edit the _package.xml_ file. Edit the file and change the description, author, etc., as desired.

   ```
   cd myworkcell_core
   gedit package.xml
   ```

   _If you forget to add a dependency when creating a package, you can add additional dependencies in the _package.xml_ file._



### STOP!  We'll go through a few more lecture slides before continuing this exercise.


## Scan-N-Plan アプリケーション: ガイダンス

### パッケージの作成

1. catkin ワークスペースの src ディレクトリに cd します．

   _注: src ディレクトリ内に全てのパッケージは作成されることを忘れないでください．_

   ```
   cd ~/catkin_ws/src
   ```

1. ROS のコマンドを利用して _roscpp_ に依存関係を有する
   _myworkcell_core_ というパッケージを作成します．

   ```
   catkin create pkg myworkcell_core --catkin-deps roscpp
   ```

   ドキュメント
   [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_create.html)
   を参照してこのコマンド理解を深めてください．

   * _本コマンドは新しいパッケージのための新しいディレクトリと必要なファイルを作成します．_
   * _最初の引数は ROS パッケージの名前です．_
   * _`--catkin-deps` を用いて新しいパッケージが依存関係にあるパッケージを設定します．_

1. _myworkcell_core_ という名前のフォルダができているはずです．
   そのフォルダに移動して _package.xml_ ファイルを編集します．
   description や author などの記述を編集します．

   ```
   cd myworkcell_core
   gedit package.xml
   ```

   _パッケージ作成時に依存関係を追加することを忘れた場合は，
   package.xml ファイルに依存関係を追加することができます．_


### ストップ！ この演習を続ける前にいくつかの講義スライドをもう一度見ていきます．


### Create a Node
1. In the package folder, edit the _CMakeLists.txt_ file using _gedit_. Browse through the example rules, and add an executable(_add_executable_), node named vision_node, source file named vision_node.cpp. Also within the _CMakeLists.txt_, make sure your new vision_node gets linked ('target_link_libraries') to the catkin libraries.

   ``` cmake
   add_compile_options(-std=c++11)
   add_executable(vision_node src/vision_node.cpp)
   target_link_libraries(vision_node ${catkin_LIBRARIES})
   ```

   These lines can be placed anywhere in `CMakeLists.txt`, but I typically:

   * Uncomment existing template examples for `add_compile_options` near the top (just below `project()`)
   * Uncomment and edit existing template examples for `add_executable` and `target_link_libraries` near the bottom
   * This helps make sure these rules are defined in the correct order, and makes it easy to remember the proper syntax.

   _Note: You're also allowed to spread most of the CMakeLists rules across multiple lines, as shown in the `target_link_libraries` template code_

1. In the package folder, create the file _src/vision_node.cpp_ (using _gedit_).

1. Add the ros header (include ros.h).

   ``` c++
   /**
   **  Simple ROS Node
   **/
   #include <ros/ros.h>
   ```

1. Add a main function (typical in c++ programs).

   ``` c++
   /**
   **  Simple ROS Node
   **/
   #include <ros/ros.h>

   int main(int argc, char* argv[])
   {

   }
   ```

1. Initialize your ROS node (within the main).

   ``` c++
   /**
   **  Simple ROS Node
   **/
   #include <ros/ros.h>

   int main(int argc, char* argv[])
   {
     // This must be called before anything else ROS-related
     ros::init(argc, argv, "vision_node");
   }
   ```

1. Create a ROS node handle.

   ``` c++
   /**
   **  Simple ROS Node
   **/
   #include <ros/ros.h>

   int main(int argc, char* argv[])
   {
     // This must be called before anything else ROS-related
     ros::init(argc, argv, "vision_node");

     // Create a ROS node handle
     ros::NodeHandle nh;
   }
   ```

1. Print a "Hello World" message using ROS print tools.

   ``` c++
   /**
   **  Simple ROS Node
   **/
   #include <ros/ros.h>

   int main(int argc, char* argv[])
   {
     // This must be called before anything else ROS-related
     ros::init(argc, argv, "vision_node");

     // Create a ROS node handle
     ros::NodeHandle nh;

     ROS_INFO("Hello, World!");
   }
   ```

1. Do not exit the program automatically - keep the node alive.

   ``` c++
   /**
   **  Simple ROS Node
   **/
   #include <ros/ros.h>

   int main(int argc, char* argv[])
   {
     // This must be called before anything else ROS-related
     ros::init(argc, argv, "vision_node");

     // Create a ROS node handle
     ros::NodeHandle nh;

     ROS_INFO("Hello, World!");

     // Don't exit the program.
     ros::spin();
   }
   ```

   _ROS_INFO_ is one of the many [logging methods](http://wiki.ros.org/roscpp/Overview/Logging).

   * It will print the message to the terminal output, and send it to the _/rosout_ topic for other nodes to monitor.
   * There are 5 levels of logging: _DEBUG, INFO, WARNING, ERROR, & FATAL._
   * To use a different logging level, replace INFO in _ROS_INFO_ or _ROS_INFO_STREAM_ with the appropriate level.
   * Use _ROS_INFO_ for printf-style logging, and _ROS_INFO_STREAM_ for cout-style logging.

1. Build your program (node), by running `catkin build` in a terminal window

   * _Remember that you must run `catkin build` from within your `catkin_ws` (or any subdirectory)_
   * This will build all of the programs, libraries, etc. in _myworkcell_core_
   * In this case, it's just a single ROS node _vision_node_


### ノードの作成

1. 本パッケージのフォルダで _gedit_ を使用して _CMakeLists.txt_ ファイルを編集します．

   例として書かれているルールを参考にしながら，実行可能ファイル（ _add_executable_ ）に
   vision_node という名前のノードと vision_node.cpp という名前のソースファイルを追加します．
   また _CMakeLists.txt_ 内で新しく作成するノード vision_node が
   catkin ライブラリにリンクされている（ 'target_link_libraries' ）ことを確認してください．

   ``` cmake
   add_compile_options(-std=c++11)
   add_executable(vision_node src/vision_node.cpp)
   target_link_libraries(vision_node ${catkin_LIBRARIES})
   ```

   これらの行は `CMakeLists.txt` どこに記述されていても大丈夫ですが，
   一般的には次のようにしています．

   * 上の `add_compile_options` の
     既存のテンプレートのコメントを外します（ `project()` の直下 ）
   * 最下部近くにある `add_executable` と `target_link_libraries` の
     既存のテンプレートのコメントを外して編集します.
   * これにより，これらのルールが正しい指示で確実に定義され，
     適切な構文を憶えやすくなります．

   _注: テンプレートコードの `target_link_libraries` のように，
    ほとんどの CMakeLists ルールは複数の行に分けて記述するできます．_

1. 本パッケージのフォルダで _src/vision_node.cpp_ というファイルを作成します．（ _gedit_ を利用 ）

1. ros のヘッダを加えます．（ ros.h のインクルード ）

   ``` c++
   /**
   **  Simple ROS Node
   **/
   #include <ros/ros.h>
   ```

1. main 関数を加えます．（ 典型的な C++ プログラム ）

   ``` c++
   /**
   **  Simple ROS Node
   **/
   #include <ros/ros.h>

   int main(int argc, char* argv[])
   {

   }
   ```

1. ROS ノードを初期化します．（ main 関数内 ）

   ``` c++
   /**
   **  Simple ROS Node
   **/
   #include <ros/ros.h>

   int main(int argc, char* argv[])
   {
     // This must be called before anything else ROS-related
     ros::init(argc, argv, "vision_node");
   }
   ```

1. ROS ノードハンドルを作成します．

   ``` c++
   /**
   **  Simple ROS Node
   **/
   #include <ros/ros.h>

   int main(int argc, char* argv[])
   {
     // This must be called before anything else ROS-related
     ros::init(argc, argv, "vision_node");

     // Create a ROS node handle
     ros::NodeHandle nh;
   }
   ```

1. "Hello World" メッセージを ROS プリントツールを用いて表示します．

   ``` c++
   /**
   **  Simple ROS Node
   **/
   #include <ros/ros.h>

   int main(int argc, char* argv[])
   {
     // This must be called before anything else ROS-related
     ros::init(argc, argv, "vision_node");

     // Create a ROS node handle
     ros::NodeHandle nh;

     ROS_INFO("Hello, World!");
   }
   ```

1. 自動的にプログラムから抜けないようにして，
   ノードが走っている状態を維持します．

   ``` c++
   /**
   **  Simple ROS Node
   **/
   #include <ros/ros.h>

   int main(int argc, char* argv[])
   {
     // This must be called before anything else ROS-related
     ros::init(argc, argv, "vision_node");

     // Create a ROS node handle
     ros::NodeHandle nh;

     ROS_INFO("Hello, World!");

     // Don't exit the program.
     ros::spin();
   }
   ```

   _ROS_INFO_ は
   [logging methods](http://wiki.ros.org/roscpp/Overview/Logging)
   の1つです．

   * メッセージをターミナルに出力し，
     他のノードから見られるように _/rosout_ トピックに送信します．
   * _DEBUG, INFO, WARNING, ERROR, FATAL_ の5つのレベルのログがあります．
   * 他のログレベルを使用するには _ROS_INFO_ または
     _ROS_INFO_STREAM_ の INFO を適切なレベルに置き換えます．
   * printf 形式のログには _ROS_INFO_ を使用し，
     cout 形式のログには _ROS_INFO_STREAM_ を使用します．

1. ターミナルのウィンドウで `catkin build` を実行して，
   プログラム（ノード）をビルドします．

   * _`catkin_ws`（または任意のサブディレクトリ）の中から
      `catkin build` を実行しなければならないことを忘れないでください．_
   * _myworkcell_core_ にあるプログラム，ライブラリなどをすべてビルドします．
   * 今回は1つの ROS ノード _vision_node_ だけです．


### Run a Node

1. Open a terminal and start the ROS master.

   ```
   roscore
   ```

   _The ROS Master must be running before any ROS nodes can function._

2. Open a second terminal to run your node.

   * In a previous exercise, we added a line to our `.bashrc` to automatically source `devel/setup.bash` in new terminal windows
   * This will automatically export the results of the build into your new terminal session.
   * If you're reusing an existing terminal, you'll need to manually source the setup files (since we added a new node):

     ```
     source ~/catkin_ws/devel/setup.bash
     ```

3. Run your node.

   ```
   rosrun myworkcell_core vision_node
   ```

   _This runs the program we just created. Remember to use TAB to help speed-up typing and reduce errors._

4. In a third terminal, check what nodes are running.

   ```
   rosnode list
   ```

   _In addition to the /rosout node, you should now see a new /vision_node listed._

5. Enter _rosnode kill /vision_node_. This will stop the node.

   _Note: It is more common to use Ctrl+C to stop a running node in the current terminal window._

### Challenge

Goal: Modify the node so that it prints your name. This will require you to run through the build process again.


### ノードを実行する

1. ターミナルを開いて ROS マスターを起動します．

   ```
   roscore
   ```

   _ROS マスターは ROS ノードが働き始めるまえに起動しておく必要があります．_

1. 2つ目のターミナルで本演習のノードを実行します．

   * 前回の演習で `.bashrc` ファイルに追記して
     `devel/setup.bash` が新しいターミナルで自動的に実行されるようにしました．
   * これによりビルドした結果が新しいターミナルのセッションに自動的に反映されます．
   * 既に起動していたターミナルを利用する場合は，
     新しいノードを追加した後に手動でセットアップファイルを実行する必要があります．

     ```
     source ~/catkin_ws/devel/setup.bash
     ```

1. ノードを実行します．

   ```
   rosrun myworkcell_core vision_node
   ```

   _これは今作成したプログラムを実行します．
    入力のスピードアップとエラーを減らすために
    Tab キーを使用することを忘れないでください．_

1. 3つ目のターミナルで，どのようなノードが走っているのかを確認します．

   ```
   rosnode list
   ```

   _/rosout ノードに加えて，
    新しい /vision_node がリストされているはずです．_

5. _rosnode kill /vision_node_ と入力してください．ノードが停止します．

   _注: 現在のターミナルウィンドウで実行中のノードを停止するには
    Ctrl+C を使用する方が一般的です．_


### チャレンジ

目標: あなたの名前を出力するようにノードを変更します．
そのためにはビルドプロセスを再度実行する必要があります．
