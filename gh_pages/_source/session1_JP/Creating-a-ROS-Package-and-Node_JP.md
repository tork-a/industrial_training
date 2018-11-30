# パッケージとノードの作成

> ROS のパッケージとノードを作成します．


## モチベーション

ROS の通信は，
ノードと呼ばれる複数の実行可能ファイルが環境内で実行され，
それらがさまざまな方法で相互に通信することが基本となっています．
これらのノードはパッケージと呼ばれる構造内に存在します．
本演習では新たに作成したパッケージ内にノードを作成します．


## リファレンス

* [Creating a ROS Package](http://wiki.ros.org/ROS/Tutorials/CreatingPackage)

* [ROSパッケージを作る](http://wiki.ros.org/ja/ROS/Tutorials/CreatingPackage)


## 追加情報とリソース

* [Building a ROS Package](http://wiki.ros.org/ROS/Tutorials/BuildingPackages)
* [Understanding ROS Nodes](http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes)


* [ROSのパッケージをビルドする](http://wiki.ros.org/ja/ROS/Tutorials/BuildingPackages)
* [ROSのノードを理解する](http://wiki.ros.org/ja/ROS/Tutorials/UnderstandingNodes)


## Scan-N-Plan アプリケーション: 演習問題

これまでに
ROS をインストールし，
ワークスペースを作成し，
さらにビルドを何回か行いました．
ここでは自分たちが行いたいことをするために
独自のパッケージと独自のノードを作成します．

目標は ROS ノードを作成することです．

 1. まず catkin ワークスペース内に
    パッケージを作成する必要があります．

 2. 次にそこに独自のノードを記述します．


## Scan-N-Plan アプリケーション: ガイダンス

### パッケージの作成

1. catkin ワークスペースの
   src ディレクトリに cd します．

   _注: src ディレクトリ内に
    全てのパッケージが作成されることを
    憶えいておいてください．_

   ```
   cd ~/catkin_ws/src
   ```

1. ROS のコマンドを利用して
   _roscpp_ に依存関係を有する
   _myworkcell_core_ という
   パッケージを作成します．

   ```
   catkin create pkg myworkcell_core --catkin-deps roscpp
   ```

   ドキュメント
   [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_create.html)
   を参照してこのコマンド理解を深めてください．

   * _本コマンドは新しいパッケージのための
      新しいディレクトリと
      必要なファイルを作成します．_
   * _最初の引数は ROS パッケージの名前です．_
   * _`--catkin-deps` を用いて
      新しいパッケージから依存関係にある
      パッケージを設定します．_

1. _myworkcell_core_ という名前の
   フォルダができているはずです．
   そのフォルダに移動して
   _package.xml_ ファイルを編集します．
   description や author など
   の記述を編集します．

   ```
   cd myworkcell_core
   gedit package.xml
   ```

   _パッケージ作成時に
    依存関係を追加することを忘れた場合は，
    後から package.xml ファイルに
    依存関係を追加することができます．_


### ストップ！ この演習を続ける前にいくつかの講義スライドをもう一度見ていきます．


### ノードの作成

1. 本パッケージのフォルダで
   _gedit_ を使用して
   _CMakeLists.txt_ ファイルを編集します．

   例として書かれているルールを参考にしながら，
   実行可能ファイル（ _add_executable_ ）に
   vision_node という名前のノードと
   vision_node.cpp という名前の
   ソースファイルを追加します．
   また _CMakeLists.txt_ 内で
   新しく作成するノード vision_node が
   catkin ライブラリにリンクされている
   （ 'target_link_libraries' ）こと
   を確認してください．

   ``` cmake
   add_compile_options(-std=c++11)
   add_executable(vision_node src/vision_node.cpp)
   target_link_libraries(vision_node ${catkin_LIBRARIES})
   ```

   これらの行は `CMakeLists.txt` の中の
   どこに記述されていても大丈夫ですが，
   一般的には次のようにしています．

   * 上の `add_compile_options` の
     既存のテンプレートのコメントを外します（ `project()` の直下 ）
   * 最下部近くにある `add_executable` と
     `target_link_libraries` の
     既存のテンプレートのコメントを外して編集します.
   * これにより，これらのルールが正しい指示で確実に定義され，
     適切な構文を憶えやすくなります．

   _注: テンプレートコードの
    `target_link_libraries` のように，
    ほとんどの CMakeLists ルールは
    複数の行に分けて記述するできます．_

1. 本パッケージのフォルダで
   _src/vision_node.cpp_ というファイルを作成します．
   （ _gedit_ を利用 ）

1. ros のヘッダを加えます．
   （ ros.h のインクルード ）

   ``` c++
   /**
   **  Simple ROS Node
   **/
   #include <ros/ros.h>
   ```

1. main 関数を加えます．
   （ 典型的な C++ プログラム ）

   ``` c++
   /**
   **  Simple ROS Node
   **/
   #include <ros/ros.h>

   int main(int argc, char* argv[])
   {

   }
   ```

1. ROS ノードを初期化します．
   （ main 関数内 ）

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

1. "Hello World" メッセージを
   ROS プリントツールを用いて表示します．

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
     他のノードから見られるように
     _/rosout_ トピックに送信します．
   * _DEBUG, INFO, WARNING, ERROR, FATAL_ の
     5つのレベルのログがあります．
   * 他のログレベルを使用するには
     _ROS_INFO_ または _ROS_INFO_STREAM_ の
      INFO を適切なレベルに置き換えます．
   * printf 形式のログには
     _ROS_INFO_ を使用し，
     cout 形式のログには
     _ROS_INFO_STREAM_ を使用します．

1. ターミナルのウィンドウで `catkin build` を実行して，
   プログラム（ノード）をビルドします．

   * _`catkin_ws`（または任意のサブディレクトリ）の中から
      `catkin build` を実行しなければならないこと
      を忘れないでください．_
   * _myworkcell_core_ にあるプログラム，
     ライブラリなどをすべてビルドします．
   * 今回は1つの ROS ノード _vision_node_ だけです．


### ノードを実行する

1. ターミナルを開いて
   ROS マスターを起動します．

   ```
   roscore
   ```

   _ROS マスターは
    ROS ノードが働き始める前に
    起動しておく必要があります．_

1. 2つ目のターミナルで
   本演習のノードを実行します．

   * 前回の演習で
     `.bashrc` ファイルに追記して
     `devel/setup.bash` が新しいターミナルで
     自動的に実行されるようにしました．
   * これによりビルドした結果が
     新しいターミナルのセッションに自動的に反映されます．
   * 既に起動していたターミナルを利用する場合は，
     新しいノードを追加した後に
     手動でセットアップファイルを実行する必要があります．

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

1. 3つ目のターミナルで
   どのようなノードが走っているのかを確認します．

   ```
   rosnode list
   ```

   _/rosout ノードに加えて，
    新しい /vision_node が
    リストされているはずです．_

5. _rosnode kill /vision_node_ と入力してください．

   ノードが停止します．

   _注: 現在のターミナルウィンドウで
    実行中のノードを停止するには
    Ctrl+C を使用する方が一般的です．_


### チャレンジ

目標:
あなたの名前を出力するようにノードを変更します．
そのためにはビルドプロセスを再度実行する必要があります．
