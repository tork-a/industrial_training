# サービス

> 本演習では .srv ファイルを定義してカスタムサービスを作成します．
そしてこのサービスを利用するためのサーバーおよびクライアントノードを作成します．


## モチベーション

前回，演習を行った最初のタイプの ROS 通信は
「トピック」と呼ばれるチャンネルを介して送信される
「メッセージ」と呼ばれる一方向の相互作用でした．<br>
ここでは異なる通信タイプ，
あるノードから別のノードへの要求と
そのノードから最初のノードへの応答を介した
双方向の相互通信の使い方を学習します．
本演習ではサービスサーバ（リクエストを待って応答を出します）と
クライアント（情報を要求してから応答を待つ）を作成します．


## リファレンス

* [Writing a Simple Service and Client (C++)](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29)

* [C++でシンプルなサービスとクライアントを書く](http://wiki.ros.org/ja/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29 )


## 追加情報とリソース

* [Creating a ROS msg and srv](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv)
* [Understanding ROS Services and Parameters](http://wiki.ros.org/ROS/Tutorials/UnderstandingServicesParams)
* [Examining the Simple Service and Client](http://wiki.ros.org/ROS/Tutorials/ExaminingServiceClient)


* [ROSのmsgとsrvを作る](http://wiki.ros.org/ja/ROS/Tutorials/CreatingMsgAndSrv)
* [ROSのサービスとパラメータを理解する](http://wiki.ros.org/ja/ROS/Tutorials/UnderstandingServicesParams)
* [シンプルなサービスとクライアントを実行してみる](http://wiki.ros.org/ja/ROS/Tutorials/ExaminingServiceClient)


## Scan-N-Plan アプリケーション: 演習問題

いくつかの情報を購読するベースとなる
ROS ノードが既にあるので
このノード上に機能を加えていきます．
更に，このノードを別の「メイン」ノードの
サブ機能として使用したいと考えています．
元の視覚ノードは AR 情報を購読して
メインのワークセルノードからの
要求に返信するようにします．

目標はより複雑なノードシステムを作成することです．

1. 視覚ノードをアップデートして
   サービス・サーバを組み込みます．

2. ゆくゆくは Scan-N-Plan アプリケーション
   を実行する新しいノードを作成します．
   * まず，新しいノード（ myworkcell_core ）を
     サービスクライアントとして作成します．
     後にその機能を拡張してゆきます．


## Scan-N-Plan アプリケーション: ガイダンス

### サービス定義の作成

1. fake_ar_publisher パッケージにある
   メッセージファイルと同じように
   サービス定義を作成しますので，
   サービスファイルを作成する必要があります．

   サービスファイルの一般的な構造は
   次のようになっています．

   ```
   #request
   ---
   #response
   ```

1. `myworkcell_core` パッケージの中に
   `srv` という名前のフォルダを作成します．
   （ パッケージの `src` フォルダと同じ階層 ）

   ```
   cd ~/catkin_ws/src/myworkcell_core
   mkdir srv
   ```

1. `srv` フォルダ内に
   `LocalizePart.srv` というファイルを
   （ gedit や QT で ）作成します．

1. ファイルの中に
   `base_frame` という名前の `string` 型の要求と
   `pose` という名前の `geometry_msgs/Pose` 型の応答を
   上に概説したようにサービスを定義してください．

   ```
   #request
   string base_frame
   ---
   #response
   geometry_msgs/Pose pose
   ```

1. パッケージの `CMakeLists.txt` と
   `package.xml` を編集して，
   キーパッケージに依存関係を追加します．

   * `message_generation` は前のステップで作成した
     .srv ファイルから C++ コードをビルドする必要があります．
   * `message_runtime` は
     新しいメッセージのランタイム依存関係を提供します．
   * `geometry_msgs` は
     サービス定義で使用される `Pose` メッセージタイプを提供します．

   1. パッケージの `CMakeLists.txt` ファイルを編集して，
      新しい **ビルドタイム** 依存関係を
      既存の `find_package` ルールに追加してください．

      ``` cmake
      find_package(catkin REQUIRED COMPONENTS
        roscpp
        fake_ar_publisher
        geometry_msgs
        message_generation
      )
      ```

   1. 同じく `CMakeLists.txt` に
      新しい **ランタイム** 依存関係を
      既存の `catkin_package` ルールに追加します．

      ``` cmake
      catkin_package(
      #  INCLUDE_DIRS include
      #  LIBRARIES myworkcell_node
        CATKIN_DEPENDS
          roscpp
          fake_ar_publisher
          message_runtime
          geometry_msgs
      #  DEPENDS system_lib
      )
      ```

   1. `package.xml` ファイルを編集して
      適切なビルド・実行依存関係を追加します．

      ``` xml
      <build_depend>message_generation</build_depend>
      <exec_depend>message_runtime</exec_depend>
      <depend>geometry_msgs</depend>
      ```

1. パッケージの `CMakeLists.txt` を編集して
   新しいサービスファイルを生成するルールを追加します．

   1. 前に定義した `LocalizePart` サービスを参照するために，
      次の `CMakeLists.txt` ルールのコメントの解除と
      編集を行ってください．

      ``` cmake
      ## Generate services in the 'srv' folder
      add_service_files(
         FILES
         LocalizePart.srv
      )
      ```

   1. メッセージとサービスの生成を有効にするには，
      次の `CMakeLists.txt` ルールのコメントの解除と
      編集を行ってください．

      ``` cmake
      ## Generate added messages and services with any dependencies listed here
      generate_messages(
         DEPENDENCIES
         geometry_msgs
      )
      ```

1. これでサービスをパッケージに定義することができましたので，
   サービスを生成するコードを _Build_ できます．

   ```
   catkin build
   ```

   _注: Qt を用いても良いです．_


### サービスサーバ

1. `vision_node.cpp` を編集します．
   [ROS Wiki ](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29)
   （ [日本語](http://wiki.ros.org/ja/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29) ）
   が基になっています．

1. 今しがた作成したサービスのヘッダーを追加します．

   ``` c++
    #include <myworkcell_core/LocalizePart.h>
   ```

1. 他の `Localizer` クラスメンバ変数の近くに
   メンバ変数（ 型:`ServiceServer`，名前:`server_` ）
   を追加してください．

   ``` c++
   ros::ServiceServer server_;
   ```

1. `Localizer` クラスのコンストラクタで
   ROS マスターにこのサービスを告知してください．

   ``` c++
   server_ = nh.advertiseService("localize_part", &Localizer::localizePart, this);
   ```

1. 上記の `advertiseService` コマンドは
   `localizePart` というサービスコールバックを参照していました．
   `Localizer` クラスにこの名前の空のブール関数を作成します．

   リクエストとレスポンスのタイプは
   `LocalizePart.srv` ファイルで定義されている
   ことを思い出してください．
   ブール関数の引数はリクエストと応答の型で，
   一般的な構造は `Package::ServiceName::Request`
   または ` Package::ServiceName::Response` です．

   ``` c++
   bool localizePart(myworkcell_core::LocalizePart::Request& req,
                     myworkcell_core::LocalizePart::Response& res)
   {

   }
   ```

1. 次に `localizePart` コールバック関数にコードを追加して
   サービスレスポンスを記入します．

   最終的にこのコールバックは
   `fake_ar_publisher`（ `visionCallback` 内 ）
   から受け取ったポーズをサービスリクエストで
   指定されたフレームに変換します．

   今回はフレーム変換を飛ばして
   `fake_ar_publisher`
   から受け取ったデータをただ渡します．
   `fake_ar_publisher`（ `last_msg_` に保存 ）
   から受け取ったポーズ測定値を
   直接サービス応答にコピーします．

   ``` c++
   bool localizePart(myworkcell_core::LocalizePart::Request& req,
                     myworkcell_core::LocalizePart::Response& res)
   {
     // Read last message
     fake_ar_publisher::ARMarkerConstPtr p = last_msg_;  
     if (!p) return false;

     res.pose = p->pose.pose;
     return true;
   }
   ```

1. 無駄な情報が画面がいっぱいにならないように
   `visionCallback` 関数内の
   `ROS_INFO_STREAM` の呼び出し
   をコメントアウトする必要があります．

1. 更新された `vision_node` をビルドして
   コンパイルエラーがないことを確認します．


### サービスクライアント

1. `myworkcell_node.cpp` という名前の新しいノードを
   同じ `myworkcell_core` パッケージ内に）作成します．

   これは最終的にメインの「アプリケーションノード」になって，
   Scan-N-Plan タスクの動作順序を制御します．
   最初に実装する動作は，
   上で作成した視覚ノードの
   `LocalizePart` サービスから
   AR ターゲットの位置を要求することです．

1. `LocalizePart` サービスのヘッダーと同様に，
   標準の ros ヘッダファイルもインクルードしてください．

   ``` c++
   #include <ros/ros.h>
   #include <myworkcell_core/LocalizePart.h>
   ```

1. 一般的な ROS ノード初期化方法を用いて
   標準的な C++ メイン関数を作成します．

   ``` c++
   int main(int argc, char **argv)
   {
     ros::init(argc, argv, "myworkcell_node");
     ros::NodeHandle nh;

     ROS_INFO("ScanNPlan node has been initialized");

     ros::spin();
   }
   ```

1. C++ クラス "ScanNPlan" に
   myworkcell_node のほとんどの機能を記述します．

   空のコンストラクタと
   内部/プライベート変数による
   プライベート空間を有する
   このクラスのスケルトン構造を作成します．

   ``` c++
   class ScanNPlan
   {
   public:
     ScanNPlan(ros::NodeHandle& nh)
     {

     }

   private:
     // Planning components

   };
   ```

1. 新しい ScanNPlan クラス内で
   クラスのプライベートメンバー変数として
   ROS ServiceClient を定義します．

   前に定義したのと同じサービス名（"localize_part"）を使用して
   ScanNPlan コンストラクタ内で
   ServiceClient を初期化します．

   ScanNPlan クラス内に引数なしで
   `start` という名前の void 関数を作成します．
   これにはほとんどの本アプリケーションの
   ワークフローを含むようになります．

   現時点ではこの関数は
   `LocalizePart`サービスを呼び出し，
   応答を出力します．

   ``` c++
   class ScanNPlan
   {
   public:
     ScanNPlan(ros::NodeHandle& nh)
     {
       vision_client_ = nh.serviceClient<myworkcell_core::LocalizePart>("localize_part");
     }

     void start()
     {
       ROS_INFO("Attempting to localize part");
       // Localize the part
       myworkcell_core::LocalizePart srv;
       if (!vision_client_.call(srv))
       {
         ROS_ERROR("Could not localize part");
         return;
       }
       ROS_INFO_STREAM("part localized: " << srv.response);
     }

   private:
     // Planning components
     ros::ServiceClient vision_client_;
   };
   ```

1. `myworkcell_node` の
   `main` 関数に戻り，
   `ScanNPlan` クラスのオブジェクトをインスタンス化し，
   オブジェクトの` start` 関数を呼び出します．

   ``` c++
   ScanNPlan app(nh);

   ros::Duration(.5).sleep();  // wait for the class to initialize
   app.start();
   ```
1. パッケージの `CMakeLists.txt` を編集して
   関連する依存関係を持つ
   新しい（実行可能な）ノードを構築します．

   適切なセクションに `vision_node` のマッチングルールの直下に
   次のルールを追加してください．

   ``` cmake
   add_executable(myworkcell_node src/myworkcell_node.cpp)

   add_dependencies(myworkcell_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

   target_link_libraries(myworkcell_node ${catkin_LIBRARIES})
   ```

5. ノードをビルドして
   コンパイル時エラーが出ないことを確認してください．

   ``` bash
   catkin build
   ```

   _注: Qt を用いても良いです．_



### サービスの利用

1. 次の各コマンドを
   各々のターミナルに入力してください．

   ```
   roscore
   rosrun fake_ar_publisher fake_ar_publisher_node
   rosrun myworkcell_core vision_node
   rosrun myworkcell_core myworkcell_node
   ```
