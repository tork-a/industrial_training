# パラメータ

> ノードを設定するためのROSパラメータを見ていきます．


## モチベーション

本チュートリアル（またはあなたのキャリア）のこの時点で，
幾度となく `int main(int argc, char** argv)`
という文をタイプしたことでしょう．
`main` への引数は
スコープ外のシステムがそのプログラムを理解して
特定のタスクを行うようにプログラムを構成できる手段です．
これらは _コマンドライン・パラメータ_ です．

ROS エコシステムには
ノードのグループ全体を構成するための
類似のシステムがあります．
それは `roscore` の一部として起動される
Key-Value ストレージプログラムです．
設定パラメータをノードに個別に渡すの
（ノードの登録先を特定するなど）にも最適ですが，
はるかに複雑な項目にも利用できます．


## リファレンス

* [Understanding Parameters](http://wiki.ros.org/ROS/Tutorials/UnderstandingServicesParams#Using_rosparam)

* [rosparam を使う](http://wiki.ros.org/ja/ROS/Tutorials/UnderstandingServicesParams#rosparam_.2BMJJPfzBG-)


## 追加情報とリソース

* [Roscpp tutorial](http://wiki.ros.org/roscpp_tutorials/Tutorials/Parameters)
* [Private Parameters](http://wiki.ros.org/roscpp_tutorials/Tutorials/AccessingPrivateNamesWithNodeHandle)
* [Parameter Server](http://wiki.ros.org/Parameter%20Server)


* [ノードハンドルからプライベートネームで接続する](http://wiki.ros.org/ja/roscpp_tutorials/Tutorials/AccessingPrivateNamesWithNodeHandle)
* [ja/Parameter Server](http://wiki.ros.org/ja/Parameter%20Server)


## Scan-N-Plan アプリケーション: 演習問題

前回の演習で
次の定義を持つサービスを追加しました．

  ```
  # request
  string base_frame
  ---
  # response
  geometry_msgs/Pose pose
  ```

これまではリクエストフィールドにある
`base_frame` を全く使用していませんでした．
この演習では ROS パラメータを使用して
このフィールドを設定します．
次のことを行ってください．

1. 通常のものに加えて，
   プライベートノード・ハンドルを
   `myworkcell_node` の
   main メソッドに追加します．

1. プライベートノード・ハンドルを使用して，
   パラメータ `base_frame` を読み込み，
   それをローカルの文字列オブジェクトに格納します．

   * パラメータが指定されていない場合は
     デフォルトでパラメータ
     "world" に設定されるようにします．

1. `vision_node` へのサービスコールを行うときは，
   このパラメータを使用して
   `request::base_frame` フィールドに記入してください．

1. 新しい値を初期化するために
   launch ファイルに `<param>` タグを追加します．


## Scan-N-Plan アプリケーション: ガイダンス

1. `myworkcell_node.cpp` を開いて編集します．

1. 新しい `ros::NodeHandle` オブジェクト
   を `main` 関数に追加し，
   そのパラメータをプライベートにします．

   詳細については
   [ROS Wiki](http://wiki.ros.org/roscpp_tutorials/Tutorials/AccessingPrivateNamesWithNodeHandle)
   の関連する箇所を参照してください．

   ``` c++
   ros::NodeHandle private_node_handle ("~");
   ```

1. テンポラリの文字列オブジェクト
   `std::string base_frame;` を作成し，
   プライベートノードハンドルの
   [API]（http://docs.ros.org/indigo/api/roscpp/html/classros_1_1NodeHandle.html）
   を使用して，パラメータ `"base_frame"`
   を読み込みます．

   ``` c++
   private_node_handle.param<std::string>("base_frame", base_frame, "world"); // parameter name, string object reference, default value
   ```

   * _`base_frame` パラメータは
      `private_node_handle` が宣言された後，
      `app.start()` が呼び出される前
      に読み込まなければなりません．_

1. base_frame 引数を受け入れる
   `myworkcell_node` の
   `start` 関数にパラメータを追加し，
   パラメータからの値を
   サービスリクエストに割り当てます．

   `main()` ルーチンの
   `app.start` 呼び出しを更新して，
   パラメータサーバから読み込んだ
   `base_frame` の値を渡すようにしてください．

   ``` c++
   void start(const std::string& base_frame)
   {
     ...
     srv.request.base_frame = base_frame;
     ROS_INFO_STREAM("Requesting pose in base frame: " << base_frame);
     ...
   }

   int main(...)
   {
     ...
     app.start(base_frame);
     ...
   }
   ```
   * _`srv.request` はサービス呼び出しに渡す **前に** 設定する必要があります．（ `vision_client.call(srv)` ）_

1. これで，既存の `workcell.launch` ファイルに
   `myworkcell_node` を追加することで，
   `base_frame` パラメータを
   launch ファイルから設定できるようになります．
   動作計画（モーション・プランニング）のためには，
   `vision_node` がワールド・フレームに対するターゲットの位置
   を返すことが望ましいです．
   デフォルト値としてそうはなっているのですが，
   一応 launch ファイルで指定しておきます．

   ``` xml
   <node name="myworkcell_node" pkg="myworkcell_core" type="myworkcell_node" output="screen">
     <param name="base_frame" value="world"/>
   </node>
   ```

1. システムを実行して試してみてください．

   ```
   catkin build
   roslaunch myworkcell_support workcell.launch
   ```

    * 実行中のノードを停止するには
      _Ctrl+C_ を押してください．
    * launch ファイルを編集して，
      base_frame パラメータを変更してみてください．
      （ 例: "test2" ）
    * workcell.launch を再起動して，
      "request frame" が変わったことを見てみてください．
         - リクエスト・フレームを扱うために
           vision_node を（まだ）更新していないため，
           レスポンス・フレームは変更されません．
           vision_node は（今のところ）常に同じフレームを返します．
    * base_frame を
      "world" に戻してください．
