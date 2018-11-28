# Coordinate Tranforms using TF
>In this exercise, we will explore the terminal and C++ commands used with TF, the transform library.


# TF による座標変換

> 座標変換ライブラリである TF で使用されるターミナルコマンドと C++ コマンドについて見ていきます．


## Motivation
It’s hard to imagine a useful, physical “robot” that doesn’t move itself or watch something else move. A useful application in ROS will inevitably have some component that needs to monitor the position of a part, robot link, or tool. In ROS, the “eco-system” and library that facilitates this is called TF.
TF is a fundamental tool that allows for the lookup the transformation between any connected frames, even back through time. It allows you to ask questions like: “What was the transform between A and B 10 seconds ago.” That’s useful stuff.


## モチベーション

実際に自身が動いたり何か他の動きを見たりしなくても便利な
「ロボット」というのはイメージし難いです．
有用な ROS におけるアプリケーションというのは必然的に
部品やロボットリンクまたはツールの位置を観察する必要のあるコンポーネントを有します．
ROS ではこれを行いやすくする「エコシステム」とライブラリを TF と呼んでいます．

TF は接続されたフレーム間の変換を
時間を遡っても調べることを可能にする基本的なツールです．
例えば「10秒前における A と B の間の変換は何ですか？」
という質問をすることも可能にする便利な道具です．


## Reference Example
[ROS TF Listener Tutorial](http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20listener%20(C%2B%2B))


## リファレンス

* [Writing a tf listener (C++)](http://wiki.ros.org/action/fullsearch/tf/Tutorials/Writing%20a%20tf%20listener%20%28C%2B%2B%29?action=fullsearch&context=180&value=linkto%3A%22tf%2FTutorials%2FWriting+a+tf+listener+%28C%2B%2B%29%22)


## Further Information and Resources
 * [Wiki Documentation](http://wiki.ros.org/tf)
 * [TF Tutorials](http://wiki.ros.org/tf/Tutorials)
 * [TF Listener API](http://docs.ros.org/kinetic/api/tf/html/)


## 追加情報とリソース

 * [tf - ROS Wiki](http://wiki.ros.org/tf)
 * [TF Tutorials](http://wiki.ros.org/tf/Tutorials)
 * [TF Listener API](http://docs.ros.org/kinetic/api/tf/html/)


 * [ja/ tf/ - ROS Wiki](http://wiki.ros.org/ja/tf)
 * [ja/ tf/ Tutorials](http://wiki.ros.org/ja/tf/Tutorials)


## Scan-N-Plan Application: Problem Statement
The part pose information returned by our (simulated) camera is given in the optical reference frame of the camera itself. For the robot to do something with this data, we need to transform the data into the robot’s reference frame.

Specifically, edit the service callback inside the vision_node to transform the last known part pose from `camera_frame` to the service call’s `base_frame` request field.


## Scan-N-Plan アプリケーション: 演習問題

（シミュレートされた）カメラにより取得された部品の姿勢情報は
カメラ自体の光学基準フレームとして得られます．
ロボットがこのデータを基に何かを行うためには
データをロボットの参照フレームに変換する必要があります．

具体的には vision_node 内のサービスコールバックを編集して，
部品姿勢の最新情報を `camera_frame` から
サービスコールの `base_frame` リクエストフィールドに変換します．


## Scan-N-Plan Application: Guidance

 1. Specify `tf` as a dependency of your core package.

    * Edit `package.xml` (1 line) and `CMakeLists.txt` (2 lines) as in previous exercises

 1. Add a `tf::TransformListener` object to the vision node (as a class member variable).

    ``` c++
    #include <tf/transform_listener.h>
    ...
    tf::TransformListener listener_;
    ```

 1. Add code to the existing `localizePart` method to convert the reported target pose from its reference frame ("camera_frame") to the service-request frame:

    1. For better or worse, ROS uses lots of different math libraries. You’ll need to transform the over-the-wire format of `geometry_msgs::Pose` into a `tf::Transform object`:

       ``` c++
       tf::Transform cam_to_target;
       tf::poseMsgToTF(p->pose.pose, cam_to_target);
       ```

    1. Use the listener object to lookup the latest transform between the `request.base_frame` and the reference frame from the `ARMarker` message (which should be "camera_frame"):

       ``` c++
       tf::StampedTransform req_to_cam;
       listener_.lookupTransform(req.base_frame, p->header.frame_id, ros::Time(0), req_to_cam);
       ```

    1. Using the above information, transform the object pose into the target frame.

       ``` c++
       tf::Transform req_to_target;
       req_to_target = req_to_cam * cam_to_target;
       ```

    1. Return the transformed pose in the service response.

       ``` c++
       tf::poseTFToMsg(req_to_target, res.pose);
       ```

 1. Run the nodes to test the transforms:

    ```
    catkin build
    roslaunch myworkcell_support urdf.launch
    roslaunch myworkcell_support workcell.launch
    ```

 1. Change the "base_frame" parameter in `workcell.launch` (e.g. to "table"), relaunch the `workcell.launch` file, and note the different pose result.  Change the "base_frame" parameter back to "world" when you're done.


## Scan-N-Plan アプリケーション: ガイダンス

 1. コアパッケージの依存関係に `tf` を指定してください．

     * 前の演習のように `package.xml`（1行）と `CMakeLists.txt`（2行）を編集します．

 1. vision node にクラスメンバ変数として
    `tf::TransformListener` オブジェクトを追加します．

    ``` c++
    #include <tf/transform_listener.h>
    ...
    tf::TransformListener listener_;
    ```

 1. 取得された目標物の姿勢を参照したフレーム（"camera_frame"）から
    サービス・リクエスト・フレームに変換するために，
    既存の `localizePart` メソッドにコードを追加します．

    1. 良かれ悪しかれ ROS はさまざまな数学ライブラリを使用します．
       `geometry_msgs::Pose` の over-the-wire 形式を
       `tf::Transform` オブジェクトに変換する必要があります．

       ``` c++
       tf::Transform cam_to_target;
       tf::poseMsgToTF(p->pose.pose, cam_to_target);
       ```

    1. リスナ・オブジェクトを用いて `request.base_frame` と
       `ARMarker` メッセージ（これは "camera_frame" 上にあるはずです）
       からの参照フレームの間の最新の変換を調べます．

       ``` c++
       tf::StampedTransform req_to_cam;
       listener_.lookupTransform(req.base_frame, p->header.frame_id, ros::Time(0), req_to_cam);
       ```

    1. 上で得られた情報を用いて対象物の姿勢を目標フレームに変換します．

       ``` c++
       tf::Transform req_to_target;
       req_to_target = req_to_cam * cam_to_target;
       ```

    1. サービス応答で変換された姿勢を返します．

       ``` c++
       tf::poseTFToMsg(req_to_target, res.pose);
       ```

 1. ノードを実行して変換をテストします．

    ```
    catkin build
    roslaunch myworkcell_support urdf.launch
    roslaunch myworkcell_support workcell.launch
    ```

 1. `workcell.launch` の `base_frame` パラメータを
    例えば "table" などに変更して，
    `workcell.launch` ファイルを再起動し，
    異なるポーズの結果を見てみてください．

    完了したら "base_frame" パラメータを "world" に戻してください．
