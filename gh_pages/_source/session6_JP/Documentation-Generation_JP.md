# ドキュメント生成

## モチベーション

ROS Scan-N-Plan アプリケーションを完成させテストを行いました．
他の開発者がこのプログラムを簡単に理解できるように，
そのコードを完全に文書化することが重要です．


## 情報とリソース

* [Doxygen - Generate documentation from source code](http://www.doxygen.org/)
* [rosdoc_lite is a ROS wrapper for doxygen](http://wiki.ros.org/rosdoc_lite)


## Scan-N-Plan アプリケーション: 演習問題

これまでに Scan-N-Plan プログラムを完成させテストを行いました．
そして，そのコードを公開リリースする必要があります．
目標はブラウザでドキュメントを閲覧可能にすることです．
これは、myworkcell_core パッケージに doxygen 構文で注釈を付けて，
rosdoc_lite でドキュメントを生成することで実現可能です．


## Scan-N-Plan アプリケーション: ガイダンス

### ソースコードに注釈を付ける

 1. 作成した myworkcell_node.cpp ファイルを開いてください．

 1. ScanNPlan クラスの上に注釈を入れます．

 ``` c++
 /**
  * @brief The ScanNPlan class is a client of the vision and path plan servers.  The ScanNPLan class takes
  * these services, computes transforms and published commands to the robot.
  */
 class ScanNPlan
 ```

 1. start メソッドの上に注釈を入れます．

 ``` c++
 /**
  * @brief start performs the robot alorithms functions of the ScanNPlan of
  * the node. The start method makes a service request for a transform that
  * localizes the part.  The start method moves the "manipulator"
  * move group to the localization target.  The start method requests
  * a cartesian path based on the localization target.  The start method
  * sends the cartesian path to the actionlib client for execution, bypassig
  * MoveIt!
  * @param base_frame is a string that specifies the reference frame
  * coordinate system.
  */
 void start()
 ```

 1. flipPose の上に注釈を入れます．

 ``` c++
 /**
  * @brief flipPose rotates the input transform by 180 degrees about the
  * x-axis
  * @param in geometry_msgs::Pose reference to the input transform
  * @return geometry_msgs::Pose of the flipped output transform
  */
 geometry_msgs::Pose transformPose(const geometry_msgs::Pose& in) const
 ```

 1. main 関数の上に注釈を入れます．

 ``` c++
 /**
  * @brief main is the ros interface for the ScanNPlan Class
  * @param argc ROS uses this to parse remapping arguments from the command line.
  * @param argv ROS uses this to parse remapping arguments from the command line.
  * @return ROS provides typical return codes, 0 or -1, depending on the
  * execution.
  */
 int main(int argc, char** argv)
 ```

 1. 注釈を追加するときは
    プライベート変数や，その他重要なコード要素の上に置くと良いでしょう．


### ドキュメントの生成

 1. rosdoc_lite をインストールしてください．

 ```
 sudo apt install ros-kinetic-rosdoc-lite
 ```

 2. パッケージをビルドしてください．

 ```
 catkin build
 ```

 3. パッケージをソース（ source ）してください．

 ```
 source ./devel/setup.bash
 ```

 4. rosdoc_lite を実行してドキュメントを生成します．

 ```
 roscd myworkcell_core
 rosdoc_lite .
 ```


### ドキュメントの閲覧

 1. ブラウザからドキュメントを開きます．

 ```
 firefox doc/html/index.html
 ```

 1. Classes -> ScanNPlan でドキュメントを表示させてください．
