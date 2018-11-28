# Launch Files
>In this exercise, we will explore starting groups of nodes at once with launch files.

# Launch ファイル

> 本演習では起動（ launch ）ファイルを使用してノードのグループを一度に起動する方法を見てゆきます．

## Motivation
The ROS architecture encourages engineers to use ''nodes'' as a fundamental unit of organization in their systems, and applications can quickly grow to require many nodes to operate. Opening a new terminal and running each node individually quickly becomes unfeasible. It'd be nice to have a tool to bring up groups of nodes at once. ROS ''launch'' files are one such tool. It even handles bringing ```roscore``` up and down for you.

## モチベーション

ROS アーキテクチャはシステム内の組織の基本単位として「ノード」を使用することを奨励しています．
そのため多くのノードを動作させる必要が出てきますのでアプリケーションが急増します．
いちいち新しいターミナルを開いて各ノードを個別に実行することは事実上困難になります．<br>
こうなるとノードのグループを一度に起動するツールがあると助かります．
ROS の "launch" ファイルはそのようなツールの1つです．
"launch" ファイルは `roscore` の起動・停止さえ扱ってくれます．


## Reference Example

[Roslaunch Examples](http://wiki.ros.org/roslaunch/XML#Example_.launch_XML_Config_Files)


## リファレンス

* [Example .launch XML Config Files](http://wiki.ros.org/roslaunch/XML#Example_.launch_XML_Config_Files)


## Further Information and Resources

[Roslaunch XML Specification](http://wiki.ros.org/roslaunch/XML)

[Debugging and Launch Files](http://wiki.ros.org/roslaunch/Tutorials/Roslaunch%20Nodes%20in%20Valgrind%20or%20GDB)


## 追加情報とリソース

* [Roslaunch XML Specification](http://wiki.ros.org/roslaunch/XML)
* [Debugging and Launch Files](http://wiki.ros.org/roslaunch/Tutorials/Roslaunch%20Nodes%20in%20Valgrind%20or%20GDB)


* [ja/roslaunch/XML](http://wiki.ros.org/ja/roslaunch/XML)
* [Valgrind か GDBでnodeをroslaunchする方法](http://wiki.ros.org/ja/roslaunch/Tutorials/Roslaunch%20Nodes%20in%20Valgrind%20or%20GDB)

## Scan-N-Plan Application: Problem Statement
In this exercise, you will:
1. Create a new package, `myworkcell_support`.
1. Create a directory in this package called `launch`.
1. Create a file inside this directory called `workcell.launch` that:
   1. Launches `fake_ar_publisher`
   1. Launches `vision_node`

You may also choose to launch `myworkcell_core` node with the others or keep it separate.  We often configure systems with two main launch files.  In this example, `fake_ar_publisher` and `vision_node` are "environment nodes", while `myworkcell_node` is an "application" node.

1. "Environment" Launch File - driver/planning nodes, config data, etc.
1. "Application" Launch File - executes a sequence of actions for a particular application.


## Scan-N-Plan アプリケーション: 演習問題

本演習では次のことを行います．

1. パッケージ `myworkcell_support` を新しく作成する．
1. `launch` ディレクトリをこのパッケージ内に新しく作成する．
1. このディレクトリ内に下記機能を持つファイル `workcell.launch` を新しく作成する．
   1. `fake_ar_publisher` を起動する．
   1. `vision_node` を起動する．

また `myworkcell_core` ノードを他のものと一緒に起動するか
別々に起動するかも選択できるようにします．

一般的には2つの主要な launch ファイルでシステムを構成します．
この例では `fake_ar_publisher` と `vision_node` は「環境」ノードですが，
`myworkcell_node`は「アプリケーション」ノードです．

1. 「環境」launch ファイル: ドライバ／動作計画ノード，設定データなど
1. 「アプリケーション」launch ファイル: 特定のアプリケーションに対してシーケンス動作を実行


## Scan-N-Plan Application: Guidance

1. In your workspace, create the new package `myworkcell_support` with a dependency on `myworkcell_core`.  Rebuild and source the workspace so that ROS can find the new package:

   ``` bash
   cd ~/catkin_ws/src
   catkin create pkg myworkcell_support --catkin-deps myworkcell_core
   catkin build
   source ~/catkin_ws/devel/setup.bash
   ```

2. Create a directory for launch files (inside the new `myworkcell_support` package):

   ``` bash
   roscd myworkcell_support
   mkdir launch
   ```

3. Create a new file, `workcell.launch` (inside the `launch` directory) with the following XML skeleton:

   ``` xml
   <launch>

   </launch>
   ```

4. Insert lines to bring up the nodes outlined in the problem statement. See the reference documentation for more information:

   ``` xml
   <node name="fake_ar_publisher" pkg="fake_ar_publisher" type="fake_ar_publisher_node" />
   <node name="vision_node" pkg="myworkcell_core" type="vision_node" />
   ```
   * _Remember: All launch-file content must be **between** the `<launch> ... </launch>` tag pair._

5. Test the launch file:

   ``` bash
   roslaunch myworkcell_support workcell.launch
   ```

   _Note: roscore and both nodes were automatically started.  Press _Ctrl+C_ to close all nodes started by the launch file. If no nodes are left running, roscore is also stopped._

6. Notice that none of the usual messages were printed to the console window.  Launch files will suppress console output below the **ERROR** severity level by default. To restore normal text output, add the `output="screen"` attribute to each of the nodes in your launch files:

   ``` xml
   <node name="fake_ar_publisher" pkg="fake_ar_publisher" type="fake_ar_publisher_node" output="screen"/>
   <node name="vision_node" pkg="myworkcell_core" type="vision_node" output="screen" />

   ```


## Scan-N-Plan アプリケーション: ガイダンス

1. ワークスペース内に `myworkcell_core` に依存関係をもつ
   新しいパッケージ `myworkcell_support` を作成してください．

   ROS が新しいパッケージを見つけることができるように
   ワークスペースをビルドした後に設定を反映させます．

   ``` bash
   cd ~/catkin_ws/src
   catkin create pkg myworkcell_support --catkin-deps myworkcell_core
   catkin build
   source ~/catkin_ws/devel/setup.bash
   ```

1. launch ファイル用のディレクトリを作成します．（新しい `myworkcell_support` パッケージ内）

   ``` bash
   roscd myworkcell_support
   mkdir launch
   ```

3. 新しいファイル `workcell.launch`（ `launch`ディレクトリ内 ）を
   次の XML スケルトンで作成します．

   ``` xml
   <launch>

   </launch>
   ```

4. 演習問題に記載されているノードを起動するための行を追加します．

   詳細はリファレンスを見てください．

   ``` xml
   <node name="fake_ar_publisher" pkg="fake_ar_publisher" type="fake_ar_publisher_node" />
   <node name="vision_node" pkg="myworkcell_core" type="vision_node" />
   ```
   * _留意: すべての起動ファイルコンテンツは
      `<launch> ... </launch>` **タグのペアの間にある** 必要があります．_

5. launch ファイルをテストします．

   ``` bash
   roslaunch myworkcell_support workcell.launch
   ```

   _注: roscore と2つのノードは自動的に起動しました．
    Ctrl+C を押して launch ファイルによって開始されたすべてのノードを閉じます．
    実行中のノードがない場合 roscore も停止します．_

6. 通常のメッセージはコンソールウィンドウに表示されませんでした．
   launch ファイルはデフォルトでは重大度 **ERROR** 未満のコンソール出力が抑制されます．
   通常のテキスト出力を復元するには launch ファイルの各ノードに `output="screen"` 属性を追加します．

   ``` xml
   <node name="fake_ar_publisher" pkg="fake_ar_publisher" type="fake_ar_publisher_node" output="screen"/>
   <node name="vision_node" pkg="myworkcell_core" type="vision_node" output="screen" />
   ```
