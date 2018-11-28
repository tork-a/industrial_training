# Introduction to URDF
>In this exercise, we will explore how to describe a robot in the URDF format.

# URDF 入門

> ロボットを URDF 形式で記述する方法を見ていきます．


## Motivation
Many of the coolest and most useful capabilities of ROS and its community involve things like collision checking and dynamic path planning. It’s frequently useful to have a code-independent, human-readable way to describe the geometry of robots and their cells. Think of it like a textual CAD description: “part-one is 1 meter left of part-two and has the following triangle-mesh for display purposes.”
The Unified Robot Description Format (URDF) is the most popular of these formats today. This module will walk you through creating a simple robot cell that we’ll expand upon and use for practical purposes later.


## モチベーション

ROS とその協同ソフトウェアの最もクールで便利な機能の多くには
干渉チェックや動作軌道計画などが含まれています．
コードに依存しなく人間が判読可能な方法で
ロボットと作業セルのジオメトリを記述することは便利なことが多いです．
これはテキストによる CAD の記述のように考えてください．

「部品1は部品2から1メートル左にあり，表示用ポリゴンメッシュがあります」

Unified Robot Description Format（URDF）は，
今日，これらのフォーマットの中で最も一般的です．
本演習では簡単なロボットセルを作成していきます．
このロボットセルは後で実際に使用するために発展させていきます．


## Reference Example

[Building a Visual Robot Model with URDF from Scratch](http://wiki.ros.org/urdf/Tutorials/Building%20a%20Visual%20Robot%20Model%20with%20URDF%20from%20Scratch)


## リファレンス

* [Building a Visual Robot Model with URDF from Scratch](http://wiki.ros.org/urdf/Tutorials/Building%20a%20Visual%20Robot%20Model%20with%20URDF%20from%20Scratch)


## Further Information and Resources

* [XML Specification](http://wiki.ros.org/urdf/XML)
* [ROS Tutorials](http://wiki.ros.org/urdf/Tutorials)
* [XACRO Extensions](http://wiki.ros.org/xacro)
* [SolidWorks to URDF Exporter](http://wiki.ros.org/sw_urdf_exporter)


## 追加情報とリソース

* [XML Specification](http://wiki.ros.org/urdf/XML)
* [ROS Tutorials](http://wiki.ros.org/urdf/Tutorials)
* [XACRO Extensions](http://wiki.ros.org/xacro)
* [SolidWorks to URDF Exporter](http://wiki.ros.org/sw_urdf_exporter)


## Scan-N-Plan Application: Problem Statement
We have the software skeleton of our Scan-N-Plan application, so let’s take the next step and add some physical context. The geometry we describe in this exercise will be used to:
1. Perform collision checking
1. Understand robot kinematics
1. Perform transformation math
Your goal is to describe a workcell that features:
1. An origin frame called `world`
1. A separate frame with “table” geometry (a flat rectangular prism)
1. A frame (geometry optional) called `camera_frame` that is oriented such that its Z axis is flipped relative to the Z axis of `world`


## Scan-N-Plan アプリケーション: 演習問題

Scan-N-Plan アプリケーションのソフトウェアスケルトンが既にあるので，
次のステップを踏んで，
そこにいくつかの物理的なコンテキストを追加していきましょう．

本演習で説明するジオメトリは次の目的に利用されます．

1. 干渉チェックの実施
1. ロボットの運動学的構成の把握
1. 座標変換の計算

目標は次の特徴を持つワークセルを記述することです．

1. `world` という名前の全ての基となるフレーム
1. `table` ジオメトリ（平坦な直方体）を有する独立したフレーム
1. `camera_frame` という名前の Z 軸が `world` の Z 軸に対して
   反対方向に付けられたフレーム（ジオメトリは任意）


## Scan-N-Plan Application: Guidance

1. It’s customary to put describing files that aren’t code into their own “support” package. URDFs typically go into their own subfolder ''urdf/''. See the [abb_irb2400_support](https://github.com/ros-industrial/abb/tree/kinetic/abb_irb2400_support) package. Add a `urdf` sub-folder to your application support package.
1. Create a new `workcell.urdf` file inside the `myworkcell_support/urdf/` folder and insert the following XML skeleton:

   ``` xml
   <?xml version="1.0" ?>
   <robot name="myworkcell" xmlns:xacro="http://ros.org/wiki/xacro">
   </robot>
   ```

1. Add the required links. See the [irb2400_macro.xacro](https://github.com/ros-industrial/abb/blob/84825661073a18e33b68bb01b5bf371edd2efd49/abb_irb2400_support/urdf/irb2400_macro.xacro#L54-L69) example from an ABB2400.  Remember that all URDF tags must be placed **between** the `<robot> ... </robot>` tags.

   1. Add the `world` frame as a "virtual link" (no geometry).

      ```
      <link name="world"/>
      ```

   1. Add the `table` frame, and be sure to specify both collision & visual geometry tags. See the `box` type in the XML specification.

      ``` xml
      <link name="table">
        <visual>
          <geometry>
            <box size="1.0 1.0 0.05"/>
          </geometry>
        </visual>
        <collision>
          <geometry>
            <box size="1.0 1.0 0.05"/>
          </geometry>
        </collision>
      </link>
      ```

   1. Add the `camera_frame` frame as another virtual link (no geometry).

      ```
      <link name="camera_frame"/>
      ```

   1. Connect your links with a pair of fixed joints  Use an `rpy` tag in the `world_to_camera` joint to set its orientation as described in the introduction.

      ``` xml
      <joint name="world_to_table" type="fixed">
        <parent link="world"/>
        <child link="table"/>
        <origin xyz="0 0 0.5" rpy="0 0 0"/>
      </joint>

      <joint name="world_to_camera" type="fixed">
        <parent link="world"/>
        <child link="camera_frame"/>
        <origin xyz="-0.25 -0.5 1.25" rpy="0 3.14159 0"/>
      </joint>
      ```

   1. It helps to visualize your URDF as you add links, to verify things look as expected:

      ```
      roslaunch urdf_tutorial display.launch model:=<RELATIVE_PATH_TO_URDF>
      ```

  _If nothing shows up in Rviz, you may need to change the base frame in RVIZ (left panel at top) to the name of one of the links in your model._



## Scan-N-Plan アプリケーション: ガイダンス

1. コード以外の記述ファイルはそれぞれのサポート（"support"）パッケージ内に置くのが通例です．
   URDF は通常，サポートパッケージのサブフォルダ "urdf/" に入れるようにしています．
   [abb_irb2400_support](https://github.com/ros-industrial/abb/tree/kinetic/abb_irb2400_support)
   パッケージを参考にしてください．
   アプリケーションサポートパッケージに `urdf` サブフォルダを追加してください。

1. `myworkcell_support/urdf/` フォルダ内に
   新しい `workcell.urdf` ファイルを作成し，
   次の XML スケルトンを挿入します．

   ``` xml
   <?xml version="1.0" ?>
   <robot name="myworkcell" xmlns:xacro="http://ros.org/wiki/xacro">
   </robot>
   ```

1. 必要なリンクを追加します．

   ABB2400 の [irb2400_macro.xacro](https://github.com/ros-industrial/abb/blob/84825661073a18e33b68bb01b5bf371edd2efd49/abb_irb2400_support/urdf/irb2400_macro.xacro#L54-L69)
   の例を参考にしてください．

   _すべての URDF タグは `<robot> ... </robot>` の
    **タグの間に** 置かなければならないことを留意してください．_

   1. `world` フレームを「仮想リンク」として追加してください．（ジオメトリなし）

      ```
      <link name="world"/>
      ```

   1. `table`フレームを追加して，
      `<collision>` と `<visual>` の両方の `<geometry>` タグを指定してください．

      XML で記述された `box` 型です．

      ``` xml
      <link name="table">
        <visual>
          <geometry>
            <box size="1.0 1.0 0.05"/>
          </geometry>
        </visual>
        <collision>
          <geometry>
            <box size="1.0 1.0 0.05"/>
          </geometry>
        </collision>
      </link>
      ```

   1. 仮想リンクとして `camera_frame` を追加してください．（ジオメトリなし）

      ```
      <link name="camera_frame"/>
      ```

   1. リンクを固定関節のペアで接続します．
      `world_to_camera` 関節を `rpy` タグを使用して
      本演習問題の説明に従って向きを設定します．

      ``` xml
      <joint name="world_to_table" type="fixed">
        <parent link="world"/>
        <child link="table"/>
        <origin xyz="0 0 0.5" rpy="0 0 0"/>
      </joint>

      <joint name="world_to_camera" type="fixed">
        <parent link="world"/>
        <child link="camera_frame"/>
        <origin xyz="-0.25 -0.5 1.25" rpy="0 3.14159 0"/>
      </joint>
      ```

   1. 追加したリンクが意図したとおりになっているか
      URDF を視覚化して確認することができます.

      ```
      roslaunch urdf_tutorial display.launch model:=<RELATIVE_PATH_TO_URDF>
      ```

      _RViz に何も表示されない場合は RViz のベースフレーム（上部の左パネル）をモデル内にあるリンクの名前に変更する必要があります．_
