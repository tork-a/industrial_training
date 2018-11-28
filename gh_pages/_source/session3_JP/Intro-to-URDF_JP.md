# URDF 入門

> ロボットを URDF 形式で記述する方法を見ていきます．


## モチベーション

ROS とその協同ソフトウェアの
最もクールで便利な機能の多くには
干渉チェックや動作軌道計画などが含まれています．
コードに依存しなく人間が判読可能な方法で
ロボットと作業セルのジオメトリを記述することは便利なことが多いです．
これはテキストによる CAD の記述のように考えてください．

「部品1は部品2から1メートル左にあり，表示用ポリゴンメッシュがあります」

Unified Robot Description Format（URDF）は，
今日，これらのフォーマットの中で最も一般的です．
本演習では簡単なロボットセルを作成していきます．
このロボットセルは後で実際に使用するために発展させていきます．


## リファレンス

* [Building a Visual Robot Model with URDF from Scratch](http://wiki.ros.org/urdf/Tutorials/Building%20a%20Visual%20Robot%20Model%20with%20URDF%20from%20Scratch)


## 追加情報とリソース

* [XML Specification](http://wiki.ros.org/urdf/XML)
* [ROS Tutorials](http://wiki.ros.org/urdf/Tutorials)
* [XACRO Extensions](http://wiki.ros.org/xacro)
* [SolidWorks to URDF Exporter](http://wiki.ros.org/sw_urdf_exporter)


## Scan-N-Plan アプリケーション: 演習問題

Scan-N-Plan アプリケーションの
ソフトウェアスケルトンが既にあるので，
次のステップを踏んで，
そこにいくつかの物理的なコンテキスト
を追加していきましょう．

本演習で説明するジオメトリは次の目的に利用されます．

1. 干渉チェックの実施
1. ロボットの運動学的構成の把握
1. 座標変換の計算

目標は次の特徴を持つワークセルを記述することです．

1. `world` という名前の
   全ての基となるフレーム
1. `table` ジオメトリ（平坦な直方体）を有する
   独立したフレーム
1. `camera_frame` という名前の
   Z 軸が `world` の Z 軸に対して反対方向に
   付けられたフレーム（ジオメトリは任意）


## Scan-N-Plan アプリケーション: ガイダンス

1. コード以外の設定ファイルは
   それぞれのサポート（"support"）パッケージ内
   に置くのが通例です．
   URDF は通常，サポートパッケージの
   サブフォルダ "urdf/" に入れるようにしています．

   [abb_irb2400_support](https://github.com/ros-industrial/abb/tree/kinetic/abb_irb2400_support)
   パッケージを参考にしてください．

   アプリケーションサポートパッケージに
   `urdf` サブフォルダを追加してください。

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

   1. `world` フレームを「仮想リンク」として追加してください．
      （ジオメトリなし）

      ```
      <link name="world"/>
      ```

   1. `table`フレームを追加して，
      `<collision>` と `<visual>` の両方の
      `<geometry>` タグを指定してください．

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

   1. 仮想リンクとして
      `camera_frame` を追加してください．（ジオメトリなし）

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

      _RViz に何も表示されない場合は
       RViz のベースフレーム（上部の左パネル）を
       モデル内にあるリンクの名前に変更する必要があります．_
