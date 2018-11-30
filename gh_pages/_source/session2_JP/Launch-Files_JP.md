# Launch ファイル

> 本演習では起動（ launch ）ファイルを使用してノードのグループを一度に起動する方法を見てゆきます．


## モチベーション

ROS アーキテクチャは
システム内の組織の基本単位として
「ノード」を使用することを奨励しています．
そのため多くのノードを動作させる必要が出てきますので
アプリケーションが急増します．
いちいち新しいターミナルを開いて
各ノードを個別に実行することは事実上困難になります．<br>
こうなるとノードのグループを一度に起動するツールがあると助かります．
ROS の "launch" ファイルは
そのようなツールの1つです．
"launch" ファイルは `roscore` の起動・停止さえ扱ってくれます．


## リファレンス

* [Example .launch XML Config Files](http://wiki.ros.org/roslaunch/XML#Example_.launch_XML_Config_Files)


## 追加情報とリソース

* [Roslaunch XML Specification](http://wiki.ros.org/roslaunch/XML)
* [Debugging and Launch Files](http://wiki.ros.org/roslaunch/Tutorials/Roslaunch%20Nodes%20in%20Valgrind%20or%20GDB)


* [ja/roslaunch/XML](http://wiki.ros.org/ja/roslaunch/XML)
* [Valgrind か GDBでnodeをroslaunchする方法](http://wiki.ros.org/ja/roslaunch/Tutorials/Roslaunch%20Nodes%20in%20Valgrind%20or%20GDB)


## Scan-N-Plan アプリケーション: 演習問題

本演習では次のことを行います．

1. パッケージ `myworkcell_support` を新しく作成する．
1. `launch` ディレクトリを
   このパッケージ内に新しく作成する．
1. このディレクトリ内に
   下記機能を持つファイル
   `workcell.launch` を新しく作成する．
   1. `fake_ar_publisher` を起動する．
   1. `vision_node` を起動する．

また `myworkcell_core` ノードを
他のものと一緒に起動するか
別々に起動するか
も選択できるようにします．

一般的には
2つの主要な launch ファイル
でシステムを構成します．
この例では `fake_ar_publisher` と
`vision_node` は「環境」ノードですが，
`myworkcell_node`は「アプリケーション」ノードです．

1. 「環境」launch ファイル:
   ドライバ／動作計画ノード，設定データなど
1. 「アプリケーション」launch ファイル:
   特定のアプリケーションに対してシーケンス動作を実行


## Scan-N-Plan アプリケーション: ガイダンス

1. ワークスペース内に
   `myworkcell_core` に依存関係をもつ
   新しいパッケージ `myworkcell_support`
   を作成してください．

   ROS が新しいパッケージを見つけることができるように
   ワークスペースをビルドした後に設定を反映させます．

   ``` bash
   cd ~/catkin_ws/src
   catkin create pkg myworkcell_support --catkin-deps myworkcell_core
   catkin build
   source ~/catkin_ws/devel/setup.bash
   ```

1. launch ファイル用のディレクトリを作成します．
   （新しい `myworkcell_support` パッケージ内）

   ``` bash
   roscd myworkcell_support
   mkdir launch
   ```

1. 新しいファイル `workcell.launch`
   （ `launch`ディレクトリ内 ）を
   次の XML スケルトンで作成します．

   ``` xml
   <launch>

   </launch>
   ```

1. 演習問題に記載されている
   ノードを起動するための行を追加します．

   詳細はリファレンスを見てください．

   ``` xml
   <node name="fake_ar_publisher" pkg="fake_ar_publisher" type="fake_ar_publisher_node" />
   <node name="vision_node" pkg="myworkcell_core" type="vision_node" />
   ```

   * _留意: すべての起動ファイルコンテンツは
      `<launch> ... </launch>`
      **タグのペアの間にある** 必要があります．_

1. launch ファイルをテストします．

   ``` bash
   roslaunch myworkcell_support workcell.launch
   ```

   _注: roscore と2つのノードは自動的に起動しました．
    Ctrl+C を押して
    launch ファイルによって開始された全てのノードを閉じます．
    実行中のノードがない場合 roscore も停止します．_

1. 通常のメッセージは
   コンソールウィンドウに表示されませんでした．
   launch ファイルはデフォルトでは
   重大度 **ERROR** 未満のコンソール出力が抑制されます．
   通常のテキスト出力を復元するには
   launch ファイルの各ノードに
   `output="screen"` 属性を追加します．

   ``` xml
   <node name="fake_ar_publisher" pkg="fake_ar_publisher" type="fake_ar_publisher_node" output="screen"/>
   <node name="vision_node" pkg="myworkcell_core" type="vision_node" output="screen" />
   ```
