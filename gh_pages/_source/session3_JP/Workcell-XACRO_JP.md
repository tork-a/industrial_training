# 作業セル XACRO

> 単純なロボット作業セルを表す XACRO ファイルを作成します．
  これにより URDF と XACRO の両方の要素を見ることができます．

## モチベーション

要素がほんの数個よりも多い URDF を書いていると，
すぐにそれが苦痛になってくるかと思います．
ワークスペースのリンクやジョイントの束をコピーして貼り付け，
その名前をわずかに変更して複製したアイテムやファイルは膨大になってしまいます．
起動時に把握される（またはされない）間違いが簡単に発生してしまいます．

コンポーネントを一度定義すれば複製無しで
どこででも再利用可能といったような，
プログラミング言語で行っているような指示方法があれば助かります．
プログラムでは関数とクラスがそれを行い，
XACRO マクロは URDF のためにそれを行います．
XACRO には他にも
（#include 的な）ファイル・インクルード・システム，
定数変数，
数式表現評価（ 例えば 1.57 ではなく PI/2.0 ）
などの優れた機能もあります．


## リファレンス

* [Using Xacro to Clean Up a URDF File](http://wiki.ros.org/urdf/Tutorials/Using%20Xacro%20to%20Clean%20Up%20a%20URDF%20File)


## 追加情報とリソース

* [Xacro Extension Documentation](http://wiki.ros.org/xacro)
* [Creating a URDF for an Industrial Robot](http://wiki.ros.org/Industrial/Tutorials/Create%20a%20URDF%20for%20an%20Industrial%20Robot)


## Scan-N-Plan アプリケーション: 演習問題

前回の演習では静的ジオメトリのみで構成される
作業セル（workcell）を作成しました．
今回は XACRO ツールを使用して
UR5 ロボット _assembly_ を追加します．

具体的には次のように行っていきます．

  1. 前回作成した `*.urdf` ファイルを
     `xacro` 拡張子を持つ XACRO ファイルに変換します．
  1. `UR5` の XACRO マクロ定義を含むファイルを
     インクルードします．
  1. 作業スペースで `UR5` をインスタンス化し，
     それを _table_ リンクに接続します．


## Scan-N-Plan アプリケーション: ガイダンス

 1. 前回の演習の `workcell.urdf` のファイル名を
    `workcell.xacro` に変更してください．

 1. `ur_description` パッケージを
    ROS 環境に導入してください．

    いくつかの方法があります．

    1. debian パッケージをインストールする．

       ```
       sudo apt install ros-kinetic-ur-description ros-kinetic-ur-kinematics
       ```

    1. [GitHub](https://github.com/ros-industrial/universal_robot)
       から catkin ワークスペースにクローンする．

       ```
       cd ~/catkin_ws/src
       git clone https://github.com/ros-industrial/universal_robot.git
       catkin build
       source ~/catkin_ws/devel/setup.bash
       ```

    > description パッケージが各 "module"，"part" または "assembly" を
      それ自身のファイルに入れることはよくあります．
      多くの場合，パッケージは特定の部品を持つ
      全体のセルを定義する以外のファイルも定義していて，
      それらを簡単に視覚的に調べることができます．
      UR パッケージは UR5（
      [ur5_robot.urdf.xacro](https://github.com/ros-industrial/universal_robot/blob/indigo-devel/ur_description/urdf/ur5_robot.urdf.xacro)
      ）のファイルを定義します．
      これは本演習にとって非常に良い例となっています．

 1. UR5 マクロを実装している xacro ファイルを探し，
    新しく名前を変更した `workcell.xacro` ファイルに組み込みます．

    `workcell.xacro` ファイル最上部の
    `<robot>` タグの下に
    次のインクルードする行を追加してください．

    ``` xml
    <xacro:include filename="$(find ur_description)/urdf/ur5.urdf.xacro" />
    ```

    > UR5 定義ファイルや Xacro マクロを定義している他のファイルを読んでいると，
      要素名に `${prefix}` をたくさん使っていることが見て取れると思います．
      Xacro は実行時に "${}" 内のものを評価します．
      基本的な数式を扱うことができ，
      プロパティ（ala-global変数）やマクロパラメータを使って変数を参照することができます．
      ほとんどのマクロは"prefix" パラメータを取得して，
      ユーザが前述のマクロの複数のインスタンスを作成できるようにしています．
      これは最終的な URDF 要素名を唯一なものにするためのメカニズムですが，
      そうならなかった場合は重複するリンク名を取得し URDF が障害を報告をします．

 1. `ur5.urdf.xacro` ファイルをインクルードしても
    URDF モデルに実際に UR5 ロボットが作成されるわけではありません．
    マクロは定義されましたが，
    マクロを呼び出してロボットのリンクとジョイントを作成する必要があります．

    _前述のように `prefix` タグの使用に注意してください．_

    ``` xml
    <xacro:ur5_robot prefix="" joint_limited="true"/>
    ```

    > Xacro のマクロはコピー＆ペーストに関する素晴らしいラッパーです．
      マクロを作ってもそれはリンクと関節の塊です．
      世界の他の部分をそのマクロの出力に関連付けなければなりません．
      マクロを見て，ベースリンクが何で，エンドリンクが何かを確認する必要があります．
      願わくば，マクロは ROS Industrial のそれのような標準に準じた
      ベースリンクは "base_link" と最後のリンクは "tool0" と名づけられると良いでしょう．

 1. UR5の `base_link` を
    既存の静的ジオメトリに固定リンクに接続します．

    ``` xml
    <joint name="table_to_robot" type="fixed">
      <parent link="table"/>
      <child link="base_link"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
    </joint>
    ```

 1. 新しい `urdf.launch` ファイル
    （ `myworkcell_support` パッケージ内 ）を作成して，
    URDF モデルをロードし，
    オプションで RViz に表示します．

    ``` xml
    <launch>
      <arg name="gui" default="true"/>
      <param name="robot_description" command="$(find xacro)/xacro --inorder '$(find myworkcell_support)/urdf/workcell.xacro'" />
      <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher"/>
      <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">
        <param name="use_gui" value="$(arg gui)"/>
      </node>
      <node name="rviz" pkg="rviz" type="rviz" if="$(arg gui)"/>
    </launch>
    ```

 1. 作成した launch ファイルを実行して，
    更新された URDF を RViz 上で確認します．

    `roslaunch myworkcell_support urdf.launch`

    * 'Fixed frame' を 'world' に設定し，
      左のツリービューに RobotModel と TF の表示を追加して
      ロボットといくつかの変換座標を表示します．
    * ジョイントスライダを動かして
      UR5 ロボットの動きを確認してください．
