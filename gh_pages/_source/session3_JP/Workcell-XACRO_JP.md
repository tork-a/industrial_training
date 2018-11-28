# Workcell XACRO
>In this exercise, we will create an XACRO file representing a simple robot workcell. This will demonstrate both URDF and XACRO elements.

# 作業セル XACRO

> 単純なロボット作業セルを表す XACRO ファイルを作成します．
  これにより URDF と XACRO の両方の要素を見ることができます．

## Motivation
Writing URDFs that involve more than just a few elements can quickly become a pain. Your file gets huge and duplicate items in your workspace means copy-pasting a bunch of links and joints while having to change their names just slightly. It’s really easy to make a mistake that may (or may not) be caught at startup.
It’d be nice if we could take some guidance from programming languages themselves: define a component once, then re-use it anywhere without excessive duplication. Functions and classes do that for programs, and XACRO macros do that for URDFs. XACRO has other cool features too, like a file include system (think #include), constant variables, math expression evaluation (e.g., say PI/2.0 instead of 1.57), and more.


## モチベーション

要素がほんの数個よりも多い URDF を書いていると，
すぐにそれが苦痛になってくるかと思います．
ワークスペースのリンクやジョイントの束をコピーして貼り付け，
その名前をわずかに変更して複製したアイテムやファイルは膨大になってしまいます．
起動時に把握される（またはされない）間違いが簡単に発生してしまいます．

コンポーネントを一度定義すれば複製無しでどこででも再利用可能といったような，
プログラミング言語で行っているような指示方法があれば助かります．
プログラムでは関数とクラスがそれを行い，
XACRO マクロは URDF のためにそれを行います．
XACRO には他にも
（#include 的な）ファイル・インクルード・システム，
定数変数，
数式表現評価（ 例えば 1.57 ではなく PI/2.0 ）
などの優れた機能もあります．


## Reference Example

[Cleaning Up URDF with XACRO Tutorial](http://wiki.ros.org/urdf/Tutorials/Using%20Xacro%20to%20Clean%20Up%20a%20URDF%20File)

## リファレンス

* [Using Xacro to Clean Up a URDF File](http://wiki.ros.org/urdf/Tutorials/Using%20Xacro%20to%20Clean%20Up%20a%20URDF%20File)


## Further Information and Resources

[Xacro Extension Documentation](http://wiki.ros.org/xacro)

[Creating a URDF for an Industrial Robot](http://wiki.ros.org/Industrial/Tutorials/Create%20a%20URDF%20for%20an%20Industrial%20Robot)


## 追加情報とリソース

* [Xacro Extension Documentation](http://wiki.ros.org/xacro)
* [Creating a URDF for an Industrial Robot](http://wiki.ros.org/Industrial/Tutorials/Create%20a%20URDF%20for%20an%20Industrial%20Robot)


## Scan-N-Plan Application: Problem Statement
In the previous exercise we created a workcell consisting of only static geometry. In this exercise, we'll add a UR5 robot _assembly_ using XACRO tools.

Specifically, you will need to:
 1. Convert the `*.urdf` file you created in the previous sample into a XACRO file with the `xacro` extension.
 1. Include a file containing the xacro-macro definition of a `UR5`
 1. Instantiate a `UR5` in your workspace and connect it to the _table_ link.


## Scan-N-Plan アプリケーション: 演習問題

前回の演習では静的ジオメトリのみで構成される作業セル（workcell）を作成しました．
今回は XACRO ツールを使用して UR5 ロボット _assembly_ を追加します．

具体的には次のように行っていきます．

  1. 前回作成した `*.urdf` ファイルを `xacro` 拡張子を持つ XACRO ファイルに変換します．
  1. `UR5` の XACRO マクロ定義を含むファイルをインクルードします．
  1. 作業スペースで `UR5` をインスタンス化し，それを _table_ リンクに接続します．


## Scan-N-Plan Application: Guidance
 1. Rename the `workcell.urdf` file from the previous exercise to `workcell.xacro`

 1. Bring in the `ur_description` package into your ROS environment. You have a few options:

    1. You can install the debian packages: `sudo apt install ros-kinetic-ur-description ros-kinetic-ur-kinematics`

    1. You can clone it from [GitHub](https://github.com/ros-industrial/universal_robot) to your catkin workspace:

       ```
       cd ~/catkin_ws/src
       git clone https://github.com/ros-industrial/universal_robot.git
       catkin build
       source ~/catkin_ws/devel/setup.bash
       ```

    >It’s not uncommon for description packages to put each “module”, “part”, or “assembly” into its own file. In many cases, a package will also define extra files that define a complete cell with the given part so that we can easily visually inspect the result. The UR package defines such a file for the UR5 ([ur5_robot.urdf.xacro](https://github.com/ros-industrial/universal_robot/blob/indigo-devel/ur_description/urdf/ur5_robot.urdf.xacro)): It’s a great example for this module.

 1. Locate the xacro file that implements the UR5 macro and include it in your newly renamed `workcell.xacro` file.  Add this include line near the top of your `workcell.xacro` file, beneath the `<robot>` tag:

    ``` xml
    <xacro:include filename="$(find ur_description)/urdf/ur5.urdf.xacro" />
    ```

    >If you explore the UR5 definition file, or just about any other file that defines a Xacro macro, you’ll find a lot of uses of `${prefix}` in element names. Xacro evaluates anything inside a “${}” at run-time. It can do basic math, and it can look up variables that come to it via properties (ala-global variables) or macro parameters. Most macros will take a “prefix” parameter to allow a user to create multiple instances of said macro. It’s the mechanism by which we can make the eventual URDF element names unique, otherwise we’d get duplicate link names and URDF would complain.

 1. Including the `ur5.urdf.xacro` file does not actually create a UR5 robot in our URDF model.  It defines a macro, but we still need to call the macro to create the robot links and joints.  _Note the use of the `prefix` tag, as discussed above._

    ``` xml
    <xacro:ur5_robot prefix="" joint_limited="true"/>
    ```

    >Macros in Xacro are just fancy wrappers around copy-paste. You make a macro and it gets turned into a chunk of links and joints. You still have to connect the rest of your world to that macro’s results. This means you have to look at the macro and see what the base link is and what the end link is. Hopefully your macro follows a standard, like the ROS-Industrial one, that says that base links are named “base_link” and the last link is called “tool0”.

 1. Connect the UR5 `base_link` to your existing static geometry with a fixed link.

    ``` xml
    <joint name="table_to_robot" type="fixed">
      <parent link="table"/>
      <child link="base_link"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
    </joint>
    ```

 1. Create a new `urdf.launch` file (in the `myworkcell_support` package) to load the URDF model and (optionally) display it in rviz:

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

 1. Check the updated URDF in RViz, using the launch file you just created:

    `roslaunch myworkcell_support urdf.launch`

    * Set the 'Fixed Frame' to 'world' and add the RobotModel and TF displays to the tree view on the left, to show the robot and some transforms.
    * Try moving the joint sliders to see the UR5 robot move.


## Scan-N-Plan アプリケーション: ガイダンス

 1. 前回の演習の `workcell.urdf` のファイル名を `workcell.xacro` に変更してください．

 1. `ur_description` パッケージをあなたの ROS 環境に導入してください．

    いくつかの方法があります．

    1. debian パッケージをインストールする．

       ```
       sudo apt install ros-kinetic-ur-description ros-kinetic-ur-kinematics
       ```

    1. [GitHub](https://github.com/ros-industrial/universal_robot) から catkin ワークスペースにクローンする．

       ```
       cd ~/catkin_ws/src
       git clone https://github.com/ros-industrial/universal_robot.git
       catkin build
       source ~/catkin_ws/devel/setup.bash
       ```

    > description パッケージが各 "module"，"part" または "assembly" を
      それ自身のファイルに入れることはよくあります．
      多くの場合，パッケージは特定の部品を持つ全体のセルを定義する以外のファイルも定義していて，
      それらを簡単に視覚的に調べることができます．
      UR パッケージは UR5（ [ur5_robot.urdf.xacro](https://github.com/ros-industrial/universal_robot/blob/indigo-devel/ur_description/urdf/ur5_robot.urdf.xacro) ）のファイルを定義します．
      これは本演習にとって非常に良い例となっています．

 1. UR5 マクロを実装している xacro ファイルを探し，
    新しく名前を変更した `workcell.xacro` ファイルに組み込みます．

    `workcell.xacro` ファイルの最上部の `<robot>` タグの下に
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

 1. UR5の `base_link` を既存の静的ジオメトリに固定リンクで接続します．

    ``` xml
    <joint name="table_to_robot" type="fixed">
      <parent link="table"/>
      <child link="base_link"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
    </joint>
    ```

 1. 新しい `urdf.launch` ファイル（ `myworkcell_support` パッケージ内 ）を作成して，
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

 1. 作成した launch ファイルを実行して，更新された URDF を RViz 上で確認します．

    `roslaunch myworkcell_support urdf.launch`

    * 'Fixed frame' を 'world' に設定し，
      左のツリービューに RobotModel と TF の表示を追加して
      ロボットといくつかの変換座標を表示します．
    * ジョイントスライダを動かして UR5 ロボットの動きを確認してください．
