<!--
# Package Setup
>In this exercise, we'll build our package dependencies and configure the package for the Qt Creator IDE.
-->

# パッケージのセットアップ
> 依存するパッケージをビルドしてから，Qt Creator IDE のためのパッケージを設定します．

<!--
## Build Package Dependencies
In a terminal type:
```
cd ~/perception_driven_ws
catkin build --cmake-args -G 'CodeBlocks - Unix Makefiles'
source devel/setup.bash
```
-->

## 依存するパッケージのビルド
ターミナルにて下記のように入力してください．
```
cd ~/perception_driven_ws
catkin build --cmake-args -G 'CodeBlocks - Unix Makefiles'
source devel/setup.bash
```

<!--
## Import Package into QT
In QT do the following:
```
File -> New -> Import ROS Project ->
```
-->

## QT へのパッケージのインポート
QT を起動して下記のように行ってください．
```
File -> New File or Project -> Import ROS Project ->
```

<!--
## Open the Main Thread Source File
  In the project tab, navigate into the `[Source directory]/collision_avoidance_pick_and_place/src/nodes` directory and open the `pick_and_place_node.cpp` file
-->

## メインスレッドのソースファイルを開く
Project タブ内で `[Source directory]/collision_avoidance_pick_and_place/src/nodes` ディレクトリに移動して `pick_and_place_node.cpp` ファイルを開きます．
