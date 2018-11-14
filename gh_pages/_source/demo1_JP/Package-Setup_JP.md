# パッケージのセットアップ
> 依存するパッケージをビルドしてから，Qt Creator IDE のためのパッケージを設定します．

## 依存するパッケージのビルド
ターミナルにて下記のように入力してください．

```
cd ~/perception_driven_ws
catkin build --cmake-args -G 'CodeBlocks - Unix Makefiles'
source devel/setup.bash
```


## QT へのパッケージのインポート

QT を起動して下記のように行ってください．

```
File -> New File or Project -> Import ROS Project ->
```

## メインスレッドのソースファイルを開く
Project タブ内で `[Source directory]/collision_avoidance_pick_and_place/src/nodes` ディレクトリに移動して `pick_and_place_node.cpp` ファイルを開きます．
