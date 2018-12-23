# ROS と Docker / Amazon Web Service

## デモ #1 - フロントエンド: ホスト Gazebo / バックエンド: Docker

### ワークスペースのセットアップ

#### フロントエンド（ ホストで実行 ） (run on host and only contains gui)

1つ目のターミナルで次を実行してください．

```
mkdir -p ./training/front_ws/src
cd ./training/front_ws/src
gazebo -v
git clone -b gazebo7 https://github.com/fetchrobotics/fetch_gazebo.git
git clone https://github.com/fetchrobotics/robot_controllers.git
git clone https://github.com/fetchrobotics/fetch_ros.git
cd ..
catkin build fetch_gazebo fetch_description
```

#### バックエンド（ Docker コンテナ内で実行 ）

このステップでは必要な実行可能ファイルを含む Docker イメージを作成します．

 - rosindustrial/core:indigo イメージ内で /bin/bash を実行してから
   apt-get でパッケージを取得
 - rosindustrial/core:indigo イメージ内で /bin/bash を実行してから
   ソースからパッケージをビルドして，その結果をコミット
 - fetch ロボットの Dockerfile で Docker コンテナを作成し，それを実行
   https://gist.github.com/AustinDeric/242c1edf1c934406f59dfd078a0ce7fa

   ```
   cd ../fetch-Dockerfile/
   docker build --network=host -t rosindustrial/fetch:indigo .
   ```


### デモンストレーションの実行

#### フロントエンドの実行

1つ目のターミナルからフロントエンドを実行します．

```
source devel/setup.bash
roslaunch fetch_gazebo playground.launch
```

#### バックエンドの実行

これを実行するには複数の方法があります．

   - fetch コンテナ内で /bin/bash を実行し，デモノードを手動で実行する方法
   - デモノードをコンテナ内で直接実行する方法（ これから本演習で行います ）

2つ目のターミナルで次のとおりバックエンドを実行してください．

```
docker run --network=host rosindustrial/fetch:indigo roslaunch fetch_gazebo_demo demo.launch
```


## デモ #2 - フロントエンド: Web サーバ / バックエンド: Docker

環境を開始します．

```
docker run --network=host rosindustrial/fetch:indigo roslaunch fetch_gazebo playground.launch headless:=true gui:=false
```

Gazebo の Web サーバを実行します．

```
docker run -v "/home/aderic/roscloud/training/front_ws/src/fetch_gazebo/fetch_gazebo/models/test_zone/meshes/:/root/gzweb/http/client/assets/test_zone/meshes/" -v "/home/aderic/roscloud/training/front_ws/src/fetch_ros/fetch_description/meshes:/root/gzweb/http/client/assets/fetch_description/meshes" -it --network=host giodegas/gzweb /bin/bash
```

サーバを実行します．

```
/root/gzweb/start_gzweb.sh && gzserver
```

3つ目のターミナルでデモを実行します．

```
docker run --network=host fetch roslaunch fetch_gazebo_demo demo.launch
```


## デモ #3 ロボット Web ツール

このデモでは産業用ロボットURDFをブラウザで表示することができます．

1つ目のターミナルで，ロボットをパラメータサーバに読み込ませます．

```
mkdir -p abb_ws/src
git clone -b kinetic-devel https://github.com/ros-industrial/abb.git
docker run -v "/home/aderic/roscloud/training/abb_ws:/abb_ws" --network=host -it rosindustrial/core:kinetic /bin/bash
cd abb_ws
catkin build
source devel/setup.bash
roslaunch abb_irb5400_support load_irb5400.launch
```

2つ目のターミナルで，ロボット Web ツールを開始します．

```
docker run --network=host rosindustrial/viz:kinetic roslaunch viz.launch
```

3つ目のターミナルで Web サーバを起動します．
まず www フォルダを開始する必要があります．

```
cp -r abb_ws/src/abb/abb_irb5400_support/ www/
```

<script src="https://gist.github.com/AustinDeric/e806e6676e6c80590b8562633c7220a4.js"></script>

<script src="https://gist.github.com/AustinDeric/ed046693bed9e55dfbe546fe8d479284.js"></script>

```
docker run -v "/home/aderic/roscloud/training/www:/data/www" -v "/home/aderic/roscloud/training/nginx_conf/:/etc/nginx/local/" -it --network=host rosindustrial/nginx:latest /bin/bash
nginx -c /etc/nginx/local/nginx.conf
```
