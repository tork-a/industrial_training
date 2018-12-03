# 知覚処理入門

> 3D データの処理により慣れ親しむために，
  ASUS Xtion Pro（ または Microsoft Kinect ）センサーから
  生成されたデータを使用してみます．
  そのデータストリームを見て
  RViz 内でで様々な方法でデータを視覚化します．


## ポイントクラウド・データファイル

ほとんどの知覚処理はセンサからの
ROS メッセージデータから始まります．
本演習では一般的な Kinect スタイルのセンサーの 3D
[Point Cloud](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html)
データを使用します。

1. まずは RViz に表示するポイントクラウドのデータを
   ROS メッセージとしてパブリッシュします．

   1. ターミナルから `roscore` を開始します．

   1. 本演習用に新しいディレクトリを作成します．

      ```
      mkdir ~/ex4.2
      cd ~/ex4.2
      cp ~/industrial_training/exercises/4.2/table.pcd .
      ```

   1. 予め記録されたポイントクラウド・ファイル `table.pcd` から
      ポイントクラウド・メッセージをパブリッシュします．

      ```
      cd ~
      rosrun pcl_ros pcd_to_pointcloud table.pcd 0.1 _frame_id:=map cloud_pcd:=orig_cloud_pcd
      ```

   1. `orig_cloud_pcd` トピックが発行されていることを
      `rostopic list` で確認してください．


## ポイントクラウドを RViz に表示する

1. ポイントクラウド処理の出力を表示するために
   RViz ウィンドウを起動します．

   ```
   rosrun rviz rviz
   ```

1. **PointCloud2** ディスプレイ・アイテムを追加して
   適切なトピック名を指定します．

   1. Displays パネル下部の **Add** をクリック
   1. **PointCloud2** を選択
   1. ディスプレイツリーにある **PointCloud2** を展開して
      topic ドロップダウンからトピックを選択
      * _ヒント: ポイントクラウド・ファイルを使用している場合，
        トピック名は `/orig_cloud_pcd` です．_


### PCL を試す

次にポイントクラウドデータを処理するために
PCL が提供するさまざまなコマンドラインツールを試します．
利用可能なコマンドラインツールは140個を超えていますが，
本演習ではいくつかのツールだけ使用します．
その趣旨としてはコードを書くことなく PCL の機能に慣れることですが，
これらのコマンドラインツールは自分でコードを書き始めるのに最適なものです．
コマンドラインツールはさまざまな処理メソッドをテストするのに役立ちますが，
ほとんどのアプリケーションでは通常は
実際の処理パイプラインに直接 C++ ライブラリを使用します．
発展的な ROS-I トレーニングコースでは，
これらの C++ PCL メソッドを詳しく解説します．

下記の各 PCL コマンドは PCL 処理コマンドの結果から
新しいポイントクラウドファイル（ `.pcd` ）を生成します．
`pcl_viewer` を使って出力を直接見るか，
`pcd_to_pointcloud` コマンドを使ってポイントクラウドデータを
RViz に表示するための ROS メッセージとしてパブリッシュします．
RViz で出力を確認した後は `pcd_to_pointcloud` コマンドを停止してください．


### pcl_voxel_grid を用いてポイントクラウドをダウンサンプリングする

1. グリッドサイズが (0.05, 0.05, 0.05) のボクセル・グリッドを使用して
   元のポイントクラウドをダウンサンプリングします．
   ボクセルグリッドでは1つのグリッドキューブ内のすべての点が
   ボクセルの中心にある単一の点に置き換えられます．
   これは複雑すぎたり詳細すぎたりするセンサデータを単純化して，
   処理ステップを高速化する一般的な方法です．

  ```
  pcl_voxel_grid table.pcd table_downsampled.pcd -leaf 0.05,0.05,0.05
  pcl_viewer table_downsampled.pcd
  ```

1. 新しいポイントクラウドを
   RViz で見てください．（任意）

  ```
  rosrun pcl_ros pcd_to_pointcloud table_downsampled.pcd 0.1 _frame_id:=map cloud_pcd:=table_downsampled
  ```

  _注: RViz の PointCloud2 では
      トピックを */table_downsampled* に変更して
      新しいデータを表示してください．_


### pcl_sac_segmentation_plane を用いてポイントクラウドからテーブルサーフェスを抽出する

1. 最大の平面を見つけ，
   その平面に属する与えられた閾値内の点を抽出する．

  ```
  pcl_sac_segmentation_plane table_downsampled.pcd only_table.pcd -thresh 0.01
  pcl_viewer only_table.pcd
  ```

1. 新しいポイントクラウドを
   RViz で見てください．（任意）

  ```
  rosrun pcl_ros pcd_to_pointcloud only_table.pcd 0.1 _frame_id:=map cloud_pcd:=only_table
  ```

  _注: RViz の PointCloud2 では
    トピックを */only_table* に変更して
    新しいデータを表示してください．_


### pcl_sac_segmentation_plane を用いてポイントクラウドからテーブル上で最大のクラスタを抽出する

1. テーブルに属する最大のポイントクラスタを抽出します．

  ```
  pcl_sac_segmentation_plane table.pcd object_on_table.pcd -thresh 0.01 -neg 1
  pcl_viewer object_on_table.pcd
  ```

1. 新しいポイントクラウドを
   RViz で見てください．（任意）

  ```
  rosrun pcl_ros pcd_to_pointcloud object_on_table.pcd 0.1 _frame_id:=map cloud_pcd:=object_on_table
  ```

  _注: RViz の PointCloud2 では
    トピックを */object_on_table* に変更して
    新しいデータを表示してください．_


### pcl_outlier_removal を用いてポイントクラウドから異常値を削除する

1. 本例では異常値を除去するために統計的方法が使用されます．
   これはノイズの多いセンサデータをクリーンアップしたり，
   追加処理する前に誤った中間生成物を除去したりするのに便利です．

  ```
  pcl_outlier_removal table.pcd table_outlier_removal.pcd -method statistical
  pcl_viewer table_outlier_removal.pcd
  ```

1. 新しいポイントクラウドを
   RViz で見てください．（任意）

  ```
  rosrun pcl_ros pcd_to_pointcloud table_outlier_removal.pcd 0.1 _frame_id:=map cloud_pcd:=table_outlier_removal
  ```

  _注: RViz の PointCloud2 では
    トピックを */table_outlier_removal* に変更して
    新しいデータを表示してください．_


### pcl_normal_estimation を用いてポイントクラウドの各ポイントの法線を計算する

1. 本例では各点におけるローカルサーフェスの
   法線（垂直）ベクトルを推定します．
   アルゴリズムは
   各点について指定された半径内の近傍点を平面にフィットさせて
   法線ベクトルを計算します．
   そして法線ベクトルについてより詳細に表示します。

  ```
  pcl_normal_estimation only_table.pcd table_normals.pcd -radius 0.1
  pcl_viewer table_normals.pcd -normals 10
  ```


### マーチングキューブ法による再構成を用いてポイントクラウドをメッシュ化する

ポイントクラウドデータは構造化されていないことが多いのですが，
処理アルゴリズムがより構造化されたサーフェスメッシュに対して
処理する必要があることが時々あります．
本例ではマーチングキューブアルゴリズムを使用して，
ポイントクラウドデータを近似したサーフェスメッシュを作成します．

  ```
  pcl_marching_cubes_reconstruction table_normals.pcd table_mesh.vtk -grid_res 20
  pcl_viewer table_mesh.vtk
  ```
