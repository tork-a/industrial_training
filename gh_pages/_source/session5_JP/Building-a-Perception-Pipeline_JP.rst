知覚パイプラインの構築
==================
本演習では，知覚パイプラインを構築するための適切なコードを記入します．
最終目標は対象オブジェクトの姿勢情報を変換したものをブロードキャストすることです．


新しいワークスペースの準備
----------------------

#. 本演習は以前の演習から独立した内容なので，
   新しい catkin ワークスペースを作成します．

   #. ``gedit ~/.bashrc``

   #. ``~/catkin_ws/devel/setup.bash`` を実行している
      最後の行をコメントアウト(`#`)してください．

      .. Note:: 新規にターミナルを開くたびに
         新しい catkin ワークスペースのセットアップファイルを
         手動で実行する必要が生じます．

   #. gedit を閉じて ROS 環境のセットアップを実行します．

      .. code-block:: bash

         source /opt/ros/kinetic/setup.bash

#. テンプレート・ワークスペース・レイアウトとファイルのコピーを行います．

   .. code-block:: bash

      cp -r ~/industrial_training/exercises/5.1/template_ws ~/perception_ws
      cd ~/perception_ws/

   #. 新しいワークスペースの初期化とビルドを行います．

      .. code-block:: bash

         catkin init
         catkin build

   #. ワークスペース環境のセットアップを行います．

      .. code-block:: bash

         source ~/perception_ws/devel/setup.bash

#. PointCloud ファイルを先の 演習 4.2 からホームディレクトリ (~) にコピーします．

   .. code-block:: bash

      cp ~/industrial_training/exercises/4.2/table.pcd ~

#. QTCreator IDE に新しいワークスペースをインポートします．

   * QTCreator にて: `File -> New File or Project -> Other Project -> ROS Workspace -> ~/perception_ws`


イントロダクション（既存コードを読む）
-------------------------------
ROS ノードの大半は既に完成していますので，
本演習の焦点は知覚のアルゴリズムとパイプラインです．
`CMakelists.txt` と `package.xml` が完成しており，実行可能ファイルが提供されています．
実行可能ファイルはそのままでも実行できますがエラーが発生します．
そこで提供されているソースコード `perception_node.cpp` を探して内容を見てください．
下記はコードの概要です．

#. ヘッダ:

   * PCL 関連のヘッダーのコメントを外す必要があります．

#. int main():

   * ``main`` 関数は ``while`` ループを含んだ状態で提供されています．

#. ROS の初期化:

   * ``ros::init`` と ``ros::NodeHandle`` の2つが呼び出され初期化されています．
     さらにノードの名前空間内に launch ファイルからパラメータを取得する場合に使用する
     プライベートノードハンドルがあります．

#. パラメータのセットアップ:

   * 現状，本例には3つの文字列パラメータがあります:
     ワールドフレーム，カメラフレーム および カメラが公開しているトピック です．
     これらのパラメータを launch ファイルから読み込む
     いくつかの ``nh.getParam`` 行を書くのは簡単だと思います．
     PCL メソッドのためにパラメータが多くなるため，
     ハードコードするよりも launch ファイルを使って調整するほうが便利です．
     時間があればこれを設定しておいた方が良いでしょう．

#. パブリッシャのセットアップ:

   * 2つのパブリッシャがポイントクラウドの ROS メッセージを公開するように設定されています．
     画像やポイントクラウドの処理では結果を視覚化すると便利なことがよくあります．

#. PointCloud2 のリスナ（ while ループ内 ）:

   * 通常は `ここ <http://wiki.ros.org/pcl/Tutorials>`__
     （ `日本語版 <http://wiki.ros.org/ja/pcl/Tutorials>`__ ）で行われているように
     ROS の購読メソッドとコールバック関数を使用して ROS メッセージを待ち受けます．
     しかしこれをコールバック関数の外で行うと便利なことが多いので
     ``ros::topic::waitForMessage `` を使ってメッセージを聞く例を示します．

#. PointCloud2 の変換（ while ループ内 ）:

   * While we could work in the camera frame, things are more understandable/useful if we are looking at the points of a point cloud in an xyz space that makes more sense with our environment. In this case we are transforming the points from the camera frame to a world frame.
   * カメラフレーム上で作業することも可能ですが，
     XYZ 空間上でポイントクラウドの点を見た方がより理解しやすく便利です．
     そのために点群をカメラフレームからワールドフレームに変換しています．

#. PointCloud2 の変換（ ROS から PCL に / while ループ内 ）:

#. PointCloud2 の変換（ PCL から ROS に / while ループ内 ）:

   * このステップは必須ではありませんが，
     ポイントクラウドの処理結果を視覚化すると便利ですので
     ROS の型に戻して，パブリッシュ用の ROS メッセージを作成しています．

このように多くのことが既に行われているので，
コードの仕上げは簡単かと思います．
必要があるのは途中の部分に書き入れることだけです．


主要なタスク: 空欄を埋める
-----------------------------------
知覚アルゴリズムを含む途中の部分を埋める作業は反復的なプロセスですので，
各ステップはそれぞれのサブタスクに分割されています．


Voxel フィルタの実装
^^^^^^^^^^^^^^^^^^^^^^
#. ファイルの最上部付近にある `voxel_grid` のインクルード・ヘッダのコメントを外します．

#. コードの変更:

   ほとんどのポイントクラウド処理パイプラインの最初のステップは Voxel フィルタです．
   このフィルタは点群をダウンサンプリングするだけでなく，
   NAN 値を排除してより先のフィルタリングや処理が
   実際的な値に対して行われるようにします．
   `PCL Voxel Filter Tutorial <http://pointclouds.org/documentation/tutorials/voxel_grid.php#voxelgrid>`_
   を参照してください．
   もしくは下記のコードをコピーしてください．

   ``perception_node.cpp`` 内の次の部分を探してください．

   .. code-block:: c++

      /* ========================================
       * Fill Code: VOXEL GRID
       * ========================================*/

   このバナーの下に次のコードをコピー＆ペーストしてください．

   .. code-block:: c++

      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ> (cloud));
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel_filtered (new pcl::PointCloud<pcl::PointXYZ> ());
      pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
      voxel_filter.setInputCloud (cloud_ptr);
      voxel_filter.setLeafSize (float(0.002), float(0.002), float(0.002));
      voxel_filter.filter (*cloud_voxel_filtered);

#. ``perception_node.cpp`` 内のパブリッシャを更新します．

   次の部分を探してください．

   .. code-block:: c++

      /* ========================================
       * CONVERT POINTCLOUD PCL->ROS
       * PUBLISH CLOUD
       * Fill Code: UPDATE AS NECESSARY
       * ========================================*/

   ``pcl::toROSMsg`` のコメントを外して
   ``*cloud_ptr`` を ``*cloud_voxel_filtered`` で置き換えます．

   *各新規アップデート後に RViz で表示するために
    パブリッシュされているポイントクラウドを置き換えます．*

   .. Note:: 時間と忍耐力があれば，
      フィルタの種類ごとに ROS パブリッシャを作成することをお勧めします．
      Rviz で異なるポイントクラウドを切り替えて
      複数のフィルタの結果を一度に見ることができると便利です．

#. コンパイル

   .. code-block:: bash

      catkin build

結果の表示
""""""""
#. 知覚パイプラインを実行します．

   .. Note:: RViz で Global Frame を **kinect_link** に変更してください．

   .. code-block:: bash

      cd ~
      roscore
      rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 world_frame kinect_link
      rosrun pcl_ros pcd_to_pointcloud table.pcd 0.1 _frame_id:=kinect_link cloud_pcd:=kinect/depth_registered/points
      rosrun rviz rviz
      rosrun lesson_perception perception_node

#. 結果を見る

   RViz 内で *PointCloud2* ディスプレイを追加してください．
   トピックには "object_cluster" を設定します．
   表示されるのはオリジナルのポイントクラウド
   （ 演習 4.2を完了して新しいデフォルト設定を保存したか，その演習の設定を保存した場合 ）
   にオーバーレイされた Voxel フィルタの結果です．

   .. image:: /_static/cloud_voxel_filtered.png

#. 結果を表示し終えたら Voxel のフィルターサイズを
   0.002 から 0.100 に変更して，
   結果をもう一度表示してみてください．
   終えたらフィルターを 0.002 に戻してください．

   * この変更の結果を確認するには Ctrl+C キーを使用して知覚ノードを終了し，
     リビルドしてから知覚ノードを再実行します．

   .. Note:: 他のノード（ RViz や ROS など ）を停止する必要はありません．

#. Voxel フィルタを確認できたら，Ctrl+C を入力して知覚ノードを停止してください．


パススルーフィルタの実装
^^^^^^^^^^^^^^^^^^^^
#. 前回と同じようにファイルの最上部付近の
   PassThrough
   フィルタのインクルード・ヘッダのコメントアウトを外します．

#. コードの変更:

   次の関心のある領域を取得するために便利なフィルタリングは一連のパススルーフィルタです．
   これらのフィルタはポイントクラウドをある空間の容積まで縮小します．
   （ x y と z フィルタを使用する場合 ）
   ここでは x, y, z の各方向にそれぞれのパススルーフィルタを適用していきます．
   ヒントは
   `PCL Pass-Through Filter Tutorial <http://pointclouds.org/documentation/tutorials/passthrough.php#passthrough>`_
   を参照するか，次のコードを使用してください．

   perception_node.cpp 内の次のセクションを探してください．

   .. code-block:: c++

      /* ========================================
       * Fill Code: PASSTHROUGH FILTER(S)
       * ========================================*/

   このバナーの下に次のコードをコピー＆ペーストしてください．

   .. code-block:: c++

      pcl::PointCloud<pcl::PointXYZ> xf_cloud, yf_cloud, zf_cloud;
      pcl::PassThrough<pcl::PointXYZ> pass_x;
      pass_x.setInputCloud(cloud_voxel_filtered);
      pass_x.setFilterFieldName("x");
      pass_x.setFilterLimits(-1.0,1.0);
      pass_x.filter(xf_cloud);

      pcl::PointCloud<pcl::PointXYZ>::Ptr xf_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(xf_cloud));
      pcl::PassThrough<pcl::PointXYZ> pass_y;
      pass_y.setInputCloud(xf_cloud_ptr);
      pass_y.setFilterFieldName("y");
      pass_y.setFilterLimits(-1.0, 1.0);
      pass_y.filter(yf_cloud);

      pcl::PointCloud<pcl::PointXYZ>::Ptr yf_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(yf_cloud));
      pcl::PassThrough<pcl::PointXYZ> pass_z;
      pass_z.setInputCloud(yf_cloud_ptr);
      pass_z.setFilterFieldName("z");
      pass_z.setFilterLimits(-1.0, 1.0);
      pass_z.filter(zf_cloud);

   *フィルタの閾値を変更すると異なる結果を見ることができます．*

#. ``pc2_cloud`` が入力されている ``pcl::toROSMsg`` 呼び出し部分を探してください．
   これは RViz ディスプレイで閲覧されるポイントクラウドです．
   現在のクラウド（ ``*cloud_voxel_filter`` ）を
   最終的なパススルーフィルタ結果（ ``zf_cloud`` ）に置き換えます．

#. コンパイルと実行

   .. code-block:: bash

      catkin build
      rosrun lesson_perception perception_node

#. 結果の表示

   RViz 内でオリジナル・カメラデータの ``/kinect/depth_registered/points`` に基づいた PointCloud2 と
   最新の処理結果の ``object_cluster`` トピックを比べてみてください．
   元のポイントクラウドの一部が「切り取られて」いることが分かると思います．

   .. image:: /_static/zf_cloud.png

   .. Note:: X/Y/Z のフィルタ閾値を変更（ 例: +/- 0.5 ）してみてください．
             リビルドと実行し直して RViz で効果を見ます．
             いろいろ試し終わりましたら閾値を +/- 1.0 に戻してください．

#. パススルー・フィルタの挙動を確認できたら，Ctrl+C キーを使用してノードを停止してください．
   他のターミナルを閉じたり他のノードを停止したりする必要はありません．


平面のセグメンテーション
^^^^^^^^^^^^^^^^^^^^
#. コードの変更

   この方法は対象物が平坦な表面上にあるあらゆるアプリケーションにとって最も有用な方法の1つです．
   テーブル上のオブジェクトを分離するには，
   ポイント群を平面にフィットさせ，
   テーブルを構成するポイント群を見つけ，
   それらのポイント群を除外して，
   テーブル上のオブジェクトに対応するポイント群だけを残すようにします．
   これは私たちが使用する最も複雑な PCL メソッドで，
   実際には RANSAC セグメンテーションモデルと
   抽出インデックスツールの2つの組み合わせとなります．
   詳細な例は
   `PCL Plane Model Segmentation Tutorial <http://pointclouds.org/documentation/tutorials/planar_segmentation.php#planar-segmentation>`_
   にあります．
   もしくは下記コードをコピーして使用してください．

   perception_node.cpp 内の次のセクションを探してください．

   .. code-block:: c++

      /* ========================================
       * Fill Code: PLANE SEGEMENTATION
       * ========================================*/

   このバナーの下に次のコードをコピー＆ペーストしてください．

   .. code-block:: c++

      pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZ>(zf_cloud));
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
      // Create the segmentation object for the planar model and set all the parameters
      pcl::SACSegmentation<pcl::PointXYZ> seg;
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      seg.setOptimizeCoefficients (true);
      seg.setModelType (pcl::SACMODEL_PLANE);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setMaxIterations (200);
      seg.setDistanceThreshold (0.004);
      // Segment the largest planar component from the cropped cloud
      seg.setInputCloud (cropped_cloud);
      seg.segment (*inliers, *coefficients);
      if (inliers->indices.size () == 0)
      {
        ROS_WARN_STREAM ("Could not estimate a planar model for the given dataset.") ;
        //break;
      }

   Once you have the inliers (points which fit the plane model),
   then you can extract the indices within the pointcloud data structure of the points
   which make up the plane.

   平面モデルに合致した点を取得したら，
   平面を構成するポイント群のポイントクラウドデータ構造内の
   インデックスを抽出することができます．

   .. code-block:: c++

      // Extract the planar inliers from the input cloud
      pcl::ExtractIndices<pcl::PointXYZ> extract;
      extract.setInputCloud (cropped_cloud);
      extract.setIndices(inliers);
      extract.setNegative (false);

      // Get the points associated with the planar surface
      extract.filter (*cloud_plane);
      ROS_INFO_STREAM("PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." );

   もちろん，これらの点をクラウドから減算またはフィルタリングして，
   平面上の点のみを取得することもできます．

   .. code-block:: c++

      // Remove the planar inliers, extract the rest
      extract.setNegative (true);
      extract.filter (*cloud_f);

#. ``pc2_cloud`` が入力されている ``pcl::toROSMsg`` 呼び出し部分を探してください．
   これは RViz ディスプレイで閲覧されるポイントクラウドです．
   現在のクラウド（ ``zf_cloud`` ）を
   平面適合から外れた結果（ ``*cloud_f`` ）に置き換えます．

#. 前のステップと同様にコンパイルと実行を行ってください．
   このステップで必要なヘッダ部分のコメントアウトを外すのを忘れないようにしてください．

#. 結果の確認

   RViz 内でオリジナル・カメラデータの ``/kinect/depth_registered/points`` に基づいた PointCloud2 と
   最新の処理結果の ``object_cluster`` トピックを比べてみてください．
   テーブル平面の上にある点だけが最終的な処理の結果として残っていることが分かると思います．

   .. image:: /_static/cloud_f.png

#. 結果の表示が終了したら，"setMaxIterations" および "setDistanceThreshold" の値を変更して，
   平面適合点／不適合点としてデータをどれくらい厳密に制御できるかを試して，
   その結果を表示してみてください．
   ``MaxIterations = 100`` と ``DistanceThreshold = 0.010`` の値を試してみてください．

#. 平面分割の結果を確認できたら，Ctrl+C を入力してノードを停止してください．
   他のターミナルを閉じたり他のノードを停止したりする必要はありません．


ユークリッドクラスタの抽出（任意／ただし推奨）
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
#. コードの変更

   この方法は複数のオブジェクトがあるすべてのアプリケーションにおいて有用です．
   またこれは複雑な PCL メソッドでもあります．
   詳細な例は
   `PCL Euclidean Cluster Extration Tutorial <http://pointclouds.org/documentation/tutorials/cluster_extraction.php#cluster-extraction>`_
   にあります．

   perception_node.cpp 内の次のセクションを探してください．

   .. code-block:: c++

      /* ========================================
       * Fill Code: EUCLIDEAN CLUSTER EXTRACTION (OPTIONAL/RECOMMENDED)
       * ========================================*/

   PCL チュートリアルに従い，このセクションに次のコードを挿入します．
   このバナーの下に次のコードをコピー＆ペーストしてください．

   .. code-block:: c++

      // Creating the KdTree object for the search method of the extraction
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
      *cloud_filtered = *cloud_f;
      tree->setInputCloud (cloud_filtered);

      std::vector<pcl::PointIndices> cluster_indices;
      pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
      ec.setClusterTolerance (0.01); // 2cm
      ec.setMinClusterSize (300);
      ec.setMaxClusterSize (10000);
      ec.setSearchMethod (tree);
      ec.setInputCloud (cloud_filtered);
      ec.extract (cluster_indices);

      std::vector<sensor_msgs::PointCloud2::Ptr> pc2_clusters;
      std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > clusters;
      for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
      {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
          cloud_cluster->points.push_back(cloud_filtered->points[*pit]);
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        std::cout << "Cluster has " << cloud_cluster->points.size() << " points.\n";
        clusters.push_back(cloud_cluster);
        sensor_msgs::PointCloud2::Ptr tempROSMsg(new sensor_msgs::PointCloud2);
        pcl::toROSMsg(*cloud_cluster, *tempROSMsg);
        pc2_clusters.push_back(tempROSMsg);
      }

#. ``pc2_cloud`` が入力されている ``pcl::toROSMsg`` 呼び出し部分を探してください．
   これは RViz ディスプレイで閲覧されるポイントクラウドです．
   現在のクラウド（ ``*cloud_f`` ）を
   最も大きいクラスタ（ ``*(clusters.at(0))`` ）に置き換えます．

#. 前のステップと同様にコンパイルと実行を行ってください．

#. 結果を RViz で見ます．
   ``setClusterTolerance`` や
   ``setMinClusterSize``，
   ``setMaxClusterSize``
   パラメタをいじって効果を RViz 上で観察してみてください．parameters,

   .. image:: /_static/clusters_at0.png

#. クラスタ抽出の結果を確認できたら，Ctrl+C を入力してノードを停止してください．
   他のターミナルを閉じたり他のノードを停止したりする必要はありません．


クロップボックス・フィルタの作成
^^^^^^^^^^^^^^^^^^^^^^^^^^^

#. コードの変更

   この方法は本演習の2つ目のサブタスクだったパススルーフィルタに似ていますが，
   一連の3つのパススルーフィルタを使う代わりに
   1つのクロップボックス（ CropBox ）フィルタの適用で済みます．

   クロップボックス・フィルタのドキュメントと必要なヘッダファイルのの情報は
   `Point Cloud Library (PCL) <http://docs.pointclouds.org/trunk/classpcl_1_1_crop_box.html>`_
   にあります．


   perception_node.cpp 内の次のセクションを探してください．

   .. code-block:: c++

      /* ========================================
       * Fill Code: CROPBOX (OPTIONAL)
       * Instead of three passthrough filters, the cropbox filter can be used
       * The user should choose one or the other method
       * ========================================*/

   パススルー・フィルタをクロップボックス・フィルタで置き換えます．
   パススルー・フィルタは削除するかコメントアウトしてください．

   参考になる PCL のチュートリアルはなく，上記のリンクにある PCL ドキュメントしかありません．
   一般的な設定方法は同じです．
   （ 結果出力の設定，フィルタ・インスタンスの宣言，入力の設定，パラメーターの設定，フィルター ）

   出力するポイントクラウドの設定:

   .. code-block:: c++

      pcl::PointCloud<pcl::PointXYZ> xyz_filtered_cloud;

   フィルタ・インスタンスの宣言:

   .. code-block:: c++

      pcl::CropBox<pcl::PointXYZ> crop;

   入力の設定:

   .. code-block:: c++

      crop.setInputCloud(cloud_voxel_filtered);

   パラメータの設定:

   ドキュメントによると CropBox は値の最大値・最小値の入力として Eigen Vector4f 型をとります．

   .. code-block:: c++

      Eigen::Vector4f min_point = Eigen::Vector4f(-1.0, -1.0, -1.0, 0);
      Eigen::Vector4f max_point = Eigen::Vector4f(1.0, 1.0, 1.0, 0);
      crop.setMin(min_point);
      crop.setMax(max_point);

   フィルタ:

   .. code-block:: c++

      crop.filter(xyz_filtered_cloud);

   パススルーフィルタを削除またはコメントアウトして
   既に平面セグメンテーションコードを記述している場合は，
   平面セグメンテーションに渡すクラウドの名前を必ず更新してください．
   zf_cloud を xyz_filtered_cloud に置き換えます．

   .. code-block:: c++

      pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZ>(xyz_filtered_cloud));

#. ``pc2_cloud`` が入力されている ``pcl::toROSMsg`` 呼び出し部分を探してください．
   これは RViz ディスプレイで閲覧されるポイントクラウドです．
   現在のクラウドを
   新しいフィルタ結果の（ ``xyz_filtered_cloud`` ）に置き換えます．

#. 前のステップと同様にコンパイルと実行を行ってください．

   次の現在使用している CropBox フィルタの画像は
   平面セグメンテーション・フィルタの画像によく似ています．

   .. image:: /_static/xyz_filtered_cloud.png


統計的な外れ値の除去
^^^^^^^^^^^^^^^^^

#. コードの変更

   この方法は最終結果に複雑性や情報を必ずしも付加するとは限りませんが，
   しばしば有用なことがあります．

   チュートリアルは
   `Removing outliers using a StatisticalOutlierRemoval filter <http://pointclouds.org/documentation/tutorials/statistical_outlier.php#statistical-outlier-removal>`_
   にあります．


   perception_node.cpp 内の次のセクションを探してください．

   .. code-block:: c++

      /* ========================================
       * Fill Code: STATISTICAL OUTLIER REMOVAL (OPTIONAL)
       * ========================================*/

   一般的な設定方法は同じです．
   （ 結果出力の設定，フィルタ・インスタンスの宣言，入力の設定，パラメーターの設定，フィルター ）

   出力するポイントクラウドの設定:

   .. code-block:: c++

      pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud_ptr= clusters.at(0);
      pcl::PointCloud<pcl::PointXYZ>::Ptr sor_cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

   フィルタ・インスタンスの宣言:

   .. code-block:: c++

      pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;

   入力の設定:

   .. code-block:: c++

      sor.setInputCloud (cluster_cloud_ptr);

   パラメータの設定:

   ドキュメントを見ると，StatisticalOutlierRemoval は
   検査する近傍の点の数と外れ値の除外に使用する標準偏差の閾値を使用します．

   .. code-block:: c++

      sor.setMeanK (50);
      sor.setStddevMulThresh (1.0);

   フィルタ:

   .. code-block:: c++

      sor.filter (*sor_cloud_filtered);

#. Find the ``pcl::toROSMsg`` call where the ``pc2_cloud`` is populated.  Replace the current cloud with the new filtered results (``*sor_cloud_filtered``).

#. ``pc2_cloud`` が入力されている ``pcl::toROSMsg`` 呼び出し部分を探してください．
   現在のクラウドを
   新しいフィルタ結果の（
   ``*sor_cloud_filtered``
   ）に置き換えます．

#. 前のステップと同様にコンパイルと実行を行ってください．

   .. image:: /_static/sor_cloud_filtered.png


TF のブロードキャストの作成
^^^^^^^^^^^^^^^^^^^^^^^

これはフィルタ・メソッドではありませんが，
処理パイプラインの結果を他のノードが使用できるようにパブリッシュする方法を提示しています．
多くの場合，処理パイプラインの目的は他のノードが使用するための測定値や
位置の定位，またはその他のメッセージを生成することです．
本サブタスクは TF 変換をブロードキャストして，
テーブル上の最も大きい箱の位置を定義します．
この変換は他のノードが把持するために箱の位置と向きを識別するのに使用されます．

#. コードの変更・挿入

   perception_node.cpp 内の次のセクションを探してください．

   .. code-block:: c++

      /* ========================================
       * BROADCAST TRANSFORM (OPTIONAL)
       * ========================================*/

   `ROS Wiki - Writing a tf broadcaster (C++) <http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28C%2B%2B%29>`_
   （ `ROS Wiki 日本語 - tf の broadcaster を書く(C++) <http://wiki.ros.org/ja/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28C%2B%2B%29>`_ ）
   に沿って行っていきます．
   重要な変更は位置と向きの情報
   （ ``setOrigin(tf::Vector3(msg->x, msg->y, 0.0)`` と ``setRotation(q)`` ）
   の設定です．

   変換を作成:

   .. code-block:: c++

      static tf::TransformBroadcaster br;
      tf::Transform part_transform;

      //Here in the tf::Vector3(x,y,z) x,y, and z should be calculated based on the pointcloud filtering results
      part_transform.setOrigin( tf::Vector3(sor_cloud_filtered->at(1).x, sor_cloud_filtered->at(1).y, sor_cloud_filtered->at(1).z) );
      tf::Quaternion q;
      q.setRPY(0, 0, 0);
      part_transform.setRotation(q);

   原点を設定するか rpy を設定するときは，
   適用した全フィルタの最終結果を用いる必要があることに注意してください．
   ここでは原点は任意に最初の点に設定されます．

   変換をブロードキャスト:

   .. code-block:: c++

      br.sendTransform(tf::StampedTransform(part_transform, ros::Time::now(), world_frame, "part"));

#. いつものようにコンパイルと実行をしてください．
   RViz の Display に TF を追加して，
   箱の上部に位置づけられた TF "part" を表示します．


Create a Polygonal Segmentation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

When using sensor data for collision detection, it is sometimes necessary to exclude "known" objects from the scene to avoid interference from these objects.  MoveIt! contains methods for masking out a robot's own geometry as a "Self Collision" filtering feature.  This example shows how to do something similar using PCL's Polygonal Segmentation filtering.

#. Change code

   This method is similar to the plane segmentation from Sub-Task 3, but instead of segmenting out a plane, you can segment and remove a prism. Documentation on the PCL Polygonal Segmentation can be found `here <http://docs.pointclouds.org/1.7.0/classpcl_1_1_convex_hull.html>`__ and `here <http://docs.pointclouds.org/trunk/classpcl_1_1_extract_polygonal_prism_data.html>`__. The goal in this sub-task is to remove the points that correspond to a known object (e.g. the box we detected earlier). This particular filter is applied to the entire point cloud (original sensor data), but only after we've already completed the processing steps to identify the position/orientation of the box.

   Within perception_node.cpp, add ``#include <tf_conversions/tf_eigen.h>`` and find section

   .. code-block:: c++

      /* ========================================
       * Fill Code: POLYGONAL SEGMENTATION (OPTIONAL)
       * ========================================*/

   Set the input cloud:

   .. code-block:: c++

      pcl::PointCloud<pcl::PointXYZ>::Ptr sensor_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>(cloud));
      pcl::PointCloud<pcl::PointXYZ>::Ptr prism_filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr pick_surface_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

   Declare instance of filter:

   .. code-block:: c++

      pcl::ExtractPolygonalPrismData<pcl::PointXYZ> prism;

   Set extraction indices:

   .. code-block:: c++

      pcl::ExtractIndices<pcl::PointXYZ> extract_ind;

   Set input and output:

   .. code-block:: c++

      prism.setInputCloud(sensor_cloud_ptr);
      pcl::PointIndices::Ptr pt_inliers (new pcl::PointIndices());

   Set parameters - looking at documentation, ExtractPolygonalPrismData uses a pointcloud defining the polygon vertices as its input.

   .. code-block:: c++

      // create prism surface
      double box_length=0.25;
      double box_width=0.25;
      pick_surface_cloud_ptr->width = 5;
      pick_surface_cloud_ptr->height = 1;
      pick_surface_cloud_ptr->points.resize(5);

      pick_surface_cloud_ptr->points[0].x = 0.5f*box_length;
      pick_surface_cloud_ptr->points[0].y = 0.5f*box_width;
      pick_surface_cloud_ptr->points[0].z = 0;

      pick_surface_cloud_ptr->points[1].x = -0.5f*box_length;
      pick_surface_cloud_ptr->points[1].y = 0.5f*box_width;
      pick_surface_cloud_ptr->points[1].z = 0;

      pick_surface_cloud_ptr->points[2].x = -0.5f*box_length;
      pick_surface_cloud_ptr->points[2].y = -0.5f*box_width;
      pick_surface_cloud_ptr->points[2].z = 0;

      pick_surface_cloud_ptr->points[3].x = 0.5f*box_length;
      pick_surface_cloud_ptr->points[3].y = -0.5f*box_width;
      pick_surface_cloud_ptr->points[3].z = 0;

      pick_surface_cloud_ptr->points[4].x = 0.5f*box_length;
      pick_surface_cloud_ptr->points[4].y = 0.5f*box_width;
      pick_surface_cloud_ptr->points[4].z = 0;

      Eigen::Affine3d eigen3d;
      tf::transformTFToEigen(part_transform,eigen3d);
      pcl::transformPointCloud(*pick_surface_cloud_ptr,*pick_surface_cloud_ptr,Eigen::Affine3f(eigen3d));

      prism.setInputPlanarHull( pick_surface_cloud_ptr);
      prism.setHeightLimits(-10,10);

   Segment:

   .. code-block:: c++

      prism.segment(*pt_inliers);

   Remember that after you use the segmentation algorithme that you either want to include or exclude the segmented points using an index extraction.

   Set input:

   .. code-block:: c++

      extract_ind.setInputCloud(sensor_cloud_ptr);
      extract_ind.setIndices(pt_inliers);

   This time, we invert the index extraction, so that we remove points inside the filter and keep points outside the filter.

   .. code-block:: c++

      extract_ind.setNegative(true);

   Filter:

   .. code-block:: c++

      extract_ind.filter(*prism_filtered_cloud);

#. Find the ``pcl::toROSMsg`` call where the ``pc2_cloud`` is populated.  This is the point cloud that is published to RViz display.  Replace the current cloud with the new filtered results (``*prism_filtered_cloud``).

#. Compile and run as before.

    .. image:: /_static/prism_filtered_cloud.png

   .. Note:: Notice that the target box has been removed from the point cloud display.


ポリゴン・セグメンテーションの作成
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

干渉検出にセンサ情報を使用する場合，
これらのオブジェクトとの干渉を避けるために，
「既知の」オブジェクトをシーンから除外する必要があることがあります．
MoveIt! にはロボット自身のジオメトリを
「自己干渉」フィルタリング機能としてマスクするためのメソッドが含まれています．

本項目では PCL のポリゴン・セグメンテーション・フィルタリングを使用して
同様のことを行う方法を提示します．

#. コードの変更

   この方法は3番目のサブタスクの平面セグメンテーションに似ていますが，
   平面を分割する代わりにプリズムを分割して削除することができます．

   PCL ポリゴン・セグメンテーションに関するドキュメントは
   `Point Cloud Library (PCL) 1.7 <http://docs.pointclouds.org/1.7.0/classpcl_1_1_convex_hull.html>`_ と
   `Point Cloud Library (PCL) 1.9 <http://docs.pointclouds.org/trunk/classpcl_1_1_extract_polygonal_prism_data.html>`_ にあります．

   本サブタスクの目標は，
   既知のオブジェクト（例えば先に検出した箱）に対応するポイントを削除することです．
   この特殊なフィルタはポイントクラウド全体（元のセンサデータ）に適用しますが，
   箱の位置・方向を特定するための処理手順を既に完了した後にだけ適用されます．

   perception_node.cpp に ``#include <tf_conversions/tf_eigen.h>`` を追加してから，
   次のセクションを探してください．

   .. code-block:: c++

      /* ========================================
       * Fill Code: POLYGONAL SEGMENTATION (OPTIONAL)
       * ========================================*/

   ポイントクラウド入力の設定:

   .. code-block:: c++

      pcl::PointCloud<pcl::PointXYZ>::Ptr sensor_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>(cloud));
      pcl::PointCloud<pcl::PointXYZ>::Ptr prism_filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr pick_surface_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

   フィルタ・インスタンスの宣言:

   .. code-block:: c++

      pcl::ExtractPolygonalPrismData<pcl::PointXYZ> prism;

   抽出インデックスの設定:

   .. code-block:: c++

      pcl::ExtractIndices<pcl::PointXYZ> extract_ind;

   入出力の設定:

   .. code-block:: c++

      prism.setInputCloud(sensor_cloud_ptr);
      pcl::PointIndices::Ptr pt_inliers (new pcl::PointIndices());

   パラメータの設定:

   ドキュメントを見ると，
   ExtractPolygonalPrismData は入力としてポリゴン頂点を定義するポイントクラウドを使用します．

   .. code-block:: c++

      // create prism surface
      double box_length=0.25;
      double box_width=0.25;
      pick_surface_cloud_ptr->width = 5;
      pick_surface_cloud_ptr->height = 1;
      pick_surface_cloud_ptr->points.resize(5);

      pick_surface_cloud_ptr->points[0].x = 0.5f*box_length;
      pick_surface_cloud_ptr->points[0].y = 0.5f*box_width;
      pick_surface_cloud_ptr->points[0].z = 0;

      pick_surface_cloud_ptr->points[1].x = -0.5f*box_length;
      pick_surface_cloud_ptr->points[1].y = 0.5f*box_width;
      pick_surface_cloud_ptr->points[1].z = 0;

      pick_surface_cloud_ptr->points[2].x = -0.5f*box_length;
      pick_surface_cloud_ptr->points[2].y = -0.5f*box_width;
      pick_surface_cloud_ptr->points[2].z = 0;

      pick_surface_cloud_ptr->points[3].x = 0.5f*box_length;
      pick_surface_cloud_ptr->points[3].y = -0.5f*box_width;
      pick_surface_cloud_ptr->points[3].z = 0;

      pick_surface_cloud_ptr->points[4].x = 0.5f*box_length;
      pick_surface_cloud_ptr->points[4].y = 0.5f*box_width;
      pick_surface_cloud_ptr->points[4].z = 0;

      Eigen::Affine3d eigen3d;
      tf::transformTFToEigen(part_transform,eigen3d);
      pcl::transformPointCloud(*pick_surface_cloud_ptr,*pick_surface_cloud_ptr,Eigen::Affine3f(eigen3d));

      prism.setInputPlanarHull( pick_surface_cloud_ptr);
      prism.setHeightLimits(-10,10);

   分割:

   .. code-block:: c++

      prism.segment(*pt_inliers);

   セグメント化アルゴリズムを使用した後は，
   インデックス抽出を使用してセグメント化されたポイントを
   含めるか除外するかを忘れないでください．

   入力の設定:

   .. code-block:: c++

      extract_ind.setInputCloud(sensor_cloud_ptr);
      extract_ind.setIndices(pt_inliers);

   今回はインデックス抽出を逆にして，
   フィルタ内のポイントを削除し，
   フィルタ外のポイントを保持します．

   .. code-block:: c++

      extract_ind.setNegative(true);

   フィルタ:

   .. code-block:: c++

      extract_ind.filter(*prism_filtered_cloud);

#. ``pc2_cloud`` が入力されている ``pcl::toROSMsg`` 呼び出し部分を探してください．
   これは RViz ディスプレイで閲覧されるポイントクラウドです．
   現在のクラウドを
   新しいフィルタ結果の（
   ``*prism_filtered_cloud``
   ）に置き換えます．


#. いつものようにコンパイルと実行を行ってください．

   .. image:: /_static/prism_filtered_cloud.png

   .. Note:: ターゲットの箱がポイントクラウド表示から除かれていることに注目してください．


launch ファイルを書く
^^^^^^^^^^^^^^^^^^

これはフィルターメソッドではありませんが，
PCL または他の知覚メソッドを使用するときに，
異なるメソッドで使用されるパラメータの数が多くなるので便利です．

#. コードの変更・挿入

   最初のサブタスクに書いてあることを詳しく読んでいる人は，
   1つの場所にパラメータを置くことが推奨されていたことに気付いているかと思います．

   perception_node.cpp の次のセクションを探してください．

   .. code-block:: c++

      /*
       * SET UP PARAMETERS (COULD TO BE INPUT FROM LAUNCH FILE/TERMINAL)
       */

   下のパラメータの例が示すように
   理想的には特定の型（ ``std::string frame;`` ）のパラメータを宣言し，
   そのパラメータの値を代入します（ ``frame="some_name";`` ）．
   下記は設定可能なパラメータの例です．

   .. code-block:: yaml

      world_frame="kinect_link";
      camera_frame="kinect_link";
      cloud_topic="kinect/depth_registered/points";
      voxel_leaf_size=0.002f;
      x_filter_min=-2.5;
      x_filter_max=2.5;
      y_filter_min=-2.5;
      y_filter_max=2.5;
      z_filter_min=-2.5;
      z_filter_max=1.0;
      plane_max_iter=50;
      plane_dist_thresh=0.05;
      cluster_tol=0.01;
      cluster_min_size=100;
      cluster_max_size=50000;

   本サブタスクの手順により，
   パラメータを launch ファイルまたは yaml ファイルから入力できるものに変えることができます．
   チュートリアル
   `Using Parameters in roscpp <http://wiki.ros.org/roscpp_tutorials/Tutorials/Parameters>`_
   で説明されているように，"getParam" メソッドを使用することができます．
   しかし，より良い選択は
   `param <http://docs.ros.org/kinetic/api/roscpp/html/classros_1_1NodeHandle.html#aa9b23d4206216ed13b5833fb1a090f1a>`_
   メソッドを使うことです．
   param メソッドはパラメータがパラメータサーバ上にない場合にはデフォルト値を返します．
   ROS パラメータサーバもしくは launch ファイルから params を取得して，
   前回のハードコードされた値を置き換えてください．
   （ただし変数宣言は残してください！）

   .. code-block:: c++

      cloud_topic = priv_nh_.param<std::string>("cloud_topic", "kinect/depth_registered/points");
      world_frame = priv_nh_.param<std::string>("world_frame", "kinect_link");
      camera_frame = priv_nh_.param<std::string>("camera_frame", "kinect_link");
      voxel_leaf_size = param<float>("voxel_leaf_size", 0.002);
      x_filter_min = priv_nh_.param<float>("x_filter_min", -2.5);
      x_filter_max = priv_nh_.param<float>("x_filter_max",  2.5);
      y_filter_min = priv_nh_.param<float>("y_filter_min", -2.5);
      y_filter_max = priv_nh_.param<float>("y_filter_max",  2.5);
      z_filter_min = priv_nh_.param<float>("z_filter_min", -2.5);
      z_filter_max = priv_nh_.param<float>("z_filter_max",  2.5);
      plane_max_iter = priv_nh_.param<int>("plane_max_iterations", 50);
      plane_dist_thresh = priv_nh_.param<float>("plane_distance_threshold", 0.05);
      cluster_tol = priv_nh_.param<float>("cluster_tolerance", 0.01);
      cluster_min_size = priv_nh_.param<int>("cluster_min_size", 100);
      cluster_max_size = priv_nh_.param<int>("cluster_max_size", 50000);

#. launch ファイルの記述

   gedit もしくは他のテキストエディタを使用して，
   新しいファイル（ "lesson_perception/launch/processing_node.launch" ）を作成して，
   下記内容を書き入れてください．

   .. code-block:: xml

      <launch>
        <node name="processing_node" pkg="lesson_perception" type="perception_node" output="screen">
          <rosparam>
            cloud_topic: "kinect/depth_registered/points"
            world_frame: "world_frame"
            camera_frame: "kinect_link"
            voxel_leaf_size: 0.001 <!-- mm -->
            x_filter_min: -2.5 <!-- m -->
            x_filter_max: 2.5 <!-- m -->
            y_filter_min: -2.5 <!-- m -->
            y_filter_max: 2.5 <!-- m -->
            z_filter_min: -2.5 <!-- m -->
            z_filter_max: 2.5 <!-- m -->
            plane_max_iterations: 100
            plane_distance_threshold: 0.03
            cluster_tolerance: 0.01
            cluster_min_size: 250
            cluster_max_size: 500000
          </rosparam>
        </node>
      </launch>

#. コンパイル

   今回は rosrun を使用する代わりに，
   新しく作成した launch ファイルを実行して処理ノードを開始します．

   結果は以前の実行と同様に見えるはずです．
   しかし，これらの設定パラメータをずっと簡単に編集できるようになりました！
   再コンパイルは不要で，launch ファイルの値を編集してノードを再起動するだけです．
   実際のアプリケーションではこのアプローチをさらに進めて
   ノードで dynamic_reconfigure サポートを実装することができます．
   これにより RViz でリアルタイムに
   パラメータ変更の結果を確認することができるようになります．

   結果が良いようでしたら，各端末で *CTRL-C* を行い終了します．

   本節の演習はすべて終わりました！
