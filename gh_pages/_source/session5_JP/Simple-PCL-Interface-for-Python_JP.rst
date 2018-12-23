コードは industrial_training リポジトリの gh_pages フォルダにあります．
kinetic ブランチを使用してください．

Python のためのシンプルな PCL インタフェースの構築
============================================

本演習では知覚パイプラインを構築するための適切なコードを記入します．
最終目標は ROS と Python 間の機能性を見るために，
ポイントクラウドのフィルタリング処理を作成します．


新規ワークスペースの準備
--------------------

本演習はこれまでの Plan-N-Scan 演習とは独立した内容ですので，
新しく catkin ワークスペースを作成します．

#. 既存の catkin ワークスペース環境を自動的にセットアップする設定を止めます．

   #. ``gedit ~/.bashrc``

   #. ``source ~/catkin_ws/devel/setup.bash`` している
      最終行を ``#`` でコメントアウトしてください．

   .. code-block:: bash

            source /opt/ros/kinetic/setup.bash
            # source ~/catkin_ws/devel/setup.bash

#. ワークスペース・レイアウト／ファイルのテンプレートをコピーしてください．

   .. code-block:: bash

            cp -r ~/industrial_training/exercises/python-pcl_ws ~
            cd ~/python-pcl_ws/

#. ワークスペースの初期化とビルドを行います．

   .. code-block:: bash

            catkin init
            catkin build


#. ワークスペースのセットアップを行います．

   .. code-block:: bash

            source ~/python-pcl_ws/devel/setup.bash

#. ポイントクラウドファイルをダンロードしてホームディレクトリ（~）に置いてください．

#. 新しいワークスペースを QTCreator IDE にインポートしてください．

   - In QTCreator: File -> New Project -> Import -> Import ROS Workspace -> ~/python-pcl_ws


既存コードの確認
--------------

ROS ノードの基盤部分の大半は既に完成していますので，
本演習の焦点は知覚アルゴリズムとそのパイプラインです．
`CMakelists.txt` と `package.xml` と完成したソースファイルも提供されています．
ソースコードをそのままビルドすることはできますがエラーが発生します．

ここでは提供されているソースコードを見ていきます．
`py_perception_node.cpp` ファイルを参照してください．

本チュートリアルはトレーニング
`演習 5.1 知覚パイプラインの構築 <http://ros-industrial.github.io/industrial_training/_source/session5/Building-a-Perception-Pipeline_JP.html>`__
の内容を Python 向けに改良したもので，
演習 5.1 の C++ コードは既にセットアップ済みとして進めます．
何か分からない内容があれば「演習 5.1 知覚パイプラインの構築」に戻って確認してください．

それでは `py_perception_node.cpp` ファイルを開いて各フィルタ機能について見ていきます．


Python パッケージの作成
^^^^^^^^^^^^^^^^^^^^

いくつかのフィルタを C++ 関数に転換してあるので，
Python ノードから呼び出す準備は整っています．

   PyCharm の Community Edition を
   インストールしていない場合はインストールしてください．
   この IDE には，編集するために必要なパーサがあります．
   これがなければ Qt で構文エラーを確認することはできません．

#. ターミナルで src フォルダに移動してください．
   pyrhon-pcl_ws 内に新しいパッケージを作成します．

   .. code-block:: bash

            cd ~/python-pcl_ws/src/
            catkin_create_pkg filter_call rospy roscpp perception_msgs

#. パッケージが新しく作成できたことを確認します．

   .. code-block:: bash

            ls

本演習ではカスタム・メッセージを作成しないため，
'perception_msgs' は使用しません．
それは更に学習が進んだ時点で学んでください．
カスタム・メッセージを実装する方法を含め，より深い説明が必要な場合は
ここに良い
`MIT Resource <http://duckietown.mit.edu/media/pdfs/1rpRisFoCYUm0XT78j-nAYidlh-cDtLCdEbIaBCnx9ew.pdf> `__
がありあす．

#. *CMakeLists.txt* を開いてください．
   Pycharm や Qt で（もしくは nano，emacs，vim，subilme でも）開くことができます．

   下記内容の行（23行目付近）のコメントアウトを外して，ファイルを保存してください．

   .. code-block:: bash

            catkin_python_setup()


setup.py の作成
^^^^^^^^^^^^^^

`setup.py` ファイルは Python モジュールを
ワークスペース全体とその後のパッケージで利用できるようにします．
デフォルトでは `catkin_create_pkg` コマンドでは作成されません．

#. ターミナルで次のように入力してください．

   .. code-block:: bash

            gedit filter_call/setup.py

#. 下記内容を `setup.py` にコピー＆ペーストしてください．

   .. code-block:: python

            ## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
            from distutils.core import setup
            from catkin_pkg.python_setup import generate_distutils_setup
            # fetch values from package.xml
            setup_args = generate_distutils_setup(
            packages=[''],
            package_dir={'': 'include'},
            )
            setup(**setup_args)

   ``packages = [ . . . ],`` の内容を変更して，
   *include* フォルダ内のフォルダ名の文字列リストを追加します．
   慣習的にはパッケージと同じ名前か ``filter_call`` です．
   これは ``filter_call/include/filter_call`` を
   ワークスペース全体で利用可能な Python モジュールとして設定します．

#. ファイルを保存して閉じてください．

    このフォルダに他の Python スクリプトがアクセスできるようにするには，
    ``__init__.py`` ファイルが存在しなければなりません。

#. ``__init__.py`` ファイルの作成

   ターミナルで次のように入力してファイルを作成します．

   .. code-block:: bash

            touch filter_call/include/filter_call/__init__.py


ポイントクラウドのパブリッシュ
^^^^^^^^^^^^^^^^^^^^^^^^^

ポイントクラウドをフィルタリングする ROS C++ ノードは既に作成されていて，
各フィルタ操作のためのサービスリクエストを Python ノードで実行した時に
新しいポイントクラウドが作成されます．

まず Python をサポートする方法で公開するように C++ コードを修正します．
C++ のコードが完成すれば，
必要があるのは Python スクリプトを書いて結果を RViz で見ることだけです．


Voxel フィルタの実装
^^^^^^^^^^^^^^^^^^

#. *py_perception_node.cpp* 内の
   ``filterCallBack`` という名前のブーリアン関数（ ``main`` 関数の真上 ）
   のコメントアウトを外して実行できるようにします．
   これは後続のフィルタリング操作を実行するために
   Python クライアントによって利用されるサービスになります．

   .. code-block:: c++

        bool filterCallback(lesson_perception::FilterCloud::Request& request,
                            lesson_perception::FilterCloud::Response& response)
        {
          pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
          pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);

          if (request.pcdfilename.empty())
          {
            pcl::fromROSMsg(request.input_cloud, *cloud);
            ROS_INFO_STREAM("cloud size: " << cloud->size());
          }
          else
          {
            pcl::io::loadPCDFile(request.pcdfilename, *cloud);
          }

          if (cloud->empty())
          {
            ROS_ERROR("input cloud empty");
            response.success = false;
            return false;
          }

          switch (request.operation)
          {

            case lesson_perception::FilterCloud::Request::VOXELGRID :
            {
              filtered_cloud = voxelGrid(cloud, 0.01);
              break;
            }
            default :
            {
              ROS_ERROR("No valid request found");
              return false;
            }

           }

        /*
         * SETUP RESPONSE
         */
          pcl::toROSMsg(*filtered_cloud, response.output_cloud);
          response.output_cloud.header=request.input_cloud.header;
          response.output_cloud.header.frame_id="kinect_link";
          response.success = true;
          return true;
        }


#. ``main`` 関数内の240行目付近の下記内容の行の
   コメントアウトを外してから保存し，ビルドします．

   .. code-block:: c++

            priv_nh_.param<double>("leaf_size", leaf_size_, 0.0f);

#. これでフィルタリングのためのフレームワークが完成しました．
   ターミナルを開いて filter_call ディレクトリにいることを確認して，
   そこで *scripts* フォルダを作成してください．

   .. code-block:: bash

            mkdir scripts

#. Pycharm が開いている場合は保存して閉じます．
   端末から Pycharm を開いて C++ ノードが正しく見て取れるかを確認します．．
   Pycharm を開くためには Pycharm のインストールディレクトリに移動します．

   .. code-block:: bash

            cd ~/pycharm-community-2018.1.3/bin
            ./pycharm.sh

   Pycharm を開いたら フォルダ *scripts* を探して右クリックし，新しい Python ファイルを作成します．
   それに *filter_call.py* というファイル名をつけます．

#. 必要なライブラリをインポートするには，
   *filter_call.py* に次のコードをコピー＆ペーストしてください．

   .. code-block:: python

            #!/usr/bin/env python

            import rospy
            import lesson_perception.srv
            from sensor_msgs.msg import PointCloud2

#. このファイルが実行されたときに Python ノードを実行するための ``if`` 文を作成します．
   次のように初期化します．

   .. code-block:: python

        if __name__ == '__main__':
            try:

            except Exception as e:
                print("Service call failed: %s" % str(e))


#. 次のように ``rospy.spin()`` を ``try`` ブロック内に設置します．

   .. code-block:: python

        if __name__ == '__main__':
            try:
                rospy.spin()
            except Exception as e:
                print("Service call failed: %s" % str(e))


#. ``try`` ブロック内に次の内容をコピー＆ペーストしてください．

   .. code-block:: python

        # =======================
        # VOXEL GRID FILTER
        # =======================

        srvp = rospy.ServiceProxy('filter_cloud', lesson_perception.srv.FilterCloud)
        req = lesson_perception.srv.FilterCloudRequest()
        req.pcdfilename = rospy.get_param('~pcdfilename', '')
        req.operation = lesson_perception.srv.FilterCloudRequest.VOXELGRID
        # FROM THE SERVICE, ASSIGN POINTS
        req.input_cloud = PointCloud2()

        # ERROR HANDLING
        if req.pcdfilename == '':
            raise Exception('No file parameter found')

        # PACKAGE THE FILTERED POINTCLOUD2 TO BE PUBLISHED
        res_voxel = srvp(req)
        print('response received')
        if not res_voxel.success:
            raise Exception('Unsuccessful voxel grid filter operation')

        # PUBLISH VOXEL FILTERED POINTCLOUD2
        pub = rospy.Publisher('/perception_voxelGrid', PointCloud2, queue_size=1, latch=True)
        pub.publish(res_voxel.output_cloud)
        print("published: voxel grid filter response")



#. Python ノードを初期化して C++ ノードのサービスを待つために，
   ``try`` ブロックの上に（ ``if`` ステートメント内に ）次の行を貼り付けます．

   .. code-block:: python

            rospy.init_node('filter_cloud', anonymous=True)
            rospy.wait_for_service('filter_cloud')

#. Python ファイルを実行可能なものとするために，
   ターミナルから下記コマンドを入力してファイルに実行権限を付与します．

   .. code-block:: bash

            chmod +x filter_call/scripts/filter_call.py


結果を見る
^^^^^^^^

#. ターミナルから次を実行してください．

   .. code-block:: bash

            roscore

#. 新しいターミナルで ROS 環境設定を行った後，
   C++ のフィルタサービス・ノードを実行します．

   .. code-block:: bash

            rosrun lesson_perception py_perception_node

#. 新しいターミナルで ROS 環境設定を行った後，
   Python のサービスコール・ノードを実行します．

   ファイルパスが違うことがありますので注意してください．

   .. code-block:: bash

            rosrun filter_call filter_call.py _pcdfilename:="/home/ros-industrial/catkin_ws/table.pcd"

#. 新しいターミナルで ROS 環境設定を行った後，
   RViz を起動します．

   .. code-block:: bash

            rosrun rviz rviz

#. RViz に新しく PointCloud2 を追加してください．

#. RViz の Global Options の Fixed Frame を kinect_link に変更してください．
   そして PointCloud2 のトピックに '/perception_voxelGrid' を選択してください．

   .. Note:: PointCloud2 のチェックボックスを切手から再び入れる必要があることがあります．


パススルーフィルタの実装
^^^^^^^^^^^^^^^^^^^^

#. ``lesson_perception`` パッケージの *py_perception_node.cpp* の ``main`` 関数内の
   次の2つの行のコメントを外し，
   また28行目と29行目付近のそれらの変数の宣言部分のコメントも解除してください．

   .. code-block:: c++

            priv_nh_.param<double>("passThrough_max", passThrough_max_, 1.0f);
            priv_nh_.param<double>("passThrough_min", passThrough_min_, -1.0f);

#. switch の内容を次のように変更してください．

   .. code-block:: bash

        switch (request.operation)
        {

          case lesson_perception::FilterCloud::Request::VOXELGRID :
          {
            filtered_cloud = voxelGrid(cloud, 0.01);
            break;
          }
          case lesson_perception::FilterCloud::Request::PASSTHROUGH :
          {
            filtered_cloud = passThrough(cloud);
            break;
          }
          default :
          {
            ROS_ERROR("No valid request found");
            return false;
          }

        }

#. 保存してビルドしてください．

   **Python コードの編集**

#. Python ノードのファイルを開き，
   次のコードをボクセルグリッドの後，``rospy.spin()`` の前に貼り付けます．
   インデントを崩さないように注意してください．

   .. code-block:: python

        # =======================
        # PASSTHROUGH FILTER
        # =======================

        srvp = rospy.ServiceProxy('filter_cloud', lesson_perception.srv.FilterCloud)
        req = lesson_perception.srv.FilterCloudRequest()
        req.pcdfilename = ''
        req.operation = lesson_perception.srv.FilterCloudRequest.PASSTHROUGH
        # FROM THE SERVICE, ASSIGN POINTS
        req.input_cloud = res_voxel.output_cloud

        # PACKAGE THE FILTERED POINTCLOUD2 TO BE PUBLISHED
        res_pass = srvp(req)
        print('response received')
        if not res_voxel.success:
            raise Exception('Unsuccessful pass through filter operation')

        # PUBLISH PASSTHROUGH FILTERED POINTCLOUD2
        pub = rospy.Publisher('/perception_passThrough', PointCloud2, queue_size=1, latch=True)
        pub.publish(res_pass.output_cloud)
        print("published: pass through filter response")

#. 保存後に端末から実行し Voxel フィルタで説明した手順を繰り返します．

   RViz で ``/kinect/depth_registered/points``（元のカメラデータ）を基にした PointCloud2 と
   ``perception_passThrough``（ここで行った処理データ）の PointCloud2 を比べてください．

   ここでのフィルタ処理でポイントクラウドの一部が「切り取り」されているはずです．

   パススルーフィルタの結果が良いようでしたら Ctrl+C でノードを停止してください．
   他のターミナルやノードを停止する必要はありません．


平面のセグメンテーション
^^^^^^^^^^^^^^^^^^^^

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

#. py_perception_node.cpp の ``main`` 関数内の
   下記2行とともに，それらの変数の宣言部分もコメントアウトを外してください．

   .. code-block:: c++

            priv_nh_.param<double>("maxIterations", maxIterations_, 200.0f);
            priv_nh_.param<double>("distThreshold", distThreshold_, 0.01f);


#. ``filterCallback`` 内の switch を次のように変更してください．

   .. code-block:: c++

        switch (request.operation)
        {

          case lesson_perception::FilterCloud::Request::VOXELGRID :
          {
            filtered_cloud = voxelGrid(cloud, 0.01);
            break;
          }
          case lesson_perception::FilterCloud::Request::PASSTHROUGH :
          {
            filtered_cloud = passThrough(cloud);
            break;
          }
          case lesson_perception::FilterCloud::Request::PLANESEGMENTATION :
          {
            filtered_cloud = planeSegmentation(cloud);
            break;
          }
          default :
          {
            ROS_ERROR("No valid request found");
            return false;
          }

        }


#. 保存してビルドしてください．

   **Python コードの編集**

#. 次のコードを filter_call.py 内のパススルーフィルタ部分の後にコピー＆ペーストしてください．
   インデントが崩れないように注意してください．

   .. code-block:: python

        # =======================
        # PLANE SEGMENTATION
        # =======================

        srvp = rospy.ServiceProxy('filter_cloud', lesson_perception.srv.FilterCloud)
        req = lesson_perception.srv.FilterCloudRequest()
        req.pcdfilename = ''
        req.operation = lesson_perception.srv.FilterCloudRequest.PLANESEGMENTATION
        # FROM THE SERVICE, ASSIGN POINTS
        req.input_cloud = res_pass.output_cloud

        # PACKAGE THE FILTERED POINTCLOUD2 TO BE PUBLISHED
        res_seg = srvp(req)
        print('response received')
        if not res_voxel.success:
            raise Exception('Unsuccessful plane segmentation operation')

        # PUBLISH PLANESEGMENTATION FILTERED POINTCLOUD2
        pub = rospy.Publisher('/perception_planeSegmentation', PointCloud2, queue_size=1, latch=True)
        pub.publish(res_seg.output_cloud)
        print("published: plane segmentation filter response")


#. 保存後に端末から実行し Voxel フィルタで説明した手順を繰り返します．

   RViz で ``/kinect/depth_registered/points``（元のカメラデータ）を基にした PointCloud2 と
   ``perception_planeSegmentation``（ここで行った処理データ）の PointCloud2 を比べてください．
   処理した結果ではテーブル平面の上にある点だけが残っているはずです．

   #. 結果の表示が終了したら，"setMaxIterations" および "setDistanceThreshold" の値を変更して，
      平面適合点／不適合点としてデータをどれくらい厳密に制御できるかを試して，
      その結果を表示してみてください．
      ``MaxIterations = 100`` と ``DistanceThreshold = 0.010`` の値を試してみてください．

   #. 平面分割の結果を確認できたら，Ctrl+C キーを使用してノードを停止してください．
      他のターミナルを閉じたり他のノードを停止したりする必要はありません．


ユークリッドクラスタの抽出
^^^^^^^^^^^^^^^^^^^^^^

この方法は複数のオブジェクトがあるすべてのアプリケーションにおいて有用です．
またこれは複雑な PCL メソッドでもあります．
詳細な例は
`PCL Euclidean Cluster Extration Tutorial <http://pointclouds.org/documentation/tutorials/cluster_extraction.php#cluster-extraction>`_
にあります．

#. py_perception_node.cpp の ``main`` 関数内の
   下記行とともに，それらの変数の宣言部分もコメントアウトを外してください．

   .. code-block:: c++

            priv_nh_.param<double>("clustTol", clustTol_, 0.01f);
            priv_nh_.param<double>("clustMax", clustMax_, 10000.0);
            priv_nh_.param<double>("clustMin", clustMin_, 300.0f);


#. ``filterCallback`` 内の switch を次のように変更してください．

   .. code-block:: c++

        switch (request.operation)
        {

          case lesson_perception::FilterCloud::Request::VOXELGRID :
          {
            filtered_cloud = voxelGrid(cloud, 0.01);
            break;
          }
          case lesson_perception::FilterCloud::Request::PASSTHROUGH :
          {
            filtered_cloud = passThrough(cloud);
            break;
          }
          case lesson_perception::FilterCloud::Request::PLANESEGMENTATION :
          {
            filtered_cloud = planeSegmentation(cloud);
            break;
          }
          case lesson_perception::FilterCloud::Request::CLUSTEREXTRACTION :
          {
            std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> temp =clusterExtraction(cloud);
            if (temp.size()>0)
            {
              filtered_cloud = temp[0];
            }
            break;
          }
          default :
          {
            ROS_ERROR("No valid request found");
            return false;
          }

        }


#. 保存してビルドしてください．

   **Python コードの編集**

#. 次のコードを filter_call.py 内の平面セグメンテーション部分の後にコピー＆ペーストしてください．
   インデントが崩れないように注意してください．

   .. code-block:: python

        # =======================
        # CLUSTER EXTRACTION
        # =======================

        srvp = rospy.ServiceProxy('filter_cloud', lesson_perception.srv.FilterCloud)
        req = lesson_perception.srv.FilterCloudRequest()
        req.pcdfilename = ''
        req.operation = lesson_perception.srv.FilterCloudRequest.CLUSTEREXTRACTION
        # FROM THE SERVICE, ASSIGN POINTS
        req.input_cloud = res_seg.output_cloud

        # PACKAGE THE FILTERED POINTCLOUD2 TO BE PUBLISHED
        res_cluster = srvp(req)
        print('response received')
        if not res_voxel.success:
            raise Exception('Unsuccessful cluster extraction operation')

        # PUBLISH CLUSTEREXTRACTION FILTERED POINTCLOUD2
        pub = rospy.Publisher('/perception_clusterExtraction', PointCloud2, queue_size=1, latch=True)
        pub.publish(res_cluster.output_cloud)
        print("published: cluster extraction filter response")


#. 保存後に端末から実行し Voxel フィルタで説明した手順を繰り返します．

   #. クラスタ抽出の結果が良いようでしたら，Ctrl+C キーを使用してノードを停止してください．
      本チュートリアルは以上ですので，終えるようでしたら他のターミナルのノードも停止してください．


将来的な学習
^^^^^^^^^^

`演習 5.1 <http://ros-industrial.github.io/industrial_training/_source/session5/Building-a-Perception-Pipeline_JP.html>`__
を呼び出し可能な関数に変換し，
より良くフィルタリング処理できるようにすることをお勧めします．

本演習では単純化のために
各フィルタリングインスタンスごとに Python コードを繰り返しました．
大量のコードを繰り返す代わりに
パブリッシングを処理するためのループを作成することをお勧めします．
また，デフォルトのものを使うだけではなく，
パラメータ操作の全機能を活用することもでき，
それらを Python から設定することもできます．
これらの関数呼び出しを作成する場合，
ここで説明されたもの以外にもいくつかのフィルタリング操作があります．
