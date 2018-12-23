.. SphinxTest documentation master file, created by
   sphinx-quickstart on Tue Oct  3 11:09:13 2017.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

ROS Industrial (Kinetic) 演習トレーニング
===========================================

PC のセットアップ
---------------

.. toctree::
   :maxdepth: 1

   PC のセットアップ <_source/setup_JP/PC-Setup---ROS-Kinetic_JP.md>

準備
----

C++
~~~~~~~~~~~~~

.. toctree::
   :maxdepth: 1

   MIT Introduction to C++ <http://ocw.mit.edu/courses/electrical-engineering-and-computer-science/6-096-introduction-to-c-january-iap-2011/assignments/>
   Bruce Eckel Thinking in C++ <http://mindview.net/Books/TICPP/ThinkingInCPP2e.html>

Linux の基礎
~~~~~~~~~~~
:download:`スライド <_downloads/slides/ROS-I Basic Developers Training - Session 0.pdf>`

.. toctree::
   :maxdepth: 1

   演習 0.1 - Ubuntu GUI 入門 <_source/prerequisites_JP/Navigating-the-Ubuntu-GUI_JP.md>
   演習 0.2 - Linux のファイルシステム <_source/prerequisites_JP/Exploring-the-Linux-File-System_JP.md>
   演習 0.3 - ターミナルを使う <_source/prerequisites_JP/The-Linux-Terminal_JP.md>

基礎編
------

セッション 1 - ROS の概念と基礎
~~~~~~~~~~~~~~~~~~~~~~~~~~~
:download:`スライド <_downloads/slides/ROS-I Basic Developers Training - Session 1.pdf>`

.. toctree::
   :maxdepth: 1

   演習 1.0 - ROS のセットアップ <_source/session1_JP/ROS-Setup_JP.md>
   演習 1.1 - ワークスペースの作成 <_source/session1_JP/Create-Catkin-Workspace_JP.md>
   演習 1.2 - パッケージのインストール <_source/session1_JP/Installing-Existing-Packages_JP.md>
   演習 1.3 - パッケージとノード <_source/session1_JP/Creating-a-ROS-Package-and-Node_JP.md>
   演習 1.4 - トピックとメッセージ <_source/session1_JP/Topics-and-Messages_JP.md>

セッション 2 - 基本的な ROS の使用法
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
:download:`スライド <_downloads/slides/ROS-I Basic Developers Training - Session 2.pdf>`

.. toctree::
   :maxdepth: 1

   演習 2.0 - サービス <_source/session2_JP/Services_JP.md>
   演習 2.1 - アクション <_source/session2_JP/Actions_JP.md>
   演習 2.2 - launch ファイル <_source/session2_JP/Launch-Files_JP.md>
   演習 2.3 - パラメータ <_source/session2_JP/Parameters_JP.md>

セッション 3 - マニュピレータの制御
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
:download:`スライド <_downloads/slides/ROS-I Basic Developers Training - Session 3.pdf>`

.. toctree::
   :maxdepth: 1

   演習 3.0 - URDF 入門 <_source/session3_JP/Intro-to-URDF_JP.md>
   演習 3.1 - 作業セルの XACRO <_source/session3_JP/Workcell-XACRO_JP.md>
   演習 3.2 - TF を用いた座標変換 <_source/session3_JP/Coordinate-Transforms-using-TF_JP.md>
   演習 3.3 - MoveIt! パッケージのビルド <_source/session3_JP/Build-a-Moveit!-Package_JP.md>
   演習 3.4 - RViz 上での動作計画 <_source/session3_JP/Motion-Planning-RVIZ_JP.md>

セッション 4 - Descartes パッケージと認識
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
:download:`スライド <_downloads/slides/ROS-I Basic Developers Training - Session 4.pdf>`

.. toctree::
   :maxdepth: 1

   演習 4.0 - C++ 上での動作計画 <_source/session4_JP/Motion-Planning-CPP_JP.md>
   演習 4.1 - 直交座標系動作軌道計画入門 <_source/session4_JP/Descartes-Path-Planning_JP.md>
   演習 4.2 - 知覚・認識系入門 <_source/session4_JP/Introduction-to-Perception_JP.md>

応用デモ 1 - センサ認識を用いたマニュピレーション
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. toctree::
   :maxdepth: 1

   デモ 1 - センサ認識を用いたマニュピレーション <_source/demo1_JP/index_JP.rst>

応用デモ 2 - Descartes パッケージによる動作計画と実行
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. toctree::
   :maxdepth: 1

   デモ 2 - Descartes パッケージによる動作計画と実行 <_source/demo2_JP/index_JP.rst>

応用編
-----

セッション 5 - 軌道生成と知覚パイプラインの作成
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
:download:`スライド <_downloads/slides/ROS-I Advanced Developers Training - Session 5.pdf>`

.. toctree::
   :maxdepth: 1

   演習 5.0 - 高度な直交座標系動作軌道計画 <_source/session5_JP/Advanced-Descartes-Path-Planning_JP.md>
   演習 5.1 - 知覚パイプラインの構築 <_source/session5_JP/Building-a-Perception-Pipeline_JP.md>
   演習 5.2 - STOMP 入門 <_source/session5_JP/Introduction-to-STOMP_JP.md>
   演習 5.3 - Python のためのシンプルな PCL インタフェースの構築 <_source/session5_JP/Simple-PCL-Interface-for-Python_JP.rst>
   演習 5.4 - OpenCV 画像処理（ Pyhton ） <_source/session5_JP/OpenCV-in-Python_JP.md>

セッション 6 - ドキュメント生成 / ユニットテスト / ROS ユーティリティ / デバッグ
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
:download:`スライド <_downloads/slides/ROS-I Advanced Developers Training - Session 5.pdf>`

.. toctree::
   :maxdepth: 1

   演習 6.0 - ドキュメント生成 <_source/session6_JP/Documentation-Generation_JP.md>
   演習 6.1 - ユニットテスト <_source/session6_JP/Unit-Testing_JP.rst>
   演習 6.2 - rqt 分析ツール <_source/session6_JP/Using-rqt-tools-for-analysis_JP.md>
   演習 6.3 - ROS スタイルガイドと ros_lint <_source/session6_JP/Style-Guide-and-ros_lint_JP.md>
   演習 6.4 - ROS と Docker / Amazon Web Service <_source/session6_JP/Docker-AWS_JP.md>
