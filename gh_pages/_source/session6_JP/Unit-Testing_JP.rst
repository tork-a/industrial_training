ユニットテスト
============

この演習ではユニットテストを myworkcell_core パッケージに記述します．


モチベーション
------------

ROS の Scan-N-Plan アプリケーションを完成し，文書化もしました．
プログラムをテストして意図したとおりに動作することを確認します．


情報とリソース
------------

* `Google Test <https://github.com/google/googletest/blob/master/googletest/docs/primer.md>`__
  : C++ XUnit test framework

* `rostest <http://wiki.ros.org/rostest>`__
  : ROS wrapper for XUnit test framework

* `catkin testing <http://catkin-tools.readthedocs.io/en/latest/verbs/catkin_build.html?highlight=run_tests#building-and-running-tests>`__
  : Building and running tests with catkin


演習問題
-------

Scan-N-Plan プログラムを完成して文書化しました．
ビルドした後にプログラムが意図したとおりに動作することを確認するために，
テストフレームワークを作成します．
ユニットテストでは
コードが意図したとおりに実行されることを確認することに加えて
新しいコードが意図しない機能変更を生じるか否かを
簡単に確認することが可能になります．

目標はユニットテストのフレームワークを作成して，
いくつかのテストを書くことです．


ガイダンス
---------

ユニットテスト・フレームワークの作成
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

#. `test` フォルダを myworkcell_core/src の下に作成します．
   そしてワークスペース・ディレクトリで次のことを行ってください．

   .. code-block:: bash

	       catkin build
	       source devel/setup.bash
	       roscd myworkcell_core
	       mkdir src/test

#. utest.cpp というファイルを `myworkcell_core/src/test` フォルダ内に作成します．

   .. code-block:: bash

	       touch src/test/utest.cpp

#. utest.cpp を QT などで編集して ros と gtest をインクルードします．

   .. code-block:: c++

	       #include <ros/ros.h>
	       #include <gtest/gtest.h>

#. 実行すると true を返すダミーテストを作成します．
   これは後でフレームワークをテストし，より有用なテストに置き換えます．

   .. code-block:: c++

            TEST(TestSuite, myworkcell_core_framework)
            {
              ASSERT_TRUE(true);
            }

#. 次に一般的な main 関数を書き込みます．

   この main 関数は後で記述するユニットテストを実行します．

   .. code-block:: c++

            int main(int argc, char **argv)
            {
              testing::InitGoogleTest(&argc, argv);
              return RUN_ALL_TESTS();
            }

#. myworkcell_core の CMakeLists.txt を編集して，
   u_test.cppファイルをビルドします．

   CMakeLists.txt に次を追加してください．

   .. code-block:: cmake

            if(CATKIN_ENABLE_TESTING)
              find_package(rostest REQUIRED)
              add_rostest_gtest(utest_node test/utest_launch.test src/test/utest.cpp)
              target_link_libraries(utest_node ${catkin_LIBRARIES})
            endif()

#. myworkcell_core の下に test フォルダを作成してください．

   .. code-block:: bash

            mkdir test

#. テスト launch ファイルを作成します．

   .. code-block:: bash

	       touch test/utest_launch.test

#. utest_launch.test ファイルを QT などで編集して，
   次のコードを書き入れてください．

   .. code-block:: xml

            <?xml version="1.0"?>
            <launch>
                <node pkg="fake_ar_publisher" type="fake_ar_publisher_node" name="fake_ar_publisher"/>
                <test test-name="unit_test_node" pkg="myworkcell_core" type="utest_node"/>
            </launch>

#. ビルドして，フレームワークをテストします．

   .. code-block:: bash

	       catkin run_tests myworkcell_core

   コンソール出力では次のように表示されるはずです．
   （ 多くのビルドメッセージの中に埋もれています．）

   .. code-block:: bash

            [ROSTEST]-----------------------------------------------------------------------

            [myworkcell_core.rosunit-unit_test_node/myworkcell_core_framework][passed]

            SUMMARY
             * RESULT: SUCCESS
             * TESTS: 1
             * ERRORS: 0
             * FAILURES: 0

   これはフレームワークが機能し，ユニットテストを追加することができる状態であることを意味します．

   .. Note:: 上記の launch ファイルを使用して，
             コマンドラインから直接テストを実行することもできます．
             ``rostest myworkcell_core utest_launch.test``
             テストファイルは通常の ``catkin build`` コマンドを使って作られていないので，
             代わりに ``catkin run_tests myworkcell_core`` を使ってください．


ストック・パブリッシャ・テストの追加
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

#. rostest パッケージには基本的なトピックの特性を検査するためのいくつかのツールがあります．
   `hztest <http://wiki.ros.org/rostest/Nodes#hztest>`__ や
   `paramtest <http://wiki.ros.org/rostest/Nodes# paramtest>`__ ，
   `publishtest <http://wiki.ros.org/rostest/Nodes#publishtest>`__ です．
   ``fake_ar_publisher`` ノードが
   期待するトピックを出力していることを検証するための
   基本的なテストをいくつか追加します。

#. 次のテストの記述を utest_launch.test ファイルに追加してください．

   .. code-block:: xml

            <test name="publishtest" test-name="publishtest" pkg="rostest" type="publishtest">
                <rosparam>
                  topics:
                    - name: "/ar_pose_marker"
                      timeout: 10
                      negative: False
                    - name: "/ar_pose_visual"
                      timeout: 10
                      negative: False
                </rosparam>
            </test>

#. テストを実行します．

   .. code-block:: bash

           catkin run_tests myworkcell_core

   次のような表示がされるはずです．

   .. code-block:: bash

           Summary: 2 tests, 0 errors, 0 failures


特定のユニットテストの記述
^^^^^^^^^^^^^^^^^^^^^^

#. fake_ar_publisher パッケージから取得したメッセージをテストしたいので，
   関連するヘッダファイルを（ ``utest.cpp`` に ）インクルードしてください．

   .. code-block:: c++

	       #include <fake_ar_publisher/ARMarker.h>

#. グローバル変数の宣言

   .. code-block:: c++

	       fake_ar_publisher::ARMarkerConstPtr test_msg_;

#. 受信したメッセージをグローバル変数にコピーするためのサブスクライバ・コールバックを追加します．

   .. code-block:: c++

            void testCallback(const fake_ar_publisher::ARMarkerConstPtr &msg)
            {
              test_msg_ = msg;
            }

#. ar_pose_marker の参照フレームをチェックするユニットテストの記述

   .. code-block:: c++

            TEST(TestSuite, myworkcell_core_fake_ar_pub_ref_frame)
            {
                ros::NodeHandle nh;
                ros::Subscriber sub = nh.subscribe("/ar_pose_marker", 1, &testCallback);

                EXPECT_NE(ros::topic::waitForMessage<fake_ar_publisher::ARMarker>("/ar_pose_marker", ros::Duration(10)), nullptr);
                EXPECT_EQ(1, sub.getNumPublishers());
                EXPECT_EQ(test_msg_->header.frame_id, "camera_frame");
            }

#. ユニットテストが実行中の ROS システムとやりとりするので，
   main() 関数にノード初期化用定型文を追加してください．
   現在の main() 関数を次の新しいコードに置き換えます．

   .. code-block:: c++

            int main(int argc, char **argv)
            {
              testing::InitGoogleTest(&argc, argv);
              ros::init(argc, argv, "MyWorkcellCoreTest");

              ros::AsyncSpinner spinner(1);
              spinner.start();
              int ret = RUN_ALL_TESTS();
              spinner.stop();
              ros::shutdown();
              return ret;
            }

#. テストを実行します．

   .. code-block:: bash

         catkin run_tests myworkcell_core

#. テスト結果の表示

   .. code-block:: bash

         catkin_test_results build/myworkcell_core

   次のように表示されるはずです．

   .. code-block:: bash

         Summary: 3 tests, 0 errors, 0 failures
