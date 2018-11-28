# RViz を利用した動作計画

> 本演習では最終的にシミュレーションロボットの
  動作計画を行って実行するための
  MoveIt! の RViz プラグインの使用方法を学習します．
  MoveIt! と RViz のプラグイン両方に関連する
  さまざまなオプションや拘束条件を見ていきます．


## 動作計画環境の起動

 1. catkin ワークスペースの環境セットアップを実行してください．

 1. 動作計画環境を起動して，ROS-I シミュレータ・ノードに接続します．

    ```
    roslaunch myworkcell_moveit_config myworkcell_planning_execution.launch
    ```


## プラグインの表示オプション

 1. ディスプレイパネル _Motion Planning_ 内の
    次の表示オプションを探して試してみてください．

    * _Scene Robot -> Show Robot Visual_

    * _Scene Robot -> Show Robot Collision_

    * _Planning Request -> Query Start State_

    * _Planning Request -> Query Goal State_

 1. ここでは _Show Robot Visual_ と
    _Query Goal State_ の表示を有効にして，
    _Show Robot Collision_ と
    _Query Start State_ は無効にしてください．

    <p align="center"><img src=http://aeswiki.datasys.swri.edu/rositraining/indigo/Exercises/3.5?action=AttachFile&do=get&target=Displays.png /></p>

 1. `Panel -> Motion Planning - Trajectory Slider`
    メニューオプションを選択すると，
    軌道プレビュースライダが表示されます．

    * _このスライダで最後に計画された軌跡の
      詳細なレビューが可能になります．_


## 基本的な動作

 1. _Motion Planning_ パネルの
    _Planning_ タブを選択してください．

 1. _Query_ セクション下の
    _Select Goal State_ の部分を展開しください．

    * _\<random valid\> を選択して
       Update をクリックします．_
    * _グラフィック・ウィンドウの
       ゴール・ポジションを見てください．_

 1. _Plan_ をクリックすると，
    MoveIt! の動作計画ライブラリが生成した
    ロボットの動作計画を見ることができます．

    * _繰り返し表示を停止するために
       Displays -> Motion Planning -> Planned Path -> Loop Animation
       を無効にしてください．_
    * _軌跡を表示するために
       Displays -> Motion Planning -> Planned Path -> Show Trail
       を有効にしてください．_

 1. _Execute_ をクリックして
    産業ロボットシミュレータで動作計画を実行します．

    * _複数色のシーン上のロボット表示が更新されて，
       ロボットが目標位置に「移動」したことを確認してください．_

 1. この手順を2〜5回ほど繰り返してください．

    * _インタラクティブ・マーカを使用して
       手動でロボットを目的の位置に移動してみる．_
    * _名前付きの姿勢（ 例: "straight up" ）を使ってみる．_

    <p align="center"><img src=http://aeswiki.datasys.swri.edu/rositraining/indigo/Exercises/3.5?action=AttachFile&do=get&target=MotionQueries.png /></p>


## 発展的な動作

 1. 動作計画アルゴリズム

    * _Context タブを選択して，
       動作計画アルゴリズム（ "OMPL" の隣にあるドロップダウンボックス ）
       をいろいろ選択して試してみてください．_

    * _RRTkConfigDefault アルゴリズムは往々にして遥かに高速です．_

 1. 環境上の障害物

    * "Goal State" を調整して
      ロボットを障害物（例えばテーブル）に干渉させて見てください．

      * 干渉するリンクは赤色に表示変更されます．

      * 位置が到達不能なため解を見つけようとするとき，
        ロボットがさまざまな位置を探索して解を見つけようとするのが見られます．

      * "Context" タブで
        "Use Collision-Aware IK" 設定を無効にしてみてください．

      * 干渉がまだ検出されていてもソルバは干渉のないソリューションを探索しません．

    * 障害物を通る軌道を計画してみてください．

      * Goal State を動かす時に
        "Collision-Aware IK" が無効だと楽です．

      * ロボットの動作計画に失敗する場合は，
        エラーログを確認して計画要求を繰り返してください．

      * デフォルトのプランナはサンプリングベースなので，
        実行ごとに異なる結果が生じることがあります．

      * 動作計画の作成に成功するように，
        計画の計算時間を増やすこともできます．

      * より複雑な動作計画タスクでは，
        異なる動作計画アルゴリズムを試してみてください．

    * シーンに新しい障害を追加してみてください．

      * "Scene Objects" タブ内に
        `I-Beam.dae` CAD モデルを追加します．

         * このファイルは industrial_training リポジトリの
           `~/industrial_training/exercises/3.4/I-Beam.dae`
           にあります．

      * 操作ハンドルを使用して
        I-Beam を面白そうな位置に移動してください．

      * Publish Scene を押して，
        更新された位置を MoveIt! に通知します．

      * 障害物の周辺における動作計画を行ってみてください．
