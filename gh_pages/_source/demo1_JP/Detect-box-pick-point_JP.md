# 箱をつかむ位置の検出

箱をつかむ座標フレームはセンサ情報処理から検出する ROS サービスから要
求できます．箱をつかむ姿勢取得のための ROS サービスを呼び出すサービス
クライアントの利用法について学習します．

## 関数の位置

  * メインプログラム内の `application.detect_box_pick()` という関数を探してください．
  * 関数の任意の部分をクリックし "F3" を押すことによってその関数のソースファイルに移動します．
  * `ROS_ERROR STREAM ...` を含む最初の行を削除してプログラムが実行されるようにしてください．

## コードを完成させる

 * 各箇所にある `Fill Code:` コメントを見つけてその記述を読んでくださ
   い．そして `ENTER CODE HERE` と書いてある各部分を適切なコードで書き
   換えてください．

```
/* Fill Code:
     .
     .
     .
*/
/* ========  ENTER CODE HERE ======== */
```

* プログラム内の `target_recognition_client` オブジェクトは `call()` メソッドを用いて ROS サービスにリクエストを送ることができます．

* その ROS サービスはセンサ情報を処理してサービス構造体メンバ `srv.response.target_pose` 内の箱をつかむ姿勢を返す要求を受け取ります．


##  コードのビルドと実行

  * Qt でピック・アンド・プレース・ノードをコンパイルします．

```
Project -> Build Project
```

* もしくはターミナルで本演習のワークスペースディレクトリに移動して次のコマンドを実行します．

```
catkin build --pkg collision_avoidance_pick_and_place
```

* launch ファイルでノードを実行します．

```
roslaunch collision_avoidance_pick_and_place ur5_pick_and_place.launch
```

* 1つの青い箱といくつかのボクセルグリッド障害物が RViz に表示され，ターミナルには次のようなメッセージが表示されるはずです．

```
[ INFO] [1400554224.057842127]: Move wait Succeeded
[ INFO] [1400554224.311158465]: Gripper opened
[ INFO] [1400554224.648747043]: target recognition succeeded
[ERROR] [1400554224.649055043]: create_pick_moves is not implemented yet.  Aborting.
```

## API リファレンス

* [call()](http://docs.ros.org/hydro/api/roscpp/html/classros_1_1ServiceClient.html#a8a0c9be49046998a830df625babd396f)
