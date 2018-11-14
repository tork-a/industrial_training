# つかみ上げ動作の生成

> グリッパは箱をつかみ上げるためのそれぞれの姿勢 : アプローチ・置く・後退 に動きます．
> 本演習ではそれらのツール中心位置 (Tool Center Point : TCP) 座標系におけるつかみ上げ動作を生成して，その後にそれらの姿勢を手首フレーム座標系に変換します．

## 関数の位置

  * メインプログラム内の `application.create_pick_moves()` という関数を探してください．
  * 関数の任意の部分をクリックし "F3" を押すことによってその関数のソースファイルに移動します．
  * `ROS_ERROR STREAM ...` を含む最初の行を削除してプログラムが実行されるようにしてください．

## コードを完成させる

 * 各箇所にある `Fill Code:` コメントを見つけてその記述を読んでください．そして `ENTER CODE HERE` と書いてある各部分を適切なコードで書き換えてください．

```
/* Fill Code:
     .
     .
     .
*/
/* ========  ENTER CODE HERE ======== */
```

 * `create_manipulation_poses()` は意図した目標にに対応するポーズを生成するためにアプローチと後退の距離を使用します．

 * MoveIt! がロボットの手首の軌道を計算するので，全てのつかむための姿勢が手首フレームの座標系に変換される必要があります．

 * [lookupTransform](http://mirror.umd.edu/roswiki/doc/hydro/api/tf/html/c++/classtf_1_1Transformer.html#ac01a9f8709a828c427f1a5faa0ced42b) メソッドが目標姿勢から他の姿勢に対する相対変換を行います．

##  コードのビルドと実行

  * Qt でピック・アンド・プレース・ノードをコンパイルします．

```
Project -> Build Project
```

* もしくはターミナルで本演習のワークスペースディレクトリに移動して次のコマンドを実行します．

```
catkin build --cmake-args -G 'CodeBlocks - Unix Makefiles' --pkg collision_avoidance_pick_and_place
source ./devel/setup.bash
```

  * Run your node with the launch file:
  * launch ファイルでノードを実行します．

```
roslaunch collision_avoidance_pick_and_place ur5_pick_and_place.launch
```

  * 次のようにツール中心姿勢とつかむ動作の手首位置がターミナルに表示されるはずです．  

```
[ INFO] [1400555434.918332145]: Move wait Succeeded
[ INFO] [1400555435.172714267]: Gripper opened
[ INFO] [1400555435.424279410]: target recognition succeeded
[ INFO] [1400555435.424848964]: tcp position at pick: 0x7fffde492790
[ INFO] [1400555435.424912520]: tcp z direction at pick: 0x7fffde492950
[ INFO] [1400555435.424993675]: wrist position at pick: x: -0.81555
y: 0.215563
z: 0.3
[ERROR] [1400555435.425051853]: pickup_box is not implemented yet.  Aborting.
```

## API リファレンス

* [lookupTransform](http://mirror.umd.edu/roswiki/doc/hydro/api/tf/html/c++/classtf_1_1Transformer.html#ac01a9f8709a828c427f1a5faa0ced42b)
* [TF Transforms and other useful data types](http://wiki.ros.org/tf/Overview/Data%20Types)
