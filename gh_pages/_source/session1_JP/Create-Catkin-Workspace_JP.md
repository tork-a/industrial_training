# Catkin ワークスペースの作成

> ROS の catkin ワークスペースを作成します．


## モチベーション

ROS プロジェクトは作業領域の作成から始まります．
このワークスペースには
プロジェクトに関連するすべてのものを配置します．
本演習では Scan-N-Planアプリケーションの
コンポーネントを作成するワークスペースを作成します．


## リファレンス

* ワークスペースの作成手順:
  * [Creating a Catkin Workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
  * [catkin の workspace を作る](http://wiki.ros.org/ja/catkin/Tutorials/create_a_workspace)

_注: ros.org の現在の多くの例は、
 古いスタイルの `catkin_init_workspace` コマンド
 を使用しています．似てはいますが
 本演習で使用する `catkin_tools` コマンドを
 そのまま置き換えることはできません．_


## 追加情報とリソース

* Catkin ワークスペスの利用:
  * [Using a Workspace](http://wiki.ros.org/catkin/Tutorials/using_a_workspace)
  * [workspace で catkin パッケージをビルドして使う](http://wiki.ros.org/ja/catkin/Tutorials/using_a_workspace)


## Scan-N-Plan アプリケーション: 演習問題

まず ROS が正しくインストールされている必要があります．
そして最初のステップとして
我々のアプリケーション固有の設定する必要があります．
目標はアプリケーションとその補足ソフトウェアのために
ワークスペース（ catkin ワークスペース）を作成することです．


## Scan-N-Plan アプリケーション: ガイダンス

### catkin ワークスペースの作成

1. ワークスペースの大本（ルート）となる
   ディレクトリを作成します．
   （ ここでは `catkin_ws` とします．)

   ```
   cd ~/
   mkdir --parents catkin_ws/src
   cd catkin_ws
   ```

1. catkin ワークスペースを初期化します．

   ```
   catkin init
   ```

   * _"Workspace configuration valid" と表示されて
     catkin ワークスペースが正常に作成されたことが表示されます．
     `src` ディレクトリを作成するのを忘れた場合や
     ワークスペースのルートから `catkin init`
     を実行しなかった場合（どちらもよくある間違いです），
     `WARNING: Source space does not yet exist`
     というエラーメッセージが表示されます．_

1. ワークスペースをビルドします．

   次のコマンドはワークスペースの
   ルートディレクトリ（つまり `catkin_ws` ）下であれば
   どこでも実行することができます．

   ```
   catkin build
   ls
   ```

   * _`catkin_ws` ディレクトリに
     追加のディレクトリ（ build, devel, logs ）
     が含まれるようになります．_

1. これらの新しいディレクトリは
   （ 手動で，または `catkin clean` を使って ）
   いつでも安全に削除できます．

   catkin は `src` ディレクトリ内の
   ファイルを決して変更しません．
   `catkin build` を再実行して
   build / devel / logs ディレクトリを再生成します．

   ```
   catkin clean
   ls
   catkin build
   ls
   ```

1. ROS から作業領域が見えるようにします．

   devel ディレクトリにある
   セットアップファイルを実行してください．

   ```
   source devel/setup.bash
   ```

   * _これは新しくターミナルを立ち上げるたびに
     実行する必要があります．_
   * 入力を省力するためには
     このコマンドを `〜/.bashrc` ファイルに追加して，
     新しくターミナルを起動するたびに自動的に実行されるようにします．

     1. `gedit ~/.bashrc`
     1. ファイルの最後に次の1行を追加:
        `source ~/catkin_ws/devel/setup.bash`
     1. 保存してエディタを閉じる
