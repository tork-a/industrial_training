# ROS のセットアップ

> ターミナルから ROS をセットアップし，roscore を起動します．


## モチベーション

ROS でプログラミングを開始するにあたり，
新しいマシンに ROS をインストールする方法や，
インストールしたものが適切に機能すること
を確認する方法も知っておいた方が良いでしょう．
本演習ではインストールされている ROS システムの
簡単なチェックを行います．
仮想マシンから作業している場合は
ROS がすでにインストールされているため
インストール手順をスキップできます．


## リファレンス

* [Installing and Configuring Your ROS Environment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)

* [ROS環境のインストールとセットアップ](http://wiki.ros.org/ja/ROS/Tutorials/InstallingandConfiguringROSEnvironment)


## 追加情報とリソース

* [Installation Instructions](http://wiki.ros.org/kinetic/Installation/Ubuntu)
* [Navigating ROS](http://wiki.ros.org/ROS/Tutorials/NavigatingTheFilesystem)


## Scan-N-Plan アプリケーション: 演習問題

ROS のインストールが正常にされているか確かめるためのテストを行います．

   * Scan-N-Plan アプリケーション: スキャンと動作計画を行うアプリケーション


## Scan-N-Plan アプリケーション: ガイダンス

### ~/.bashrc の設定

1. ROS パッケージを探したり使用したりする際に
   問題が発生した場合は，
   環境が適切に設定されていることを確認してください．
   ROS_ROOT や ROS_PACKAGE_PATH などの
   環境変数がどのような設定になっているのか
   を確認すると良いです．

   ```
   printenv | grep ROS
   ```

1. 環境設定が適切でなかった場合は
   _setup.*sh_ ファイルを `source` する必要があります．

   ```
   source /opt/ros/kinetic/setup.bash
   ```

1. ROS をインストールした素の状態では
   ROS コマンドにアクセスするために開く
   新規のシェル（ターミナル）でこの
   コマンドを実行する必要があります．
   典型的な ROS インストールにおいては
   設定手順の1つとしてこのコマンドを
   `~/.bashrc` ファイルの最後に追加して，
   ターミナルウィンドウを開くたびに
   自動で実行されるようにします．
   `~/.bashrc` ファイルが
   既に ROS-kinetic の `setup.bash` を実行するように
   設定されているかを確認してください．

   ```
   tail ~/.bashrc
   ```

このプロセスにより
複数の ROS ディストリビューション
（例えば indigo と kinetic）を
同じコンピュータにインストールし，
各ディストリビューションの
`setup.bash` ファイルをソースして
それらを切り替えることができます．


### roscore の起動

1. _roscore_ は ROS ベースのシステムに必要な
   ノードとプログラムの集合です．
   ROS ノードが通信できるようにするには
   _roscore_ を実行しておく必要があります．
   `roscore` コマンドを使用して起動します。

   ```
   roscore
   ```

   _roscore_ が次を起動します．

   * ROS マスター
   * ROS パラメータサーバ
   * ROS ロギングノード

   `started core service [/rosout]`
   で終わる表示がされているはずです．
   `roscore: command not found` と表示されたら，
   ROS の環境が設定されていません．
   詳しくは 演習 5.1 の
   .bashrc のセットアップを参照してください．

1. ロギングノードを見るには
   新しくターミナルを開いて
   次のコマンドを実行してください．

   ```
   rosnode list
   ```

   ロギングノードは _/rosout_ です．

1. 1つ目のターミナルで
   _Ctrl+C_ を押して
   _roscore_ を停止します．
   _Ctrl+C_ は
   ほとんどの ROS コマンド
   を停止するために使用される一般的な方法です．
