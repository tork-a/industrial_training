# ROS-Setup
> In this exercise, we will setup ROS to be used from the terminal, and start roscore
***

# ROS のセットアップ

> ターミナルから ROS をセットアップし，roscore を起動します．

## Motivation
In order to start programming in ROS, you should know how to install ROS on a new machine as well and check that the installation worked properly. This module will walk you through a few simple checks of your installed ROS system. Assuming you are working from the VM, you can skip any installation instructions as ROS is already installed.

## モチベーション

ROS でプログラミングを開始するにあたり，
新しいマシンに ROS をインストールする方法や，
インストールしたものが適切に機能することを確認する方法も知っておいた方が良いでしょう．
本演習ではインストールされている ROS システムの簡単なチェックを行います．
仮想マシンから作業している場合は ROS がすでにインストールされているため
インストール手順をスキップできます．

## Reference Example
[Configuring ROS](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)

## リファレンス

* [Installing and Configuring Your ROS Environment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
* [ROS環境のインストールとセットアップ](http://wiki.ros.org/ja/ROS/Tutorials/InstallingandConfiguringROSEnvironment)

## Further Information and Resources
[Installation Instructions](http://wiki.ros.org/kinetic/Installation/Ubuntu)

[Navigating ROS](http://wiki.ros.org/ROS/Tutorials/NavigatingTheFilesystem)

## 追加情報とリソース

* [Installation Instructions](http://wiki.ros.org/kinetic/Installation/Ubuntu)
* [Navigating ROS](http://wiki.ros.org/ROS/Tutorials/NavigatingTheFilesystem)


## Scan-N-Plan Application: Problem Statement
We believe we have a good installation of ROS but let's test it to make sure.


## Scan-N-Plan アプリケーション: 演習問題

ROS のインストールが正常にされているか確かめるためのテストを行います．

   * Scan-N-Plan アプリケーション: スキャンと動作計画を行うアプリケーション


## Scan-N-Plan Application: Guidance

### Setup ~/.bashrc
1. If you are ever having problems finding or using your ROS packages make sure that you have your environment properly setup. A good way to check is to ensure that environment variables like ROS_ROOT and ROS_PACKAGE_PATH are set:

   ```
   printenv | grep ROS
   ```

2. If they are not then you might need to 'source' some _setup.*sh_ files.

   ```
   source /opt/ros/kinetic/setup.bash
   ```

3. In a "bare" ROS install, you will need to run this command on every new shell you open to have access to the ROS commands.  One of the setup steps in a _typical_ ROS install is to add that command to the end of your `~/.bashrc` file, which is run automatically in every new terminal window.  Check that your `.bashrc` file has already been configured to source the ROS-kinetic `setup.bash` script:

   ```
   tail ~/.bashrc
   ```

This process allows you to install several ROS distributions (e.g. indigo and kinetic) on the same computer and switch between them by sourcing the distribution-specific `setup.bash` file.


## Scan-N-Plan アプリケーション: ガイダンス

### ~/.bashrc の設定

1. ROS パッケージを探したり使用したりする際に問題が発生した場合は，
   環境が適切に設定されていることを確認してください．
   ROS_ROOT や ROS_PACKAGE_PATH などの
   環境変数がどのような設定になっているのかを確認すると良いです．

   ```
   printenv | grep ROS
   ```

1. 環境設定が適切でなかった場合は _setup.*sh_ ファイルを `source` する必要があります．

   ```
   source /opt/ros/kinetic/setup.bash
   ```

1. ROS をインストールした素の状態では ROS コマンドにアクセスするために開く
   新規のシェル（ターミナル）でこのコマンドを実行する必要があります．
   典型的な ROS インストールにおいては設定手順の1つとして
   このコマンドを `~/.bashrc` ファイルの最後に追加して，
   ターミナルウィンドウを開くたびに自動で実行されるようにします．
   `~/.bashrc` ファイルが既に ROS-kinetic の `setup.bash` を実行するように
   設定されているかを確認してください．

   ```
   tail ~/.bashrc
   ```

このプロセスにより複数の ROS ディストリビューション（例えば indigo と kinetic）を
同じコンピュータにインストールし，各ディストリビューションの `setup.bash` ファイルを
ソースしてそれらを切り替えることができます．


### Starting roscore
1. _roscore_ is a collection of nodes and programs that are pre-requisites of a ROS-based system. You must have a roscore running in order for ROS nodes to communicate. It is launched using the _roscore_ command.

   ```
   roscore
   ```

   _roscore_ will start up:

   * a ROS Master
   * a ROS Parameter Server
   * a rosout logging node

   You will see ending with `started core service [/rosout]`. If you see `roscore: command not found` then you have not sourced your environment, please refer to section 5.1. .bashrc Setup.

2. To view the logging node, open a new terminal and enter:

   ```
   rosnode list
   ```

   The logging node is named _/rosout_

3. Press _Ctrl+C_ in the first terminal window to stop roscore.  Ctrl-C is the typical method used to stop most ROS commands.


### roscore の起動

1. _roscore_ は ROS ベースのシステムに必要なノードとプログラムの集合です．
   ROS ノードが通信できるようにするには _roscore_ を実行しておく必要があります．
   `roscore` コマンドを使用して起動します。

   ```
   roscore
   ```

   _roscore_ が次を起動します．

   * ROS マスター
   * ROS パラメータサーバ
   * ROS ロギングノード

   `started core service [/rosout]` で終わる表示がされているはずです．
   `roscore: command not found` と表示されたら，ROS の環境が設定されていません．
   詳しくは 演習 5.1 の .bashrc のセットアップを参照してください．

1. ロギングノードを見るには新しくターミナルを開いて次のコマンドを実行してください．

   ```
   rosnode list
   ```

   ロギングノードは _/rosout_ です．

1. 1つ目のターミナルで _Ctrl+C_ を押して _roscore_ を停止します．
   _Ctrl+C_ は，ほとんどの ROS コマンドを停止するために使用される一般的な方法です．
