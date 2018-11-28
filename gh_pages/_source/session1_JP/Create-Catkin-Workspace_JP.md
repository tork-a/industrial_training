# Create Catkin Workspace
> In this exercise, we will create a ROS catkin workspace.

# Catkin ワークスペースの作成

> ROS の catkin ワークスペースを作成します．

## Motivation
Any ROS project begins with making a workspace. In this workspace, you will put all the things related to this particular project. In this module we will create the workspace where we will build the components of our Scan-N-Plan application.


## モチベーション

ROS プロジェクトは作業領域の作成から始まります．
このワークスペースにはプロジェクトに関連するすべてのものを配置します．
本演習では Scan-N-Planアプリケーションの
コンポーネントを作成するワークスペースを作成します．


## Reference Example
Steps to creating a workspace: [Creating a Catkin Workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

_Note: Many current examples on ros.org use the older-style `catkin_init_workspace` commands.  These are similar, but not directly interchangeable with the `catkin_tools` commands used in this course._


## リファレンス

* ワークスペースの作成手順:
  * [Creating a Catkin Workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
  * [catkin の workspace を作る](http://wiki.ros.org/ja/catkin/Tutorials/create_a_workspace)

_注: ros.org の現在の多くの例は、
 古いスタイルの `catkin_init_workspace` コマンドを使用しています．
 似てはいますが本演習で使用する `catkin_tools` コマンドをそのまま置き換えることはできません．_


## Further Information and Resources
Using a Catkin Workspace: [Using a Workspace](http://wiki.ros.org/catkin/Tutorials/using_a_workspace)


## 追加情報とリソース

* Catkin ワークスペスの利用:
  * [Using a Workspace](http://wiki.ros.org/catkin/Tutorials/using_a_workspace)
  * [workspace で catkin パッケージをビルドして使う](http://wiki.ros.org/ja/catkin/Tutorials/using_a_workspace)


## Scan-N-Plan Application: Problem Statement
We have a good installation of ROS, and we need to take the first step to setting up our particular application. Your goal is to create a workspace - a catkin workspace - for your application and its supplements.


## Scan-N-Plan アプリケーション: 演習問題

まず ROS が正しくインストールされている必要があります．
そして最初のステップとして我々のアプリケーション固有の設定する必要があります．
目標はアプリケーションとその補足ソフトウェアのために
ワークスペース（ catkin ワークスペース）を作成することです．


## Scan-N-Plan Application: Guidance

### Create a Catkin Workspace

1. Create the root workspace directory (we'll use `catkin_ws`)

   ```
   cd ~/
   mkdir --parents catkin_ws/src
   cd catkin_ws
   ```

1. Initialize the catkin workspace

   ```
   catkin init
   ```
   * _Look for the statement "Workspace configuration appears valid", showing that your catkin workspace was created successfully.  If you forgot to create the `src` directory, or did not run `catkin init` from the workspace root (both common mistakes), you'll get an error message like "WARNING: Source space does not yet exist"._

1. Build the workspace. This command may be issued anywhere under the workspace root-directory (i.e. `catkin_ws`).

   ```
   catkin build
   ls
   ```

   * _See that the `catkin_ws` directory now contains additional directories (build, devel, logs)._

1. These new directories can be safely deleted at any time (either manually, or using `catkin clean`).  Note that catkin never changes any files in the `src` directory.  Re-run `catkin build` to re-create the build/devel/logs directories.

   ```
   catkin clean
   ls
   catkin build
   ls
   ```

1. Make the workspace visible to ROS. Source the setup file in the devel directory.

   ```
   source devel/setup.bash
   ```

   * _This file MUST be sourced for every new terminal._
   * To save typing, add this to your `~/.bashrc` file, so it is automatically sourced for each new terminal:

     1. `gedit ~/.bashrc`
     1. add to the end: `source ~/catkin_ws/devel/setup.bash`
     1. save and close the editor


## Scan-N-Plan アプリケーション: ガイダンス

### catkin ワークスペースの作成

1. ワークスペースの大本（ルート）となるディレクトリを作成します．
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
     ワークスペースのルートから `catkin init`を実行しなかった場合（どちらもよくある間違いです），
     "WARNING: Source space does not yet exist"
     というエラーメッセージが表示されます．_

1. ワークスペースをビルドします．

   このコマンドはワークスペースのルートディレクトリ（つまり `catkin_ws` ）下であれば
   どこでも実行することができます．

   ```
   catkin build
   ls
   ```

   * _`catkin_ws` ディレクトリに追加のディレクトリ（ build, devel, logs ）
     が含まれるようになります．_

1. これらの新しいディレクトリは（ 手動で，または `catkin clean` を使って ）
   いつでも安全に削除できます．

   catkin は `src` ディレクトリ内のファイルを決して変更しません．
   `catkin build` を再実行して build / devel / logs ディレクトリを再生成します．

   ```
   catkin clean
   ls
   catkin build
   ls
   ```

1. ROS から作業領域が見えるようにします．

   devel ディレクトリにあるセットアップファイルを実行してください．

   ```
   source devel/setup.bash
   ```

   * _これは新しくターミナルを立ち上げるたびに実行する必要があります．_
   * 入力を省力するためにはこのコマンドを `〜/.bashrc` ファイルに追加して，
     新しくターミナルを起動するたびに自動的に実行されるようにします．

     1. `gedit ~/.bashrc`
     1. ファイルの最後に次の1行を追加: `source ~/catkin_ws/devel/setup.bash`
     1. 保存してエディタを閉じる
