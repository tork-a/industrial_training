# Installing Packages

# パッケージのインストール

## Motivation
Many of the coolest and most useful capabilities of ROS already exist somewhere in its community. Often, stable resources exist as easily downloadable debian packages. Alternately, some resources are less tested or more "cutting edge" and have not reached a stable release state; you can still access many of these resources by downloading them from their repository (usually housed on Github). Getting these git packages takes a few more steps than the debian packages. In this module we will access both types of packages and install them on our system.


## モチベーション

ROS の最もクールで最も便利な機能の多くは既に ROS コミュニティのどこかに存在しています．
多くの場合，安定版のリソースがダウンロードしやすい debian パッケージとして存在しています．<br>
テストされていないか，より「最先端」であり，安定版のリリースには達していないリソースもあります．
これらのリソースの多くはリポジトリ（通常は GitHub に置かれています）
からダウンロードしてアクセスすることができます．
これらの git パッケージを入手するには debian パッケージよりもいくつかのステップが必要となります．
本演習では両方のタイプのパッケージにアクセスし，
それらをシステムにインストールします．


## Reference Example
[apt-get usage](http://www.tecmint.com/useful-basic-commands-of-apt-get-and-apt-cache-for-package-management/)


## リファレンス

[apt-get usage](http://www.tecmint.com/useful-basic-commands-of-apt-get-and-apt-cache-for-package-management/)


## Further Information and Resources
[Ubuntu apt-get How To](https://help.ubuntu.com/community/AptGet/Howto)

[Git Get Repo](https://git-scm.com/book/en/v2/Git-Basics-Getting-a-Git-Repository)

[Git Clone Documentation](https://git-scm.com/docs/git-clone)


## 追加情報とリソース

* [Ubuntu apt-get How To](https://help.ubuntu.com/community/AptGet/Howto)
* [Git Get Repo](https://git-scm.com/book/en/v2/Git-Basics-Getting-a-Git-Repository)
* [Git Clone Documentation](https://git-scm.com/docs/git-clone)


## Scan-N-Plan Application: Problem Statement
We have a good installation of ROS, and we have an idea of some packages that exist in ROS that we would like to use within our program. We have found a package which is stable and has a debian package we can download. We've also found a less stable git package that we are interested in. Go out into the ROS world and download these packages!

 1. A certain message type exists which you want to use. The stable ROS package is called: calibration_msgs

 2. You are using an AR tag, but for testing purposes you would like a node to publish similar info : fake_ar_publisher

Your goal is to have access to both of these packages' resources within your package/workspace:

 1. calibration_msgs (using apt-get)

 2. fake_ar_publisher (from git)


## Scan-N-Plan アプリケーション: 演習問題

まず ROS が正しくインストールされている必要があります．
そして ROS に存在するいくつかのパッケージを本プログラム内で使用したいと考えています．
安定版のパッケージがあり，そのダウンロード可能な debian パッケージが見つかったとします．
また，興味深い安定版の git パッケージも見つけたとします．
ROS の世界に出て，これらのパッケージをダウンロードしてください！

 1. 使用したい特定のメッセージタイプがあります．
    calibration_msgs という安定版 ROS パッケージがあります．

 2. AR タグを使用したいのですが，
    テスト目的のためノードが似たような情報を発行するものが欲しいです．: fake_ar_publisher

パッケージ／ワークスペース内の
これらの両パッケージのリソースにアクセスすることが目標です．

 1. calibration_msgs（ apt-get を使用 ）

 2. fake_ar_publisher ( git から取得 )


## Scan-N-Plan Application: Guidance

### Install Package from apt Repository

1. Open a terminal window. Type roscd calibration_msgs.

   ```
   roscd calibration_msgs
   ```

   * This command changes the working directory to the directory of the ROS _calibration_msgs_ package.
   * You should see an error message `**No such package/stack 'calibration_msgs**'` .
   * _This package is not installed on the system, so we will install it._

2. Type _apt install ros-kinetic-calibration-msgs_.

   ```
   apt install ros-kinetic-calibration-msgs
   ```

   * The program will say it cannot install the package, and suggests that we must run the program as root.
   * Try pressing the _TAB_ key while typing the package name.
     * The system will try to automatically complete the package name, if possible.
     * Frequent use of the TAB key will help speed up entry of many typed commands.

3. Type _sudo apt install ros-kinetic-calibration-msgs_.

   ```
   sudo apt install ros-kinetic-calibration-msgs
   ```

   * Note the use of the _sudo_ command to run a command with "root" (administrator) privileges.
   * Enter your password, and (if asked) confirm you wish to install the program.

4. Type _roscd calibration_msgs_ again.

   ```
   roscd calibration_msgs
   ```

   * This time, you will see the working directory change to _/opt/ros/kinetic/share/calibration_msgs_.

6. Type _sudo apt remove ros-kinetic-calibration-msgs_ to remove the package.

   ```
   sudo apt remove ros-kinetic-calibration-msgs
   ```

   * _Don't worry. We won't be needing this package for any future exercises, so it's safe to remove._

7. Type _cd ~_ to return to your home directory.

   ```
   cd ~
   ```

### Download and Build a Package from Source

1. Identify the source repository for the desired package:
   1. Go to [github](http://github.com/search).
   1. Search for fake_ar_publisher.
   1. Click on this repository, and look to the right for the _Clone or Download_, then copy to clipboard.

1. Clone the _fake_ar_publisher_ [repository](https://github.com/jmeyer1292/fake_ar_publisher.git) into the catkin workspace's _src_ directory.

   ```
   cd ~/catkin_ws/src
   git clone https://github.com/jmeyer1292/fake_ar_publisher.git
   ```

   * _Use Ctrl-Shift-V to paste within the terminal, or use your mouse to right-click and select paste_
   * _Git commands are outside of the scope of this class, but there are good tutorials available [here](https://help.github.com/articles/git-and-github-learning-resources/)_

1. Build the new package using `catkin build`<BR>
  _The build command can be issued from anywhere inside the catkin workspace_

1. Once the build completes, notice the instruction to _"re-source setup files to use them"_.

   * In the previous exercise, we added a line to our `~/.bashrc` file to automatically re-source the catkin setup files in each new terminal.
   * This is sufficient for most development activities, but you may sometimes need to re-execute the `source` command in your current terminal (e.g. when adding new packages):

     ```
     source ~/catkin_ws/devel/setup.bash
     ```

1. Once the build completes, explore the _build_ and _devel_ directories to see what files were created.

1. Run _rospack find fake_ar_publisher_ to verify the new packages are visible to ROS.

   ```
   rospack find fake_ar_publisher
   ```

   * This is a helpful command to troubleshoot problems with a ROS workspace.
   * If ROS can't find your package, try re-building the workspace and then re-sourcing the workspace's `setup.bash` file.


## Scan-N-Plan アプリケーション: ガイダンス

### apt リポジトリからのパッケージのインストール

1. ターミナルのウィンドウを開いて，
   `roscd calibration_msgs` と入力してください．

   ```
   roscd calibration_msgs
   ```

   * このコマンドは作業ディレクトリを
     ROS _calibration_msgs_ パッケージのディレクトリに変更します．
   * 次のようなエラーメッセージが表示されるはずです．
     `**No such package/stack 'calibration_msgs**'`
   * _このパッケージはシステム内にインストールされていないのでインストールします．_

1. `apt install ros-kinetic-calibration-msgs` と入力してください．

   ```
   apt install ros-kinetic-calibration-msgs
   ```

   * プログラムはパッケージをインストールできないと表示して
     ルート（root）としてプログラムを実行する必要があることを提示します．
   * パッケージ名を入力する際に _TAB_ キーを押してください．
     * インストール可能であればシステムは自動的にパッケージ名を完成させようとします．
     * TAB キーを常用すると多くの定型コマンドの入力を高速化できます．

1. `sudo apt install ros-kinetic-calibration-msgs` と入力してください．

   ```
   sudo apt install ros-kinetic-calibration-msgs
   ```

   * _sudo_ コマンドを使用して
     "root"（管理者）権限を持つコマンドを実行することに注意してください．
   * パスワードを入力し，（尋ねられたら）プログラムのインストールを確認します．

1. `roscd calibration_msgs` と再び入力してください．

   ```
   roscd calibration_msgs
   ```

   * 今度は作業ディレクトリが
     _/opt/ros/kinetic/share/calibration_msgs_ に変更されたはずです．

1. `sudo apt remove ros-kinetic-calibration-msgs` と入力してパッケージを削除します．

   ```
   sudo apt remove ros-kinetic-calibration-msgs
   ```

   * _安心してください．今後の演習にはこのパッケージは必要ないため削除しても大丈夫です．_

7. `cd ~` と入力してホームディレクトリに戻ります．

   ```
   cd ~
   ```

### ソースからダウンロードしてパッケージをビルドする

1. 目的のパッケージのソースリポジトリを特定します．

   1. [GitHub](http://github.com/search) を開きます．
   1. fake_ar_publisher を検索します．
   1. このリポジトリをクリックして右の方にある _Clone or download_ をクリックして，
      そのパネル内から URL をクリップボードにコピーします．

1. _fake_ar_publisher_ をクローンします．

   [fake_ar_publisher のリポジトリ](https://github.com/jmeyer1292/fake_ar_publisher.git) から
   catkin ワークスペース内の _src_ ディレクトリにクローンします．

   ```
   cd ~/catkin_ws/src
   git clone https://github.com/jmeyer1292/fake_ar_publisher.git
   ```

   * _Ctrl-Shift-V を押してターミナルに貼り付けるか，
      マウスの右クリックから貼り付けを選択します．_
   * _Git コマンドは本課題の範囲外ですが，
      [GitHub に良いチュートリアル](https://help.github.com/articles/git-and-github-learning-resources/)
      があります．_

1. `catkin build` を使って新しいパッケージをビルドします．

  _ビルドコマンドは catkin ワークスペースのどこからでも実行できます．_

1. ビルドが終了したら _"re-source setup files to use them"_ との指示が出ます．

   * 前回の演習で `〜/.bashrc` ファイルに行を追加して
     新しくターミナルを起動するたびに
     catkin 設定ファイルを自動的に再実行するようにしました．
   * ほとんどの開発の場面ではこれで十分ですが，
     例えば新しいパッケージを追加するような場合など，
     現在のターミナルで `source` コマンドを再実行する必要が生じることがあります．

     ```
     source ~/catkin_ws/devel/setup.bash
     ```

1. ビルドが終了したら _build_ と _devel_ ディレクトリを調べて
   作成されたファイルを確認してください．

1. _rospack find fake_ar_publisher_ を実行して
   新しいパッケージが ROS から見えていることを確認します．

   ```
   rospack find fake_ar_publisher
   ```

   * このコマンドは ROS ワークスペースの問題のトラブルシューティングに役立ちます．
   * ROS がパッケージを見つけることができない場合は，
     ワークスペースの再ビルドを行い，
     その後ワークスペースの `setup.bash` ファイルを再実行してみてください．
