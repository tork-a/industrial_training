# パッケージのインストール


## モチベーション

ROS の最もクールで最も便利な機能の多くは
既に ROS コミュニティのどこかに存在しています．
多くの場合，安定版のリソースがダウンロードしやすい
debian パッケージとして存在しています．<br>
テストされていないか，より「最先端」であり，
安定的なリリースには達していないリソースもあります．
これらのリソースの多くはリポジトリ
（通常は GitHub に置かれています）
からダウンロードしてアクセスすることができます．
これらの git パッケージを入手するには
debian パッケージよりもいくつかのステップが多く必要となります．
本演習では両方のタイプのパッケージにアクセスし，
それらをシステムにインストールします．


## リファレンス

* [apt-get usage](http://www.tecmint.com/useful-basic-commands-of-apt-get-and-apt-cache-for-package-management/)


## 追加情報とリソース

* [Ubuntu apt-get How To](https://help.ubuntu.com/community/AptGet/Howto)
* [Git Get Repo](https://git-scm.com/book/en/v2/Git-Basics-Getting-a-Git-Repository)
* [Git Clone Documentation](https://git-scm.com/docs/git-clone)


## Scan-N-Plan アプリケーション: 演習問題

まず ROS が正しくインストールされている必要があります．
そして ROS に存在するいくつかのパッケージを
本プログラム内で使用したいと考えています．
安定版のパッケージがあり，
そのダウンロード可能な debian パッケージが見つかったとします．
また，興味深い安定版の git パッケージも見つけたとします．
ROS の世界を訪れて
これらのパッケージをダウンロードしてください！

 1. 使用したい特定のメッセージタイプがあります．
    calibration_msgs という
    安定版 ROS パッケージがあります．

 2. AR タグを使用したいのですが，
    テスト目的のために
    ノードが似たような情報を発行するものが欲しいです．
    : fake_ar_publisher

パッケージ／ワークスペース内の
これらの両パッケージのリソースにアクセスすることが目標です．

 1. calibration_msgs（ apt-get を使用 ）

 2. fake_ar_publisher ( git から取得 )


## Scan-N-Plan アプリケーション: ガイダンス

### apt リポジトリからのパッケージのインストール

1. ターミナルのウィンドウを開いて，
   `roscd calibration_msgs` と入力してください．

   ```
   roscd calibration_msgs
   ```

   * このコマンドは作業ディレクトリを
     ROS _calibration_msgs_ パッケージの
     ディレクトリに変更します．
   * 次のようなエラーメッセージが表示されるはずです．
     `**No such package/stack 'calibration_msgs**'`
   * _このパッケージはシステム内にインストールされていないので
      インストールします．_

1. `apt install ros-kinetic-calibration-msgs` と入力してください．

   ```
   apt install ros-kinetic-calibration-msgs
   ```

   * プログラムはパッケージをインストールできないと表示して
     ルート（root）としてプログラムを実行する必要があることを提示します．
   * パッケージ名を入力する際に _TAB_ キーを押してください．
     * インストール可能であれば
       システムは自動的にパッケージ名を完成させようとします．
     * TAB キーを常用すると多くの定型コマンドの入力を高速化できます．

1. `sudo apt install ros-kinetic-calibration-msgs` と入力してください．

   ```
   sudo apt install ros-kinetic-calibration-msgs
   ```

   * _sudo_ コマンドを使用して
     "root"（管理者）権限を持つコマンド
     を実行することに注意してください．
   * パスワードを入力し，
     （尋ねられたら）プログラムのインストールを確認します．

1. `roscd calibration_msgs` と再び入力してください．

   ```
   roscd calibration_msgs
   ```

   * 今度は作業ディレクトリが
     _/opt/ros/kinetic/share/calibration_msgs_ に変更されたはずです．

1. `sudo apt remove ros-kinetic-calibration-msgs` と入力して
   パッケージを削除します．

   ```
   sudo apt remove ros-kinetic-calibration-msgs
   ```

   * _安心してください．
      今後の演習にはこのパッケージは必要ないため
      削除しても大丈夫です．_

1. `cd ~` と入力して
   ホームディレクトリに戻ります．

   ```
   cd ~
   ```

### ソースからダウンロードしてパッケージをビルドする

1. 目的のパッケージのソースリポジトリを特定します．

   1. [GitHub](http://github.com/search) を開きます．
   1. fake_ar_publisher を検索します．
   1. このリポジトリをクリックして
      右の方にある _Clone or download_ をクリックして，
      そのパネル内から URL をクリップボードにコピーします．

1. _fake_ar_publisher_ をクローンします．

   [fake_ar_publisher のリポジトリ](https://github.com/jmeyer1292/fake_ar_publisher.git)
   から catkin ワークスペース内の
   _src_ ディレクトリにクローンします．

   ```
   cd ~/catkin_ws/src
   git clone https://github.com/jmeyer1292/fake_ar_publisher.git
   ```

   * _Ctrl-Shift-V を押してターミナルに貼り付けるか，
      マウスの右クリックから貼り付けを選択します．_
   * _Git コマンドは本課題の範囲外ですが，
      [GitHub に良いチュートリアル](https://help.github.com/articles/git-and-github-learning-resources/)
      があります．_

1. `catkin build` を使って
   新しいパッケージをビルドします．

  _ビルドコマンドは catkin ワークスペースのどこからでも実行できます．_

1. ビルドが終了したら
   _"re-source setup files to use them"_
   との指示が出ます．

   * 前回の演習で `〜/.bashrc` ファイルに行を追加して
     新しくターミナルを起動するたびに
     catkin 設定ファイルを
     自動的に再実行するようにしました．
   * ほとんどの開発の場面ではこれで十分ですが，
     例えば新しいパッケージを追加するような場合など，
     現在のターミナルで `source` コマンド
     を再実行する必要が生じることがあります．

     ```
     source ~/catkin_ws/devel/setup.bash
     ```

1. ビルドが終了したら
   _build_ と _devel_ ディレクトリを調べて
   作成されたファイルを確認してください．

1. _rospack find fake_ar_publisher_ を実行して
   新しいパッケージが ROS から見えていることを確認します．

   ```
   rospack find fake_ar_publisher
   ```

   * このコマンドは ROS ワークスペースに関連する問題の
     トラブルシューティングに役立ちます．
   * ROS がパッケージを見つけることができない場合は，
     ワークスペースの再ビルドを行い，
     その後ワークスペースの `setup.bash` ファイル
     を再実行してみてください．
