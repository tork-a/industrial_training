# PC のセットアップ

> ROS Industrial トレーニング教材を利用するには
  2つの方法があります．
  **推奨方法** は事前に設定された仮想マシンを利用することです．
  もう1つの方法は必要なソフトウェアを備えた
  ネイティブの Ubuntu マシンをインストールすることです．
  仮想マシンを用いた方法は，はるかに簡単な選択肢で
  トレーニング中のビルドエラーを最小限に抑えますが，
  特定のハードウェア，特に USB（ Kinect のようなデバイスなど ）
  に接続するには機能的な限界があります．
  本トレーニングコースの知覚機能トレーニングにおいて
  USB 接続を必要としなくて済むように
  .bag ファイルを用意しています．


## 仮想マシンの設定（**推奨**）

仮想マシン（VM）はトレーニング教材を利用する最も便利な方法です．

 1. [VirtualBox をダウンロード](https://www.virtualbox.org/wiki/Downloads)
 1. [ROS Kinetic Industrial Training VM イメージのダウンロード](https://rosi-images.datasys.swri.edu)
 1. [VirtualBox への VM イメージのインポート](https://www.virtualbox.org/manual/ch01.html#ovf)
 1. 仮想マシンの起動
 1. ユーザ名: ```ros-industrial``` ，パスワード: ```rosindustrial```（スペース・ハイフンなし）でログイン
 1. ターミナルを開いて最新の状態に更新

    ```
    cd ~/industrial_training
    git checkout kinetic
    git pull
    ./.check_training_config.bash
    ```

### VirtualBox で制限される機能

VirtualBox はハードウェア機能（VMの制限による）と
パッケージのインストール（スペースを節約するため）の
2点で制限があります．
USB の制約のため Kinect を用いたデモはできません．

### 仮想マシン一般の問題

最近のほとんどのシステムでは
VirtualBox などの仮想マシンはそのまま利用できます．
次は他の人が遭遇した問題とその解決法です．

  * 仮想化を有効にする必要があります - 古いシステムでは
    デフォルトで仮想化が有効になっていません．
    BIOS で仮想化を有効にする必要があります．
    <http://www.sysprobs.com/disable-enable-virtualization-technology-bios>
    を参考にしてください．


## ネイティブ Linux PC の設定（**非推奨**）

[インストール シェルスクリプト]( https://github.com/ros-industrial/industrial_training/blob/kinetic/gh_pages/_downloads/ros-kinetic-industrial-training.sh) は
Ubuntu Linux 16.04（Xenial Xerus）LTS で動作するようになっています．
このスクリプトは本トレーニングに使用する環境に必要な
ROS およびその他のパッケージをインストールします．

この手順の後（もしくは既に ROS 環境がある場合）
トレーニング教材リポジトリをホームディレクトリに複製します．

```
git clone -b kinetic https://github.com/ros-industrial/industrial_training.git ~/industrial_training
```


## 設定の確認

以下は適切なパッケージがインストールされ
`industrial_training` git リポジトリが最新であること
を確認するための簡単なチェック方法です．
ターミナルで次のコマンドを実行してください．

```
~/industrial_training/.check_training_config.bash
```
