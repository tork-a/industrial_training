# PC Setup
>There are two options for utilizing the ROS-Industrial training materials.  The first **recommended** option is to utilize a pre-configured virtual machine.  The second option is to install a native Ubuntu machine with the required software.  The virtual machine approach is by far the easiest option and ensures the fewest build errors during training but is limited in its ability to connect to certain hardware, particularly over USB (i.e. kinect-like devices).  For the perception training a .bag file is provided so that USB connection is not required for this training course.

# PC のセットアップ

> ROS Industrial トレーニング教材を利用するには2つの方法があります．
**推奨方法** は事前に設定された仮想マシンを利用することです．
もう1つの方法は必要なソフトウェアを備えたネイティブの Ubuntu マシンをインストールすることです．
仮想マシンを用いた方法は，はるかに簡単な選択肢でトレーニング中のビルドエラーを最小限に抑えますが，
特定のハードウェア，特に USB（ Kinect のようなデバイスなど）に接続するには機能的な限界があります．
本トレーニングコースの知覚機能トレーニングにおいて USB 接続を必要としないように
.bag ファイルを用意しています．

## Virtual Machine Configuration (**Recommended**)
The VM method is the most convenient method of utilizing the training materials:

 1. [Download virtual box](https://www.virtualbox.org/wiki/Downloads)
 1. [Download ROS Kinetic training VM](https://rosi-images.datasys.swri.edu)
 1. [Import image into virtual box](https://www.virtualbox.org/manual/ch01.html#ovf)
 1. Start virtual machine
 1. Log into virtual machine, user: ```ros-industrial```, pass: ```rosindustrial``` (no spaces or hyphens)
 1. Get the latest changes (Open Terminal).

    ```
    cd ~/industrial_training
    git checkout kinetic
    git pull
    ./.check_training_config.bash
    ```

### 仮想マシンの設定（**推奨**）

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

### Limitations of Virtual Box
The Virtual Box is limited both in hardware capability(due to VM limitations) and package installs (to save space).  Kinect-based demos aren't possible due to USB limitations.   

### VirtualBox で制限される機能

VirtualBox はハードウェア機能（VMの制限による）とパッケージのインストール（スペースを節約するため）の2点で制限があります．USB の制約のため Kinect を用いたデモはできません．

### Common VM Issues
On most new systems, Virtual Box and VMs work out of the box.  The following is a list of issues others have encountered and solutions:
  * Virtualization must be enabled - Older systems do not have virtualization enabled (by default).  Virtualization must be enabled in the BIOS.  See <http://www.sysprobs.com/disable-enable-virtualization-technology-bios> for more information.

### 仮想マシン一般の問題

最近のほとんどのシステムでは VirtualBox などの仮想マシンはそのまま利用できます．
次は他の人が遭遇した問題とその解決法です．

  * 仮想化を有効にする必要があります - 古いシステムではデフォルトで仮想化が有効になっていません．
  BIOS で仮想化を有効にする必要があります．
  <http://www.sysprobs.com/disable-enable-virtualization-technology-bios> を参考にしてください．


## Direct Linux PC Configuration (**NOT Recommended**)
An installation [shell script](https://github.com/ros-industrial/industrial_training/blob/kinetic/gh_pages/_downloads/ros-kinetic-industrial-training.sh)
is provided to run in Ubuntu Linux 16.04 (Xenial Xerus) LTS. This script installs ROS and any other packages needed for the environment used for this training.

After this step (or if you already have a working ROS environment), clone the training material repository into your home directory:

`git clone -b kinetic https://github.com/ros-industrial/industrial_training.git ~/industrial_training`


## Linux PC の設定（**非推奨**）

インストール [シェルスクリプト]( https://github.com/ros-industrial/industrial_training/blob/kinetic/gh_pages/_downloads/ros-kinetic-industrial-training.sh) は
Ubuntu Linux 16.04（Xenial Xerus）LTS で動作するようになっています．
このスクリプトは本トレーニングに使用する環境に必要な
ROS およびその他のパッケージをインストールします．

この手順の後（もしくは既に ROS 環境がある場合）
トレーニング教材リポジトリをホームディレクトリに複製します．

```
git clone -b kinetic https://github.com/ros-industrial/industrial_training.git ~/industrial_training
```


## Configuration Check
The following is a quick check to ensure that the appropriate packages have been installed and the the `industrial_training` git repository is current.  Enter the following into the terminal:

`~/industrial_training/.check_training_config.bash`


## 設定の確認

以下は適切なパッケージがインストールされ
`industrial_training` git リポジトリが最新であること
を確認するための簡単なチェック方法です．
ターミナルで次のコマンドを実行してください．

```
~/industrial_training/.check_training_config.bash
```
