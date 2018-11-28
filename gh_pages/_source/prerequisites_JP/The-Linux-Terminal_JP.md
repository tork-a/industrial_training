# The Linux Terminal
> In this exercise, we will familiarize ourselves with the Linux terminal.

# Linux のターミナル

> Linux のターミナル操作に慣れるようにします．

## Starting the Terminal

 1. To open the terminal, click on the terminal icon:

    ![](../../_static/ubuntu_terminal_icon.png)

 1. Create a second terminal window, either by:

    * Right-clicking on the terminal and selecting the "Open Terminal" or
    * Selecting "Open Terminal" from the "File" menu

 1. Create a second terminal within the same window by pressing "Ctrl+Shift+T" while the terminal window is selected.

 1. Close the 2nd terminal tab, either by:

    * clicking the small 'x' in the terminal tab (not the main terminal window)
    * typing `exit` and hitting enter.

 1. The window will have a single line, which looks like this:

    `ros-industrial@ros-i-kinetic-vm:~$`

 1. This is called the prompt, where you enter commands. The prompt, by default, provides three pieces of information:

    1. _ros-industrial_ is the login name of the user you are running as.
    1. _ros-i-kinetic-vm_ is the host name of the computer.
    1. ~ is the directory in which the terminal is currently in. (More on this later).

 1. Close the terminal window by typing `exit` or clicking on the red 'x' in the window's titlebar.


# ターミナルの起動

 1. ターミナルを開くには下記のアイコンをクリックします．

    ![](../../_static/ubuntu_terminal_icon.png)

 1. 2つ目のターミナルを次のいずれかの方法で開きます．

    * ターミナル上で右クリックして "Open Terminal" を選択する．
    * "File" メニューから "Open Terminal" を選択する．

 1. ターミナルウィンドウが選択されているときに "Ctrl+Shift+T" を押すと，
    同じウィンドウ内に2つ目のターミナルが開きます．

 1. 2つ目のターミナルのタブを次のいずれかの方法で閉じてください．

    * ターミナルタブにある小さな 'x' をクリックする．
      （メインのターミナルウィンドウの 'x' ではありません）
    * `exit` とタイプして Enter を押す．

 1. ウィンドウは次のような1行が表示されています．

    `ros-industrial@ros-i-kinetic-vm:~$`

 1. これは「プロンプト」と呼ばれるものでコマンドを入力する場所です．
    このプロンプトにはデフォルトでは3つの情報が表示されています．

    1. _ros-industrial_ は現在のユーザのログイン名です．
    1. _ros-i-kinetic-vm_ はコンピュータのホスト名です．
    1. ~ は現在のターミナルがいるディレクトリです．（後で詳述）

 1. `exit` を入力するか
    ウィンドウタイトルバーにある赤い 'x' をクリックして
    ターミナルウィンドウを閉じます．


## Navigating Directories and Listing Files

### Prepare your environment ###
 1. Open your home folder in the file browser.
 1. Double-click on the `ex0.3` folder we created in the previous step.
    * _We'll use this to illustrate various file operations in the terminal._
 1. Right click in the main file-browser window and select "Open in Terminal" to create a terminal window at that location.
 1. In the terminal window, type the following command to create some sample files that we can study later:
    * `cp -a ~/industrial_training/exercises/0.3/. .`


## ディレクトリの移動とファイルのリスト表示

## 環境の準備

 1. ファイルブラウザでホームフォルダを開いてください．
 1. 前の演習で作成した `ex0.3` フォルダをダブルクリックしてください．
    * このフォルダを利用してターミナルのさまざまなファイル操作を説明します．
 1. 現在のディレクトリでターミナルを開くために，
    ファイルブラウザのメインウィンドウで右クリックして
    "Open in Terminal" を選択してください．
 1. ターミナルウィンドウで次のコマンドを入力して
    この後の演習で使ういくつかのサンプルファイルを作成します．
    * `cp -a ~/industrial_training/exercises/0.3/. .`


### ls Command ###
 1. Enter `ls` into the terminal.
    * You should see `test.txt`, and `new` listed. (If you don't see 'new', go back and complete the [previous exercise](Exploring-the-Linux-File-System.md)).
    * Directories, like `new`, are colored in blue.
    * The file `sample_job` is in green; this indicates it has its "execute" bit set, which means it can be executed as a command.  
 1. Type `ls *.txt`.  Only the file `test.txt` will be displayed.
 1. Enter `ls -l` into the terminal.
    * Adding the `-l` option shows one entry per line, with additional information about each entry in the directory.
    * The first 10 characters indicate the file type and permissions
    * The first character is `d` if the entry is a directory.
    * The next 9 characters are the permissions bits for the file
    * The third and fourth fields are the owning user and group, respectively.
    * The second-to-last field is the time the file was last modified.
    * If the file is a symbolic link, the link's target file is listed after the link's file name.
 1. Enter `ls -a` in the terminal.
    * You will now see one additional file, which is hidden.
 1. Enter `ls -a -l` (or `ls -al`) in the command.
    * You'll now see that the file `hidden_link.txt` points to `.hidden_text_file.txt`.


### ls コマンド

 1. ターミナルで `ls` と入力してください．
    * `test.txt` と `new` がリスト表示されるはずです．
      （ `new` が表示されない場合は
         [前回の演習](Exploring-the-Linux-File-System_JP.md) を完遂してください．）
    * `new` のようなディレクトリは青色になっています．
    * ファイル `sample_job` は緑色になっています．
      これは「実行」のためのビットセットを有しているという印で，
      コマンドとして実行可能であることを意味しています．
 1. `ls *.txt` と入力してください．
    `test.txt` だけが表示されるはずです．
 1. `ls -l` とターミナルに入力してください．
    * `-l`オプションを追加すると，1行に1つのエントリが表示され，
      ディレクトリ内の各エントリに関する追加情報が表示されます．
    * 最初の10文字はファイルの種類とアクセス許可を示します．
    * エントリがディレクトリの場合，最初の文字は `d` です．
    * 次の9文字はファイルのパーミション（許可）ビットです．
    * 3番目と4番目のフィールドはそれぞれ所有するユーザーとグループです．
    * 2番目から最後までのフィールドは、ファイルが最後に更新された時刻です．
    * ファイルがシンボリックリンクの場合，
      リンクのファイル名の後ろにリンクのターゲットファイルがリストされます．
 1. `ls -a` とターミナルに入力してください．
    * ここでは隠しファイルが1つ追加で表示されます．
 1. `ls -a -l` もしくは `ls -al` とコマンド入力してください．
    * これで `hidden_link.txt` ファイルが
      `.hidden_text_file.txt` を指していることがわかります．


### `pwd` and `cd` Commands ###
 1. Enter `pwd` into the terminal.
    * This will show you the full path of the directory you are working in.
 1. Enter `cd new` into the terminal.
    * The prompt should change to `ros-industrial@ros-i-kinetic-vm:~/ex0.3/new$`.
    * Typing `pwd` will show you now in the directory `/home/ros-industrial/ex0.3/new`.
  1. Enter `cd ..` into the terminal.
    * In the [previous exercise](Exploring-the-Linux-File-System.md), we noted that `..` is the parent folder.
    * The prompt should therefore indicate that the current working directory is `/home/ros-industrial/ex0.3`.
 1. Enter `cd /bin`, followed by `ls`.
    * This folder contains a list of the most basic Linux commands.<BR>
    _Note that `pwd` and `ls` are both in this folder._
 1. Enter `cd ~/ex0.3` to return to our working directory.
    * Linux uses the `~` character as a shorthand representation for your home directory.
    * It's a convenient way to reference files and paths in command-line commands.
    * You'll be typing it a lot in this class... remember it!

_If you want a full list of options available for any of the commands given in this section, type `man <command>` (where `<command>` is the command you want information on) in the command line.  This will provide you with built-in documentation for the command.  Use the arrow and page up/down keys to scroll, and `q` to exit._


### `pwd` と `cd` コマンド

 1. `pwd` とターミナルに入力してください．
    * 現在の作業ディレクトリの完全なパスを表示します．
 1. `cd new` とターミナルに入力してください．
    * プロンプトが変わって
      `ros-industrial@ros-i-kinetic-vm:~/ex0.3/new$` と表示されるはずです．
    * ここで `pwd` を入力すると
      `/home/ros-industrial/ex0.3/new` ディレクトリにいることを表示します．
 1. `cd ..` とターミナルに入力してください．
    * [前の演習](Exploring-the-Linux-File-System_JP.md) で
      `..` が親フォルダであることを示しました．
    * プロンプトは現在の作業ディレクトリが
      `/home/ros-industrial/ex0.3` であることを示すはずです．
 1. `cd /bin` と入力してから `ls` を実行してください．
    * このフォルダには最も基本的な Linux コマンドのリストが含まれています．
      * _`pwd` と `ls` はこのフォルダにあります．_
 1. `cd ~/ex0.3` と入力して本演習の作業ディレクトリに戻ります．
    * Linux では `~` 文字をホームディレクトリの省略表現として使用します．
    * コマンドラインのコマンドでファイルとパスを参照する便利な方法です．
    * 本演習で沢山この `~` を入力しますので是非憶えてください．

_本節のコマンドの全オプションを知りたい場合は，
 コマンドラインに `man <command>` と入力します．
 （ `<command>` は欲しい情報があるコマンド名です．）
 これによりコマンドの組込みドキュメントが表示されます．
 スクロールするには矢印と Page Up/Down キーを使い，
 終了するには `q` を使います．_


## Altering Files ##

### mv Command ###
 1. Type `mv test.txt test2.txt`, followed by `ls`.
    * You will notice that the file has been renamed to `test2.txt`.<BR>
    _This step shows how `mv` can rename files._
 1. Type `mv test2.txt new`, then `ls`.
    * The file will no longer be present in the folder.
 1. Type `cd new`, then `ls`.
    * You will see `test2.txt` in the folder.<BR>
    _These steps show how `mv` can move files._
 1. Type `mv test2.txt ../test.txt`, then `ls`.
    * `test2.txt` will no longer be there.
 1. Type `cd ..`, then `ls`.
    * You will notice that `test.txt` is present again.<BR>
    _This shows how `mv` can move and rename files in one step._


## ファイルの変更

### mv コマンド

 1. `mv test.txt test2.txt` と入力してから `ls` を行ってください．
    * `test2.txt` にファイル名が変更されているはずです．<BR>
      _ここでは `mv` でファイル名を変更する方法を見ました．_
 1. `mv test2.txt new` と入力してから `ls` を行ってください．
    * ファイルがフォルダ内にないことがわかると思います．
 1. `cd new` と入力してから `ls` を行ってください．
    * フォルダ内にファイル `test2.txt` があるはずです．<BR>
      _ここでは `mv` でファイルを移動する方法を見ました．_
 1. `mv test2.txt ../test.txt` と入力してから `ls` を行ってください．
    * `test2.txt` はそこにはないはずです．
 1. `cd ..` と入力してから `ls` を行ってください．
    * `test.txt` が再び表示されているはずです．<BR>
      _ここでは `mv` でファイル名の変更と移動を同時に行う方法を見ました．_

### cp Command ###
 1. Type `cp test.txt new/test2.txt`, then `ls new`.
    * You will see `test2.txt` is now in the `new` folder.
 1. Type `cp test.txt "test copy.txt"`, then `ls -l`.
    * You will see that `test.txt` has been copied to `test copy.txt`.<BR>
    _Note that the quotation marks are necessary when spaces or other special characters are included in the file name._

### cp コマンド

 1. `cp test.txt new/test2.txt` と入力してから `ls new` を行ってください．
    * `test2.txt` が `new` フォルダ内にあるはずです．
 1. `cp test.txt "test copy.txt"` と入力してから `ls -l` を行ってください．
    * `test.txt` が `test copy.txt` にコピーされているはずです．<BR>
    _引用符 `"` はスペースや特殊文字がファイル名に含まれる場合に必要となります．_

### rm Command ###
 1. Type `rm "test copy.txt"`, then `ls -l`.
    * You will notice that `test copy.txt` is no longer there.

### rm コマンド

 1. `rm "test copy.txt"` を入力してから `ls -l` を行ってください．
    * `test copy.txt` が無くなっているはずです．

### mkdir Command ###
 1. Type `mkdir new2`, then `ls`.
    * You will see there is a new folder `new2`.

_You can use the  `-i` flag with `cp`, `mv`, and `rm` commands to prompt you when a file will be overwritten or removed._

### mkdir コマンド

 1. `mkdir new2` を入力してから `ls` を行ってください．
    * `new2` というフォルダができているはずです．

_`-i` フラグを `cp` や `mv` および `rm` コマンドとともに使用すると，
 ファイルの上書きまたは削除を行うときに注意喚起されます．_

## Job management ##

### Stopping Jobs ###
 1. Type `./sample_job`.
    * The program will start running.
 1. Press Control+C.
    * The program should exit.
 1. Type `./sample_job sigterm`.
    * The program will start running.
 1. Press Control+C.
    * This time the program will not die.


## ジョブの管理

### ジョブの停止

 1. `./sample_job` を実行してください．
    * プログラムが開始します．
 1. Control+C を押してください．
    * プログラムから抜けます．
 1. `./sample_job sigterm` を実行してください．
    * プログラムが開始します．
 1. Control+C を押してください．
    * 今回はプログラムは停止しません．


### Stopping "Out of Control" Jobs ###
 1. Open a new terminal window.
 1. Type `ps ax`.
 1. Scroll up until you find `python ./sample_job sigterm`.
    * This is the job that is running in the first window.
    * The first field in the table is the ID of the process (use `man ps` to learn more about the other fields).
 1. Type `ps ax | grep sample`.
    * You will notice that only a few lines are returned.
    * This is useful if you want to find a particular process
    * _Note: this is an advanced technique called "piping", where the output of one program is passed into the input of the next.  This is beyond the scope of this class, but is useful to learn if you intend to use the terminal extensively._
 1. Type `kill <id>`, where `<id>` is the job number you found with the `ps ax`.
 1. In the first window, type `./sample_job sigterm sigkill`.
    * The program will start running.
 1. In the second window, type `ps ax | grep sample` to get the id of the process.
 1. Type `kill <id>`.
    * This time, the process will not die.
 1. Type `kill -SIGKILL <id>`.
    * This time the process will exit.

### 制御不能ジョブの停止

 1. 新たにターミナルのウィンドウを開きます．
 1. `ps ax` と入力してください．
 1. ターミナルをスクロールアップして `python ./sample_job sigterm` を見つけてください．
    * これは先程の1つ目のウィンドウで実行したジョブです．
    * テーブルの最初のフィールドはプロセス ID です．
      （ `man ps` で他のフィールドについても学習してください．）
 1. `ps ax | grep sample` と入力してください．
    * より少ない行が表示されているはずです．
    * これは特定のプロセスを見つけたい場合に便利です．
    * _注: これはパイピングという応用技術で，
      あるプログラムの出力が次のプログラムの入力に渡されます．
      これは本演習の目的の範囲を超えていますが，
      ターミナルをより広範に使用するようでしたら便利な機能です．_
 1. `kill <id>` と入力してください．
    `<id>` は `ps ax` で調べたジョブ番号です．
 1. 1つ目のターミナルで `./sample_job sigterm sigkill` と入力してください．
    * プログラムが開始されます．
 1. 2つ目のターミナルでプロセス ID を得るために
    `ps ax | grep sample` と入力してください．
 1. `kill <id>` と入力してください．
    * 今回はプロセスが終了しません．
 1. `kill -SIGKILL <id>` と入力してください．
    * 今回はプロセスが終了するはずです．


### Showing Process and Memory usage ###
 1. In a terminal, type `top`.
    * A table will be shown, updated once per second, showing all of the processes on the system, as well as the overall CPU and memory usage.
 1. Press the Shift+P key.
    * This will sort processes by CPU utilization.<BR>
    _This can be used to determine which processes are using too much CPU time._
 1. Press the Shift+M key.
    * This will sort processes by memory utilization<BR>
    _This can be used to determine which processes are using too much memory._
 1. Press q or Ctrl+C to exit the program.


### プロセスとメモリの使用状況の表示

 1. ターミナルで `top` と入力してください．
    * テーブルが表示され，1秒につき1回更新されます．
      システムの全プロセスと全体の CPU とメモリの使用状況が表示されます．
 1. Shift+P キーを押してください．
    * これは CPU の利用割合の順番で並び替えを行います．<BR>
    _どのプロセスが最も CPU を使用しているかを判断するのに利用できます．_
 1. Shift+M キーを押してください．
    * これはメモリの利用割合の順番で並び替えを行います．<BR>
    _どのプロセスが最もメモリを使用しているかを判断するのに利用できます．_
 1. q か Ctrl+C でプログラムを終了します．


### Editing Text (and Other GUI Commands) ###
 1. Type `gedit test.txt`.
    * You will notice that a new text editor window will open, and `test.txt` will be loaded.
    * The terminal will not come back with a prompt until the window is closed.
 1. There are two ways around this limitation.  Try both...
 1. **Starting the program and immediately returning a prompt:**
    1. Type `gedit test.txt &`.
       * The `&` character tells the terminal to run this command in "the background", meaning the prompt will return immediately.
    1. Close the window, then type `ls`.
       * In addition to showing the files, the terminal will notify you that `gedit` has finished.
 1. **Moving an already running program into the background:**
    1. Type `gedit test.txt`.
       * The window should open, and the terminal should not have a prompt waiting.
    1. In the terminal window, press Ctrl+Z.
       * The terminal will indicate that `gedit` has stopped, and a prompt will appear.
    1. Try to use the `gedit` window.
       * Because it is paused, the window will not run.
    1. Type `bg` in the terminal.
       * The `gedit` window can now run.
    1. Close the `gedit` window, and type `ls` in the terminal window.
       * As before, the terminal window will indicate that `gedit` is finished.


### テキストの編集とその他の GUI コマンド

 1. `gedit test.txt` と入力してください．
    * 新しくテキストエディタのウィンドウが開いて `test.txt` が読み込まれます．
    * ウィンドウを閉じるまでターミナルのプロンプトは戻ってきません．
 1. プロンプトが戻ってくるようにする方法は2つありますので両方とも試してみてください．
   1. **プログラム開始と同時にプロンプトを回復させる:**
     1. `gedit test.txt &` と入力してください．
        * `&` の文字はコマンドの実行をバックグラウンドで行うことをターミナルに指定します．
          つまりプロンプトがすぐに戻ってくることを意味します．
     1. gedit ウィンドウを閉じて，ターミナルで `ls` と入力してください．
        * ターミナルにはファイルリストが表示されるだけでなく
          `gedit` が完了したことも通知されます．
   1. **既に開始しているプログラムをバックグラウンド実行に変更する:**
     1. `gedit test.txt` と入力してください．
        * ウィンドウが開いてターミナルではプロンプトが待機状態にはなっていないはずです．
     1. ターミナルのウィンドウ内で Ctrl+Z を押してください．
        * ターミナルには `gedit` が終了した旨が表示されて，プロンプトが現れます．
     1. `gedit` ウィンドウを使ってみてください．
        * 一時停止状態のためウィンドウは動作しません．
     1. ターミナルで `bg` と入力してください．
        * `gedit` ウィンドウは動作をします．
     1. gedit ウィンドウを閉じて，ターミナルで `ls` と入力してください．
        * 前と同じようにターミナルには `gedit` が完了したことが通知されます．


### Running Commands as Root ###
 1. In a terminal, type `ls -a /root`.
    * The terminal will indicate that you cannot read the folder `/root`.
    * Many times you will need to run a command that cannot be done as an ordinary user, and must be done as the "super user"
 1. To run the previous command as root, add `sudo` to the beginning of the command.
    * In this instance, type `sudo ls -a /root` instead.
    * The terminal will request your password (in this case, `rosindustrial`) in order to proceed.
    * Once you enter the password, you should see the contents of the `/root` directory.

_**Warning**: `sudo` is a powerful tool which doesn't provide any sanity checks on what you ask it to do, so be **VERY** careful in using it._


### ルートとしてのコマンド実行

 1. ターミナルで `ls -a /root` と入力してください．
    * 端末は `/root` フォルダを読むことができないことを示します．
    * 普通のユーザーとしては実行できないコマンドを実行する必要がある場合が多くあり，
      その時は「スーパーユーザー」としてコマンドを実行する必要があります．
 1. 前のコマンドを root として実行するには，
    コマンドの先頭に `sudo` を追加します．
    * この場合 `sudo ls -a /root` と入力してください．
    * 実行において端末はパスワードを要求します．
      （本例では `rosindustrial` ）
    * パスワードを入力すると，`/root` ディレクトリの内容が表示されます．

_**警告**:`sudo`は
 実行に際して指示の健全性のチェックを行わない強力なツールですので
 使用する際に **特に注意** してください．_
