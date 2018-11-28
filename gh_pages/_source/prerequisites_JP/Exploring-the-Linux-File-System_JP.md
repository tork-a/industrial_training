# Exploring the Linux File System
> In this exercise, we will look at how to navigate and move files in the Linux file system using the Ubuntu GUI, and learn some of the basics of Linux file attributes.

# Linux のファイルシステム

> Ubuntu GUI を使用して
Linux ファイルシステム内のファイルをナビゲートして移動する方法と，
Linux ファイル属性の基本について学びます．


## Using the File Browser to Navigate

 1. Open the folder browser application we opened in the [previous exercise](Navigating-the-Ubuntu-GUI.md). You should see an window like the one below. The icons and text above the main window show the current location of the window in the file system.

 ![](../../_static/ubuntu_file_system.png)

 2. The icons at the top constitute the "location bar" of the file browser. While the location bar is very useful for navigating in the GUI, it hides the exact location of the window. You can show the location by pressing _Ctrl+L_. You should see the location bar turn into something like the image below:

 ![](../../_static/ubuntu_file_system_location.png)

 3. The folder browser opens up in the user's home folder by default. This folder is typically _/home/<username>_, which in the ROS-Industrial training computer is _/home/ros-industrial_. This folder is the only one which the user has full access to. This is by design for security's sake.

 4. By default, the file browser doesn't show hidden files (files which begin with a . character) or "backup" files (which end with a ~ character). To show these files, click on the "View" menu, and select "Show Hidden Files" (or press Ctrl+H). This will show all of the hidden files.  Uncheck the option to re-hide those files.

 5. Two hidden directories are _never_ shown: The . folder, which is a special folder that represents the current folder, and .., which represents the folder which contains the current folder. These will become important in the [next exercise](The-Linux-Terminal.md).

 6. On the left hand side of the window are some quick links to removable devices, other hard drives, bookmarks, etc. Click on the "Computer" shortcut link. This will take you to the "root" of the file system, the / folder. All of the files on the computer are in sub-folders under this folder.

 7. Double click on the _opt_ folder, then the _ros_ folder. This is where all of the ROS software resides. Each version is stored in its own folder; we should see a kinetic folder there. Double-click on that folder. The _setup.bash_ file will be used in the [terminal exercise](The-Linux-Terminal.md) to configure the terminal for ROS. The programs, data, etc. are in the _bin_ and _share_ folders. You generally do not need to modify any of these files directly, but it is good to know where they reside.


## ファイルブラウザの利用

 1. [前の演習](Navigating-the-Ubuntu-GUI_JP.md)
    で開いたファイルブラウザ・アプリケーションを開きます．
    次のようなウィンドウが表示されます．
    メインウィンドウ上部のアイコンとテキストはファイルシステム内の現在の位置を示します．

    ![](../../_static/ubuntu_file_system.png)

 1. 上部のアイコンはファイルブラウザの「ロケーションバー」を構成しています．
    ロケーションバーは GUI でのナビゲートに非常に便利ですが
    ウィンドウの正確な位置を表示していません．
    _Ctrl + L_ を押すと位置を表示できます．
    ロケーションバーが次のように変わっているはずです．

    ![](../../_static/ubuntu_file_system_location.png)

 1. デフォルトではユーザのホームフォルダにファイルブラウザが開きます．
    このフォルダは通常 _/home/ <username>_ で，
    ROS-Industrial トレーニング・コンピュータでは _/home/ros-industrial_ です．
    このフォルダはユーザが完全にアクセスできる唯一のフォルダです．
    これはセキュリティ上の理由によるものです．

 1. デフォルトではファイルブラウザは隠しファイル（ . 文字で始まるファイル）
    またはバックアップ・ファイル（ ~ 文字で終わる）を表示しません．
    これらのファイルを表示するには [View] メニューをクリックし
    [Show Hidden Files] を選択する，または _Ctrl + H_ キーを押してください．
    すべての隠しファイルが表示されます．
    これらのファイルを再び隠すオプションをオフにします．

 1. 2つの隠しディレクトリは表示されません．
    現在のフォルダーを表す特別な . フォルダーと，
    現在のフォルダーを含むフォルダーを表す .. フォルダの2つです．
    これらは [次の演習](The-Linux-Terminal_JP.md) で重要になります．

 1. ウィンドウの左側にはリムーバブルデバイスや他のハードドライブ，
    ブックマークなどへのクイックリンクがあります．
    「Computer」ショートカットリンクをクリックします．
    これでファイルシステムの "root" の / フォルダに移動します．
    コンピュータ上のすべてのファイルはこのフォルダの下のサブフォルダにあります．

 1. _opt_ フォルダ，次に _ros_ フォルダをダブルクリックします．
    ここは全ての ROS ソフトウェアが存在する場所です．
    各バージョンはそれぞれのフォルダに格納されています．
    そこに _kinetic_ フォルダがあるはずです．
    そのフォルダをダブルクリックします．
    _setup.bash_ ファイルは [ターミナル演習](The-Linux-Terminal_JP.md) で使用され，
    ターミナルを ROS 用に設定します．
    プログラムやデータなどは _bin_ と _share_ フォルダにあります．
    一般的にこれらのファイルを直接変更する必要はありませんが，
    それらがどこにあるのかを知っていると良いです．


## Making Changes

### Copying, Moving, and Removing Files

 1. Create a directory and file

    1. Make a directory  _\<Home\>/ex0.3_. We will be working within this folder.

       * Inside the file browser, click on the "Home" shortcut in the left sidebar.
       * Right click in the file browser's main panel and select "New Folder".
       * Name the folder "ex0.3" and press "return".

    1. Make a file _test.txt_ inside the newly-created _ex0.3_ folder.

       * Double-click on the _ex0.3_ folder.  Note how the File Browser header changes to show the current folder.
       * Right click in the file browser's main panel and select "New Document", then "Empty Document".
       * Name the file "test.txt" and press "return".

 1. Copying Files

    1. Copy the file using one of the following methods:

       * Click and hold on the _test.txt_ file, hold down on the control key, drag somewhere else on the folder, and release.
       * Click on the file, go to the "Copy" from the "Edit" menu, and then "Paste" from the "Edit" menu.<BR>
     _Remember: to see the Menu, hover your mouse above the bar at the top of the screen_

    1. Rename the copied file to _copy.txt_ using one of the following methods:

       * Right-click on the copied file, select "Rename..." and enter _copy.txt_.
       * Click on the file, press the F2 key, and enter _copy.txt_.

    1. Create a folder _new_ using one of the following methods:

       * Right-click on an open area of the file browser window, select "New Folder", and naming it _new_
       * Select "New Folder" from the "File" menu, and naming it _new_

    1. Move the file _copy.txt_ into the _new_ folder by dragging the file into the _new_ folder.

    1. Copy the file _test.txt_ by holding down the Control key while dragging the file into the new folder.

    1. Navigate into the _new_ folder, and delete the _test.txt_ folder by clicking on the file, and pressing the delete key.


## フォルダ・ファイルの変更

### ファイルのコピー・移動・削除

 1. 新しいフォルダ・ファイルの作成

    1. _\<Home\>/ex0.3_ フォルダを作成します．今後はここで演習を行います．

       * ファイルブラウザ内で左のサイドバーにある "Home" ショートカットをクリックします．
       * ファイルブラウザのメインパネル内で右クリックして "New Folder" を選択します．
       * フォルダ名を "ex0.3" にして Enter キーを押します．

    1. 新しく作成した _ex0.3_ フォルダ内に _test.txt_ というファイルを作成します．

       * _ex0.3_ フォルダをダブルクリックします．
         ファイルブラウザのヘッダがどのように変更されて現在のフォルダを表示するのかに着目してください．
       * ファイルブラウザのメインパネル内で右クリックして
         "New Document" と次に "Empty Document" を選択してください．
       * ファイル名を "test.txt" としてリターンを押します．

 1. ファイルのコピー

    1. 次のいずれかの方法でファイルをコピーしてください．

       * _test.txt_ ファイルをクリック&ホールドし，Control キーを押したまま，
         どこか他のフォルダにドラッグして放します．
       * ファイルをクリックし "Edit" メニューの "Copy" をクリックしてから
         "Edit" メニューから "Paste" をクリックします．<BR>
         * _注: メニューを表示するには画面上部のバーの上にマウスを置いてください．_

    1. 次のいずれかの方法でコピーしたファイルの名前を _copy.txt_ に変更してください．

       * コピーしたファイルを右クリックして "Rename..." を選択し _copy.txt_ と入力する．
       * ファイルをクリックし F2 キーを押して _copy.txt_ と入力する．

    1. 次のいずれかの方法で _new_ フォルダを作成してください．

       * ファイルブラウザ・ウィンドウの空いている領域を右クリックし
         "New Folder" を選択して名前を _new_ にする．
       * "File" メニューから "New Folder" を選択し _new_ という名前をつける．

    1. ファイルを _new_ フォルダにドラッグして _copy.txt_ を _new_ フォルダに移動します．

    1. Control キーを押しながらファイルを新しいフォルダにドラッグして
       test.txt ファイルをコピーします．

    1. _new_ フォルダに移動し， _test.txt_ ファイルをクリックした後，
       Delete キーを押してファイルを削除します．
