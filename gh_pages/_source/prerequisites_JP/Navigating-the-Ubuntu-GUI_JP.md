# Navigating the Ubuntu GUI
> In this exercise, we will familiarize ourselves with the graphical user interface (GUI) of the Ubuntu operating system.

# Ubuntu の GUI 操作

> オペレーティング・システム Ubuntu のグラフィカル・ユーザ・インタフェース（GUI）に慣れるようにします．

## Task 0: Presentation Slides
Don't forget about the [presentation slides](../../_downloads/slides/ROS-I%20Basic%20Developers%20Training%20-%20Session%200.pdf) that accompany this Lesson!

## タスク 0: プレゼンテーション・スライド

[プレゼンテーション・スライド](../../_downloads/slides/ROS-I%20Basic%20Developers%20Training%20-%20Session%200.pdf) の PDF ファイルがこのトレーニングには添付されています．

## Task 1: Familiarize Yourself with the Ubuntu Desktop
At the log-in screen, click in the password input box, enter `rosindustrial` for the password, and hit enter. The screen should look like the image below when you log in:

![](../../_static/ubuntu_desktop.png)

There are several things you will notice on the desktop:

![](../../_static/ubuntu_desktop_details.png)

1. The gear icon on the top right of the screen brings up a menu which allows the user to log out, shut down the computer, access system settings, etc...
2. The bar on the left side shows running and "favorite" applications, connected thumb drives, etc.
3. The top icon is used to access all applications and files. We will look at this in more detail later.
 1. The next icons are either applications which are currently running or have been "pinned" (again, more on pinning later)
 3. Any removable drives, like thumb drives, are found after the application icons.
 4. If the launcher bar gets "too full", clicking and dragging up/down allows you to see the applications that are hidden.
 5. To reorganize the icons on the launcher, click and hold the icon until it "pops out", then move it to the desired location.

## タスク 1: Ubuntu デスクトップに慣れる

ログイン画面でパスワード入力欄に `rosindustrial` を入力して Enter キーを押してください．
ログインできた場合には次のような画面が見えているはずです．

![](../../_static/ubuntu_desktop.png)

デスクトップ上のいくつかの項目について見ていきます．

![](../../_static/ubuntu_desktop_details.png)

1. 画面の右上にある歯車アイコンは，ユーザのログアウトやコンピュータのシャットダウン，システム設定へのアクセスなどを行うためのメニューを表示します．
1. 左側のバーには，実行中の「お気に入り」アプリケーションや接続された USB ドライブなどが表示されます．
1. 最上部のアイコンは，全てのアプリケーションとファイルにアクセスするために使用されます．これについては後で詳しく見ていきます．
  1. 続くアイコンは，現在実行中のアプリケーションか「ピン留め」されているアプリケーションです．（後で詳述）
  1. USB ドライブのようなリムーバブル・ドライブはアプリケーションのアイコンの後にあります．
  1. ランチャーバーがあまりにもいっぱいになった場合，クリックして上下にドラッグすると非表示のアプリケーションを見ることができます．
  1. ランチャーのアイコンを再編成するには「飛び出す」までアイコンをクリックしたままにして目的の場所に移動します．

## Task 2: Open and Inspect an Application
Click on the filing-cabinet icon in the launcher. A window should show up, and your desktop should look like something below:

![](../../_static/ubuntu_folder_browser.png)

Things to notice:

1. The close, minimize, and maximize buttons typically found on the right-hand side of the window title bar are found on the left-hand side.
2. The menu for windows are found on the menu bar at the top of the screen, much in the same way Macs do. The menus, however, only show up when you hover the mouse over the menu bar.
3. Notice that there are triangles on the left and right of the folder icon. The triangles on the left show how many windows of this application are open, and the right shows which application is currently in the foreground, or "has focus". Clicking on these icons when the applications are open does one of two things:
 * If there is only one window open, this window gets focus.
 * If more than one are open, clicking a second time causes all of the windows to show up in the foreground, so that you can choose which window to go to (see below):

![](../../_static/ubuntu_inspect.png)


## タスク 2: アプリケーションを開く・調べる

ランチャーのファイリング・キャビネットの形をしたアイコンをクリックします．
ウィンドウが表示され，デスクトップは次のようになります．

![](../../_static/ubuntu_folder_browser.png)

注記事項:

1. 通常は 閉じる・最小化・最大化 のボタンがウィンドウのタイトルバーの左側にあります．
1. ウィンドウのメニューは，画面の上部にあるメニューバーに表示されます．
   これは Mac の場合と同じです．
   ただしメニューはメニューバーの上にマウスを置いたときのみ表示されます．
1. フォルダアイコンの左右に三角形があることに注目してください．
   左の三角はこのアプリケーションでいくつのウィンドウが開いているかを示し，
   右は現在どのアプリケーションが最前面にあるか，
   または「フォーカスがある」ことを示しています．
   アプリケーションが開いているときにアイコンをクリックすると，
   次の2つのうちのいずれかが実行されます．
  * 開いているウィンドウが1つだけの場合，そのウィンドウにフォーカスが移ります．
  * 複数のウィンドウが開いている場合は，2回目のクリックをすると，
    すべてのウィンドウが最前面に表示されるため，どのウィンドウを表示するかを選択できます．
    （下図参照）
    ![](../../_static/ubuntu_inspect.png)


## Task 3: Start an Application & Pin it to the Launcher Bar
Click on the launcher button (top left) and type gedit in the search box. The "Text Editor" application (this is actually gedit) should show up (see below):

![](../../_static/ubuntu_start_application.png)

Click on the application. The text editor window should show up on the screen, and the text editor icon should show up on the launcher bar on the left-hand side (see below):

![](../../_static/ubuntu_application_pin.png)

1. Right-click on the text editor launch icon, and select "Lock to Launcher".
2. Close the gedit window. The launcher icon should remain after the window closes.
3. Click on the gedit launcher icon. You should see a new gedit window appear.


## タスク 3: アプリケーションを開始する・ランチャーバーへのピン留め

ランチャーボタン（左上）をクリックし，検索ボックスに「 gedit 」と入力します．
「Text Editor」アプリケーション（これがまさに gedit です）が表示されます．
（下図参照）

![](../../_static/ubuntu_start_application.png)

アプリケーションをクリックします．
テキストエディタのウィンドウが画面に表示されて
テキストエディタ・アイコンが左側のランチャーバーに表示されます．
（下記参照）

![](../../_static/ubuntu_application_pin.png)

1. ランチャーアイコンを右クリックして "Lock to Launcher" を選択してください．
1. gedit ウィンドウを閉じてください．閉じてもランチャーアイコンがまだ残っているはずです．
1. gedit ランチャーアイコンをクリックしてください．新しい gedit ウィンドウが表示されるはずです．
