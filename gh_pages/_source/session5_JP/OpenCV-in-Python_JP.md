# OpenCV 画像処理（ Pyhton ）

> 本演習では簡単な 2D 画像処理アプリケーションを通じて OpenCV と Python の両方に親しんでいきます．


## モチベーション

OpenCV は成熟し安定した 2D 画像処理ライブラリで，
多種多様なアプリケーションで利用されています．
ROS の多くは 3D センサーとポイントクラウドデータを利用していますが，
従来の 2D カメラや画像処理を使用するアプリケーションはまだまだ多くあります．

本演習では Python を使用して画像処理パイプラインを構築します．
Python は迅速なプロトタイプ作成が容易で
OpenCV ライブラリへの接合手段が既にあるため
このアプリケーションに適しています．


## 情報とリソース

* [OpenCV Website](https://opencv.org/)
* [OpenCV API](https://docs.opencv.org/3.0-beta/modules/refman.html)
* [OpenCV Python Tutorials](https://docs.opencv.org/3.4.2/d6/d00/tutorial_py_root.html)

* [Converting between ROS images and OpenCV images (Python)](http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython)
* [Writing a Publisher and Subscriber (Python)](http://wiki.ros.org/rospy_tutorials/Tutorials/WritingPublisherSubscriber)
* [sensor_msgs/Image](http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/Image.html)

* [シンプルな Publisher と Subscriber を書く (Python)](http://wiki.ros.org/rospy_tutorials/Tutorials/WritingPublisherSubscriber)



## 演習問題

本演習では OpenCV 画像処理ライブラリを用いて
ポンプハウジングの角度姿勢を特定する新しいノードを作成します．
ポンプの向きは，ジオメトリフィーチャを抽出して比較する
一連の処理ステップを使用して計算されます．

  1. 画像のリサイズ（処理の高速化のため）
  1. 画像の閾値処理（白黒への変換）
  1. ポンプのアウターハウジングの位置の特定（円の抽出）
  1. ピストン・スリーブ位置の特定（連結領域 blob の検出）
  1. バウンディング・ボックスを使用して主軸を推定
  1. ピストン・スリーブの位置を使用して方向を特定
  1. 基準（水平）軸に対する軸の向きを計算

![pump images](../../_static/pump_images.png)


## 実装

### パッケージの作成

この演習では任意の catkin ワークスペースに配置できる単一のパッケージを使用します．
以下の例では，以前の演習の `~/catkin_ws` ワークスペースを使用します．

 1. これから作成する新しい Python ノードを格納する新しいパッケージ `detect_pump` を作成します．

    ```bash
    cd ~/catkin_ws/src
    catkin create pkg detect_pump --catkin-deps rospy cv_bridge
    ```
    * 全ての ROS パッケージは `rospy` に依存します．
    * `cv_bridge` を使って ROS の標準画像メッセージと OpenCV の画像オブジェクトを変換します．
    * また `cv_bridge` は依存関係にある OpenCV モジュールを自動的に引っ張ってきます．

 1. このパッケージの Python モジュールを作成します．

    ```bash
    cd detect_pump
    mkdir nodes
    ```

    * このようなシンプルなパッケージのために
      [Python Style Guide](http://docs.ros.org/kinetic/api/catkin/html/howto/format2/installing_python.html)
      はパッケージ構造を完結にすることを薦めています．
    * より複雑なパッケージ（例えばエクスポート可能なモジュール，msg/srv 定義など）は，
      `__init__.py` と `setup.py` を使ってより複雑なパッケージ構造にしてください．

        * 参考
          * [Installing Python Scripts](http://docs.ros.org/kinetic/api/catkin/html/howto/format2/installing_python.html)
          * [Handling setup.py](http://docs.ros.org/api/catkin/html/user_guide/setup_dot_py.html)


### 画像パブリッシャの作成

まず最初のノードは
ファイルから画像を読み込んで
ROS の `image` トピック上の
[Image](http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/Image.html)
メッセージとしてパブリッシュします．

 * Note: ROS にはこの機能を実行する既存の `image_publisher` パッケージ/ノードがありますが，
 ここではそれをコピーして Python の ROS Publishers について学んでゆきます．

1. 画像パブリッシャ・ノード（ `nodes/image_pub.py` ）用の
    新しい Python スクリプトを作成します．
    ROS Python ノードのスケルトン・テンプレートに次のコードを書き入れてください．

    ```python
    #!/usr/bin/env python
    import rospy

    def start_node():
        rospy.init_node('image_pub')
        rospy.loginfo('image_pub node started')

    if __name__ == '__main__':
        try:
            start_node()
        except rospy.ROSInterruptException:
            pass
    ```

1. 新しいスクリプトファイルに実行権限を付与します．

    ```bash
    chmod u+x nodes/image_pub.py
    ```

1. 画像パブリッシャをテスト実行します．

    ```bash
    roscore
    rosrun detect_pump image_pump.py
    ```

    * "node started" と表示されるはずです．

1. コマンドラインから提供されたファイル名を使用して，公開するイメージファイルを読み込みます．

    1. `sys` と `cv2` (OpenCV) モジュールをインポートします．

       ```python
       import sys
       import cv2
       ```

    1. コマンドライン引数を `start_node` 関数に渡します．

       ```python
       def start_node(filename):
       ...
       start_node( rospy.myargv(argv=sys.argv)[1] )
       ```
       * ROS 固有のコマンドライン引数を取り除くために `rospy.myargv()` を使用することに注意してください．

    1. `start_node` 関数内で OpenCV を呼び出します．

       * [imread](https://docs.opencv.org/3.0-beta/modules/imgcodecs/doc/reading_and_writing_images.html#imread)
       関数が画像を読み込みます．
       * [imshow](https://docs.opencv.org/3.0-beta/modules/highgui/doc/user_interface.html#imshow)
       を使って表示します．

       ```python
       img = cv2.imread(filename)
       cv2.imshow("image", img)
       cv2.waitKey(2000)
       ```

    1. 画像ファイルを指定してノードを実行してください．

       ```bash
       rosrun detect_pump image_pub.py ~/industrial_training/exercises/5.4/pump.jpg
       ```
       * ディスプレイに画像が表示されるはずです．
       * `imshow`/`waitKey` をコメントアウトすると，それらが表示されなくなります．
       * Python ファイルを編集した後にコンパイルは必要ありませんので
         `catkin build` を行う必要はありません．

1. 画像を OpenCV オブジェクトから ROS の Image メッセージに変換します．

    1. `CvBridge` と `Image`（ROS メッセージ）モジュールをインポートします．

       ```python
       from cv_bridge import CvBridge
       from sensor_msgs.msg import Image
       ```

    1. CvBridge [cv2_to_imgmsg](https://docs.ros.org/api/cv_bridge/html/python/)
       メソッドへの呼び出しを追加します．

       ```python
       bridge = CvBridge()
       imgMsg = bridge.cv2_to_imgmsg(img, "bgr8")
       ```

1. Image メッセージを継続的に `image` トピックに公開するために ROS パブリッシャを作成します．
   メッセージを発行するために 1 Hz のスロットルを持つループを使用します．

    ```python
    pub = rospy.Publisher('image', Image, queue_size=10)
    while not rospy.is_shutdown():
        pub.publish(imgMsg)
        rospy.Rate(1.0).sleep()  # 1 Hz
    ```

1. ノードを実行して，新しくパブリッシュされた画像メッセージを調べてください．

    1. ノードの実行

       ```bash
       rosrun detect_pump image_pub.py ~/industrial_training/exercises/5.4/pump.jpg
       ```

    1. コマンドラインツールを用いたメッセージの調査

       ```bash
       rostopic list
       rostopic hz /image
       rosnode info /image_pub
       ```

    1. スタンドアロンの
       [image_view](http://wiki.ros.org/image_view#image_view.2BAC8-diamondback.image_view)
       ノードを用いたパブリッシュされた画像の調査

       ```bash
       rosrun image_view image_view
       ```


### イメージ処理ノード Detect_Pump の作成

次のノードは `image` トピックを購読して，
一連の処理ステップを実行して，
画像水平軸に対するポンプの傾きを識別します．

 1. 先程と同じように，
    基本的な ROS の Python ノード（`detect_pump.py`）を作成して
    ファイルに実行権限を付与します．

    ```python
    #!/usr/bin/env python
    import rospy

    # known pump geometry
    #  - units are pixels (of half-size image)
    PUMP_DIAMETER = 360
    PISTON_DIAMETER = 90
    PISTON_COUNT = 7

    def start_node():
        rospy.init_node('detect_pump')
        rospy.loginfo('detect_pump node started')

    if __name__ == '__main__':
        try:
            start_node()
        except rospy.ROSInterruptException:
            pass
    ```

    ```bash
    chmod u+x nodes/detect_pump.py
    ```

    * Python はコンパイルする必要がないので，
      各スクリプトの新しいビルドルールを作成するために
      `CMakeLists` を編集する必要はありません．

 1. `image` トピックを購読する ROS サブスクライバを追加して，
    処理するための画像ソースを提供します．

    1. ヘッダ部分で `Image` メッセージをインポートします．

       ```python
       from sensor_msgs.msg import Image
       ```

    1. `start_node` 関数の上に，新しい Image メッセージが受信されたときに呼び出される
       空のコールバック（ `process_image` ）を作成します．

       ```python
       def process_image(msg):
           try:
               pass
           except Exception as err:
               print err
       ```

       * try/except のエラー処理は，処理パイプライン中にエラーがあっても
         コードが実行され続けることを可能にします．

    1. `start_node` 関数内に ROS のサブスクライバ・オブジェクトを作成します．

       * `image` トピックを購読し，`Image` 型メッセージをモニタリングします．
       * 上で定義したコールバック関数を登録します．

       ```python
       rospy.Subscriber("image", Image, process_image)
       rospy.spin()
       ```

       * 参考
         * [rospy.Subscriber](http://docs.ros.org/kinetic/api/rospy/html/rospy.topics.Subscriber-class.html)
         * [rospy.spin](http://docs.ros.org/kinetic/api/rospy/html/rospy-module.html#spin)

    1. 新しいノードを実行して，
       意図したとおりにトピックを購読しているかを確認してください．

       ```bash
       rosrun detect_pump detect_pump.py
       rosnode info /detect_pump
       rqt_graph
       ```

 1. 受信した `Image` メッセージを OpenCV `Image` オブジェクトに変換して表示します．

    前と同じように `CvBridge` モジュールを使って変換を行います．

    1. `CvBridge` モジュールをインポートします．

       ```python
       from cv_bridge import CvBridge
       ```

    1. `process_image` コールバックの中で CvBridge の
       [imgmsg_to_cv2](https://docs.ros.org/api/cv_bridge/html/python/)
       メソッドへのコールを追加します．

       ```python
       # convert sensor_msgs/Image to OpenCV Image
       bridge = CvBridge()
       orig = bridge.imgmsg_to_cv2(msg, "bgr8")
       ```
       * このコード（および他のすべての画像処理コード）では，
         処理エラーがノードをクラッシュさせないように
         `try` ブロックの中に入れると良いです．
       * これは先程 `try` ブロック内に仮に置かれた `pass` コマンドを置き換えます．

    1. 受信した画像を表示するには OpenCV の `imshow` メソッドを使用します．
       各画像処理ステップの結果を表示するために再利用できるパターンを作成します．

       1. OpenCV `cv2` モジュールをインポートします．

          ```python
          import cv2
          ```

       1. `process_image` コールバックの前に表示補助の関数を追加します．

          ```python
          def showImage(img):
              cv2.imshow('image', img)
              cv2.waitKey(1)
          ```

       1. 受信した画像を新しい "drawImg" 変数に代入します．

          ```python
          drawImg = orig
          ```

       1. `except` ブロックの **下に** （そのスコープの外側で）
          `process_image` スコープのあるところで `drawImg` 変数を表示します．

          ```python
          # show results
          showImage(drawImg)
          ```

    1. ノードを実行して，受信した画像が表示されることを確認してください．

 1. 画像処理パイプラインの最初のステップは，
    画像のサイズを変更して将来の処理ステップを高速化することです．
    `try` ブロック内に次のコードを追加しノードを再実行してください．

    ```python
    # resize image (half-size) for easier processing
    resized = cv2.resize(orig, None, fx=0.5, fy=0.5)
    drawImg = resized
    ```

    * 前よりも小さい画像が表示されるはずです．
    * 参考
       * [resize()](https://docs.opencv.org/3.0-beta/modules/imgproc/doc/geometric_transformations.html#resize)

 1. 次に画像をカラーからグレースケールに変換します．
    ノードを実行してエラーをチェックしますが，
    画像は以前と同じように見えるでしょう．

    ```python
    # convert to single-channel image
    gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
    drawImg = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    ```

    * 元の画像はグレーに見えますが
      JPG ファイル，Image メッセージ，および `orig` OpenCV イメージは
      すべて3チャネルのカラー画像です．
    * 多くの OpenCV の機能は個々の画像チャンネルで動作します．
      グレー表示の画像を「真の」1チャンネルグレースケール画像に変換すると，
      取り違えを避けることができます．
    * 再変換をして `drawImg` のカラー画像に戻して，
      画像の上に色付きのオーバーレイを描画し，
      今後の処理ステップの結果を表示することができます．
    * 参考
      *  [cvtColor()](https://docs.opencv.org/3.0-beta/modules/imgproc/doc/miscellaneous_transformations.html#cvtcolor)

 1. 閾値処理を適用してグレースケール画像を二値画像に変換します．
    ノードを実行して，二値化された画像を表示します．

    ```python
    # threshold grayscale to binary (black & white) image
    threshVal = 75
    ret,thresh = cv2.threshold(gray, threshVal, 255, cv2.THRESH_BINARY)
    drawImg = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)
    ```

    `threshVal` パラメータをいろいろ試して，
    この画像に最も適した値を探してください．
    このパラメータの有効な値は [0-255] の間にあり，
    グレースケールのピクセル強度レンジと一致します．
    ポンプ面の形状を明確に強調する値を見つけます．
    150 という値が良いようです．

    * 参考
      *  [threshold](https://docs.opencv.org/3.0-beta/modules/imgproc/doc/miscellaneous_transformations.html#threshold)

 1. 外側のポンプハウジングの円形状を検出します．

     これは実際にはポンプ角度を検出するためには使用されませんが，特徴検出の良い例となっています．
     より複雑なシーンでは OpenCV の関心領域（ROI）機能を使用して，
     このポンプハウジング円形状内の特徴だけに処理を制限することができます．

    1. `HoughCircles` メソッドを使用して，既知のサイズのポンプハウジングを検出します．

       ```python
       # detect outer pump circle
       pumpRadiusRange = ( PUMP_DIAMETER/2-2, PUMP_DIAMETER/2+2)
       pumpCircles = cv2.HoughCircles(thresh, cv2.HOUGH_GRADIENT, 1, PUMP_DIAMETER, param2=2, minRadius=pumpRadiusRange[0], maxRadius=pumpRadiusRange[1])
       ```

       * 参考
         * [HoughCircles](https://docs.opencv.org/3.0-beta/modules/imgproc/doc/feature_detection.html#houghcircles)

    1. 検出されたすべてのサークルを表示する関数を追加します．（ `process_image` コールバックの前 ）

       ```python
       def plotCircles(img, circles, color):
           if circles is None: return

           for (x,y,r) in circles[0]:
               cv2.circle(img, (int(x),int(y)), int(r), color, 2)
       ```

    1. 円検出の後で表示機能を呼び出し予想される円の数を確認します．（1）

       ```python
       plotCircles(drawImg, pumpCircles, (255,0,0))
       if (pumpCircles is None):
           raise Exception("No pump circles found!")
       elif len(pumpCircles[0])<>1:
           raise Exception("Wrong # of pump circles: found {} expected {}".format(len(pumpCircles[0]),1))
       else:
           pumpCircle = pumpCircles[0][0]
       ```

    1. ノードを実行して，円形状が検出されるのを確認してください．

       * `HoughCircles` への `param2` 入力を調整して，
         うまくいくと思われる値を見つけてみてください．
         このパラメータは検出器の感度を表します．
         値を小さくするほど円が多く検出されますが，誤検知も多くなります．
       * `min/maxRadius` パラメータを削除するか，
         円間の最小距離（4番目のパラメータ）を減らして，
         他のサークルが検出されたことを確認します．
       * `param2=7` あたりの値で良い具合に検出できるようです．

 1. 連結領域（ブロブ）の検出を使用してピストン・スリーブを検出します．

    ブロブ検出は類似した色の連結領域（ブロブ）を識別するために画像を分析します．
    サイズや形状またはその他の特性に関する結果のブロブフィーチャのフィルタリングは，
    関心のある特徴を識別するのに役立ちます．
    OpenCV の
    [SimpleBlobDetector](https://docs.opencv.org/3.2.0/d0/d7a/classcv_1_1SimpleBlobDetector.html)
    を利用します．

    1. 二値化した画像に対してブロブ検出を行うために次のコードを追加してください．

       ```python
       # detect blobs inside pump body
       pistonArea = 3.14159 * PISTON_DIAMETER**2 / 4
       blobParams = cv2.SimpleBlobDetector_Params()
       blobParams.filterByArea = True;
       blobParams.minArea = 0.80 * pistonArea;
       blobParams.maxArea = 1.20 * pistonArea;
       blobDetector = cv2.SimpleBlobDetector_create(blobParams)
       blobs = blobDetector.detect(thresh)
       ```

       * 予想されるピストンスリーブ面積の 20％ 以内のブロブを選択するために
         エリアフィルタを使用することに注目してください．
       * デフォルトでは，ブロブ検出は白い背景上の黒いブロブを検出するように設定されています．
         そのため追加のカラーフィルタリングは必要ありません．

    1. ブロブ検出の後で OpenCV ブロブ表示機能を呼び出して，
       期待されるピストン・スリーブ数（7）を確認します．

       ```python
       drawImg = cv2.drawKeypoints(drawImg, blobs, (), (0,255,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
       if len(blobs) <> PISTON_COUNT:
           raise Exception("Wring # of pistons: found {} expected {}".format(len(blobs), PISTON_COUNT))
       pistonCenters = [(int(b.pt[0]),int(b.pt[1])) for b in blobs]
       ```

    1. ノードを実行して，すべてのピストン・スリーブが適切に識別されているかどうかを確認します．

 1. ポンプボディの主軸を検出します．

    この軸はキー・ピストン・スリーブの特徴を識別するために使用されます．
    画像を輪郭（アウトライン）に変換し，
    次に最大の輪郭を見つけ，
    長方形のボックスにフィットさせ（最適に回転させる），
    そのボックスの長軸を特定します．

    1. 画像の輪郭を計算して，最大の面積を持つ輪郭を選択します．

       ```python
       # determine primary axis, using largest contour
       im2, contours, h = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
       maxC = max(contours, key=lambda c: cv2.contourArea(c))
       ```

    1. 最大の輪郭にバウンディング・ボックスをフィットさせます．

       ```python
       boundRect = cv2.minAreaRect(maxC)
       ```

    1. 次の3つの補助関数をコピー＆ペーストして（ `process_image` コールバックの上 ），
       長方形の主軸の端点を計算します．

       ```python
       import math
       ...

       def ptDist(p1, p2):
           dx=p2[0]-p1[0]; dy=p2[1]-p1[1]
           return math.sqrt( dx*dx + dy*dy )

       def ptMean(p1, p2):
           return ((int(p1[0]+p2[0])/2, int(p1[1]+p2[1])/2))

       def rect2centerline(rect):
           p0=rect[0]; p1=rect[1]; p2=rect[2]; p3=rect[3];
           width=ptDist(p0,p1); height=ptDist(p1,p2);

           # centerline lies along longest median
           if (height > width):
               cl = ( ptMean(p0,p1), ptMean(p2,p3) )
           else:
               cl = ( ptMean(p1,p2), ptMean(p3,p0) )

           return cl
       ```

    1. 先に計算された境界矩形データを用いて，
       上記の `rect2centerline` 関数を呼び出します．
       表示画像上に中心線を描画します．

       ```python
       centerline = rect2centerline(cv2.boxPoints(boundRect))
       cv2.line(drawImg, centerline[0], centerline[1], (0,0,255))
       ```

 1. 最後のステップは（中心線に最も近い）キー・ピストン・スリーブを特定し，
    ポンプ角度を計算するためにその位置を用います．

    1. 点と中心線の間の距離を計算する補助関数を追加します．

       ```python
       def ptLineDist(pt, line):
           x0=pt[0]; x1=line[0][0]; x2=line[1][0];
           y0=pt[1]; y1=line[0][1]; y2=line[1][1];
           return abs((x2-x1)*(y1-y0)-(x1-x0)*(y2-y1))/(math.sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1)))
       ```

    1. `ptLineDist` 関数を呼び出すと，中心線に最も近いピストンのブロブを見つけることができます．
       どのブロブが識別されたかを示す drawImg を更新します．

       ```python
       # find closest piston to primary axis
       closestPiston = min( pistonCenters, key=lambda ctr: ptLineDist(ctr, centerline))
       cv2.circle(drawImg, closestPiston, 5, (255,255,0), -1)
       ```

    1. 3つのキーとなる点間の角度を計算します．

        * ピストン・スリーブ中心点
        * ポンプ中心点
        * 水平軸に沿った任意の点（基準の「ゼロ」位置）

        1. 3点間の角度を計算する補助関数 `findAngle` を追加します．

           ```python
           import numpy as np

           def findAngle(p1, p2, p3):
               p1=np.array(p1); p2=np.array(p2); p3=np.array(p3);
               v1=p1-p2; v2=p3-p2;
               return math.atan2(-v1[0]*v2[1]+v1[1]*v2[0],v1[0]*v2[0]+v1[1]*v2[1]) * 180/3.14159
           ```

        1. 適切なキーとなる3つの点を用いて `findAngle` 関数を呼び出します．

           ```python
           # calculate pump angle
           p1 = (orig.shape[1], pumpCircle[1])
           p2 = (pumpCircle[0], pumpCircle[1])
           p3 = (closestPiston[0], closestPiston[1])
           angle = findAngle(p1, p2, p3)
           print "Found pump angle: {}".format(angle)
           ```

 1. 全てを終えました！
    前と同じようにノードを実行します．
    報告されたポンプ角度は 24° 付近になるはずです．


### 演習チャレンジ

より大きなチャレンジとして，
下記の提案を試してこの画像処理例の操作を変更してみてください．

 1. パブリッシュ・ステップ間に画像を 10° 回転するように
    `image_pub` ノードを修正してください．
    次のコードで画像を回転させることができます：

    ```python
    def rotateImg(img, angle):
        rows,cols,ch = img.shape
        M = cv2.getRotationMatrix2D((cols/2,rows/2),angle,1)
        return cv2.warpAffine(img,M,(cols,rows))
    ```

 1. `detect_pump` ノードを変更して，
    画像検出を行う **サービス** を提供してください．
    入力画像を取り込み，ポンプ角度を出力する，カスタムサービスタイプを定義します．
    `image` トピックを購読して，
    `detect_pump` サービスを呼び出す新しいアプリケーション・ノードを作成します．

 1. `BlobDetector` の代わりに
    `HoughCircles` を使って
     ピストン・スリーブを探してみてください．
