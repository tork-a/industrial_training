# rqt 分析ツール

> 本演習では ROS システムの動作を理解するために
  rqt_graph ,
  rqt_console ,
  rqt_plot
  を使ってみます．


## モチベーション

複雑なマルチノードの ROS システムが動作している場合は，
ノードの相互作用を理解することが重要になります．


## 情報とリソース

* [Building and using catkin packages in a workspace](http://wiki.ros.org/catkin/Tutorials/using_a_workspace)
* [workspace で catkin パッケージをビルドして使う](http://wiki.ros.org/ja/catkin/Tutorials/using_a_workspace)


## 演習問題

Scan-N-Plan アプリケーションは完了しました．
さまざまな ROS rqt ツールを用いてアプリケーションをさらに検査したいと考えています．


## ガイダンス

### rqt_graph : ノードの相互作用状況の表示

複雑なアプリケーションでは ROS ノードの相互作用の状況を視覚的に表示すると便利です．

 1. Scan-N-Plan ワークセルを起動します．

    ```
    roslaunch myworkcell_support setup.launch
    ```

 1. 2つ目のターミナルで rqt_graph を起動します．

    ```
    rqt_graph
    ```

 1. これで Scan-N-Plan アプリケーションの基本的なレイアウトを見ることができます．

    ![](../../_static/basic_rqt_graph.png)

 1. 3つ目のターミナルで直交座標系動作軌道プランナを実行します．

    ```
    rosrun myworkcell_core myworkcell_node
    ```

 1. グラフが自動的に更新されないため，ノードの実行中にグラフを更新する必要があります．

    更新すると ROS ネットワークに myworkcell_node が含まれている表示がされるはずです．
    また，myworkcell_node は move_group ノードによって購読される
    新しいトピック `/move_group/goal` をパブリッシュしています．

    ![](../../_static/planned_rqt_graph.png)


### rqt_console : メッセージの表示

動作軌道プランナの出力を見たいと思います．
rqt_console は ROS トピックを表示するための優れた GUI です．

 1. 2つ目のターミナルで rqt_graph アプリケーションを停止して
    rqt_console を実行します．

    ```
    rqt_console
    ```

 1. 動作軌道プランナを実行します．

    ```
    rosrun myworkcell_core myworkcell_node
    ```

 1. rqt_console は自動的に更新されます．

    次のように動作軌道プランナの背後にあるロジックが表示されるはずです．

    ![](../../_static/rqt_console_output.png)


### `rqt_plot`: データのプロット表示

 rqt_plot は ROS データをリアルタイムでプロットする簡単な方法です．
 ここの例では動作軌道計画からロボットの関節角度をプロットします．

 1. 2つ目のターミナルで rqt_console アプリケーションを停止して
    rqt_plot を実行します．

    ```
    rqt_plot
    ```

 1. `Topic` 欄に下記のトピックを追加します．

    ```
    /joint_states/position[0]
    /joint_states/position[1]
    /joint_states/position[2]
    /joint_states/position[3]
    /joint_states/position[4]
    /joint_states/position[5]
    ```

 1. 動作軌道プランナを実行します．

    ```
    rosrun myworkcell_core myworkcell_node
    ```

 1. 関節角度がリアルタイムでストリーミングされ，プロットされているはずです．

 ![](../../_static/plot.png)
