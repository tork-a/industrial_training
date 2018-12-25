# ROS スタイルガイドと ros_lint

## モチベーション

> ROS Scan-N-Plan アプリケーションは完成し，テストされ，文書化されています．
他の開発者がコードを簡単に理解できるように，
スタイルガイドに従ってコードをクリーンアップする必要があります．


## 情報とリソース

* [ROS C++ Style Guide](http://wiki.ros.org/CppStyleGuide)
* [Automated Style Guide Enforcement](http://wiki.ros.org/roslint)

* [ROS C++ スタイルガイド](http://wiki.ros.org/ja/CppStyleGuide)


## Scan-N-Plan アプリケーション: 演習問題

これまでに Scan-N-Plan プログラムを完成させ，テストもしました．
そのコードを公開する必要があります．

目標は作成したコードが ROS C ++スタイルガイド に準拠していることを確認することです．


## Scan-N-Plan アプリケーション: ガイダンス

### パッケージの設定

 1. パッケージの package.xml に roslint のビルド依存関係を追加します．

    ``` xml
    <build_depend>roslint</build_depend>
    ```

 1. CMakeLists.txt の catkin REQUIRED COMPONENTS に roslint を追加します．

    ``` cmake
    find_package(catkin REQUIRED COMPONENTS
      ...
      roslint
    )
    ```

 1. CMakeLists.txt から roslint 関数を呼び出します．

    ``` cmake
    roslint_cpp()
    ```


### roslint の実行

 1. roslint を実行します．

    ``` bash
    roscd myworkcell_core
    catkin_make --make-args roslint
    ```
