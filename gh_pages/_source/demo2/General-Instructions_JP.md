# General Instructions
>In this exercise, we'll demonstrate how to run the demo as you make progress through the exercises.  Also, it will be shown how to run the system in simulation mode and on the real robot.

# 全体の手順
> 演習を通してデモをどのように進めていくかについて提示します．また，シミュレーションモードや実機ロボットでどのようにしてシステムを実行するかについても見てゆきます．

## Main Objective

 In general, you'll be implementing a `plan_and_run` node incrementally.  This means that in each exercise you'll be adding individual pieces that are needed to complete the full application demo. Thus, when an exercise is completed run the demo in simulation mode in order to verify your results.  Only when all of the exercises are finished should you run it on the real robot.

## 主要目標

 全体としては `plan_and_run` を段階的に実装してゆきます．
 つまり，デモアプリケーションを完成させるために必要な個々のピースを各演習で加えてゆきます．
 各演習が完成したら，それらの結果を評価するためにシミュレーションでデモを実行してください．
 全ての演習を終えた場合にのみ実機のロボットで成果を実行するようにしてください．

## Complete Exercises

 1. To complete an exercise, open the corresponding source file under the `src/plan_and_run/src/tasks/` directory.  For instance, in Exercise 1 you'll open `load_parameters.cpp`.

 1. Take a minute to read the header comments for specific instructions for how to complete this particular exercise.  For instance, the `load_parameters.cpp`  file contains the following instructions and hints:

``` c++
/* LOAD PARAMETERS
  Goal:
    - Load missing application parameters into the node from the ros parameter server.
    - Use a private NodeHandle in order to load parameters defined in the node's namespace.
  Hints:
    - Look at how the 'config_' structure is used to save the parameters.
    - A private NodeHandle can be created by passing the "~" string to its constructor.
*/
```

 1. Don't forget to comment out the line:

 ``` c++
ROS_ERROR_STREAM("Task '"<<__FUNCTION__ <<"' is incomplete. Exiting"); exit(-1);
 ```

 This line is usually located at the beginning of each function.  Omitting this step will cause the program to exit immediately when it reaches this point.

 1. When you run into a comment block that starts with `/*  Fill Code:` this means that the line(s) of code that follow are incorrect, commented out or incomplete at best.  Read the instructions following `Fill Code` and complete that task as described.  An example of instructions comment block is the following:

``` c++
  /*  Fill Code:
   * Goal:
   * - Create a private handle by passing the "~" string to its constructor
   * Hint:
   * - Replace the string in ph below with "~" to make it a private node.
   */
```

 1. The ```[ COMPLETE HERE ]``` entries are meant to be replaced by the appropriate `code entry`.  The right code entries could either be program variables, strings or numeric constants.  One example is shown below:

``` c++
ros::NodeHandle ph("[ COMPLETE HERE ]: ?? ");
```

In this case the correct replacement would be the string `"~"`, so this line would look like this:

``` c++
ros::NodeHandle ph("~");
```

 1. As you are completing each task in this exercise, you can run the demo (see following sections) to verify that it was completed properly.


## 演習課題を完成させる

 1. 演習課題を完成させるために `src/plan_and_run/src/tasks/` ディレクトリの中にある対応したソースファイルを開いてください．例えば 演習1 では `load_parameters.cpp` を開きます．

 1. 各々の演習を完成するための方法の具体的な指示がヘッダのコメントにありますのでそれを読んでください．
例としてファイル `load_parameters.cpp` には下記のとおり説明とヒントが書いてあります．

 ``` c++
 /* LOAD PARAMETERS
   Goal:
     - Load missing application parameters into the node from the ros parameter server.
     - Use a private NodeHandle in order to load parameters defined in the node's namespace.
   Hints:
     - Look at how the 'config_' structure is used to save the parameters.
     - A private NodeHandle can be created by passing the "~" string to its constructor.
 */
 ```

 1. 下記の行をコメントアウトしてください．

 ``` c++
 ROS_ERROR_STREAM("Task '"<<__FUNCTION__ <<"' is incomplete. Exiting"); exit(-1);
 ```

 この行は大抵，各関数の先頭に書いてあります．
 このステップを省略すると，プログラムはこの行に到達した時点ですぐに終了してしまいます．

 1. When you run into a comment block that starts with `/*  Fill Code:` で始まるコメントブロックに進んでください．この部分は後続のコードが正しくないか，コメントアウトされているか，または不完全であることを示しています．  Read the instructions following `Fill Code` に続く指示を読んで記述されているようにタスクを完成させてください．指示書きのコメントブロックは次の例のように書いてあります．

 ``` c++
   /*  Fill Code:
    * Goal:
    * - Create a private handle by passing the "~" string to its constructor
    * Hint:
    * - Replace the string in ph below with "~" to make it a private node.
    */
 ```

 1. ```[ COMPLETE HERE ]``` とある部分が適切なコードエントリに置き換えられます．正しいコードエントリはプログラムの変数や文字列，定数だったりします．1つの例を下に記します．

 ``` c++
 ros::NodeHandle ph("[ COMPLETE HERE ]: ?? ");
 ```

 In this case the correct replacement would be the string `"~"`, so this line would look like this:
 この場合，正しくは文字列が `"~"` となりますので下記のように行が書き換わります．

 ``` c++
 ros::NodeHandle ph("~");
 ```

 1. 各演習の全タスクができたら正しく完成したかを評価するためにデモを実行します．（実行方法は次の節を読んでください．）


## Run Demo in Simulation Mode
## シミュレーションモードでのデモの実行

<!-- 1. In a terminal, run the setup launch file as follows: -->

1. ターミナルから次のように入力して実行します．

```
roslaunch plan_and_run demo_setup.launch
```

 * When the virtual robot is ready , Rviz should be up and running with a UR5 arm in the home position and you'll see the following messages in the terminal:
 * シミュレーションロボットが準備された後に RViz が起動し，UR5 アームがホームポジションの状態で表示されます．また，ターミナルに次のようなメッセージが表示されるはずです．

```
      .
      .
      .
********************************************************
* MoveGroup using:
*     - CartesianPathService
*     - ExecutePathService
*     - KinematicsService
*     - MoveAction
*     - PickPlaceAction
*     - MotionPlanService
*     - QueryPlannersService
*     - StateValidationService
*     - GetPlanningSceneService
*     - ExecutePathService
********************************************************

[ INFO] [1430359645.694537917]: MoveGroup context using planning plugin ompl_interface/OMPLPlanner
[ INFO] [1430359645.694700640]: MoveGroup context initialization complete

All is well! Everyone is happy! You can start planning now!
```

  * This launch file only needs to be run once.
  * この launch フィアルは1回だけ起動する必要があります．


<!-- 1. In a separate terminal, run the application launch file: -->
2. もう1つのターミナルで次のアプリケーション launch ファイルを実行します．

```
roslaunch plan_and_run demo_run.launch
```

  * Look in the Rviz window and the arm should start moving.  
  * RViz ウィンドウのアームが動作を開始します．


## Run Demo on the Real Robot
## 実機ロボットでのデモの実行

<!--
   **Notes**

   * Make sure that you can `ping` the robot and that there aren't any obstacles near it.
-->
   **注意**

   * ロボットに `ping` が通ることと，ロボット周辺に障害物がないことを確認してください．

<!-- 1. In a terminal, run the setup launch file as follows: -->

 1. ターミナルから次のように入力して実行します．

```
roslaunch plan_and_run demo_setup.launch sim:=false robot_ip:=000.000.0.00
```

<!--
  **Notes:**

   * Enter the robot's actual IP address into the `robot_ip` argument.  The robot model in Rviz should be in about the same position as the real robot.
-->

  **注意**
   * ロボットの実際の IP アドレスを引数 `robot_ip` に入力してください．RViz 内のロボットも実機ロボットと同じ姿勢に表示されているはずです．


<!-- 1. In a separate terminal, run the application launch file: -->

 2. もう1つのターミナルで次のアプリケーション launch ファイルを実行します．

```
roslaunch plan_and_run demo_run.launch
```

  * This time the real robot should start moving.
  * 実機ロボットが動き始めます．
