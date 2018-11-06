<!--
# Application Structure
>In this exercise, we'll take a look at all the packages and files that will be used during the completion of these exercises.
-->

# アプリケーション構成
> 演習を完成するまでに利用する全てのパッケージとファイルについて見てゆきます．

<!-- ## Acquire and initialize the Workspace -->
## ワークスペースのコピーと初期化
```
cd ~/industrial_training/exercises/Descartes_Planning_and_Execution
cp -r template_ws ~/descartes_ws
cd ~/descartes_ws
source /opt/ros/kinetic/setup.bash
catkin init
```

<!--
## Download source dependencies
>Use the [wstool](http://wiki.ros.org/wstool) command to download the repositories listed in the **src/.rosinstall** file
-->

## 依存するソースのダウンロード
> **src/.rosinstall** にリストアップされたリポジトリをダウンロードするために [wstool](http://wiki.ros.org/wstool) コマンドを使います．
```
cd ~/descartes_ws/src/
wstool update
```

<!--
## Download debian dependencies
>Make sure you have installed and configured the [rosdep tool](http://wiki.ros.org/rosdep).
>Then, run the following command from the **src** directory of your workspace.
-->

## 依存するDebian パッケージのダウンロード
> 前提として [rosdep tool](http://wiki.ros.org/rosdep) がインストールされていることを確認してください．
> 確認できましたらワークスペース内の **src** ディレクトリで下記コマンドを実行してください．
```
rosdep install --from-paths . --ignore-src -y
```

<!-- ## Build your workspace -->
## ワークスペースのビルド
```
catkin build
```
<!-- >If the build fails then revisit the previous two steps to make sure all the dependencies were downloaded. -->

> もしビルドが失敗するようでしたら前の2つのステップを再び行い，依存関係にあるものがすべてダウンロードされていることを確認してください．

<!--
## Source the workspace
> Run the following command from your workspace parent directory
-->

## ワークスペース設定の反映
> ワークスペースの大本のディレクトリにて下記コマンドを実行します．
```
source devel/setup.bash
```

<!-- ## List All the Packages in the Application -->
## アプリケーションの全パッケージの確認
```
cd ~/descartes_ws/src
ls -la
```

<!--
 * `plan_and_run` :           Contains the source code for the `plan_and_run` application.  You'll be completing the exercises by editing source files in this package
 * `ur5_demo_moveit_config` : Contains support files for planning and execution robot motions with Moveit.  This package was generated with the Moveit Setup Assistant
 * `ur5_demo_support` :       Provides the robot definition as a URDF file.  This URDF is loaded at run time by our `plan_and_run` application.
 * `ur5_demo_descartes` :     Provides a custom Descartes Robot Model for the UR5 arm.  It uses a Inverse-Kinematics closed form solution; which is significantly faster than the numerical approach used by the **MoveitStateAdapter**.
-->

 * `plan_and_run` : `plan_and_run` アプリケーションのためのソースコードが含まれている．本パッケージにあるソースファイルを編集して演習課題を完成させます．
 * `ur5_demo_moveit_config` : MoveIt! でロボット動作を計画・実行するための補助ファイルが含まれている．本パッケージは MoveIt! のセットアップアシスタントで生成されている．
 * `ur5_demo_support` : ロボットが URDF ファイルとして定義されている．この URDF ファイルは `plan_and_run` アプリケーション実行時に読み込まれます．
 * `ur5_demo_descartes` : UR5 アームのカスタム直交座標系ロボットモデルが用意されています．逆運動学の閉形式解を用いており，**MoveitStateAdapter** で使用されている数値解析的アプローチよりも著しく速く解を得られます．

<!-- ## The `plan_and_run` package -->
## `plan_and_run` パッケージ
```
roscd plan_and_run
ls -la
```

<!--
 * `src` :     Application source files.
  * `src/demo_application.cpp` : A class source file that contains the application implementation code.
  * `src/plan_and_run.cpp`     : The application main access point. It invokes all the tasks in the application and wraps them inside the "`main`" routine.
  * `src/tasks`                : A directory that contains all of the source files that you'll be editing or completing as you make progress through the exercises.
 * `include` : Header files
  * `include/plan_and_run/demo_application.h` : Defines the application skeleton and provides a number of global variables for passing data at various points in the exercises.
 * `launch`: Launch files needed to run the application
  * `launch/demo_setup.launch` : Loads `roscore`, `moveit` and the runtime resources needed by our application.  
  * `launch/demo_run.launch` : Starts our application main executable as a ROS node.
 * `config`: Directory that contains non-critical configuration files.
-->

 * `src` : アプリケーション・ソースファイル・ディレクトリ
  * `src/demo_application.cpp` : アプリケーション実行するコードを含むクラスソースファイル
  * `src/plan_and_run.cpp` : アプリケーションのメインアクセスファイル．全てのタスクを呼び出し`main` ルーチンの中から順次実行します．
  * `src/tasks` : 本演習を通して編集・完成させる全てのソースファイルが含まれているディレクトリ
 * `include` : ヘッダファイルの・ディレクトリ
  * `include/plan_and_run/demo_application.h` : アプリケーションの骨格の定義され，演習で使用されるグローバル変数が用意されている．
 * `launch`: アプリケーションを実行するための launch ファイル
  * `launch/demo_setup.launch` : `roscore` や `moveit`，アプリケーションで必要となるランタイム・リソースが読み込まれる．
  * `launch/demo_run.launch` : ROS ノードとしてアプリケーションのメインプログラムを実行する．
 * `config`: 重要性の低い設定ファイルのディレクトリ

<!-- ## Main Application Source File -->
## メインアプリケーション・ソースファイル

<!--  In the "`plan_and_run/src/plan_and_run_node.cpp`" you'll find the following code: -->
 "`plan_and_run/src/plan_and_run_node.cpp`" 内に下記のコードがあります．

``` c++
int main(int argc,char** argv)
{
  ros::init(argc,argv,"plan_and_run");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // creating application
  plan_and_run::DemoApplication application;

  // loading parameters
  application.loadParameters();

  // initializing ros components
  application.initRos();

  // initializing descartes
  application.initDescartes();

  // moving to home position
  application.moveHome();

  // generating trajectory
  plan_and_run::DescartesTrajectory traj;
  application.generateTrajectory(traj);


  // planning robot path
  plan_and_run::DescartesTrajectory output_path;
  application.planPath(traj,output_path);

  // running robot path
  application.runPath(output_path);

  // exiting ros node
  spinner.stop();

  return 0;
}
```

<!--
  In short, this program will run through each exercise by calling the corresponding function from the `application` object.  For instance, in order to initialize Descartes the program calls `application.iniDescartes()`.  Thus each exercise consists of editing the source file where that exercise is implemented, so for `application.initDescartes()` you'll be editing the `plan_and_run/src/tasks/init_descartes.src` source file.
-->

  このプログラムは `application` オブジェクトから各演習で対応する関数を呼び出して実行します．
例えばこのプログラムは直交座標系の定義を行うために `application.iniDescartes()` を呼び出します．
つまり各々の機能が実装されるソースファイルを編集することで各演習を行っていき，
この例では `application.initDescartes（）` の `plan_and_run/src/tasks/init_descartes.src` ソースファイルを編集します．

<!-- ## The DemoApplication Class -->
## DemoApplication クラス

<!--
  In the header file "`plan_and_run/include/plan_and_run/demo_application.h`" you'll find the definition for the application's main class along with several support constructs. Some of the important components to take notice of are as follows:
-->

  ヘッダファイル "`plan_and_run/include/plan_and_run/demo_application.h`" には補助構成要素とともにアプリケーションのメインクラスの定義があります．重要な構成要素のいくつかを下に書き出しています．

  * Program Variables: Contain hard coded values that are used at various points in the application.
  * プログラムの変数: アプリケーションの様々なところから利用されるハードコードされた値も含む

``` c++
const std::string ROBOT_DESCRIPTION_PARAM = "robot_description";
const std::string EXECUTE_TRAJECTORY_SERVICE = "execute_kinematic_path";
const std::string VISUALIZE_TRAJECTORY_TOPIC = "visualize_trajectory_curve";
const double SERVICE_TIMEOUT = 5.0f; // seconds
const double ORIENTATION_INCREMENT = 0.5f;
const double EPSILON = 0.0001f;
const double AXIS_LINE_LENGHT = 0.01;
const double AXIS_LINE_WIDTH = 0.001;
const std::string PLANNER_ID = "RRTConnectkConfigDefault";
const std::string HOME_POSITION_NAME = "home";
```

<!--  * Trajectory Type: Convenience type that represents an array of Descartes Trajectory Points. -->
 * Trajectory 型: 直交座標系軌道点群の配列を表現するのに便利

``` c++
typedef std::vector<descartes_core::TrajectoryPtPtr> DescartesTrajectory;
```


<!--  * **DemoConfiguration Data Structure**:  Provides variables whose values are initialize at run-time from corresponding ros parameters. -->
 * **DemoConfiguration Data Structure**: 対応する ROS パラメータから実行時が初期化される変数を提供

``` c++
struct DemoConfiguration
{
  std::string group_name;                 /* Name of the manipulation group containing the relevant links in the robot */
  std::string tip_link;                   /* Usually the last link in the kinematic chain of the robot */
  std::string base_link;                  /* The name of the base link of the robot */
  std::string world_frame;                /* The name of the world link in the URDF file */
  std::vector<std::string> joint_names;   /* A list with the names of the mobile joints in the robot */


  /* Trajectory Generation Members:
   *  Used to control the attributes (points, shape, size, etc) of the robot trajectory.
   *  */
  double time_delay;              /* Time step between consecutive points in the robot path */
  double foci_distance;           /* Controls the size of the curve */
  double radius;                  /* Controls the radius of the sphere on which the curve is projected */
  int num_points;                 /* Number of points per curve */
  int num_lemniscates;            /* Number of curves*/
  std::vector<double> center;     /* Location of the center of all the lemniscate curves */
  std::vector<double> seed_pose;  /* Joint values close to the desired start of the robot path */

  /*
   * Visualization Members
   * Used to control the attributes of the visualization artifacts
   */
  double min_point_distance;      /* Minimum distance between consecutive trajectory points. */
};
```

<!-- * **DemoApplication Class**:  Main component of the application which provides functions for each step in our program.  It also contains several constructs that turn this application into a ROS node. -->
 * **DemoApplication Class**: プログラム各ステップの関数を提供するアプリケーションの主要コンポーネント．このアプリケーションを ROS ノードに変換するいくつかの構成要素も含まれています．

``` c++
class DemoApplication
{
public:
  /*  Constructor
   *    Creates an instance of the application class
   */
  DemoApplication();
  virtual ~DemoApplication();

  /* Main Application Functions
   *  Functions that allow carrying out the various steps needed to run a
   *  plan an run application.  All these functions will be invoked from within
   *  the main routine.
   */

  void loadParameters();
  void initRos();
  void initDescartes();
  void moveHome();
  void generateTrajectory(DescartesTrajectory& traj);
  void planPath(DescartesTrajectory& input_traj,DescartesTrajectory& output_path);
  void runPath(const DescartesTrajectory& path);

protected:

  /* Support methods
   *  Called from within the main application functions in order to perform convenient tasks.
   */

  static bool createLemniscateCurve(double foci_distance, double sphere_radius,
                                    int num_points, int num_lemniscates,
                                    const Eigen::Vector3d& sphere_center,
                                    EigenSTL::vector_Affine3d& poses);

  void fromDescartesToMoveitTrajectory(const DescartesTrajectory& in_traj,
                                              trajectory_msgs::JointTrajectory& out_traj);

  void publishPosesMarkers(const EigenSTL::vector_Affine3d& poses);


protected:

  /* Application Data
   *  Holds the data used by the various functions in the application.
   */
  DemoConfiguration config_;



  /* Application ROS Constructs
   *  Components needed to successfully run a ros-node and perform other important
   *  ros-related tasks
   */
  ros::NodeHandle nh_;                        /* Object used for creating and managing ros application resources*/
  ros::Publisher marker_publisher_;           /* Publishes visualization message to Rviz */
  ros::ServiceClient moveit_run_path_client_; /* Sends a robot trajectory to moveit for execution */



  /* Application Descartes Constructs
   *  Components accessing the path planning capabilities in the Descartes library
   */
  descartes_core::RobotModelPtr robot_model_ptr_; /* Performs tasks specific to the Robot
                                                     such IK, FK and collision detection*/
  descartes_planner::SparsePlanner planner_;      /* Plans a smooth robot path given a trajectory of points */

};
```

<!--
## Application Launch File
 This file starts our application as a ROS node and loads up the necessary parameters into the ROS parameter server. Observe how this is done by opening the "`plan_and_run/launch/demo_run.launch`" file:
-->

## アプリケーション launch ファイル
 アプリケーションを ROS ノードとして開始し，必要なパラメーターを ROS パラメーター・サーバーにロードします．
 "`plan_and_run/launch/demo_run.launch`" を見てこれがどのように行われているか確認してください．

``` xml
<launch>
  <node name="plan_and_run_node" type="plan_and_run_node" pkg="plan_and_run" output="screen">
    <param name="group_name" value="manipulator"/>
    <param name="tip_link" value="tool"/>
    <param name="base_link" value="base_link"/>
    <param name="world_frame" value="world"/>
    <param name="trajectory/time_delay" value="0.1"/>
    <param name="trajectory/foci_distance" value="0.07"/>
    <param name="trajectory/radius" value="0.08"/>
    <param name="trajectory/num_points" value="200"/>
    <param name="trajectory/num_lemniscates" value="4"/>
    <rosparam param="trajectory/center">[0.36, 0.2, 0.1]</rosparam>
    <rosparam param="trajectory/seed_pose">[0.0, -1.03, 1.57 , -0.21, 0.0, 0.0]</rosparam>
    <param name="visualization/min_point_distance" value="0.02"/>
  </node>
</launch>
```

<!-- * Some of the important parameters are explained as follows: -->
 * 重要なパラメータ
<!--  * group_name: A namespace that points to the list of links in the robot that are included in the arm's kinematic chain (base to end-of-tooling links).  This list is defined in the `ur5_demo_moveit_config` package. -->
  * group_name: アームの運動学的なつながり（基礎部リンクからツール端部リンクまで）を含むリンクのリストを示す名前空間
<!--  * tip_link: Name of the last link in the kinematic chain, usually the tool link. -->
  * tip_link: 運動学的なつながりの最端部のリンク名（多くの場合はツールリンク）
<!--  * base_link: Name for the base link of the robot. -->
  * base_link: ロボットの基礎部分のリンク名
<!--  * world_frame: The absolute coordinate frame of reference for all the objects defined in the planning environment. -->
  * world_frame: 動作計画空間上に定義された全ての物体が参照する絶対座標系フレーム
<!-- * The parameters under the "`trajectory`" namespace are used to generate the trajectory that is feed into the Descartes planner. -->
  * "trajectory" 名前空間にあるパラメータは直交座標系プランナに供給される軌道の生成に用いられます．
<!--  * trajectory/seed_pose: This is of particular importance because it is used to indicate preferred start and end joint configurations of the robot when planning the path.  If a ''seed_pose'' wasn't specified then planning would take longer since multiple start and end joint configurations would have to be taken into account, leading to multiple path solutions that result from combining several start and end poses. -->
  * trajectory/seed_pose: 特に重要なパラメータで，軌道計画時に理想的な開始と終了のロボット関節設定を示すために使われます．
  "seed_pose" が指定されていない場合，複数の開始と終了の関節設定を考慮して，いくつかの開始・終了の姿勢を組み合わせた複数の軌道解を得るために動作計画の時間が長くなってしまいます．
