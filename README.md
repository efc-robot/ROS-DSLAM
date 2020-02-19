# ROS-based DSLAM

该项目的目的是构建完全给予ROS的模块化 DLSAM 系统 （现在只完成了单一VO）

## 准备工作

    - 项目基于ubuntu，需要安装 ROS
    - 需要安装 caffe-ssd ，https://github.com/efc-robot/caffe-ssd.git
    - 项目包括子模块，superpoint，采用 git clone --recursive 即可克隆子模块

## 编译使用

    - 切换到工作空间，并新建src文件家
        ```Shell
        cd ~/catkin_ws_d
        mkdir src
        cd  src
        ```
    
    - 下载代码并且更新子模块
        ```Shell
        git clone --recursive https://github.com/efc-robot/ROS-DSLAM.git
        cd ROS-DSLAM
        ```

    - 使用catkin_make 编译
        ```Shell
        cd ~/catkin_ws_d
        catkin_make
        ```

## 在ROS中运行
    要先准备数据集，不然会报错，数据集可以问xuzhilin要。
    
    ```
    source ~/catkin_ws_d/devel/setup.bash
    roslaunch dslam_sp SP_VO_file.launch
    ```

    但是SP可能需要显卡资源比较多。
    可以只跑orb的方法。然后在rviz中可视化。
    ```
    source ~/catkin_ws_d/devel/setup.bash
    roslaunch dslam_sp ORB_VO_file.launch
    ```

    注意修改 launch file 中的路径。
    如果不想编译caffe，只想用orb，可以在CMakeLists.txt 中修改，不再变异SP相关的文件。
    
