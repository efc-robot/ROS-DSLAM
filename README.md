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
    
    - 下载代码
        ```Shell
        git clone --recursive https://github.com/efc-robot/ROS-DSLAM.git
        ```

    - 使用catkin_make 编译
        ```Shell
        cd ~/catkin_ws_d
        catkin_make
        ```

## 在ROS中运行
    
    ```
    source ~/catkin_ws_d/devel/setup.bash
    roslaunch dslam_sp SP_VO_file.launch
    ```
