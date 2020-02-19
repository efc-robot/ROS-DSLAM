#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>   
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/String.h>
#include <dslam_sp/image_depth.h>
#include <dslam_sp/Point2f.h>
#include <dslam_sp/KeyPoint.h>
#include <dslam_sp/KeyPoints_vector.h>
#include <dslam_sp/Descriptor.h>
#include <dslam_sp/Descriptors_vector.h>
#include <dslam_sp/EF_output.h>
#include <dslam_sp/top_k.h>
#include <dslam_sp/bundleAdjustment.h>

#include <boost/make_shared.hpp>
#include <sstream>
// #include <dslam_sp/SuperPoint.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <chrono>


using namespace cv;
using namespace std;

#define KEEP_K_POINTS 200
#define MATCHER "BF"
#define NN_thresh 0.7

sensor_msgs::CameraInfo camera_info;
int frame_id = 0;
char FileDir[200] = "/home/yujc/robotws/dataset/image_503_loop";
