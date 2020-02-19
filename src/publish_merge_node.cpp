#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <dslam_sp/image_depth.h>

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "publish_merge_node");
  
  char FileDir[200] = "/home/yujc/robotws/dataset/image_503_loop/";
  if(argc >= 2) {
    snprintf(FileDir, sizeof(FileDir), argv[1]);
  }
  
  ros::NodeHandle n;
  
  int tmp = 1;
  // cv::namedWindow(FileDir);
  
  dslam_sp::image_depth img_depth_msg;
  sensor_msgs::CameraInfo camera_info;
  camera_info.P = {367.5128813981231, 0.0, 391.8034210205078, 0.0, 0.0, 367.5128813981231, 246.19532012939453, 0.0, 0.0, 0.0, 1.0, 0.0};
  
  ros::Publisher pub = n.advertise<dslam_sp::image_depth>("/merge/img_depth_file", 1); //创建publisher，往话题上发布消息
  ros::Publisher pub_info = n.advertise<sensor_msgs::CameraInfo>("/mynteye/left_rect/camera_info", 1); //创建publisher，往话题上发布消息
  ros::Publisher pub_image = n.advertise<sensor_msgs::Image>("/mynteye/left_rect/image", 1); //创建publisher，往话题上发布消息
  ros::Publisher pub_depth = n.advertise<sensor_msgs::Image>("/mynteye/left_rect/depth", 1); //创建publisher，往话题上发布消息
  ros::Rate loop_rate(6);   //定义发布的频率，20HZ 
  
  while (ros::ok())   //循环发布msg
  {
    char depth_name[200],image_name[200];
    
    sprintf(image_name, "%s/left_%d.png", FileDir, tmp);
    sprintf(depth_name, "%s/depth_%d.png", FileDir, tmp);
    
    Mat Image = imread ( image_name, CV_LOAD_IMAGE_UNCHANGED );
    Mat Depth = imread ( depth_name, CV_LOAD_IMAGE_UNCHANGED );
    // cout << "Image.type(): " << Image.type() << endl;
    // cout << "imshow" << image_name << endl;
    // cv::imshow(FileDir, Image);
    // cv::waitKey(1); 
    
    // cout << "CvImage.toImageMsg" << endl;
    sensor_msgs::ImagePtr ptr = cv_bridge::CvImage(std_msgs::Header(), "mono8", Image).toImageMsg();
    img_depth_msg.image = *ptr;
    ptr = cv_bridge::CvImage(std_msgs::Header(), "mono16", Depth).toImageMsg();
    img_depth_msg.depth = *ptr;

    img_depth_msg.header.frame_id = std::to_string(tmp);
    tmp++;
    
    cout << "publish: " << tmp << endl;
    if ( tmp %4 == 0){
      pub.publish(img_depth_msg);//以1Hz的频率发布msg
      pub_image.publish(img_depth_msg.image);
      pub_depth.publish(img_depth_msg.depth);
      pub_info.publish(camera_info);//以1Hz的频率发布msg
      loop_rate.sleep();//根据前面的定义的loop_rate,设置1s的暂停
    }
    
    // cv_bridge::CvImageConstPtr cv_ptr1;
    // sensor_msgs::ImagePtr img_ptr = boost::make_shared<::sensor_msgs::Image>(img_depth_msg.image);
    // cout << img_depth_msg.image.encoding << endl;
    // cv_ptr1=cv_bridge::toCvShare(img_ptr, img_depth_msg.image.encoding);
    // const cv::Mat &image_raw = cv_ptr1->image;
    // cv::imshow(FileDir, image_raw);
    // cv::waitKey(1); 
  }
  
  return 0;
}