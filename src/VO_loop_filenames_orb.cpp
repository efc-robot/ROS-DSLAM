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

#define KEEP_K_POINTS 1000
#define MATCHER "BF"

string g_window_name;
// SuperPoint superpoint;
ros::Publisher pub;
sensor_msgs::CameraInfo camera_info;
int frame_id = 0;
char FileDir[200] = "/PATH/TO/DATASET/";

void info_Callback(const sensor_msgs::CameraInfo::ConstPtr &msg)
{  
    camera_info.P = msg->P;
}

void KeyPoint_cv2ros(cv::KeyPoint cv_kp, dslam_sp::KeyPoint& ros_kp)
{
    ros_kp.angle = cv_kp.angle;
    ros_kp.class_id = cv_kp.class_id;
    ros_kp.octave = cv_kp.octave;
    ros_kp.pt.x = cv_kp.pt.x;
    ros_kp.pt.y = cv_kp.pt.y;
    ros_kp.response = cv_kp.response;
    ros_kp.size = cv_kp.size;
}


Point2d pixel2cam ( const Point2d& p, const Mat& K )
{
    return Point2d
           (
               ( p.x - K.at<double> ( 0,2 ) ) / K.at<double> ( 0,0 ),
               ( p.y - K.at<double> ( 1,2 ) ) / K.at<double> ( 1,1 )
           );
}

void fnames_Callback(const std_msgs::String::ConstPtr &msg, Ptr<FeatureDetector> &detector, Ptr<DescriptorExtractor> &descriptor){
    
    char fname1[200],fname2[200],dfname1[200],dfname2[200];
    int intfname1,intfname2;
    stringstream sstream;
    sstream << msg->data.c_str();
    sstream >> intfname1;
    sstream >> intfname2;
    cout<< "fname1 : " << intfname1 <<  ". fname2 : " << intfname2 << endl;
    
    sprintf(fname1, "%s/left_%d.png", FileDir, intfname1);
    sprintf(fname2, "%s/left_%d.png", FileDir, intfname2);
    sprintf(dfname1, "%s/depth_%d.png", FileDir, intfname1);
    sprintf(dfname2, "%s/depth_%d.png", FileDir, intfname2);
    Mat Image1 = imread ( fname1, CV_LOAD_IMAGE_UNCHANGED );
    Mat Image2 = imread ( fname2, CV_LOAD_IMAGE_UNCHANGED );
    Mat DImage1 = imread ( dfname1, CV_LOAD_IMAGE_UNCHANGED );
    Mat DImage2 = imread ( dfname2, CV_LOAD_IMAGE_UNCHANGED );
    Mat image1,image2;
    Mat depth1,depth2;

    sensor_msgs::CameraInfo camera_info_tmp = camera_info;
    if((float)Image1.rows/(float)Image1.cols  > 0.75) {
        int height_ = Image1.cols /4 *3;
        int margin = (Image1.rows - height_)/2;
        image1 = Image1(Rect(0,margin,Image1.cols,height_));
        depth1 = DImage1(Rect(0,margin,Image1.cols,height_));
        image2 = Image2(Rect(0,margin,Image1.cols,height_));
        depth2 = DImage2(Rect(0,margin,Image1.cols,height_));
        camera_info_tmp.P[6] = camera_info_tmp.P[6] - margin;
    }
    else if((float)Image1.rows/(float)Image1.cols < 0.75) {
        int width_ = Image1.rows /3 *4;
        int margin = (Image1.cols - width_)/2;
        image1 = Image1(Rect(margin,0,width_,Image1.rows));
        depth1 = DImage1(Rect(margin,0,width_,Image1.rows));
        image2 = Image2(Rect(margin,0,width_,Image1.rows));
        depth2 = DImage2(Rect(margin,0,width_,Image1.rows));
        camera_info_tmp.P[2] = camera_info_tmp.P[2] - margin;
        // cout <<  "width_" << width_ << "  margin" << margin << endl;
    }

    cout <<  "finish p1" << endl;
    
    Size dsize = Size(640,480);
    if(image1.size() != dsize){
        resize(image1, image1, dsize);
        resize(image2, image2, dsize);
        resize(depth1, depth1, dsize);
        resize(depth2, depth2, dsize);
        camera_info_tmp.P[2] = camera_info_tmp.P[2] * (640.0/image1.cols);
        camera_info_tmp.P[6] = camera_info_tmp.P[6] * (480.0/image1.rows);
    }   

    std::vector<cv::KeyPoint> kpts1,kpts2;
    std::vector<std::vector<float> > dspts1, dspts2;

    Mat descriptor_results1, descriptor_results2;
    detector->detect ( image1,kpts1 );
    detector->detect ( image2,kpts2 );
    //NMS
    // selectMax(NMS_Threshold, kpts);
    cout << "keypoints size:" << kpts1.size() << endl;
    cout << "keypoints size:" << kpts2.size() << endl;

    //-- 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute ( image1, kpts1, descriptor_results1 );
    cout << "descriptor_results: "<< descriptor_results1.size() << endl;
    cout << "cols: "<< descriptor_results1.cols << endl;
    cout << "rows: "<< descriptor_results1.rows << endl;
    cout << "type: "<< descriptor_results1.type() << endl;

    for (int i = 0;  i< kpts1.size(); i++){
        std::vector<float> dspts_tmp;
        dspts_tmp.resize(32);
        uchar* pData = descriptor_results1.ptr<uchar>(i);
        for (int j = 0; j < descriptor_results1.cols; j++){
            dspts_tmp[j] = (float)pData[j] ;
        }
        // cout << descriptor_results1.at()
        dspts1.push_back(dspts_tmp);
    }

    
    descriptor->compute ( image2, kpts2, descriptor_results2 );
    cout << "descriptor_results: "<< descriptor_results2.size() << endl;
    cout << "cols: "<< descriptor_results2.cols << endl;
    cout << "rows: "<< descriptor_results2.rows << endl;
    cout << "type: "<< descriptor_results2.type() << endl;

    for (int i = 0;  i< kpts2.size(); i++){
        std::vector<float> dspts_tmp;
        dspts_tmp.resize(32);
        uchar* pData = descriptor_results2.ptr<uchar>(i);
        for (int j = 0; j < descriptor_results2.cols; j++){
            dspts_tmp[j] = (float)pData[j] ;
        }
        // cout << descriptor_results2.at()
        dspts2.push_back(dspts_tmp);
    }



    
    std::vector<dslam_sp::KeyPoint> keypoints1,keypoints2;
    std::vector<dslam_sp::Descriptor> descriptors1,descriptors2;
    for(int i = 0; i < kpts1.size(); i++){ 
        
        dslam_sp::KeyPoint kp_ros;
        KeyPoint_cv2ros(kpts1[i],kp_ros);
        keypoints1.push_back(kp_ros);
        
        dslam_sp::Descriptor ds_ros;
        ds_ros.desc = dspts1[i];
        descriptors1.push_back(ds_ros);
    }
    for(int i = 0; i < kpts2.size(); i++){ 
        
        dslam_sp::KeyPoint kp_ros;
        KeyPoint_cv2ros(kpts2[i],kp_ros);
        keypoints2.push_back(kp_ros);
        
        dslam_sp::Descriptor ds_ros;
        ds_ros.desc = dspts2[i];
        descriptors2.push_back(ds_ros);
    }

    
    cout <<  "finish ExactSP" << endl;
    // cout << "LOG start" << endl;
    // for ( auto keypoint1 : keypoints1){
    //     cout << keypoint1 << endl;
    // }
    // for ( auto descriptor1 : descriptors1){
    //     cout << descriptor1 << endl;
    // }
    // cout << desc_prev << endl;
    // cout << depth_prev << endl;

    // for ( auto curr_point : points_curr){
    //     cout << curr_point << endl;
    // }
    // cout << desc_curr << endl;
    // cout << depth_curr << endl;


    if(keypoints1.size() > KEEP_K_POINTS) {
        top_k(keypoints1, descriptors1, keypoints1.size(), KEEP_K_POINTS);
    }
    if(keypoints2.size() > KEEP_K_POINTS) {
        top_k(keypoints2, descriptors2, keypoints2.size(), KEEP_K_POINTS);
    }

    vector<Point2f> points_curr;
    Mat desc_curr;
    desc_curr.create( int(keypoints2.size()), int(descriptors2[0].desc.size()), CV_8UC1);
    for(int i=0; i<keypoints2.size(); i++)
    {
        points_curr.push_back(Point2f(keypoints2[i].pt.x, keypoints2[i].pt.y));
        uchar* pData = desc_curr.ptr<uchar>(i);   //第i+1行的所有元素  
        for(int j = 0; j < desc_curr.cols; j++){
            pData[j] = (uchar) descriptors2[i].desc[j];
        }
    }

    
    vector<Point2f> points_prev;
    Mat desc_prev;
    desc_prev.create( int(keypoints1.size()), int(descriptors1[0].desc.size()), CV_8UC1);
    for(int i=0; i<keypoints1.size(); i++)
    {
        points_prev.push_back(Point2f(keypoints1[i].pt.x, keypoints1[i].pt.y));
        uchar* pData = desc_prev.ptr<uchar>(i);   //第i+1行的所有元素  
        for(int j = 0; j < desc_prev.cols; j++){
            pData[j] = (uchar) descriptors1[i].desc[j];
        }
        
    }
    
    cv::Mat depth_curr = depth2;
    cv::Mat depth_prev = depth1;
        //判定是否有prev数据，第一帧不做match
    if(true)
    {
        //match
        // cout << "LOG start" << endl;
        // for ( auto prev_point : points_prev){
        //     cout << prev_point << endl;
        // }
        // cout << desc_prev << endl;
        // cout << depth_prev << endl;

        // for ( auto curr_point : points_curr){
        //     cout << curr_point << endl;
        // }
        // cout << desc_curr << endl;
        // cout << depth_curr << endl;
        

        vector<DMatch> matches;
        if( MATCHER == "BF" ) {

            Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create ( "BruteForce-Hamming" );
            cout << "BF1" << endl;
            matcher->match ( desc_prev, desc_curr, matches );
            cout << "BF2" << endl;
            // BFMatcher matcher(NORM_L2, true);
            // matcher.match(desc_prev, desc_curr, matches);
        }
        else {
            FlannBasedMatcher matcher;
            matcher.match(desc_prev, desc_curr, matches);
        }
        
        //-- 匹配点对筛选
        double min_dist=10000, max_dist=0;

        //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
        for ( int i = 0; i < desc_prev.rows; i++ )
        {
            double dist = matches[i].distance;
            if ( dist < min_dist ) min_dist = dist;
            if ( dist > max_dist ) max_dist = dist;
        }

        printf ( "-- Max dist : %f \n", max_dist );
        printf ( "-- Min dist : %f \n", min_dist );



        //配对后的筛选
        vector <Point2f> RAN_KP1, RAN_KP2;
        for(int i=0; i<matches.size(); i++) {
            if ( matches[i].distance> max ( 3*min_dist, 30.0 ) )
                continue;
            // if ( abs(points1[matches[i].queryIdx].x-points2[matches[i].trainIdx].x)> Width/10 )
                // continue;
            // if ( abs(points1[matches[i].queryIdx].y-points2[matches[i].trainIdx].y)> Height/10 )
                // continue;
            RAN_KP1.push_back(points_prev[matches[i].queryIdx]);
            RAN_KP2.push_back(points_curr[matches[i].trainIdx]);
            //cout << matches[i].distance << endl;
            //RAN_KP1是要存储img01中能与img02匹配的点
        }
        cout <<  "RAN_KP1 size:" << RAN_KP1.size() << endl;
        
        //通过RansacStatus来删除误匹配点
        vector<uchar> RansacStatus;
        Mat Fundamental = findFundamentalMat(RAN_KP1, RAN_KP2, RansacStatus, FM_RANSAC);
        
        //2d to 3d的投射
        vector<Point3f> points_3d;
        vector<Point2f> points_2d;        //vectors to store the coordinates of the feature points
        Mat K = ( Mat_<double> ( 3,3 ) << camera_info_tmp.P[0], 0, camera_info_tmp.P[2], 0, camera_info_tmp.P[5], camera_info_tmp.P[6], 0, 0, 1 );
        for ( int i=0; i<RAN_KP1.size(); i++ )
        {
            if (RansacStatus[i] == 0)
                continue;
            ushort d = depth_prev.ptr<unsigned short> (int ( RAN_KP1[i].y )) [ int ( RAN_KP1[i].x ) ];
            if ( d == 0 )   // bad depth
                continue;
            float dd = d/1000.0;
            Point2d p1 = pixel2cam ( RAN_KP1[i], K );
            points_3d.push_back ( Point3f ( p1.x*dd, p1.y*dd, dd ) );
            points_2d.push_back ( RAN_KP2[i] );
        }
        // cout<<"3d-2d pairs: "<<points_3d.size() <<endl;
        // imshow( g_window_name, img_2 );
        // waitKey(1);
        
        if (points_3d.size()<8) {
            ROS_INFO("3d-2d pairs:%d",  points_3d.size());
            return;
        }
        
        Mat r, t, R;
        solvePnP ( points_3d, points_2d, K, Mat(), r, t, false, cv::SOLVEPNP_EPNP ); // 调用OpenCV 的 PnP 求解，可选择EPNP，DLS等方法
        cv::Rodrigues ( r, R ); // r为旋转向量形式，用Rodrigues公式转换为矩阵
        
        bundleAdjustment ( points_3d, points_2d, K, R, t );
        
        Eigen::Matrix3d R_eigen;
        cv2eigen(R, R_eigen);
        Eigen::Quaterniond q = Eigen::Quaterniond(R_eigen);
         
        cout <<  "finish PnP" << endl;
            
            geometry_msgs::TransformStamped TF_msg;
            TF_msg.header.frame_id = std::to_string(intfname2);
            TF_msg.child_frame_id = std::to_string(intfname1);
            TF_msg.transform.translation.x = t.at<double>(0);
            TF_msg.transform.translation.y = t.at<double>(1);
            TF_msg.transform.translation.z = t.at<double>(2);
            TF_msg.transform.rotation.x = q.x();
            TF_msg.transform.rotation.y = q.y();
            TF_msg.transform.rotation.z = q.z();
            TF_msg.transform.rotation.w = q.w();
            pub.publish(TF_msg);//发布msg
            
            cout << "child_frame_id:" << TF_msg.child_frame_id << " frame_id:" << TF_msg.header.frame_id << endl;
            ROS_INFO("stamp:%d.%d translation:%f %f %f rotation:%f %f %f %f ;", TF_msg.header.stamp.sec, TF_msg.header.stamp.nsec,
                        TF_msg.transform.translation.x, TF_msg.transform.translation.y, TF_msg.transform.translation.z,
                        TF_msg.transform.rotation.x, TF_msg.transform.rotation.y, TF_msg.transform.rotation.z, TF_msg.transform.rotation.w); //输出

        }


}

int main(int argc, char **argv)
{
    int o;
    const char *optstring = "p:"; // 有三个选项-abc，其中c选项后有冒号，所以后面必须有参数
    while ((o = getopt(argc, argv, optstring)) != -1) {
        switch (o) {
            case 'p':
                strcpy(FileDir, optarg);
                break;
            default:
                break;
        }
    }
  Ptr<FeatureDetector> detector = ORB::create( 1000 );
  Ptr<DescriptorExtractor> descriptor = ORB::create();
  ros::init(argc, argv, "VO_loop", ros::init_options::AnonymousName);
  if (ros::names::remap("filenames") == "filenames") {
    ROS_WARN("Topic 'filenames' has not been remapped! Typical command-line usage:\n"
             "\t$ rosrun superpoint_EF superpoint_EF_node filenames:=<filenames topic> [transport]");
  }

  ros::NodeHandle n;
  std::string topic = n.resolveName("filenames");
  g_window_name = topic;
  
  pub = n.advertise<geometry_msgs::TransformStamped>("VO_loop_relpose", 1); //创建publisher，往"featurepoints_descriptors"话题上发布消息
  
  // namedWindow( g_window_name, WINDOW_AUTOSIZE );// Create a window for display.
  
  ros::Subscriber sub_info = n.subscribe("/mynteye/left_rect/camera_info", 1, info_Callback);
  ros::Subscriber sub = n.subscribe<std_msgs::String>(topic, 1, boost::bind(&fnames_Callback,_1, detector, descriptor)  );  //设置回调函数gpsCallback
  ros::spin(); //ros::spin()用于调用所有可触发的回调函数，将进入循环，不会返回，类似于在循环里反复调用spinOnce() 
  //而ros::spinOnce()只会去触发一次
  return 0;
}