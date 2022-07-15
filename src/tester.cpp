#include <iostream>
#include <calibration_server.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/CameraInfo.h>



bool lidarReceived = false;
bool cameraReceived = false;

pcl::PointCloud<Velodyne::Point>::Ptr cloudBuffer(new pcl::PointCloud<Velodyne::Point>);
cv::Mat imageBuffer;
cv::Mat cameraMatrix(3, 3, CV_32F);
cv::Mat distCoeffs(1, 5, CV_32F);

void lidar_callback(const sensor_msgs::PointCloud2::ConstPtr &laser_cloud)
{
    cloudBuffer->clear();
    //TODO : Change to Kritin PointType
    pcl::PointCloud<pcl::PointXYZIRT>::Ptr velocloud(new pcl::PointCloud<pcl::PointXYZIRT>);
    fromROSMsg(*laser_cloud, *velocloud);
    pcl::copyPointCloud(*velocloud, *cloudBuffer);
    if (cloudBuffer->size() == 0){
        std::cout << "No points in the cloud" << std::endl;
        return;
    }
    lidarReceived = true;
}


void camera_callback(const sensor_msgs::ImageConstPtr &msg, 
        const sensor_msgs::CameraInfoConstPtr &left_info)
{
  // Note that camera matrix is K, not P; both are interchangeable only
  // if D is zero
    cameraMatrix.at<float>(0, 0) = left_info->K[0];
    cameraMatrix.at<float>(0, 1) = left_info->K[1];
    cameraMatrix.at<float>(0, 2) = left_info->K[2];
    cameraMatrix.at<float>(1, 0) = left_info->K[3];
    cameraMatrix.at<float>(1, 1) = left_info->K[4];
    cameraMatrix.at<float>(1, 2) = left_info->K[5];
    cameraMatrix.at<float>(2, 0) = left_info->K[6];
    cameraMatrix.at<float>(2, 1) = left_info->K[7];
    cameraMatrix.at<float>(2, 2) = left_info->K[8];

    for (int i = 0; i < left_info->D.size(); i++)
        distCoeffs.at<float>(0, i) = left_info->D[i];

    cv_bridge::CvImageConstPtr cv_img_ptr;
    try {
        cv_img_ptr = cv_bridge::toCvShare(msg);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat image = cv_img_ptr->image;
    cv::undistort(image, imageBuffer, cameraMatrix, distCoeffs);
    cameraReceived = true;

    cv::imshow("test", imageBuffer);
    cv::waitKey(1);

}

bool pushDataFrame(CalibrationServer& server){
    if (lidarReceived && cameraReceived){
        lidarReceived = false;
        cameraReceived = false;
        
        if (server.pushSynchronisedData(cloudBuffer, imageBuffer, cameraMatrix, distCoeffs)){
            std::cout << "Successfully pushed camera and lidar frame" << std::endl;
            return true;
        }
            
    }
    return false;

}

int main(int argc, char **argv){
    ros::init(argc, argv, "tester");
    std::cout << "Starting....................................\n";
    ros::NodeHandle n;

    ServerConfig config;
    CalibrationServer server(config);

    std::string image_topic{"/camera/image"};
    std::string lidar_topic{"/lidar_points"};
    std::string cinfo_topic{"/camera/camera_info"};

    message_filters::Subscriber<sensor_msgs::Image> image_sub(n, image_topic,10);
    message_filters::Subscriber<sensor_msgs::CameraInfo> cinfo_sub(n, cinfo_topic, 10);

    ros::Subscriber lidar_sub = n.subscribe(lidar_topic, 10,  lidar_callback);


    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo>
        sync(image_sub, cinfo_sub, 1000);
    sync.registerCallback(boost::bind(&camera_callback, _1, _2));

    ros::Rate loop_rate(10);
    while(ros::ok()){
        ros::spinOnce();
        // ROS_INFO("Processing images and point clouds\n");
        if (pushDataFrame(server)){
            ROS_ERROR(" Extrinsic is ready\n");
            std::cout << server.getExtrinsics().rotation.toRotationMatrix() << std::endl;
            std::cout << server.getExtrinsics().translation << std::endl;
            return 0;
        }
        loop_rate.sleep();
    }
    return 0;

}