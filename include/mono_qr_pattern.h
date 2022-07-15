#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <velo2cam_utils.h>
#include <Eigen/Geometry>

#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>

#include <iostream>
#include <functional>
#include <params.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>


class MonoCameraFeatureDetector{

public:
  
  MonoCameraFeatureDetector(MonoConfig& config);
  MonoCameraFeatureDetector(cv::Mat& K, cv::Mat& D, MonoConfig& config);
  ~MonoCameraFeatureDetector() = default;
  MonoCameraFeatureDetector() = default;
  cv::Point2f projectPointDist(cv::Point3f pt_cv, const cv::Mat& intrinsics, const cv::Mat& distCoeffs );
  
  // Return the frame used in calculating the cumulative cloud, return -1 if cloud is invalid
  
  int imageCallback(const cv::Mat& image, const cv::Mat& K, const cv::Mat& D,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr& clusters_cloud);

  void setReady();
  bool isReady();


private:
  void publishROSCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_in);
  std::pair<cv::Vec3d, cv::Vec3f> getAverageBoardPose(std::vector<cv::Vec3d>& rvecs,
                           std::vector<cv::Vec3d>& tvecs, int n_marker);
  int imageCallback(const cv::Mat& image, pcl::PointCloud<pcl::PointXYZ>::Ptr& clusters_cloud);
  void estimateBoardCorners(std::vector<std::vector<cv::Point3f>>& boardCorners);
  void estimateBoardCircles(std::vector<cv::Point3f>& boardCircleCenters);
  void transformToCameraFrame(std::vector<cv::Point3f>& points, const cv::Vec3d& rvec,
                              const cv::Vec3d& tvec, pcl::PointCloud<pcl::PointXYZ>::Ptr & candidates_cloud);

  bool validateCentersCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& candidates_cloud,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr& centers_cloud);
  void clearCumulativeCloud();

  void transformCloudCoordinateFrame(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
                                const Eigen::Quaternionf& rotation);

  cv::Ptr<cv::aruco::Dictionary> mDictionary;

  int mFramesProc = 0, mFrameUsed = 0;
  pcl::PointCloud<pcl::PointXYZ>::Ptr mCumulativeCloud;

  MonoConfig mEnvironmentConfig;
 
  bool mWarmUpDone = false;

  cv::Mat mCameraIntrinsic{3,3, CV_32F};
  cv::Mat mDistCoeffs{1, 5, CV_32F};

  std::vector<int> mBoardIds{0, 1, 3, 2};  // Zigzag ordering

  // std::function<void (pcl::PointCloud<pcl::PointXYZ>::ConstPtr, long)> mCbPub;
  ros::Publisher mCumulativePub;
  ros::NodeHandle nh_;
};
