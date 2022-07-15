#pragma once
#define PCL_NO_PRECOMPILE

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <velo2cam_utils.h>

#include <iostream>
#include <functional>
#include <string>
#include <params.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

namespace pcl
{

// mimicking velodyne's - https://github.com/ros-drivers/velodyne/blob/master/velodyne_pcl/include/velodyne_pcl/point_types.h
struct EIGEN_ALIGN16 PointXYZIRT {
    PCL_ADD_POINT4D;
    float intensity;
    std::uint16_t ring;
    float time; // This time is relative to the last point received (which should be approx equivalent to timestamp of the pointcloud), so the earliest point should be around -0.1 
    float distance;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace lslidar_c16_decoder

POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::PointXYZIRT,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint16_t, ring, ring)
    (float, time, time)
)

class LidarFeatureDetector {
public:
    LidarFeatureDetector (LidarConfig config);
    ~LidarFeatureDetector() = default;
    LidarFeatureDetector() = default;
    int callback(const pcl::PointCloud<Velodyne::Point>::Ptr &laser_cloud,
                 pcl::PointCloud<pcl::PointXYZ>::Ptr &centers_cloud);
    void setReady();
    bool isReady();

private:
    void publishROSCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_in);
    bool validateCentersCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& candidates_cloud,
                            Eigen::Affine3f& rotation, 
                            pcl::ModelCoefficients::Ptr& coefficients,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr& centers_cloud);

    void rangePassThroughFilterCloud(pcl::PointCloud<Velodyne::Point>::Ptr& cloud_in, 
                                pcl::PointCloud<Velodyne::Point>::Ptr& cloud_out);
    bool planeSegmentation(pcl::PointCloud<Velodyne::Point>::Ptr& cloud_in,
                            pcl::ModelCoefficients::Ptr& coefficients, pcl::PointIndices::Ptr& inliers);

    bool edgeCloudFilter(pcl::PointCloud<Velodyne::Point>::Ptr& cloud_in,
                        pcl::PointCloud<Velodyne::Point>::Ptr& cloud_out);

    bool planeEdgeCloudFilter(pcl::PointCloud<Velodyne::Point>::Ptr& cloud_in,
                        Eigen::VectorXf& plane_coeffs,
                        pcl::PointCloud<Velodyne::Point>::Ptr& cloud_out);
    bool velodyneRingsToPointCloud(std::vector<std::vector<Velodyne::Point*>>& rings_in,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out);

    bool circleSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
                            double zcoord_xyplane,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out);
    void clearCumulativeCloud();
    // Dynamic parameters
    bool WARMUP_DONE = false;
    int clouds_proc_ = 0, clouds_used_ = 0;

    double threshold_;


    LidarConfig mEnvironmentConfig;

    ros::Publisher mCumulativePub;
    ros::NodeHandle nh_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr mCumulativeCloud;
    // std::function<void(pcl::PointCloud<pcl::PointXYZ>::Ptr, long)> mCbPub;



};