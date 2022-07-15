#define PCL_NO_PRECOMPILE

#include <pcl/registration/transformation_estimation_svd.h>

#include <Eigen/Geometry>
#include <velo2cam_utils.h>
#include <iostream>
#include <string>
#include <functional>
#include <params.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>

#include <lidar_pattern.h>
#include <mono_qr_pattern.h>
#include <stereo_pattern.h>


struct VizData
{
    std::vector<std::pair<int, int>> imageFeatures;
    std::vector<pcl::PointXYZ> lidarFeatures;
    std::vector<pcl::PointXYZ> cameraFeatures;
};

struct Extrinsics{
    Eigen::Vector3f translation;
    Eigen::Quaternionf rotation;
};

class CalibrationServer{
public:
    CalibrationServer(ServerConfig config);
    ~CalibrationServer() = default;
    CalibrationServer() = default;

    // TODO: Change to Kritin cloud type
    // TODO: Return if the data iscvalid
    bool pushSynchronisedData(const pcl::PointCloud<Velodyne::Point>::Ptr & lidar_cloud , 
                                    const cv::Mat& image, const cv::Mat& K, const cv::Mat& D);

    // Get the latest Extrinsics from the class
    Extrinsics getExtrinsics(); 
    bool removeDataPt(size_t dataInd);
    bool clearAllDataPts();

private:
    bool process_camera(pcl::PointCloud<pcl::PointXYZ>::Ptr camera_centroids
                        , long int frame_used);
    bool process_lidar(pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_centroids
                        , long int frame_used);
    bool calibrateExtrinsics();
    bool getExtrinsicCalibration();


protected:
    bool cameraReceived{false};
    bool lidarReceived{false};

    pcl::PointCloud<pcl::PointXYZ>::Ptr camera_cloud, lidar_cloud;
    //
    std::vector<pcl::PointXYZ> camera_vector{4};
    std::vector<pcl::PointXYZ> lidar_vector{4};

    bool skip_warmup, single_pose_mode;
    std::string camera_frame_id{"camera"};
    std::string lidar_frame_id{"lidar"};

    Extrinsics myExtrinsics;

    //TODO: If fixed size, use array instead
    std::vector<std::vector<std::tuple<pcl::PointCloud<pcl::PointXYZ>,
                                    std::vector<pcl::PointXYZ>>>>
        camera_buffer;
    std::vector<std::vector<std::tuple<pcl::PointCloud<pcl::PointXYZ>,
                                    std::vector<pcl::PointXYZ>>>>
        lidar_buffer;

    int CAMERA_WARMUP_COUNT = 0, LIDAR_WARMUP_COUNT = 0;
    int TARGET_POSITIONS_COUNT = 0;
    int TARGET_ITERATIONS = 50;

    bool sync_iterations;
    bool calibration_ended{false};
    bool results_every_pose;

    long int camera_count, lidar_count;
    
    MonoConfig mono_config;
    LidarConfig lidar_config;

    cv::Mat cameraIntrinsic{3, 3, CV_32F};
    cv::Mat distCoeffs{1, 5, CV_32F};

    MonoCameraFeatureDetector camera{cameraIntrinsic, distCoeffs, mono_config};
    LidarFeatureDetector lidar{lidar_config};

    ros::NodeHandle nh_;
    ros::Publisher verification_pub;
};

