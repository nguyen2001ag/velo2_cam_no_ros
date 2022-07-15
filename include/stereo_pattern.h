#define TARGET_NUM_CIRCLES 4

#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <velo2cam_utils.h>
#include <params.h>
#include <string>
#include <functional>
#include <iostream>

class StereoCameraFeatureDetector{
private:
    double delta_width_circles_, delta_height_circles_;
    double circle_threshold_;
    double plane_distance_inliers_;
    double target_radius_tolerance_;
    double cluster_tolerance_;
    double min_cluster_factor_;
    int min_centers_found_;
    
    bool skip_warmup_;
    bool save_to_file_;
    std::string savefile;
    
    int images_proc_ = 0, images_used_ = 0;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cumulative_cloud;    
    std::function<void(pcl::PointCloud<pcl::PointXYZ>::Ptr, long)> mCbPub;
    

public:
    bool WARMUP_DONE = false;

    StereoCameraFeatureDetector(const StereoConfig &config);
    ~StereoCameraFeatureDetector();
    StereoCameraFeatureDetector() = default;
    void callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &camera_cloud,
              const Eigen::VectorXf &coefficients_plane);
    void pushPointCloudCbFn(std::function<void(pcl::PointCloud<pcl::PointXYZ>::Ptr, long)> cbIn);
};