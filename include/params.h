#pragma once
#include <string>
#include <velo2cam_utils.h>

// #ifndef DEBUG
#define DEBUG 1

// TODO: Dynamic parameter not set

struct MonoConfig{
    double delta_width_circles{0.5};
    double delta_height_cirlces{0.4};
    double marker_size{0.18};
    double delta_width_qr_center{0.55};
    double delta_height_qr_center{0.35};
    int min_detected_markers{3};
    double cluster_tolerance{0.05};
    double min_cluster_factor{2.0/3.0};
    bool skip_warmup{false};
};


struct LidarConfig{
    double delta_width_circles{0.5};
    double delta_height_cirlces{0.4};
    double plane_threshold{0.1};
    double gradient_threshold{0.1};
    double plane_distance_inliers{0.1};
    double circle_threshold{0.05};
    double target_radius_tolerance{0.01};
    double cluster_tolerance{0.05};
    int min_centers_found_{TARGET_NUM_CIRCLES};
    double min_cluster_factor{0.5};
    int rings_count{64};

    bool skip_warmup{false};

    double passthrough_radius_min{1.0f};
    double passthrough_radius_max{20.0f};
    double circle_radius{0.12f};
    Eigen::Vector3f axis {0.0f, 0.0f, 1.0f};
    double angle_threshold{0.55f};
    double centroid_distance_min{0.15f};
    double centroid_distance_max{0.8f};


};

struct StereoConfig{
    double delta_width_circles{0.5};
    double delta_height_cirlces{0.4};
    double plane_distance_inliers{0.1};
    double target_radius_tolerance{0.01};
    double min_centers_found{TARGET_NUM_CIRCLES}; 
    double cluster_tolerance{0.05};
    double min_cluster_factor{2.0/3.0};
    bool skip_warmup{false};

    double circle_threshold;
};

struct ServerConfig{
    bool sync_iterations{false};
    bool save_to_file{false};
    bool skip_warmup{false};
    bool single_pose_mode{false};
    bool results_every_pose{false};
};