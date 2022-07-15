#include <stereo_pattern.h>
//TODO : Add debugging and save to file feature


StereoCameraFeatureDetector::StereoCameraFeatureDetector(const StereoConfig& config):
    delta_width_circles_(config.delta_width_circles),
    delta_height_circles_(config.delta_height_cirlces),
    plane_distance_inliers_(config.plane_distance_inliers),
    target_radius_tolerance_(config.target_radius_tolerance),
    cluster_tolerance_(config.cluster_tolerance),
    min_cluster_factor_(config.min_cluster_factor),
    min_centers_found_(config.min_centers_found),
    skip_warmup_(config.skip_warmup),
    save_to_file_(config.save_to_file),
    circle_threshold_(config.circle_threshold)
{
  if (skip_warmup_) {
    std::cout << "Skipping warmup" << std::endl;
    WARMUP_DONE = true;
  }

  cumulative_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);

}


void StereoCameraFeatureDetector::pushPointCloudCbFn(std::function<void(pcl::PointCloud<pcl::PointXYZ>::Ptr, long)> cbIn){
  mCbPub = std::move(cbIn);
}

// TODO: Callback on subscription to cloud 2 and coefficient plane
void StereoCameraFeatureDetector::callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &camera_cloud,
              const Eigen::VectorXf &coefficients_plane) {
//   if (DEBUG) ROS_INFO("[Stereo] Processing image...");

  images_proc_++;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cam_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());

  pcl::copyPointCloud(*camera_cloud, *cam_cloud);

  // 1.FILTER THE EDGES-CLOUD ACCORDING TO THE DETECTED PLANE
  // Segment inliers
  Eigen::VectorXf coefficients_v(4);
  coefficients_v = coefficients_plane;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cam_plane_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr dit(
      new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cam_cloud));
  std::vector<int> plane_inliers;
  dit->selectWithinDistance(coefficients_v, plane_distance_inliers_,
                            plane_inliers);
  pcl::copyPointCloud<pcl::PointXYZ>(*cam_cloud, plane_inliers,
                                     *cam_plane_cloud);

  // Publish plane as "plane_edges_cloud"
//   if (DEBUG) {
//     PointCloud2 plane_edges_ros;
//     pcl::toROSMsg(*cam_plane_cloud, plane_edges_ros);
//     plane_edges_ros.header = camera_cloud->header;
//     plane_edges_pub.publish(plane_edges_ros);
//   }

  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  // 2.ROTATE CLOUD TO FACE PATTERN PLANE
  pcl::PointCloud<pcl::PointXYZ>::Ptr xy_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  Eigen::Vector3f xy_plane_normal_vector, floor_plane_normal_vector;
  xy_plane_normal_vector[0] = 0.0;
  xy_plane_normal_vector[1] = 0.0;
  xy_plane_normal_vector[2] = -1.0;

  floor_plane_normal_vector[0] = coefficients_plane(0);
  floor_plane_normal_vector[1] = coefficients_plane(1);
  floor_plane_normal_vector[2] = coefficients_plane(2);

  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cam_plane_cloud, *cam_plane_cloud, indices);

  Eigen::Affine3f rotation =
      getRotationMatrix(floor_plane_normal_vector, xy_plane_normal_vector);
  pcl::transformPointCloud(*cam_plane_cloud, *xy_cloud, rotation);

  pcl::PointCloud<pcl::PointXYZ>::Ptr aux_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointXYZ aux_point;
  aux_point.x = 0;
  aux_point.y = 0;
  aux_point.z = (-coefficients_v(3) / coefficients_v(2));
  aux_cloud->push_back(aux_point);

  pcl::PointCloud<pcl::PointXYZ>::Ptr auxrotated_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*aux_cloud, *auxrotated_cloud, rotation);

  double zcoord_xyplane = auxrotated_cloud->at(0).z;

  // Force pattern points to belong to computed plane
  for (pcl::PointCloud<pcl::PointXYZ>::iterator pt = xy_cloud->points.begin();
       pt < xy_cloud->points.end(); ++pt) {
    pt->z = zcoord_xyplane;
  }

  // Publishing "xy_pattern" (pattern transformed to be aligned with XY)
//   if (DEBUG) {
//     PointCloud2 xy_pattern_ros;
//     pcl::toROSMsg(*xy_cloud, xy_pattern_ros);
//     xy_pattern_ros.header = camera_cloud->header;
//     xy_pattern_pub.publish(xy_pattern_ros);
//   }

  // 3.FIND CIRCLES
  pcl::SACSegmentation<pcl::PointXYZ> seg;

  seg.setModelType(pcl::SACMODEL_CIRCLE2D);
  seg.setDistanceThreshold(circle_threshold_);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setOptimizeCoefficients(true);
  seg.setMaxIterations(1000);
  seg.setRadiusLimits(TARGET_RADIUS - target_radius_tolerance_,
                      TARGET_RADIUS + target_radius_tolerance_);

  pcl::PointCloud<pcl::PointXYZ>::Ptr circle_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);

  std::vector<std::vector<float>> found_centers;

  do {
    seg.setInputCloud(xy_cloud);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0) {
      if (found_centers.size() < 1) {
        // TODO: Warning level
        std::cout <<
            "[Stereo] Could not estimate a circle model for the given "
            "dataset." << std::endl;
      }
      break;
    } else {
      if (DEBUG)
        std::cout << "[Stereo] Found circle: " << inliers->indices.size() << ", ";
    }

    // Extract the inliers
    extract.setInputCloud(xy_cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*circle_cloud);

    // Add center point to cloud (only if it makes sense)
    pcl::PointXYZ center;
    center.x = *coefficients->values.begin();
    center.y = *(coefficients->values.begin() + 1);
    center.z = zcoord_xyplane;

    std::vector<float> found_center;
    found_center.push_back(center.x);
    found_center.push_back(center.y);
    found_center.push_back(center.z);
    found_centers.push_back(found_center);

    // Remove inliers from pattern cloud to find next circle
    extract.setNegative(true);
    extract.filter(*temp_cloud);
    xy_cloud.swap(temp_cloud);

  } while (xy_cloud->points.size() > 3);

  if (found_centers.size() <
      min_centers_found_) {  // Usually min_centers_found_ = TARGET_NUM_CIRCLES
    // Exit 1: centers not found
    // TODO: Warning level,, std
    std::cout << "Not enough centers: " <<  found_centers.size() << std::endl; 
    return;
  }

  /**
    4. GEOMETRIC CONSISTENCY CHECK
    At this point, circles' center candidates have been computed
  (found_centers). Now we need to select the set of 4 candidates that best fit
  the calibration target geometry. To that end, the following steps are
  followed: 1) Create a cloud with 4 points representing the exact geometry of
  the calibration target 2) For each possible set of 4 points: compute
  similarity score 3) Rotate back the candidates with the highest score to their
  original position in the cloud, and add them to cumulative cloud
  **/
  std::vector<std::vector<int>> groups;
  comb(found_centers.size(), TARGET_NUM_CIRCLES, groups);
  double groups_scores[groups.size()];  // -1: invalid; 0-1 normalized score
  for (int i = 0; i < groups.size(); ++i) {
    std::vector<pcl::PointXYZ> candidates;
    // Build candidates set
    for (int j = 0; j < groups[i].size(); ++j) {
      pcl::PointXYZ center;
      center.x = found_centers[groups[i][j]][0];
      center.y = found_centers[groups[i][j]][1];
      center.z = found_centers[groups[i][j]][2];
      candidates.push_back(center);
    }

    // Compute candidates score
    Square square_candidate(candidates, delta_width_circles_,
                            delta_height_circles_);
    groups_scores[i] = square_candidate.is_valid()
                           ? 1.0
                           : -1;  // -1 when it's not valid, 1 otherwise
  }

  int best_candidate_idx = -1;
  double best_candidate_score = -1;
  for (int i = 0; i < groups.size(); ++i) {
    if (best_candidate_score == 1 && groups_scores[i] == 1) {
      // Exit 2: Several candidates fit target's geometry
      // TODO: Error level
      std::cout << 
          "[Stereo] More than one set of candidates fit target's geometry. "
          "Please, make sure your parameters are well set. Exiting callback" << std::endl;
      return;
    }
    if (groups_scores[i] > best_candidate_score) {
      best_candidate_score = groups_scores[i];
      best_candidate_idx = i;
    }
  }

  if (best_candidate_idx == -1) {
    // Exit 3: No candidates fit target's geometry
    // TODO: Warining level
    std::cout << 
        "[Stereo] Unable to find a candidate set that matches target's "
        "geometry" << std::endl;
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr rotated_back_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  for (int j = 0; j < groups[best_candidate_idx].size(); ++j) {
    pcl::PointXYZ point;
    point.x = found_centers[groups[best_candidate_idx][j]][0];
    point.y = found_centers[groups[best_candidate_idx][j]][1];
    point.z = found_centers[groups[best_candidate_idx][j]][2];

    pcl::PointXYZ center_rotated_back =
        pcl::transformPoint(point, rotation.inverse());
    center_rotated_back.z =
        (-coefficients_plane(0) * center_rotated_back.x -
         coefficients_plane(1) * center_rotated_back.y -
         coefficients_plane(3)) /
        coefficients_plane(2);

    rotated_back_cloud->push_back(center_rotated_back);
    cumulative_cloud->push_back(center_rotated_back);
  }

  // Publishing "cumulative_cloud" (centers found from the beginning)
//   if (DEBUG) {
//     PointCloud2 cumulative_ros;
//     pcl::toROSMsg(*cumulative_cloud, cumulative_ros);
//     cumulative_ros.header = camera_cloud->header;
//     cumulative_pub.publish(cumulative_ros);
//   }

//   pcl_msgs::PointIndices p_ind;

//   pcl_conversions::moveFromPCL(*inliers, p_ind);
//   p_ind.header = camera_cloud->header;

//   pcl_msgs::ModelCoefficients m_coeff;

//   pcl_conversions::moveFromPCL(*coefficients, m_coeff);
//   m_coeff.header = camera_cloud->header;

//   if (DEBUG) {
//     inliers_pub.publish(p_ind);
//     coeff_pub.publish(m_coeff);
//   }

  images_used_++;
//   if (DEBUG) {
//     ROS_INFO("[Stereo] %d/%d frames: %ld pts in cloud", images_used_,
//              images_proc_, cumulative_cloud->points.size());
//   }
  pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);

  // Compute circles centers
  if (!WARMUP_DONE) {  // Compute clusters from detections in the latest frame
    getCenterClusters(cumulative_cloud, final_cloud, cluster_tolerance_, 1, 1);
  } else {  // Use cumulative information from previous frames
    getCenterClusters(cumulative_cloud, final_cloud, cluster_tolerance_,
                      min_cluster_factor_ * images_used_, images_used_);
    if (final_cloud->points.size() > TARGET_NUM_CIRCLES) {
      getCenterClusters(cumulative_cloud, final_cloud, cluster_tolerance_,
                        3.0 * images_used_ / 4.0, images_used_);
    }
  }

  // Exit 4: clustering failed
  if (final_cloud->points.size() == TARGET_NUM_CIRCLES) {
    mCbPub(final_cloud, images_used_);
  }


  // Clear cumulative cloud during warm-up phase
  if (!WARMUP_DONE) {
    cumulative_cloud->clear();
    images_proc_ = 0;
    images_used_ = 0;
  }
}

