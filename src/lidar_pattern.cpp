#define PCL_NO_PRECOMPILE

#include <lidar_pattern.h>

// TODO : Add debugging and save to file feature

LidarFeatureDetector::LidarFeatureDetector(const LidarConfig config):
  mEnvironmentConfig(config)
{

  if (mEnvironmentConfig.skip_warmup) {
    // TODO : Flag this as a warning
    std::cout << "Skipping warmup\n";
    WARMUP_DONE = true;
  }

  mCumulativeCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
  mCumulativePub = nh_.advertise<sensor_msgs::PointCloud2>("lidar_cumulative_cloud", 1);
}

// void LidarFeatureDetector::pushPointCloudCbFn(std::function<void(pcl::PointCloud<pcl::PointXYZ>::Ptr, long)> cbIn){
//   mCbPub = std::move(cbIn);
// }

// TODO: Callback from topic cloud 1, figure it out what is that
int LidarFeatureDetector::callback(const pcl::PointCloud<Velodyne::Point>::Ptr &laser_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr &centers_cloud)
{
  // if (DEBUG) ROS_INFO("[LiDAR] Processing cloud...");

  // laser_cloud is already xyz filtered

  pcl::PointCloud<Velodyne::Point>::Ptr velocloud(
      new pcl::PointCloud<Velodyne::Point>),
      velo_filtered(new pcl::PointCloud<Velodyne::Point>),
      pattern_cloud(new pcl::PointCloud<Velodyne::Point>),
      edges_cloud(new pcl::PointCloud<Velodyne::Point>);

  pcl::copyPointCloud(*laser_cloud, *velocloud);

  clouds_proc_++;

  Velodyne::addRange(*velocloud);

  rangePassThroughFilterCloud(velocloud, velo_filtered);

  // Plane segmentation
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

  

  if (!planeSegmentation(velo_filtered, coefficients, inliers)){
    std::cout << "[LiDAR] Could not estimate a planar model for the given dataset.\n";
    return -1;
  }

  // Copy coefficients to proper object for further filtering
  Eigen::VectorXf coefficients_v(4);
  coefficients_v(0) = coefficients->values[0];
  coefficients_v(1) = coefficients->values[1];
  coefficients_v(2) = coefficients->values[2];
  coefficients_v(3) = coefficients->values[3];

  
  

  if (!edgeCloudFilter(velocloud, edges_cloud)) {
    // Exit 2: pattern edges not found
    // TODO: Flag a warning here
    std::cout<< "[LiDAR] Could not detect pattern edges.\n";
    return -1;
  }

  ROS_WARN("[LiDar] Edge cloud has %d points\n", edges_cloud->size());

  // Get points belonging to plane in pattern pointcloud
  if (!planeEdgeCloudFilter(edges_cloud, coefficients_v, pattern_cloud)){
    std::cout << "Cannot detect edge from the plane" << std::endl;
    return -1;
  }

  // Velodyne specific info no longer needed for calibration
  // so standard PointXYZ is used from now on
  pcl::PointCloud<pcl::PointXYZ>::Ptr circles_cloud(new pcl::PointCloud<pcl::PointXYZ>),
      xy_cloud(new pcl::PointCloud<pcl::PointXYZ>),
      aux_cloud(new pcl::PointCloud<pcl::PointXYZ>),
      auxrotated_cloud(new pcl::PointCloud<pcl::PointXYZ>),
      centroid_candidates(new pcl::PointCloud<pcl::PointXYZ>);

  std::vector<std::vector<Velodyne::Point *>> rings2 =
      Velodyne::getRings(*pattern_cloud, mEnvironmentConfig.rings_count);

  if (!velodyneRingsToPointCloud(rings2, circles_cloud)){
    std::cout << "Failed to transform Velodyne Point Type to PointXYZ PCL" << std::endl;
    return -1;
  }

  // if (DEBUG) 
  //     publishROSCloud(circles_cloud);


  // Rotate cloud to face pattern plane
  Eigen::Vector3f xy_plane_normal_vector, floor_plane_normal_vector;
  xy_plane_normal_vector[0] = 0.0;
  xy_plane_normal_vector[1] = 0.0;
  xy_plane_normal_vector[2] = -1.0;

  floor_plane_normal_vector[0] = coefficients->values[0];
  floor_plane_normal_vector[1] = coefficients->values[1];
  floor_plane_normal_vector[2] = coefficients->values[2];

  Eigen::Affine3f rotation =
      getRotationMatrix(floor_plane_normal_vector, xy_plane_normal_vector);
  pcl::transformPointCloud(*circles_cloud, *xy_cloud, rotation);

  // if (DEBUG) 
  //     publishROSCloud(xy_cloud);

  // Publishing "rotated_pattern" cloud (plane transformed to be aligned with
  // XY)
//   if (DEBUG) {
//     sensor_msgs::PointCloud2 ros_rotated_pattern;
//     pcl::toROSMsg(*xy_cloud, ros_rotated_pattern);
//     ros_rotated_pattern.header = laser_cloud->header;
//     rotated_pattern_pub.publish(ros_rotated_pattern);
//   }

  // Force pattern points to belong to computed plane
  pcl::PointXYZ aux_point;
  aux_point.x = 0;
  aux_point.y = 0;
  aux_point.z = (-coefficients_v(3) / coefficients_v(2));
  aux_cloud->push_back(aux_point);
  pcl::transformPointCloud(*aux_cloud, *auxrotated_cloud, rotation);
  double zcoord_xyplane = auxrotated_cloud->at(0).z;

  for (pcl::PointCloud<pcl::PointXYZ>::iterator pt = xy_cloud->points.begin();
      pt < xy_cloud->points.end(); ++pt) {
    pt->z = zcoord_xyplane;
  }

  if (!circleSegmentation(xy_cloud, zcoord_xyplane, centroid_candidates)) {
    // Exit 3: all centers not found
    // TODO: Flag a warning here
    std::cout << "[LiDAR] Not enough centers: "<< centroid_candidates->size() << std::endl;
    return -1;
  }

  /**
    Geometric consistency check
    At this point, circles' center candidates have been computed
  (found_centers). Now we need to select the set of 4 candidates that best fit
  the calibration target geometry. To that end, the following steps are
  followed: 1) Create a cloud with 4 points representing the exact geometry of
  the calibration target 2) For each possible set of 4 points: compute
  similarity score 3) Rotate back the candidates with the highest score to their
  original position in the cloud, and add them to cumulative cloud
  **/
  
  if(!validateCentersCloud(centroid_candidates, rotation, coefficients, mCumulativeCloud)){
    std::cout << "Failed to validate circle point cloud" << std::endl;
    return -1;
  }


  xy_cloud.reset();  // Free memory

  ++clouds_used_;

  if (DEBUG)
    ROS_INFO("[LiDAR] %d/%d frames: %ld pts in cloud", clouds_used_,
            clouds_proc_, mCumulativeCloud->points.size());


  // Compute circles centers
  if (!WARMUP_DONE) {  // Compute clusters from detections in the latest frame
    getCenterClusters(mCumulativeCloud, centers_cloud, mEnvironmentConfig.cluster_tolerance, 1,
                      1);
  } else {  // Use cumulative information from previous frames
    getCenterClusters(mCumulativeCloud, centers_cloud, mEnvironmentConfig.cluster_tolerance,
                      mEnvironmentConfig.min_cluster_factor * clouds_used_, clouds_used_);
    if (centers_cloud->points.size() > TARGET_NUM_CIRCLES) {
      getCenterClusters(mCumulativeCloud, centers_cloud, mEnvironmentConfig.cluster_tolerance,
                        3.0 * clouds_used_ / 4.0, clouds_used_);
    }
  }

  // Exit 6: clustering failed
  if (centers_cloud->points.size() != TARGET_NUM_CIRCLES) {
    std::cout << "Not enough circles in a Point Cloud" << std::endl;
    return -1;
    
  }

  if (DEBUG) 
      publishROSCloud(centers_cloud);

  // if (save_to_file_) {
  //   savefile << std::endl;
  // }

  // Clear cumulative cloud during warm-up phase
  
  if (!WARMUP_DONE) {
    clearCumulativeCloud();
    clouds_proc_ = 0;
    clouds_used_ = 0;
  }
  ROS_WARN("[LIDAR] Frame used: %d\n", clouds_used_);
  return clouds_used_;
}


void LidarFeatureDetector::publishROSCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_in){
  sensor_msgs::PointCloud2 cumulative_pointcloud;
  pcl::toROSMsg(*cloud_in, cumulative_pointcloud);
  cumulative_pointcloud.header.frame_id = "map";
  mCumulativePub.publish(cumulative_pointcloud);
}


bool LidarFeatureDetector::validateCentersCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& candidates_cloud,
                            Eigen::Affine3f& rotation, 
                            pcl::ModelCoefficients::Ptr& coefficients,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr& centers_cloud)
{
  /**
    Geometric consistency check
    At this point, circles' center candidates have been computed
  (found_centers). Now we need to select the set of 4 candidates that best fit
  the calibration target geometry. To that end, the following steps are
  followed: 1) Create a cloud with 4 points representing the exact geometry of
  the calibration target 2) For each possible set of 4 points: compute
  similarity score 3) Rotate back the candidates with the highest score to their
  original position in the cloud, and add them to cumulative cloud
  **/
  std::vector<std::vector<int>> groups;
  comb(candidates_cloud->size(), TARGET_NUM_CIRCLES, groups);
  double groups_scores[groups.size()];  // -1: invalid; 0-1 normalized score
  for (int i = 0; i < groups.size(); ++i) {
    std::vector<pcl::PointXYZ> candidates;
    // Build candidates set
    for (int j = 0; j < groups[i].size(); ++j) {
      pcl::PointXYZ center;
      center.x = candidates_cloud->at(groups[i][j]).x;
      center.y = candidates_cloud->at(groups[i][j]).y;
      center.z = candidates_cloud->at(groups[i][j]).z;
      candidates.push_back(center);
    }

    // Compute candidates score
    Square square_candidate(candidates, mEnvironmentConfig.delta_width_circles,
                            mEnvironmentConfig.delta_height_cirlces);
    groups_scores[i] = square_candidate.is_valid()
                          ? 1.0
                          : -1;  // -1 when it's not valid, 1 otherwise
  }

  int best_candidate_idx = -1;
  double best_candidate_score = -1;
  for (int i = 0; i < groups.size(); ++i) {
    if (best_candidate_score == 1 && groups_scores[i] == 1) {
      // Exit 4: Several candidates fit target's geometry
      // TODO : Flag an error here
      std::cout << 
          "[LiDAR] More than one set of candidates fit target's geometry. "
          "Please, make sure your parameters are well set. Exiting callback" << std::endl;
      return false;
    }
    if (groups_scores[i] > best_candidate_score) {
      best_candidate_score = groups_scores[i];
      best_candidate_idx = i;
    }
  }

  if (best_candidate_idx == -1) {
    // Exit 5: No candidates fit target's geometry'
    // TODO: Flag a warning here
    std::cout <<
        "[LiDAR] Unable to find a candidate set that matches target's "
        "geometry" << std::endl;
    return false;
  }

  // Build selected centers set

  for (int j = 0; j < groups[best_candidate_idx].size(); ++j) {
    pcl::PointXYZ center_rotated_back = pcl::transformPoint(
        candidates_cloud->at(groups[best_candidate_idx][j]),
        rotation.inverse());
    center_rotated_back.x = (-coefficients->values[1] * center_rotated_back.y -
                            coefficients->values[2] * center_rotated_back.z -
                            coefficients->values[3]) /
                            coefficients->values[0];

    centers_cloud->push_back(center_rotated_back);
  }

  if (centers_cloud->size() < TARGET_NUM_CIRCLES)
    return false;
  return true;
}

void LidarFeatureDetector::rangePassThroughFilterCloud(pcl::PointCloud<Velodyne::Point>::Ptr& cloud_in, 
                                pcl::PointCloud<Velodyne::Point>::Ptr& cloud_out)
{
  // Range passthrough filter
  pcl::PointCloud<Velodyne::Point>::Ptr tmp(new pcl::PointCloud<Velodyne::Point>);

  pcl::PassThrough<Velodyne::Point> pass2;
  pass2.setInputCloud(cloud_in);
  pass2.setFilterFieldName("range");
  pass2.setFilterLimits(mEnvironmentConfig.passthrough_radius_min, mEnvironmentConfig.passthrough_radius_max);
  pass2.filter(*tmp);

  pcl::CropBox<Velodyne::Point> cropBox;
  cropBox.setInputCloud(tmp);
  cropBox.setMin(Eigen::Vector4f(2.0f, -1.0f, -20.0f, 1.0f));
  cropBox.setMax(Eigen::Vector4f(4.0f,  1.0f,  20.0f, 1.0f));
  cropBox.filter(*cloud_out);
}

bool LidarFeatureDetector::planeSegmentation(pcl::PointCloud<Velodyne::Point>::Ptr& cloud_in,
                            pcl::ModelCoefficients::Ptr& coefficients, pcl::PointIndices::Ptr& inliers)
{
  pcl::SACSegmentation<Velodyne::Point> plane_segmentation;
  plane_segmentation.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
  plane_segmentation.setDistanceThreshold(mEnvironmentConfig.plane_threshold);
  plane_segmentation.setMethodType(pcl::SAC_RANSAC);
  plane_segmentation.setAxis(mEnvironmentConfig.axis);
  plane_segmentation.setEpsAngle(mEnvironmentConfig.angle_threshold);
  plane_segmentation.setOptimizeCoefficients(true);
  plane_segmentation.setMaxIterations(1000);
  plane_segmentation.setInputCloud(cloud_in);
  plane_segmentation.segment(*inliers, *coefficients);
  ROS_INFO("[LiDar]Doing plane segmentation\n");

  if (inliers->indices.size() == 0) 
    return false;
  return true;
}

bool LidarFeatureDetector::edgeCloudFilter(pcl::PointCloud<Velodyne::Point>::Ptr& cloud_in,
                        pcl::PointCloud<Velodyne::Point>::Ptr& cloud_out)
{
  // Get edges points by range
  std::vector<std::vector<Velodyne::Point *>> rings =
      Velodyne::getRings(*cloud_in, mEnvironmentConfig.rings_count);
  for (std::vector<std::vector<Velodyne::Point *>>::iterator ring = rings.begin();
      ring < rings.end(); ++ring) {
    Velodyne::Point *prev, *succ;
    if (ring->empty()) continue;

    (*ring->begin())->intensity = 0;
    (*(ring->end() - 1))->intensity = 0;
    for (std::vector<Velodyne::Point *>::iterator pt = ring->begin() + 1;
        pt < ring->end() - 1; pt++) {
      Velodyne::Point *prev = *(pt - 1);
      Velodyne::Point *succ = *(pt + 1);
      (*pt)->intensity =
          std::max(std::max(prev->range - (*pt)->range, succ->range - (*pt)->range), 0.f);
    }
  }

  float THRESHOLD =
      mEnvironmentConfig.gradient_threshold;  // 10 cm between the pattern and the background
  for (pcl::PointCloud<Velodyne::Point>::iterator pt =
          cloud_in->points.begin();
      pt < cloud_in->points.end(); ++pt) {
    if (pt->intensity > THRESHOLD) {
      cloud_out->push_back(*pt);
    }
  }
  ROS_INFO("[LiDar] Filtering out edges\n");
  if (cloud_out->points.size() == 0)
    return false;
  return true;
}

bool LidarFeatureDetector::planeEdgeCloudFilter(pcl::PointCloud<Velodyne::Point>::Ptr& cloud_in,
                        Eigen::VectorXf& plane_coeffs,
                        pcl::PointCloud<Velodyne::Point>::Ptr& cloud_out)
{
  // Get points belonging to plane in pattern pointcloud
  pcl::SampleConsensusModelPlane<Velodyne::Point>::Ptr dit(
      new pcl::SampleConsensusModelPlane<Velodyne::Point>(cloud_in));
  std::vector<int> inliers2;
  dit->selectWithinDistance(plane_coeffs, mEnvironmentConfig.plane_distance_inliers, inliers2);
  pcl::copyPointCloud<Velodyne::Point>(*cloud_in, inliers2, *cloud_out);
  ROS_INFO("[LiDar]Doing plane edge segmentation\n");
  if (cloud_out->size() == 0)
    return false;

  return true;
}

bool LidarFeatureDetector::velodyneRingsToPointCloud(std::vector<std::vector<Velodyne::Point *>>& rings_in,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out)
{
  // Conversion from Velodyne::Point to pcl::PointXYZ
  for (std::vector<std::vector<Velodyne::Point *>>::iterator ring = rings_in.begin();
      ring < rings_in.end(); ++ring) {
    for (std::vector<Velodyne::Point *>::iterator pt = ring->begin();
        pt < ring->end(); ++pt) {
      pcl::PointXYZ point;
      point.x = (*pt)->x;
      point.y = (*pt)->y;
      point.z = (*pt)->z;
      cloud_out->push_back(point);
    }
  }
  ROS_INFO("[LiDar]Converted Rings to Point Cloud\n");
  if (cloud_out->size() == 0)
    return false;
  return true;
}

bool LidarFeatureDetector::circleSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
                            double zcoord_xyplane,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out)
{
  // RANSAC circle detection
  pcl::ModelCoefficients::Ptr coefficients3(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers3(new pcl::PointIndices);

  pcl::SACSegmentation<pcl::PointXYZ> circle_segmentation;
  circle_segmentation.setModelType(pcl::SACMODEL_CIRCLE2D);
  circle_segmentation.setDistanceThreshold(mEnvironmentConfig.circle_threshold);
  circle_segmentation.setMethodType(pcl::SAC_RANSAC);
  circle_segmentation.setOptimizeCoefficients(true);
  circle_segmentation.setMaxIterations(1000);
  circle_segmentation.setRadiusLimits(
      mEnvironmentConfig.circle_radius - mEnvironmentConfig.target_radius_tolerance,
      mEnvironmentConfig.circle_radius + mEnvironmentConfig.target_radius_tolerance);

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
//   if (DEBUG)
//     ROS_INFO("[LiDAR] Searching for points in cloud of size %lu",
//              xy_cloud->points.size());
  while (cloud_in->points.size() > 3) {
    circle_segmentation.setInputCloud(cloud_in);
    circle_segmentation.segment(*inliers3, *coefficients3);
    if (inliers3->indices.size() == 0) {
    //   if (DEBUG)
    //     ROS_INFO(
    //         "[LiDAR] Optimized circle segmentation failed, trying unoptimized "
    //         "version");
      circle_segmentation.setOptimizeCoefficients(false);

      circle_segmentation.setInputCloud(cloud_in);
      circle_segmentation.segment(*inliers3, *coefficients3);

      // Reset for next iteration
      circle_segmentation.setOptimizeCoefficients(true);
      if (inliers3->indices.size() == 0) {
        break;
      }
    }

    // Add center point to cloud
    pcl::PointXYZ center;
    center.x = *coefficients3->values.begin();
    center.y = *(coefficients3->values.begin() + 1);
    center.z = zcoord_xyplane;

    cloud_out->push_back(center);

    // Remove inliers from pattern cloud to find next circle
    extract.setInputCloud(cloud_in);
    extract.setIndices(inliers3);
    extract.setNegative(true);
    extract.filter(*cloud_f);
    cloud_in.swap(cloud_f);
  }

  ROS_INFO("[LiDar]Doing CIRCLE segmentation\n");
  if (cloud_out->size() < TARGET_NUM_CIRCLES)
    return false;
  return true;
}

void LidarFeatureDetector::clearCumulativeCloud()
{
  mCumulativeCloud->clear();
}

void LidarFeatureDetector::setReady(){
  WARMUP_DONE = true;
}

bool LidarFeatureDetector::isReady(){
  return WARMUP_DONE ;
}