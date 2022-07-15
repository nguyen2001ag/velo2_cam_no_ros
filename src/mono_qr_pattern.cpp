#include <mono_qr_pattern.h>

MonoCameraFeatureDetector::MonoCameraFeatureDetector(cv::Mat& K, cv::Mat& D, MonoConfig& config)
          :mEnvironmentConfig(config), mCameraIntrinsic(K), mDistCoeffs(D)
          {
            mDictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
            mCumulativeCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

            mCumulativePub = nh_.advertise<sensor_msgs::PointCloud2>("canera_cumulative_cloud", 1);
          }


MonoCameraFeatureDetector::MonoCameraFeatureDetector(MonoConfig& config)
          :mEnvironmentConfig(config)
          {
            mDictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50);
            mCumulativeCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

            mCumulativePub = nh_.advertise<sensor_msgs::PointCloud2>("canera_cumulative_cloud", 1);

          }

// void MonoCameraFeatureDetector::pushPointCloudCbFn(std::function<void(pcl::PointCloud<pcl::PointXYZ>::ConstPtr, long)> cbIn){
//   mCbPub = std::move(cbIn);
// }

cv::Point2f MonoCameraFeatureDetector::projectPointDist(cv::Point3f pt_cv, const cv::Mat& intrinsics,
                        const cv::Mat& distCoeffs) {
  // Project a 3D point taking into account distortion
  std::vector<cv::Point3f> input{pt_cv};
  std::vector<cv::Point2f> projectedPoints;
  projectedPoints.resize(
      1);  // TODO: Do it batched? (cv::circle is not batched anyway)
  projectPoints(input, cv::Mat::zeros(3, 1, CV_64FC1), cv::Mat::zeros(3, 1, CV_64FC1),
                intrinsics, distCoeffs, projectedPoints);
  return projectedPoints[0];
}

int MonoCameraFeatureDetector::imageCallback(const cv::Mat& image, const cv::Mat& K, const cv::Mat& D
                ,pcl::PointCloud<pcl::PointXYZ>::Ptr& clusters_cloud)
{
  mCameraIntrinsic = K;
  mDistCoeffs = D;
  return imageCallback(image, clusters_cloud);
}

int MonoCameraFeatureDetector::imageCallback(const cv::Mat& image, pcl::PointCloud<pcl::PointXYZ>::Ptr& clusters_cloud) {
  mFramesProc++;

  cv::Mat imageCopy;
  image.copyTo(imageCopy);
  // TODO End of block to move

  // Create vector of markers corners. 4 markers * 4 corners
  // Markers order:
  // 0-------1
  // |       |
  // |   C   |
  // |       |
  // 3-------2

  // WARNING: IDs are in different order:
  // Marker 0 -> aRuCo ID: 1
  // Marker 1 -> aRuCo ID: 2
  // Marker 2 -> aRuCo ID: 4
  // Marker 3 -> aRuCo ID: 3

  std::vector<std::vector<cv::Point3f>> boardCorners(4);
  std::vector<cv::Point3f> boardCircleCenters;

  estimateBoardCircles(boardCircleCenters);
  estimateBoardCorners(boardCorners);
  

  cv::Ptr<cv::aruco::Board> board =
      cv::aruco::Board::create(boardCorners, mDictionary, mBoardIds);

  cv::Ptr<cv::aruco::DetectorParameters> parameters =
      cv::aruco::DetectorParameters::create();
  // set tp use corner refinement for accuracy, values obtained
  // for pixel coordinates are more accurate than the neaterst pixel

#if (CV_MAJOR_VERSION == 3 && CV_MINOR_VERSION <= 2) || CV_MAJOR_VERSION < 3
  parameters->doCornerRefinement = true;
#else
  parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
#endif

  // Detect markers
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> arucoCorners;
  cv::aruco::detectMarkers(image, mDictionary, arucoCorners, ids, parameters);

  // Draw detections if at least one marker detected
  if (ids.size() > 0) {
    cv::aruco::drawDetectedMarkers(imageCopy, arucoCorners, ids);
    cv::imshow("marker", imageCopy);
    cv::waitKey(1);
  }
  // cv::Vec3d rvec(0, 0, 0), tvec(0, 0, 0);  // Vectors to store initial guess

  // Compute initial guess as average of individual markers poses
  if (ids.size() >= mEnvironmentConfig.min_detected_markers && ids.size() <= TARGET_NUM_CIRCLES) {
    // Estimate 3D position of the markers
    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(arucoCorners, mEnvironmentConfig.marker_size, mCameraIntrinsic,
                                        mDistCoeffs, rvecs, tvecs);

    // Draw markers' axis and centers in color image (Debug purposes)
    auto [rvec, tvec] = getAverageBoardPose(rvecs, tvecs, ids.size());


    pcl::PointCloud<pcl::PointXYZ>::Ptr centers_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr candidates_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);

// Estimate 3D position of the board using detected markers
#if (CV_MAJOR_VERSION == 3 && CV_MINOR_VERSION <= 2) || CV_MAJOR_VERSION < 3
    int valid = cv::aruco::estimatePoseBoard(corners, ids, board, cameraMatrix,
                                            distCoeffs, rvec, tvec);
#else
    int valid = cv::aruco::estimatePoseBoard(arucoCorners, ids, board, mCameraIntrinsic,
                                            mDistCoeffs, rvec, tvec, true);
#endif


    transformToCameraFrame(boardCircleCenters, rvec, tvec, candidates_cloud);


    if(!validateCentersCloud(candidates_cloud, centers_cloud))
      std::cout << "Unable to validate centers cluster cloud" << std::endl;
    

    // Add centers to cumulative for further clustering
    for (int i = 0; i < centers_cloud->size(); i++) {
      mCumulativeCloud->push_back(centers_cloud->at(i));
    }
    mFrameUsed++;

    if (DEBUG){
      ROS_INFO("[Mono] %d/%d frames: %ld pts in cloud", mFrameUsed,
              mFramesProc, mCumulativeCloud->points.size());
    }

    if (DEBUG) {  // Draw centers
      for (int i = 0; i < centers_cloud->size(); i++) {
        cv::Point3f pt_circle1(centers_cloud->at(i).x, centers_cloud->at(i).y,
                              centers_cloud->at(i).z);
        cv::Point2f uv_circle1;
        uv_circle1 = projectPointDist(pt_circle1, mCameraIntrinsic, mDistCoeffs);
        cv::circle(imageCopy, uv_circle1, 2, cv::Scalar(255, 0, 255), -1);
        cv::imshow("circles", imageCopy);
        cv::waitKey(1);
      }
    }

    if (!mWarmUpDone) {  // Compute clusters from detections in the latest frame
      copyPointCloud(*centers_cloud, *clusters_cloud);
    } else {  // Use cumulative information from previous frames
      getCenterClusters(mCumulativeCloud, clusters_cloud, mEnvironmentConfig.cluster_tolerance,
                        mEnvironmentConfig.min_cluster_factor * mFrameUsed, mFrameUsed);
      if (clusters_cloud->points.size() > TARGET_NUM_CIRCLES) {
        getCenterClusters(mCumulativeCloud, clusters_cloud, mEnvironmentConfig.cluster_tolerance,
                          3.0 * mFrameUsed / 4.0, mFrameUsed);
      }
    }


    if (clusters_cloud->size() != TARGET_NUM_CIRCLES) {
      ROS_WARN("CLusters cloud has %d points\n",clusters_cloud->size() );
      return -1;
    }

    transformCloudCoordinateFrame(clusters_cloud, Eigen::Quaternionf(0.5f, -0.5f, 0.5f, -0.5f));
    if (DEBUG) {
      publishROSCloud(clusters_cloud);
      ROS_WARN("camera point clouds has %d pts\n", clusters_cloud->size());
      ROS_WARN("Publishing camera point clouds\n");
    }

  } else {  // Markers found != TARGET_NUM_CIRCLES
    std::cout << ids.size() <<" marker(s) found, " << TARGET_NUM_CIRCLES << " expected. Skipping frame...\n";
    return -1;
  }

  // if (DEBUG) {
  //   cv::namedWindow("out", WINDOW_NORMAL);
  //   cv::imshow("out", imageCopy);
  //   cv::waitKey(1);
  // }

  // Clear cumulative cloud during warm-up phase
  
  if (!mWarmUpDone) {
    clearCumulativeCloud();
    mFramesProc = 0;
    mFrameUsed = 0;
    return -1;
  }
  ROS_WARN("[MONO] Frame used: %d\n", mFrameUsed);
  return mFrameUsed;
}

void MonoCameraFeatureDetector::publishROSCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_in){
  sensor_msgs::PointCloud2 cumulative_pointcloud;
  pcl::toROSMsg(*cloud_in, cumulative_pointcloud);
  cumulative_pointcloud.header.frame_id = "map";
  mCumulativePub.publish(cumulative_pointcloud);
}

std::pair<cv::Vec3d, cv::Vec3f> MonoCameraFeatureDetector::getAverageBoardPose(std::vector<cv::Vec3d>& rvecs,
                           std::vector<cv::Vec3d>& tvecs, int n_marker)
{
  cv::Vec3d rvec(0, 0, 0), tvec(0, 0, 0);  // Vectors to store initial guess
  cv::Vec3f rvec_sin(0.0f, 0.0f, 0.0f), rvec_cos(0.0f, 0.0f, 0.0f);
  for (int i = 0; i < n_marker; i++) {
      double x = tvecs[i][0];
      double y = tvecs[i][1];
      double z = tvecs[i][2];

      cv::Point3f pt_cv(x, y, z);
      cv::Point2f uv;
      uv = projectPointDist(pt_cv, mCameraIntrinsic, mDistCoeffs);
      //TODO : Draw visualiaztion for axis found
      // cv::aruco::drawAxis(imageCopy, cameraIntrinsic, distCoeffs, rvecs[i],
      //                     tvecs[i], 0.1);

      // Accumulate pose for initial guess
      tvec[0] += tvecs[i][0];
      tvec[1] += tvecs[i][1];
      tvec[2] += tvecs[i][2];
      rvec_sin[0] += sin(rvecs[i][0]);
      rvec_sin[1] += sin(rvecs[i][1]);
      rvec_sin[2] += sin(rvecs[i][2]);
      rvec_cos[0] += cos(rvecs[i][0]);
      rvec_cos[1] += cos(rvecs[i][1]);
      rvec_cos[2] += cos(rvecs[i][2]);
    }

    // Compute average pose. Rotation computed as atan2(sin/cos)
    tvec = tvec / int(n_marker);
    rvec_sin = rvec_sin / int(n_marker);  // Average sin
    rvec_cos = rvec_cos / int(n_marker);  // Average cos
    rvec[0] = atan2(rvec_sin[0], rvec_cos[0]);
    rvec[1] = atan2(rvec_sin[1], rvec_cos[1]);
    rvec[2] = atan2(rvec_sin[2], rvec_cos[2]);

    ROS_INFO("[MONO]: Getting average pose\n");

    return std::pair<cv::Vec3d, cv::Vec3f>(rvec, tvec);
}

void MonoCameraFeatureDetector::estimateBoardCircles(std::vector<cv::Point3f>& boardCircleCenters){
  float circle_width = mEnvironmentConfig.delta_width_circles / 2.;
  float circle_height = mEnvironmentConfig.delta_height_cirlces / 2.;
  for (int i = 0; i < 4; ++i) {
    int x_qr_center =
        (i % 3) == 0 ? -1 : 1;  // x distances are substracted for QRs on the
                                // left, added otherwise
    int y_qr_center =
        (i < 2) ? 1 : -1;  // y distances are added for QRs above target's
                          // center, substracted otherwise
    cv::Point3f circleCenter3d(x_qr_center * circle_width,
                              y_qr_center * circle_height, 0);
    boardCircleCenters.push_back(circleCenter3d);
  }

  ROS_INFO("[MONO]: Estimating Board circles\n");
}

void MonoCameraFeatureDetector::estimateBoardCorners(std::vector<std::vector<cv::Point3f>>& boardCorners){
  float width = mEnvironmentConfig.delta_width_qr_center;
  float height = mEnvironmentConfig.delta_height_qr_center;

  for (int i = 0; i < 4; ++i) {
    int x_qr_center =
        (i % 3) == 0 ? -1 : 1;  // x distances are substracted for QRs on the
                                // left, added otherwise
    int y_qr_center =
        (i < 2) ? 1 : -1;  // y distances are added for QRs above target's
                          // center, substracted otherwise
    float x_center = x_qr_center * width;
    float y_center = y_qr_center * height;

    for (int j = 0; j < 4; ++j) {
      int x_qr = (j % 3) == 0 ? -1 : 1;  // x distances are added for QRs 0 and
                                        // 3, substracted otherwise
      int y_qr = (j < 2) ? 1 : -1;  // y distances are added for QRs 0 and 1,
                                    // substracted otherwise
      cv::Point3f pt3d(x_center + x_qr * mEnvironmentConfig.marker_size / 2.,
                      y_center + y_qr * mEnvironmentConfig.marker_size / 2., 0);
      boardCorners[i].push_back(pt3d);
    }
  }

  ROS_INFO("[MONO]: Estimating Board corners\n");
}

void MonoCameraFeatureDetector::transformToCameraFrame (std::vector<cv::Point3f>& points, const cv::Vec3d& rvec,
                          const cv::Vec3d& tvec, pcl::PointCloud<pcl::PointXYZ>::Ptr & candidates_cloud)
{

  //TODO : have something for visualization by drawing axis found
    // cv::aruco::drawAxis(imageCopy, cameraIntrincic, distCoeffs, rvec, tvec, 0.2);

  // Build transformation matrix to calibration target axis
  cv::Mat R(3, 3, cv::DataType<float>::type);
  cv::Rodrigues(rvec, R);

  cv::Mat t = cv::Mat::zeros(3, 1, CV_32F);
  t.at<float>(0) = tvec[0];
  t.at<float>(1) = tvec[1];
  t.at<float>(2) = tvec[2];

  cv::Mat board_transform = cv::Mat::eye(3, 4, CV_32F);
  R.copyTo(board_transform.rowRange(0, 3).colRange(0, 3));
  t.copyTo(board_transform.rowRange(0, 3).col(3));

  // Compute coordintates of circle centers
  for (int i = 0; i < points.size(); ++i) {
    cv::Mat mat = cv::Mat::zeros(4, 1, CV_32F);
    mat.at<float>(0, 0) = points[i].x;
    mat.at<float>(1, 0) = points[i].y;
    mat.at<float>(2, 0) = points[i].z;
    mat.at<float>(3, 0) = 1.0;

    // Transform center to target coords
    cv::Mat mat_qr = board_transform * mat;
    cv::Point3f center3d;
    center3d.x = mat_qr.at<float>(0, 0);
    center3d.y = mat_qr.at<float>(1, 0);
    center3d.z = mat_qr.at<float>(2, 0);

    // Add center to list
    pcl::PointXYZ qr_center;
    qr_center.x = center3d.x;
    qr_center.y = center3d.y;
    qr_center.z = center3d.z;
    candidates_cloud->push_back(qr_center);
  }

  ROS_INFO("[MONO]: To camera Coordinates frame\n");
}

void MonoCameraFeatureDetector::clearCumulativeCloud(){
  mCumulativeCloud->clear();
}


bool MonoCameraFeatureDetector::validateCentersCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& candidates_cloud,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr& centers_cloud)
{
  /**
      NOTE: This is included here in the same way as the rest of the modalities
    to avoid obvious misdetections, which sometimes happened in our experiments.
    In this modality, it should be impossible to have more than a set of
    candidates, but we keep the ability of handling different combinations for
    eventual future extensions.

      Geometric consistency check
      At this point, circles' center candidates have been computed
    (found_centers). Now we need to select the set of 4 candidates that best fit
    the calibration target geometry. To that end, the following steps are
    followed: 1) Create a cloud with 4 points representing the exact geometry of
    the calibration target 2) For each possible set of 4 points: compute
    similarity score 3) Rotate back the candidates with the highest score to
    their original position in the cloud, and add them to cumulative cloud
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
      // TODO: Flag this as error
      std::cout << 
          "[Mono] More than one set of candidates fit target's geometry. "
          "Please, make sure your parameters are well set. Exiting callback\n";
      return false;
    }
    if (groups_scores[i] > best_candidate_score) {
      best_candidate_score = groups_scores[i];
      best_candidate_idx = i;
    }
  }

  if (best_candidate_idx == -1) {
    // Exit: No candidates fit target's geometry
    // TODO : Flag this as warning
    std::cout << 
        "[Mono] Unable to find a candidate set that matches target's "
        "geometry\n";
    return false;
  }

  for (int j = 0; j < groups[best_candidate_idx].size(); ++j) {
    centers_cloud->push_back(
        candidates_cloud->at(groups[best_candidate_idx][j]));
  }

  return true;

}

void MonoCameraFeatureDetector::transformCloudCoordinateFrame(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
                                const Eigen::Quaternionf& rotation)
{
  // xy_camera_cloud is camera cloud in robot coordinate frame
  pcl::PointCloud<pcl::PointXYZ>::Ptr xy_camera_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());

  pcl::copyPointCloud(*cloud_in, *xy_camera_cloud);
  cloud_in->clear();

  // Since this is a camera, do a transformation
  pcl::transformPointCloud(*xy_camera_cloud, *cloud_in, Eigen::Vector3f(0.0, 0.0, 0.0), rotation);
  ////
}

void MonoCameraFeatureDetector::setReady(){
  mWarmUpDone = true;
}

bool MonoCameraFeatureDetector::isReady(){
  return mWarmUpDone ;
}