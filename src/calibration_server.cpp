#include <calibration_server.h>


CalibrationServer::CalibrationServer(ServerConfig config):
  sync_iterations(config.sync_iterations),
  results_every_pose(config.results_every_pose), skip_warmup(config.skip_warmup),
  single_pose_mode(config.single_pose_mode)
{

  camera_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  lidar_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

  verification_pub = nh_.advertise<sensor_msgs::PointCloud2>("verification", 1);

  cameraReceived = false;
  lidarReceived = false;

  calibration_ended = false;

  if (skip_warmup) {
    camera.setReady();
    lidar.setReady();
    // TODO : Warning level
    std::cout << "Skipping warmup" << std::endl;
  } else {
    std::cout << "Please, adjust the filters for each sensor before the calibration "
            "starts."
         << std::endl;
  }
}


bool CalibrationServer::pushSynchronisedData(const pcl::PointCloud<Velodyne::Point>::Ptr & lidar_cloud,
                           const cv::Mat& image, const cv::Mat& K, const cv::Mat& D)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr circles_image_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr circles_lidar_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  int camera_frame_used = camera.imageCallback(image, K, D, circles_image_cloud);
  ROS_ERROR("Camera frame used to generate Point CLoud %d", camera_frame_used);
  if(camera_frame_used <= 0){
    std::cout << "Failed to retrieve camera Point Cloud" << std::endl;
  }

  // pcl::PointCloud<Velodyne::Point>::Ptr processed_cloud;
  // pcl::copyPointCloud(*lidar_cloud, *processed_cloud);

  int lidar_frame_used = lidar.callback(lidar_cloud, circles_lidar_cloud);
  ROS_ERROR("Lidar frame used to generate Point CLoud: %d", lidar_frame_used);
  if(lidar_frame_used <= 0){
    std::cout << "Failed to retrieve initial Lidar Point Cloud" << std::endl;
  }
    

  std::cout << "Number of points from circles cloud: " << circles_image_cloud-> size();

  if (!process_camera(circles_image_cloud, camera_frame_used))
    std::cout << "Failed to add CAMERA POINT CLOUD to queue" << std::endl;
  else
    std::cout << "Successfully to add CAMERA POINT CLOUD to queue" << std::endl;
  if (!process_lidar(circles_lidar_cloud, lidar_frame_used))
    std::cout << "Failed to add LIDAR POINT CLOUD to queue" << std::endl;
  else
    std::cout << "Successfully to add LIDAR POINT CLOUD to queue" << std::endl;
  if(getExtrinsicCalibration()){
    auto [translation, rotation] = getExtrinsics();
    pcl::PointCloud<pcl::PointXYZ>::Ptr final_img_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr final_lidar_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*circles_image_cloud, *final_img_cloud);
    pcl::transformPointCloud(*final_img_cloud, *final_lidar_cloud, translation, rotation);

    sensor_msgs::PointCloud2 verification_pointcloud;
    pcl::toROSMsg(*final_lidar_cloud, verification_pointcloud);
    verification_pointcloud.header.frame_id = "map";
    verification_pub.publish(verification_pointcloud);
    return true;
  }
  return false;

}


//TODO : Have a synchronization method on this calibration



// Assume that camera is a camera
// TODO: Add transformation from camera to robot coordinate system

/***
@params
  camera_centroids: Point Cloud contains 3D coordinates of circles
  frame_used: the number of frame used to generate a cumulative pointcloud.

***/
bool CalibrationServer::process_camera(pcl::PointCloud<pcl::PointXYZ>::Ptr camera_centroids
                                        , long int frame_used = 0) 
{
  

  if (!camera.isReady()) {
    CAMERA_WARMUP_COUNT++;
    std::cout << "Clusters from " << camera_frame_id << ": " << CAMERA_WARMUP_COUNT
         << "/10" << '\r' << std::flush;
    if (CAMERA_WARMUP_COUNT >= 10)  // TODO: Change to param?
    {
      camera.setReady();
      ROS_ERROR("[SERVER] CAMERA warm up done\n");
    }
    return false;
  }

  if (camera_centroids->size() == 0){
    return false;
  }

  std::cout << "Camera frame used: " << frame_used << std::endl;
  if (frame_used < TARGET_ITERATIONS) return false; // Return if not enough frame to support


  // if (DEBUG) ROS_INFO("camera (%s) pattern ready!", camera_frame_id.c_str());

  if (camera_buffer.size() == TARGET_POSITIONS_COUNT) {
    camera_buffer.resize(TARGET_POSITIONS_COUNT + 1);
    std::cout << "Resize buffer" << std::endl;
  }
  camera_cloud->clear();
  pcl::copyPointCloud(*camera_centroids, *camera_cloud);

  cameraReceived = true;
  std::cout << "Process camera number of points: " << camera_cloud->size();
  sortPatternCenters(camera_centroids, camera_vector);
  

  std::cout << "Finished sorting centers" << std::endl;

  camera_buffer[TARGET_POSITIONS_COUNT].emplace_back(*camera_centroids, camera_vector);
  camera_vector.clear();
  return true;
}
/***
@params
  lidar_centroids: Point Cloud contains 3D coordinates of circles
  frame_used: the number of frame used to generate a cumulative pointcloud.

***/

bool CalibrationServer::process_lidar (pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_centroids
                                        , long int frame_used = 0) 
{
  if (!lidar.isReady()) {
    LIDAR_WARMUP_COUNT++;
    std::cout << "Clusters from " << lidar_frame_id << ": " << LIDAR_WARMUP_COUNT
         << "/10" << '\r' << std::endl;
    if (LIDAR_WARMUP_COUNT >= 10)  // TODO: Change to param?
    {
      lidar.setReady();
      ROS_ERROR("[SERVER] LIDAR warm up done\n");
    }
    return false;
  } 
  if (lidar_centroids->size() == 0){
    return false;
  }
  std::cout << "Lidar frame used: " << frame_used << std::endl;
  if (frame_used < TARGET_ITERATIONS) return false; // Return if not enough frame to supprt

  if (lidar_buffer.size() == TARGET_POSITIONS_COUNT) {
    lidar_buffer.resize(TARGET_POSITIONS_COUNT + 1);
  }

  lidar_cloud->clear();
  pcl::copyPointCloud(*lidar_centroids, *lidar_cloud);

  lidarReceived = true;

  sortPatternCenters(lidar_centroids, lidar_vector);

  lidar_buffer[TARGET_POSITIONS_COUNT].emplace_back(*lidar_centroids, lidar_vector);
  lidar_vector.clear();
  return true;
}

// Performing merging 2 pipeline for calibration
bool CalibrationServer::getExtrinsicCalibration(){
  if (!(cameraReceived && lidarReceived))
    return false;
  
  bool succeed = calibrateExtrinsics();
  if (succeed){
    TARGET_POSITIONS_COUNT++;
    return true;
  }
  
  return false;
}

bool CalibrationServer::calibrateExtrinsics() {
  std::vector<pcl::PointXYZ> local_camera_vector, local_lidar_vector;
  pcl::PointCloud<pcl::PointXYZ>::Ptr local_camera_cloud(new pcl::PointCloud<pcl::PointXYZ>),
      local_lidar_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  int used_lidar, used_camera;

  // Get final frame names for TF broadcaster
  std::string camera_final_transformation_frame = camera_frame_id;

  int total_camera, total_lidar;

  
  for (int i = 0; i < TARGET_POSITIONS_COUNT + 1; ++i) {
    // Sensor 1
    local_camera_vector.insert(
        local_camera_vector.end(),
        std::get<1>(camera_buffer[i].back()).begin(),
        std::get<1>(camera_buffer[i].back())
            .end());  // Add sorted centers (for equations)
    *local_camera_cloud += std::get<0>(
        camera_buffer[i].back());  // Add centers cloud (for registration)
    // used_camera = std::get<1>(lidar_buffer[i].back());

    // Sensor 2
    local_lidar_vector.insert(
        local_lidar_vector.end(),
        std::get<1>(lidar_buffer[i].back()).begin(),
        std::get<1>(lidar_buffer[i].back())
            .end());  // Add sorted centers (for equations)
    *local_lidar_cloud += std::get<0>(
        lidar_buffer[i].back());  // Add centers cloud (for registration)
  }


  // SVD code
  pcl::PointCloud<pcl::PointXYZ>::Ptr sorted_centerCAMERA(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr sorted_centerLIDAR(
      new pcl::PointCloud<pcl::PointXYZ>());

  for (int i = 0; i < local_camera_vector.size(); ++i) {
    sorted_centerCAMERA->push_back(local_camera_vector[i]);
    sorted_centerLIDAR->push_back(local_lidar_vector[i]);
  }

  Eigen::Matrix4f final_transformation;
  const pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,
                                                       pcl::PointXYZ>
      trans_est_svd(true);
  trans_est_svd.estimateRigidTransformation(*sorted_centerCAMERA, *sorted_centerLIDAR,
                                            final_transformation);

  Eigen::Matrix3f tf3d;
  tf3d << final_transformation(0, 0), final_transformation(0, 1), final_transformation(0, 2)
        , final_transformation(1, 0), final_transformation(1, 1), final_transformation(1, 2)
        , final_transformation(2, 0), final_transformation(2, 1), final_transformation(2, 2);

  Eigen::Quaternionf tfqt(tf3d);
  myExtrinsics.rotation = tfqt;

  Eigen::Vector3f origin{final_transformation(0, 3), final_transformation(1, 3), final_transformation(2, 3)};
  myExtrinsics.translation = origin;

  std::cout << std::setprecision(4) << std::fixed;
  std::cout << "Calibration finished succesfully." << std::endl;
  std::cout << "Extrinsic parameters:" << std::endl;
  std::cout << "x = " << origin(0) << "\ty = " << origin(1) << "\tz = " << origin(2) << std::endl;

  cameraReceived = false;
  lidarReceived = false;

  return true;
}


Extrinsics CalibrationServer::getExtrinsics(){
  getExtrinsicCalibration();
  return myExtrinsics;
}


bool CalibrationServer::removeDataPt(size_t dataInd){
  if (dataInd >= camera_buffer.size() || dataInd >= lidar_buffer.size())
  {
    std::cout << "Index out of range when removing data points" << std::endl;
    return false;
  }  

  camera_buffer.erase(camera_buffer.begin() + dataInd);
  lidar_buffer.erase(lidar_buffer.begin() + dataInd);
  return true;
}

bool CalibrationServer::clearAllDataPts()
{
  camera_buffer.clear();
  lidar_buffer.clear();

}

