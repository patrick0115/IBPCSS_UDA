#include <mutex>
#include <memory>
#include <iostream>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <eigen_conversions/eigen_msg.h>

#include <std_srvs/Empty.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <pcl/filters/voxel_grid.h>

#include <pclomp/ndt_omp.h>
#include <fast_gicp/ndt/ndt_cuda.hpp>

#include <hdl_localization/pose_estimator.hpp>
#include <hdl_localization/delta_estimater.hpp>

#include <hdl_localization/ScanMatchingStatus.h>
#include <hdl_global_localization/SetGlobalMap.h>
#include <hdl_global_localization/QueryGlobalLocalization.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
namespace hdl_localization {

class HdlLocalizationNodelet : public nodelet::Nodelet {
public:
  using PointT = pcl::PointXYZI;

  HdlLocalizationNodelet() : tf_buffer(), tf_listener(tf_buffer) {
  }
  virtual ~HdlLocalizationNodelet() {
  }

  void onInit() override {
    nh = getNodeHandle();
    mt_nh = getMTNodeHandle();
    private_nh = getPrivateNodeHandle();

    initialize_params();

    robot_odom_frame_id = private_nh.param<std::string>("robot_odom_frame_id", "robot_odom");
    odom_child_frame_id = private_nh.param<std::string>("odom_child_frame_id", "base_link");
    

    image_sub = mt_nh.subscribe("/segmented_image", 1, &HdlLocalizationNodelet::image_callback, this);

    points_sub = mt_nh.subscribe("/velodyne_points", 2, &HdlLocalizationNodelet::points_callback, this);
    globalmap_sub = nh.subscribe("/globalmap", 1, &HdlLocalizationNodelet::globalmap_callback, this);
    initialpose_sub = nh.subscribe("/initialpose", 8, &HdlLocalizationNodelet::initialpose_callback, this);

    pose_pub = nh.advertise<nav_msgs::Odometry>("/odom", 5, false);
    aligned_pub = nh.advertise<sensor_msgs::PointCloud2>("/aligned_points", 5, false);
    status_pub = nh.advertise<ScanMatchingStatus>("/status", 5, false);
    globalmap_pub = nh.advertise<sensor_msgs::PointCloud2>("/transformed_globalmap", 1, true);
    projected_image_pub = nh.advertise<sensor_msgs::Image>("/projected_image", 1);
    colored_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/colored_cloud", 1);  // 初始化新的發布器  

    map_pub_timer = private_nh.createTimer(ros::Duration(1.0), &HdlLocalizationNodelet::map_publish_callback, this);

    // global localization
    use_global_localization = private_nh.param<bool>("use_global_localization", true);
    if(use_global_localization) {
      NODELET_INFO_STREAM("wait for global localization services");
      ros::service::waitForService("/hdl_global_localization/set_global_map");
      ros::service::waitForService("/hdl_global_localization/query");

      set_global_map_service = nh.serviceClient<hdl_global_localization::SetGlobalMap>("/hdl_global_localization/set_global_map");
      query_global_localization_service = nh.serviceClient<hdl_global_localization::QueryGlobalLocalization>("/hdl_global_localization/query");

      relocalize_server = nh.advertiseService("/relocalize", &HdlLocalizationNodelet::relocalize, this);
    }
  }

private:
  pcl::Registration<PointT, PointT>::Ptr create_registration() const {
    std::string reg_method = private_nh.param<std::string>("reg_method", "NDT_OMP");
    std::string ndt_neighbor_search_method = private_nh.param<std::string>("ndt_neighbor_search_method", "DIRECT7");
    double ndt_neighbor_search_radius = private_nh.param<double>("ndt_neighbor_search_radius", 2.0);
    double ndt_resolution = private_nh.param<double>("ndt_resolution", 1.0);

    if(reg_method == "NDT_OMP") {
      NODELET_INFO("NDT_OMP is selected");
      pclomp::NormalDistributionsTransform<PointT, PointT>::Ptr ndt(new pclomp::NormalDistributionsTransform<PointT, PointT>());
      ndt->setTransformationEpsilon(0.01);
      ndt->setResolution(ndt_resolution);
      if (ndt_neighbor_search_method == "DIRECT1") {
        NODELET_INFO("search_method DIRECT1 is selected");
        ndt->setNeighborhoodSearchMethod(pclomp::DIRECT1);
      } else if (ndt_neighbor_search_method == "DIRECT7") {
        NODELET_INFO("search_method DIRECT7 is selected");
        ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
      } else {
        if (ndt_neighbor_search_method == "KDTREE") {
          NODELET_INFO("search_method KDTREE is selected");
        } else {
          NODELET_WARN("invalid search method was given");
          NODELET_WARN("default method is selected (KDTREE)");
        }
        ndt->setNeighborhoodSearchMethod(pclomp::KDTREE);
      }
      return ndt;
    } else if(reg_method.find("NDT_CUDA") != std::string::npos) {
      NODELET_INFO("NDT_CUDA is selected");
      boost::shared_ptr<fast_gicp::NDTCuda<PointT, PointT>> ndt(new fast_gicp::NDTCuda<PointT, PointT>);
      ndt->setResolution(ndt_resolution);

      if(reg_method.find("D2D") != std::string::npos) {
        ndt->setDistanceMode(fast_gicp::NDTDistanceMode::D2D);
      } else if (reg_method.find("P2D") != std::string::npos) {
        ndt->setDistanceMode(fast_gicp::NDTDistanceMode::P2D);
      }

      if (ndt_neighbor_search_method == "DIRECT1") {
        NODELET_INFO("search_method DIRECT1 is selected");
        ndt->setNeighborSearchMethod(fast_gicp::NeighborSearchMethod::DIRECT1);
      } else if (ndt_neighbor_search_method == "DIRECT7") {
        NODELET_INFO("search_method DIRECT7 is selected");
        ndt->setNeighborSearchMethod(fast_gicp::NeighborSearchMethod::DIRECT7);
      } else if (ndt_neighbor_search_method == "DIRECT_RADIUS") {
        NODELET_INFO_STREAM("search_method DIRECT_RADIUS is selected : " << ndt_neighbor_search_radius);
        ndt->setNeighborSearchMethod(fast_gicp::NeighborSearchMethod::DIRECT_RADIUS, ndt_neighbor_search_radius);
      } else {
        NODELET_WARN("invalid search method was given");
      }
      return ndt;
    }

    NODELET_ERROR_STREAM("unknown registration method:" << reg_method);
    return nullptr;
  }
  void initialize_params() {
    // intialize scan matching method
    double downsample_resolution = private_nh.param<double>("downsample_resolution", 0.1);
    boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
    voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
    downsample_filter = voxelgrid;

    NODELET_INFO("create registration method for localization");
    registration = create_registration();

    // global localization
    NODELET_INFO("create registration method for fallback during relocalization");
    relocalizing = false;
    delta_estimater.reset(new DeltaEstimater(create_registration()));

    // initialize pose estimator
    if(private_nh.param<bool>("specify_init_pose", true)) {
      NODELET_INFO("initialize pose estimator with specified parameters!!");
      pose_estimator.reset(new hdl_localization::PoseEstimator(registration,
        Eigen::Vector3f(private_nh.param<double>("init_pos_x", 0.0), private_nh.param<double>("init_pos_y", 0.0), private_nh.param<double>("init_pos_z", 0.0)),
        Eigen::Quaternionf(private_nh.param<double>("init_ori_w", 1.0), private_nh.param<double>("init_ori_x", 0.0), private_nh.param<double>("init_ori_y", 0.0), private_nh.param<double>("init_ori_z", 0.0)),
        private_nh.param<double>("cool_time_duration", 0.5)
      ));
    }
  }
private:
  cv::Mat projectPointCloudToImage(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_rgb, const cv::Mat& image) {
    cv::Mat projected_image = image.clone();
    cv::Mat rvec = rvecs;
    cv::Mat tvec = tvecs;
    cv::Mat R;
    cv::Rodrigues(rvec, R); 

    for (auto& pt : *cloud_rgb) {
        cv::Mat point3D(3, 1, CV_64F);
        point3D.at<double>(0) = pt.x;
        point3D.at<double>(1) = pt.y;
        point3D.at<double>(2) = pt.z;

        cv::Mat pointCam = R * point3D + tvec;
        cv::Mat point2D = mtx * pointCam;    

        int u = static_cast<int>(point2D.at<double>(0) / point2D.at<double>(2));
        int v = static_cast<int>(point2D.at<double>(1) / point2D.at<double>(2));  
        //  && pt.r ==254 && pt.r==254  &&pt.r==254
        if (u >= 0 && u < projected_image.cols && v >= 0 && v < projected_image.rows && pt.x>0 ) {
            cv::Vec3b color = image.at<cv::Vec3b>(v, u); 
            pt.r = color[2];  // BGR -> RGB
            pt.g = color[1];
            pt.b = color[0];
            cv::circle(projected_image, cv::Point(u, v), 3, cv::Scalar(255, 0, 0), -1);  // 使用點雲中的 RGB 顏色
        }
      }
      return projected_image;
    }
  void map_publish_callback(const ros::TimerEvent& event) {
    if(!globalmap) {
      NODELET_WARN("Global map has not been received yet!");
      return;
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>());
    cloud_rgb->header = globalmap->header;
    for(const auto& point : *globalmap) {
      pcl::PointXYZRGB color_point;
      color_point.x = point.x;
      color_point.y = point.y;
      color_point.z = point.z;
    // 設定固定的RGB顏色，例如紅色 (255, 0, 0)
      uint8_t r = 0, g = 50, b = 100;    // 紅色
      uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
      color_point.rgb = *reinterpret_cast<float*>(&rgb);
      cloud_rgb->points.push_back(color_point);
    }

    // 獲取 base_link 到 globalmap 的轉換
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    try {
      geometry_msgs::TransformStamped transform_stamped = tf_buffer.lookupTransform(odom_child_frame_id, cloud_rgb->header.frame_id, ros::Time(0), ros::Duration(1.0));
      transform = tf2::transformToEigen(transform_stamped).matrix().cast<float>();
    } catch (tf2::TransformException &ex) {
      ROS_WARN("%s", ex.what());
      return;
    }

    // 應用轉換到每個點
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    // pcl::transformPointCloud(*cloud_rgb, *transformed_cloud, transform);
    pcl::transformPointCloud(*pcss_map, *transformed_cloud, transform);

    // sensor_msgs::PointCloud2 map_msg;
    // pcl::toROSMsg(*globalmap, map_msg);
    // map_msg.header.frame_id = "map";
    // map_msg.header.stamp = ros::Time::now();
    // globalmap_pub.publish(map_msg);

    if (!last_image.empty()) {
      
      cv::Mat projected_image = projectPointCloudToImage(transformed_cloud, last_image);
      sensor_msgs::ImagePtr projected_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", projected_image).toImageMsg();
      projected_image_pub.publish(projected_msg);

      // 計算逆變換矩陣
      Eigen::Matrix4f inverse_transform = transform.inverse();

      // 應用逆變換到 transformed_cloud
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr original_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
      pcl::transformPointCloud(*transformed_cloud, *original_cloud, inverse_transform);

    // 更新 pcss_map
      *pcss_map = *original_cloud;

      colored_cloud_pub.publish(pcss_map);
    } 

  }
  void image_callback(const sensor_msgs::ImageConstPtr& image_msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
      last_image = cv_ptr->image;
    }
    catch (cv_bridge::Exception& e) {
      NODELET_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  }
  void points_callback(const sensor_msgs::PointCloud2ConstPtr& points_msg) {
    if(!globalmap) {
      // NODELET_ERROR("globalmap has not been received!!");
      return;
    }
     
    const auto& stamp = points_msg->header.stamp;
    pcl::PointCloud<PointT>::Ptr pcl_cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*points_msg, *pcl_cloud);
    if(pcl_cloud->empty()) {
      // NODELET_ERROR("cloud is empty!!");
      return;
    }
    
    // transform pointcloud into odom_child_frame_id
    std::string tfError;
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());

    if(this->tf_buffer.canTransform(odom_child_frame_id, pcl_cloud->header.frame_id, stamp, ros::Duration(0.1), &tfError))
    {
        if(!pcl_ros::transformPointCloud(odom_child_frame_id, *pcl_cloud, *cloud, this->tf_buffer)) {
            NODELET_ERROR("point cloud cannot be transformed into target frame!!");
            return;
        }
    }else
    {
        NODELET_ERROR(tfError.c_str());
        return;
    }

    auto filtered = downsample(cloud);
    last_scan = filtered;

    if(relocalizing) {
      delta_estimater->add_frame(filtered);
    }

    std::lock_guard<std::mutex> estimator_lock(pose_estimator_mutex);
    if(!pose_estimator) {
      // NODELET_ERROR("waiting for initial pose input!!");
      return;
    }
    Eigen::Matrix4f before = pose_estimator->matrix();

    // odometry-based prediction
    ros::Time last_correction_time = pose_estimator->last_correction_time();
    if(private_nh.param<bool>("enable_robot_odometry_prediction", false) && !last_correction_time.isZero()) {
      geometry_msgs::TransformStamped odom_delta;
      if(tf_buffer.canTransform(odom_child_frame_id, last_correction_time, odom_child_frame_id, stamp, robot_odom_frame_id, ros::Duration(0.1))) {
        odom_delta = tf_buffer.lookupTransform(odom_child_frame_id, last_correction_time, odom_child_frame_id, stamp, robot_odom_frame_id, ros::Duration(0));
      } else if(tf_buffer.canTransform(odom_child_frame_id, last_correction_time, odom_child_frame_id, ros::Time(0), robot_odom_frame_id, ros::Duration(0))) {
        odom_delta = tf_buffer.lookupTransform(odom_child_frame_id, last_correction_time, odom_child_frame_id, ros::Time(0), robot_odom_frame_id, ros::Duration(0));
      }

      if(odom_delta.header.stamp.isZero()) {
        NODELET_WARN_STREAM("failed to look up transform between " << cloud->header.frame_id << " and " << robot_odom_frame_id);
      } else {
        Eigen::Isometry3d delta = tf2::transformToEigen(odom_delta);
        pose_estimator->predict_odom(delta.cast<float>().matrix());
      }
    }

    // correct
    auto aligned = pose_estimator->correct(stamp, filtered);

    if(aligned_pub.getNumSubscribers()) {
      aligned->header.frame_id = "map";
      aligned->header.stamp = cloud->header.stamp;
      aligned_pub.publish(aligned);
    }

    if(status_pub.getNumSubscribers()) {
      publish_scan_matching_status(points_msg->header, aligned);
    }

    publish_odometry(points_msg->header.stamp, pose_estimator->matrix());



    
  }
  void globalmap_callback(const sensor_msgs::PointCloud2ConstPtr& points_msg) {
    NODELET_INFO("globalmap received!");
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*points_msg, *cloud);
    globalmap = cloud;

    registration->setInputTarget(globalmap);

    if(use_global_localization) {
      NODELET_INFO("set globalmap for global localization!");
      hdl_global_localization::SetGlobalMap srv;
      pcl::toROSMsg(*globalmap, srv.request.global_map);

      if(!set_global_map_service.call(srv)) {
        NODELET_INFO("failed to set global map");
      } else {
        NODELET_INFO("done");
      }
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(*points_msg, *colored_cloud);
    // 从 ROS 消息转换到 PCL 点云，保留颜色信息
    // pcl::fromROSMsg(*points_msg, *cloud);
    // pcss_map.reset(new pcl::PointCloud<pcl::PointXYZRGB>());    
    // for(const auto& point : *cloud) {
    //   pcl::PointXYZRGB color_point;
    //   color_point.x = point.x;
    //   color_point.y = point.y;
    //   color_point.z = point.z;
    //   // 可以根據需要設置顏色
    //   // color_point.r = color_point.g = color_point.b = 254;
    //   pcss_map->points.push_back(color_point);
    // }
    pcss_map=colored_cloud;
    pcss_map->header.frame_id = "map";
  }

  bool relocalize(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res) {
    if(last_scan == nullptr) {
      NODELET_INFO_STREAM("no scan has been received");
      return false;
    }

    relocalizing = true;
    delta_estimater->reset();
    pcl::PointCloud<PointT>::ConstPtr scan = last_scan;

    hdl_global_localization::QueryGlobalLocalization srv;
    pcl::toROSMsg(*scan, srv.request.cloud);
    srv.request.max_num_candidates = 1;

    if(!query_global_localization_service.call(srv) || srv.response.poses.empty()) {
      relocalizing = false;
      NODELET_INFO_STREAM("global localization failed");
      return false;
    }

    const auto& result = srv.response.poses[0];

    NODELET_INFO_STREAM("--- Global localization result ---");
    NODELET_INFO_STREAM("Trans :" << result.position.x << " " << result.position.y << " " << result.position.z);
    NODELET_INFO_STREAM("Quat  :" << result.orientation.x << " " << result.orientation.y << " " << result.orientation.z << " " << result.orientation.w);
    NODELET_INFO_STREAM("Error :" << srv.response.errors[0]);
    NODELET_INFO_STREAM("Inlier:" << srv.response.inlier_fractions[0]);

    Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
    pose.linear() = Eigen::Quaternionf(result.orientation.w, result.orientation.x, result.orientation.y, result.orientation.z).toRotationMatrix();
    pose.translation() = Eigen::Vector3f(result.position.x, result.position.y, result.position.z);
    pose = pose * delta_estimater->estimated_delta();

    std::lock_guard<std::mutex> lock(pose_estimator_mutex);
    pose_estimator.reset(new hdl_localization::PoseEstimator(
      registration,
      pose.translation(),
      Eigen::Quaternionf(pose.linear()),
      private_nh.param<double>("cool_time_duration", 0.5)));

    relocalizing = false;

    return true;
  }
  void initialpose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_msg) {
    NODELET_INFO("initial pose received!!");
    std::lock_guard<std::mutex> lock(pose_estimator_mutex);
    const auto& p = pose_msg->pose.pose.position;
    const auto& q = pose_msg->pose.pose.orientation;
    pose_estimator.reset(
          new hdl_localization::PoseEstimator(
            registration,
            Eigen::Vector3f(p.x, p.y, p.z),
            Eigen::Quaternionf(q.w, q.x, q.y, q.z),
            private_nh.param<double>("cool_time_duration", 0.5))
    );
  }
  pcl::PointCloud<PointT>::ConstPtr downsample(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
    if(!downsample_filter) {
      return cloud;
    }

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    downsample_filter->setInputCloud(cloud);
    downsample_filter->filter(*filtered);
    filtered->header = cloud->header;

    return filtered;
  }
  void publish_odometry(const ros::Time& stamp, const Eigen::Matrix4f& pose) {
    // if tou have imu to get odom
    if(tf_buffer.canTransform(robot_odom_frame_id, odom_child_frame_id, ros::Time(0))) {
      geometry_msgs::TransformStamped map_wrt_frame = tf2::eigenToTransform(Eigen::Isometry3d(pose.inverse().cast<double>()));
      map_wrt_frame.header.stamp = stamp;
      map_wrt_frame.header.frame_id = odom_child_frame_id;
      map_wrt_frame.child_frame_id = "map";

      geometry_msgs::TransformStamped frame_wrt_odom = tf_buffer.lookupTransform(robot_odom_frame_id, odom_child_frame_id, ros::Time(0), ros::Duration(0.1));
      Eigen::Matrix4f frame2odom = tf2::transformToEigen(frame_wrt_odom).cast<float>().matrix();

      geometry_msgs::TransformStamped map_wrt_odom;
      tf2::doTransform(map_wrt_frame, map_wrt_odom, frame_wrt_odom);

      tf2::Transform odom_wrt_map;
      tf2::fromMsg(map_wrt_odom.transform, odom_wrt_map);
      odom_wrt_map = odom_wrt_map.inverse();

      geometry_msgs::TransformStamped odom_trans;
      odom_trans.transform = tf2::toMsg(odom_wrt_map);
      odom_trans.header.stamp = stamp;
      odom_trans.header.frame_id = "map";
      odom_trans.child_frame_id = robot_odom_frame_id;

      tf_broadcaster.sendTransform(odom_trans);
    } else {
      geometry_msgs::TransformStamped odom_trans = tf2::eigenToTransform(Eigen::Isometry3d(pose.cast<double>()));
      odom_trans.header.stamp = stamp;
      odom_trans.header.frame_id = "map";
      odom_trans.child_frame_id = odom_child_frame_id;
      tf_broadcaster.sendTransform(odom_trans);
    }
    // publish the transform
    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "map";

    tf::poseEigenToMsg(Eigen::Isometry3d(pose.cast<double>()), odom.pose.pose);
    odom.child_frame_id = odom_child_frame_id;
    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = 0.0;

    pose_pub.publish(odom);
  }
  void publish_scan_matching_status(const std_msgs::Header& header, pcl::PointCloud<pcl::PointXYZI>::ConstPtr aligned) {
    ScanMatchingStatus status;
    status.header = header;

    status.has_converged = registration->hasConverged();
    status.matching_error = 0.0;

    const double max_correspondence_dist = private_nh.param<double>("status_max_correspondence_dist", 0.5);
    const double max_valid_point_dist = private_nh.param<double>("status_max_valid_point_dist", 25.0);

    int num_inliers = 0;
    int num_valid_points = 0;
    std::vector<int> k_indices;
    std::vector<float> k_sq_dists;
    for(int i = 0; i < aligned->size(); i++) {
      const auto& pt = aligned->at(i);
      if (pt.getVector3fMap().norm() > max_valid_point_dist) {
        continue;
      }
      num_valid_points++;

      registration->getSearchMethodTarget()->nearestKSearch(pt, 1, k_indices, k_sq_dists);
      if(k_sq_dists[0] < max_correspondence_dist * max_correspondence_dist) {
        status.matching_error += k_sq_dists[0];
        num_inliers++;
      }
    }

    status.matching_error /= num_inliers;
    status.inlier_fraction = static_cast<float>(num_inliers) / std::max(1, num_valid_points);
    status.relative_pose = tf2::eigenToTransform(Eigen::Isometry3d(registration->getFinalTransformation().cast<double>())).transform;

    status.prediction_labels.reserve(2);
    status.prediction_errors.reserve(2);

    std::vector<double> errors(6, 0.0);

    if(pose_estimator->wo_prediction_error()) {
      status.prediction_labels.push_back(std_msgs::String());
      status.prediction_labels.back().data = "without_pred";
      status.prediction_errors.push_back(tf2::eigenToTransform(Eigen::Isometry3d(pose_estimator->wo_prediction_error().get().cast<double>())).transform);
    }

    if(pose_estimator->odom_prediction_error()) {
      status.prediction_labels.push_back(std_msgs::String());
      status.prediction_labels.back().data = "odom";
      status.prediction_errors.push_back(tf2::eigenToTransform(Eigen::Isometry3d(pose_estimator->odom_prediction_error().get().cast<double>())).transform);
    }

    status_pub.publish(status);
  }

private:
  // ROS
  
  ros::NodeHandle nh;
  ros::NodeHandle mt_nh;
  ros::NodeHandle private_nh;
  ros::Publisher projected_image_pub; 
  ros::Subscriber image_sub; 
  ros::Timer colorize_map_timer; 
  ros::Timer map_pub_timer; 

  std::string robot_odom_frame_id;
  std::string odom_child_frame_id;
  cv::Mat last_image;
  cv::Mat rvecs = (cv::Mat_<double>(3, 1) << 1.05609027, -1.20615636, 1.36252147);
  cv::Mat tvecs = (cv::Mat_<double>(3, 1) << 0.04703533,  0.64852852, 0.18442012);
  cv::Mat mtx = (cv::Mat_<double>(3, 3) << 575.416493, 0.0, 462.921798, 0.0, 570.435706, 272.781949, 0.0, 0.0, 1.0);
  cv::Mat dist = (cv::Mat_<double>(5, 1) << 0.157403 ,-0.231388, -0.003486, 0.003449, 0.000000);

  // bool use_imu;
  bool invert_acc;
  // bool invert_gyro;
  // ros::Subscriber imu_sub;
  ros::Subscriber points_sub;
  ros::Subscriber globalmap_sub;
  ros::Subscriber initialpose_sub;
  ros::Publisher colored_cloud_pub; 
  ros::Publisher pose_pub;
  ros::Publisher aligned_pub;
  ros::Publisher status_pub;
  ros::Publisher globalmap_pub;
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;
  tf2_ros::TransformBroadcaster tf_broadcaster;

  // globalmap and registration method
  pcl::PointCloud<PointT>::Ptr globalmap;
  pcl::Filter<PointT>::Ptr downsample_filter;
  pcl::Registration<PointT, PointT>::Ptr registration;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcss_map;
  // pose estimator
  std::mutex pose_estimator_mutex;
  std::unique_ptr<hdl_localization::PoseEstimator> pose_estimator;

  // global localization
  bool use_global_localization;
  std::atomic_bool relocalizing;
  std::unique_ptr<DeltaEstimater> delta_estimater;

  pcl::PointCloud<PointT>::ConstPtr last_scan;
  ros::ServiceServer relocalize_server;
  ros::ServiceClient set_global_map_service;
  ros::ServiceClient query_global_localization_service;
};
}
PLUGINLIB_EXPORT_CLASS(hdl_localization::HdlLocalizationNodelet, nodelet::Nodelet)
