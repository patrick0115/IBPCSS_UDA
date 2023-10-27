// SPDX-License-Identifier: BSD-2-Clause

#include <hdl_graph_slam/map_cloud_generator.hpp>

#include <pcl/octree/octree_search.h>

namespace hdl_graph_slam {

MapCloudGenerator::MapCloudGenerator() {}

MapCloudGenerator::~MapCloudGenerator() {}

pcl::PointCloud<MapCloudGenerator::PointRGBT>::Ptr MapCloudGenerator::generateColoredPointCloud(const std::vector<KeyFrameSnapshot::Ptr>& keyframes, double resolution) const {
  if(keyframes.empty()) {
    std::cerr << "warning: keyframes empty!!" << std::endl;
    return nullptr;
  }

  pcl::PointCloud<PointRGBT>::Ptr cloudRGB(new pcl::PointCloud<PointRGBT>());

  for(const auto& keyframe : keyframes) {
    Eigen::Matrix4f pose = keyframe->pose.matrix().cast<float>();
    

    for(const auto& src_pt : keyframe->cloudRGB->points) {
      PointRGBT dst_pt;
      dst_pt.getVector4fMap() = pose * src_pt.getVector4fMap();
      dst_pt.r = src_pt.r;
      dst_pt.g = src_pt.g;
      dst_pt.b = src_pt.b;
      cloudRGB->push_back(dst_pt);
    }
  }

  cloudRGB->width = cloudRGB->size();
  cloudRGB->height = 1;
  cloudRGB->is_dense = false;


  if (resolution <=0.0)
    return cloudRGB;

  return cloudRGB;

}

pcl::PointCloud<MapCloudGenerator::PointT>::Ptr MapCloudGenerator::generate(const std::vector<KeyFrameSnapshot::Ptr>& keyframes, double resolution) const {

  if(keyframes.empty()) {
    std::cerr << "warning: keyframes empty!!" << std::endl;
    return nullptr;
  }

  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());

  cloud->reserve(keyframes.front()->cloud->size() * keyframes.size());

  for(const auto& keyframe : keyframes) {
    Eigen::Matrix4f pose = keyframe->pose.matrix().cast<float>();
    for(const auto& src_pt : keyframe->cloud->points) {
      PointT dst_pt;
      dst_pt.getVector4fMap() = pose * src_pt.getVector4fMap();
      dst_pt.intensity = src_pt.intensity;
      cloud->push_back(dst_pt);
    }
  }

  cloud->width = cloud->size();
  cloud->height = 1;
  cloud->is_dense = false;



  if (resolution <=0.0)
    return cloud; // To get unfiltered point cloud with intensity

  pcl::octree::OctreePointCloud<PointT> octree(resolution);
  octree.setInputCloud(cloud);
  octree.addPointsFromInputCloud();

  pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
  octree.getOccupiedVoxelCenters(filtered->points);

  filtered->width = filtered->size();
  filtered->height = 1;
  filtered->is_dense = false;

  return filtered;
}


}  // namespace hdl_graph_slam
