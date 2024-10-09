#pragma once

#include <any>
#include <atomic>
#include <thread>
#include <chrono>
#include <deque>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>


#include <nav_msgs/msg/occupancy_grid.hpp>

#include <glim/odometry/estimation_frame.hpp>
#include <glim/mapping/sub_map.hpp>
#include <glim/util/extension_module.hpp>
#include <glim/util/extension_module_ros2.hpp>

#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.hpp"


#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>

#include "octomap_msgs/msg/octomap.hpp"
#include "octomap_msgs/srv/get_octomap.hpp"
#include "octomap_msgs/srv/bounding_box_query.hpp"
#include "octomap_msgs/conversions.h"

#include "octomap_ros/conversions.hpp"

namespace spdlog {
class logger;
}

namespace glim {

class TrajectoryManager;

using nav_msgs::msg::MapMetaData;
using nav_msgs::msg::OccupancyGrid;
using octomap_msgs::msg::Octomap;

/**
 * @brief Rviz-based viewer
 */
class GlimOctomap : public ExtensionModuleROS2 {
public:

using PCLPoint = pcl::PointXYZ;
using PCLPointCloud = pcl::PointCloud<pcl::PointXYZ>;
using OcTreeT = octomap::OcTree;

  GlimOctomap();
  ~GlimOctomap();

  virtual std::vector<GenericTopicSubscription::Ptr> create_subscriptions(rclcpp::Node& node) override;

private:
  void set_callbacks();
  void globalmap_on_update_submaps(const std::vector<SubMap::Ptr>& submaps);
  void invoke(const std::function<void()>& task);

  void spin_once();

private:
  inline static void updateMinKey(const octomap::OcTreeKey & in, octomap::OcTreeKey & min)
  {
    for (size_t i = 0; i < 3; ++i) {
      min[i] = std::min(in[i], min[i]);
    }
  }

  inline static void updateMaxKey(const octomap::OcTreeKey & in, octomap::OcTreeKey & max)
  {
    for (size_t i = 0; i < 3; ++i) {
      max[i] = std::max(in[i], max[i]);
    }
  }

  inline size_t mapIdx(const int i, const int j) const
  {
    return gridmap_.info.width * j + i;
  }

  inline size_t mapIdx(const octomap::OcTreeKey & key) const
  {
    return mapIdx(
      (key[0] - padded_min_key_[0]) / multires_2d_scale_,
      (key[1] - padded_min_key_[1]) / multires_2d_scale_);
  }

  inline bool mapChanged(const MapMetaData & old_map_info, const MapMetaData & new_map_info)
  {
    return old_map_info.height != new_map_info.height ||
           old_map_info.width != new_map_info.width ||
           old_map_info.origin.position.x != new_map_info.origin.position.x ||
           old_map_info.origin.position.y != new_map_info.origin.position.y;
  }

    /// Test if key is within update area of map (2D, ignores height)
  inline bool isInUpdateBBX(const OcTreeT::iterator & it) const
  {
    // 2^(tree_depth-depth) voxels wide:
    unsigned voxelWidth = (1 << (max_tree_depth_ - it.getDepth()));
    octomap::OcTreeKey key = it.getIndexKey();  // lower corner of voxel
    return key[0] + voxelWidth >= update_bbox_min_[0] &&
           key[1] + voxelWidth >= update_bbox_min_[1] &&
           key[0] <= update_bbox_max_[0] &&
           key[1] <= update_bbox_max_[1];
  }

  void adjustMapData(OccupancyGrid & map, const MapMetaData & old_map_info) const;
  void update2DMap(const OcTreeT::iterator & it, bool occupied);
  void insert_pointcloud_to_octomap(gtsam_points::PointCloud::ConstPtr submap, const Eigen::Isometry3d& pose);
  void handlePreNodeTraversal(const rclcpp::Time & rostime);
  bool isSpeckleNode(const octomap::OcTreeKey & n_key) const;

  /// hook that is called when traversing all nodes of the updated Octree (does nothing here)
  virtual void handleNode([[maybe_unused]] const OcTreeT::iterator & it) {}

  /// hook that is called
  /// when traversing all nodes of the updated Octree in the updated area (does nothing here)
  virtual void handleNodeInBBX([[maybe_unused]] const OcTreeT::iterator & it) {}

  /// hook that is called when traversing occupied nodes of the updated Octree
  virtual void handleOccupiedNode(const OcTreeT::iterator & it);

  /// hook that is called
  /// when traversing occupied nodes in the updated area (updates 2D map projection here)
  virtual void handleOccupiedNodeInBBX(const OcTreeT::iterator & it);

  /// hook that is called when traversing free nodes of the updated Octree
  virtual void handleFreeNode(const OcTreeT::iterator & it);

  /// hook that is called
  /// when traversing free nodes in the updated area (updates 2D map projection here)
  virtual void handleFreeNodeInBBX(const OcTreeT::iterator & it);

  /// hook that is called after traversing all nodes
  virtual void handlePostNodeTraversal(const rclcpp::Time & rostime);

  void reset_octree();

  void publish_2d_map(const rclcpp::Time & rostime);

private:
  std::atomic_bool kill_switch;
  std::thread thread;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

  rclcpp::Time last_globalmap_pub_time;

  std::string imu_frame_id;
  std::string lidar_frame_id;
  std::string base_frame_id;
  std::string odom_frame_id;
  std::string map_frame_id;
  bool publish_imu2lidar;
  double tf_time_offset;
  double rviz_random_sampling_rate;


  std::mutex submaps_mutex;
  // std::unique_ptr<TrajectoryManager> trajectory;
  std::vector<gtsam_points::PointCloud::ConstPtr> submap_pointclouds_;
  std::vector<Eigen::Isometry3d> submap_poses_;

  std::vector<SubMap::Ptr> submaps;

  std::mutex invoke_queue_mutex;
  std::vector<std::function<void()>> invoke_queue;

  // Logging
  std::shared_ptr<spdlog::logger> logger;

  std::unique_ptr<OcTreeT> octree_;
  OccupancyGrid gridmap_;
  octomap::KeyRay key_ray_;  // temp storage for ray casting
  octomap::OcTreeKey update_bbox_min_;
  octomap::OcTreeKey update_bbox_max_;

  double max_range_;
  double res_;
  size_t tree_depth_;
  size_t max_tree_depth_;
  bool compress_map_;
  bool publish_2d_map_;
  bool project_complete_map_;

  octomap::OcTreeKey padded_min_key_;
  unsigned multires_2d_scale_;
  bool incremental_2D_projection_;
  rclcpp::Publisher<OccupancyGrid>::SharedPtr map_pub_;
  double point_cloud_min_x_;
  double point_cloud_max_x_;
  double point_cloud_min_y_;
  double point_cloud_max_y_;
  double point_cloud_min_z_;
  double point_cloud_max_z_;
  double occupancy_min_z_;
  double occupancy_max_z_;
  double min_x_size_;
  double min_y_size_;
  bool filter_speckles_;


};
}  // namespace glim