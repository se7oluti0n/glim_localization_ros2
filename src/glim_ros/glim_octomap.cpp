#include <glim_ros/glim_octomap.hpp>

#include <mutex>
#include <spdlog/spdlog.h>
#include <rclcpp/clock.hpp>

#define GLIM_ROS2
#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <glim/odometry/callbacks.hpp>
#include <glim/mapping/callbacks.hpp>
#include <glim/util/logging.hpp>
#include <glim/util/config.hpp>
#include <glim/util/ros_cloud_converter.hpp>

namespace glim {

GlimOctomap::GlimOctomap() : logger(create_module_logger("glim_octomap")) {
  const Config config_ros(GlobalConfig::get_config_path("config_ros"));

  imu_frame_id = config_ros.param<std::string>("glim_ros", "imu_frame_id", "imu");
  lidar_frame_id = config_ros.param<std::string>("glim_ros", "lidar_frame_id", "lidar");
  base_frame_id = config_ros.param<std::string>("glim_ros", "base_frame_id", "");
  if (base_frame_id.empty()) {
    base_frame_id = imu_frame_id;
  }

  odom_frame_id = config_ros.param<std::string>("glim_ros", "odom_frame_id", "odom");
  map_frame_id = config_ros.param<std::string>("glim_ros", "map_frame_id", "map");
  publish_imu2lidar = config_ros.param<bool>("glim_ros", "publish_imu2lidar", true);
  tf_time_offset = config_ros.param<double>("glim_ros", "tf_time_offset", 1e-6);
  rviz_random_sampling_rate = config_ros.param<double>("glim_ros", "rviz_random_sampling_rate", 0.1);

  const Config config_octomap(GlobalConfig::get_config_path("config_octomap"));

  res_ = config_octomap.param<double>("octomap", "resolution", 0.1);
  point_cloud_min_x_ = config_octomap.param<double>("octomap", "point_cloud_min_x", -std::numeric_limits<double>::max());
  point_cloud_min_y_ = config_octomap.param<double>("octomap", "point_cloud_min_y", -std::numeric_limits<double>::max());
  point_cloud_max_x_ = config_octomap.param<double>("octomap", "point_cloud_max_x", std::numeric_limits<double>::max());
  point_cloud_max_y_ = config_octomap.param<double>("octomap", "point_cloud_max_y", std::numeric_limits<double>::max());

  point_cloud_min_z_ = config_octomap.param<double>("octomap", "point_cloud_min_z", -100.0);
  point_cloud_max_z_ = config_octomap.param<double>("octomap", "point_cloud_max_z", 100.0);

  occupancy_max_z_ = config_octomap.param<double>("octomap", "occupancy_max_z", 100.0);
  occupancy_min_z_ = config_octomap.param<double>("octomap", "occupancy_min_z", -100.0);

  min_x_size_ = config_octomap.param<double>("octomap", "min_x_size", 0.0);
  min_y_size_ = config_octomap.param<double>("octomap", "min_y_size", 0.0);

  filter_speckles_ = config_octomap.param<bool>("octomap", "filter_speckles", false);
  publish_2d_map_ = config_octomap.param<bool>("octomap", "publish_2d_map", false);
  compress_map_ = config_octomap.param<bool>("octomap", "compress_map", false);
  incremental_2D_projection_ = config_octomap.param<bool>("octomap", "incremental_2D_projection", false);
  // filter_ground_plane = config_octomap.param<bool>("octomap", "filter_ground_plane", false);
  max_range_ = config_octomap.param<double>("octomap", "max_range", 100.0);

  const double prob_hit = config_octomap.param<double>("octomap", "sensor_model_hit", 0.7);
  const double prob_miss = config_octomap.param<double>("octomap", "sensor_model_miss", 0.4);

  const double prob_min = config_octomap.param<double>("octomap", "sensor_model_min_prob", 0.12);
  const double prob_max = config_octomap.param<double>("octomap", "sensor_model_max_prob", 0.97);

  octree_ = std::make_unique<OcTreeT>(res_);
  octree_->setProbHit(prob_hit);
  octree_->setProbMiss(prob_miss);
  octree_->setClampingThresMin(prob_min);
  octree_->setClampingThresMax(prob_max);
  tree_depth_ = octree_->getTreeDepth();
  max_tree_depth_ = tree_depth_;

  gridmap_.info.resolution = res_;


  last_globalmap_pub_time = rclcpp::Clock(rcl_clock_type_t::RCL_ROS_TIME).now();
  // trajectory.reset(new TrajectoryManager);

  set_callbacks();

  kill_switch = false;
  thread = std::thread([this] {
    while (!kill_switch) {
      const auto expected = std::chrono::milliseconds(10);
      const auto t1 = std::chrono::high_resolution_clock::now();
      spin_once();
      const auto t2 = std::chrono::high_resolution_clock::now();

      if (t2 - t1 < expected) {
        std::this_thread::sleep_for(expected - (t2 - t1));
      }
    }
  });
}

GlimOctomap::~GlimOctomap() {
  kill_switch = true;
  thread.join();
}

std::vector<GenericTopicSubscription::Ptr> GlimOctomap::create_subscriptions(rclcpp::Node& node) {
  tf_buffer = std::make_unique<tf2_ros::Buffer>(node.get_clock());
  tf_listener = std::make_unique<tf2_ros::TransformListener>(*tf_buffer);
  tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(node);

  // auto qos = rclcpp::QoS{1};
  rclcpp::QoS map_qos(10);  // initialize to default
    // if (map_subscribe_transient_local_)
  {
    map_qos.transient_local();
    map_qos.reliable();
    map_qos.keep_last(1);
  }
  map_pub_ = node.create_publisher<OccupancyGrid>("projected_map", map_qos);
  return {};
}

void GlimOctomap::set_callbacks() {
  using std::placeholders::_1;
  // using std::placeholders::_2;
  GlobalMappingCallbacks::on_update_submaps.add(std::bind(&GlimOctomap::globalmap_on_update_submaps, this, _1));
}

void GlimOctomap::insert_pointcloud_to_octomap(
  gtsam_points::PointCloud::ConstPtr submap, const Eigen::Isometry3d& pose) {

  logger->info("Insert sumap to octomap");
  const auto sensor_origin = octomap::point3d(
    pose.translation().x(), pose.translation().y(), pose.translation().z());

  if (!octree_->coordToKeyChecked(sensor_origin, update_bbox_min_) ||
    !octree_->coordToKeyChecked(sensor_origin, update_bbox_max_))
  {
    logger->error("Could not generate Key for origin: ");
  }

    // instead of direct scan insertion, compute update to filter ground:
  octomap::KeySet free_cells, occupied_cells;
  // insert ground points only as free:
  // for (size_t i = 0; i < submap->size(); ++i) {
  //   octomap::point3d point(submap->points[i].x(), submap->points[i].y(), submap->points[i].z());
  //   // maxrange check
  //   if ((max_range_ > 0.0) && ((point - sensor_origin).norm() > max_range_) ) {
  //     point = sensor_origin + (point - sensor_origin).normalized() * max_range_;
  //   }

  //   // only clear space (ground points)
  //   if (octree_->computeRayKeys(sensor_origin, point, key_ray_)) {
  //     free_cells.insert(key_ray_.begin(), key_ray_.end());
  //   }

  //   octomap::OcTreeKey end_key;
  //   if (octree_->coordToKeyChecked(point, end_key)) {
  //     updateMinKey(end_key, update_bbox_min_);
  //     updateMaxKey(end_key, update_bbox_max_);
  //   } else {
  //     // RCLCPP_ERROR_STREAM(get_logger(), "Could not generate Key for endpoint " << point);
  //     logger->error("Could not generate Key for endpoint: ");
  //   }
  // }

    // all other points: free on ray, occupied on endpoint:
  for (size_t i = 0; i < submap->size(); ++i) {
    octomap::point3d point(submap->points[i].x(), submap->points[i].y(), submap->points[i].z());
    if ( point.x() < point_cloud_min_x_ || point.x() > point_cloud_max_x_
      || point.y() < point_cloud_min_y_ || point.y() > point_cloud_max_y_
      || point.z() < point_cloud_min_z_ || point.z() > point_cloud_max_z_
    )
      continue;
    // maxrange check
    if ((max_range_ < 0.0) || ((point - sensor_origin).norm() <= max_range_) ) {
      // free cells
      if (octree_->computeRayKeys(sensor_origin, point, key_ray_)) {
        free_cells.insert(key_ray_.begin(), key_ray_.end());
      }
      // occupied endpoint
      octomap::OcTreeKey key;
      if (octree_->coordToKeyChecked(point, key)) {
        occupied_cells.insert(key);

        updateMinKey(key, update_bbox_min_);
        updateMaxKey(key, update_bbox_max_);
      }
    } else {  // ray longer than maxrange
      octomap::point3d new_end = sensor_origin + (point - sensor_origin).normalized() * max_range_;
      if (octree_->computeRayKeys(sensor_origin, new_end, key_ray_)) {
        free_cells.insert(key_ray_.begin(), key_ray_.end());

        octomap::OcTreeKey end_key;
        if (octree_->coordToKeyChecked(new_end, end_key)) {
          free_cells.insert(end_key);
          updateMinKey(end_key, update_bbox_min_);
          updateMaxKey(end_key, update_bbox_max_);
        } else {
          // RCLCPP_ERROR_STREAM(get_logger(), "Could not generate Key for endpoint " << new_end);
           logger->error("Could not generate Key for endpoint: ");
        }
      }
    }
  }

  // mark free cells only if not seen occupied in this cloud
  for (auto it = free_cells.begin(), end = free_cells.end(); it != end; ++it) {
    if (occupied_cells.find(*it) == occupied_cells.end()) {
      octree_->updateNode(*it, false);
    }
  }

  // now mark all occupied cells:
  for (auto it = occupied_cells.begin(), end = occupied_cells.end(); it != end; it++) {
    octree_->updateNode(*it, true);
  }

  // TODO(someone): eval lazy+updateInner vs. proper insertion
  // non-lazy by default (updateInnerOccupancy() too slow for large maps)
  // octree_->updateInnerOccupancy();

  if (compress_map_) {
    octree_->prune();
  }
}

void GlimOctomap::handlePostNodeTraversal([[maybe_unused]] const rclcpp::Time & rostime)
{
  if (publish_2d_map_) {
    map_pub_->publish(gridmap_);
  }
}

void GlimOctomap::handleOccupiedNode(const OcTreeT::iterator & it)
{
  if (publish_2d_map_ && project_complete_map_) {
    update2DMap(it, true);
  }
}

void GlimOctomap::handleFreeNode(const OcTreeT::iterator & it)
{
  if (publish_2d_map_ && project_complete_map_) {
    update2DMap(it, false);
  }
}

void GlimOctomap::handleOccupiedNodeInBBX(const OcTreeT::iterator & it)
{
  if (publish_2d_map_ && project_complete_map_) {
    update2DMap(it, true);
  }
}

void GlimOctomap::handleFreeNodeInBBX(const OcTreeT::iterator & it)
{
  if (publish_2d_map_ && project_complete_map_) {
    update2DMap(it, false);
  }
}

void GlimOctomap::update2DMap(const OcTreeT::iterator & it, bool occupied)
{
  // update 2D map (occupied always overrides):
  if (it.getDepth() == max_tree_depth_) {
    unsigned idx = mapIdx(it.getKey());
    if (occupied) {
      gridmap_.data[mapIdx(it.getKey())] = 100;
    } else if (gridmap_.data[idx] == -1) {
      gridmap_.data[idx] = 0;
    }

  } else {
    int int_size = 1 << (max_tree_depth_ - it.getDepth());
    octomap::OcTreeKey min_key = it.getIndexKey();
    for (int dx = 0; dx < int_size; dx++) {
      int i = (min_key[0] + dx - padded_min_key_[0]) / multires_2d_scale_;
      for (int dy = 0; dy < int_size; dy++) {
        unsigned idx = mapIdx(i, (min_key[1] + dy - padded_min_key_[1]) / multires_2d_scale_);
        if (occupied) {
          gridmap_.data[idx] = 100;
        } else if (gridmap_.data[idx] == -1) {
          gridmap_.data[idx] = 0;
        }
      }
    }
  }
}

void GlimOctomap::handlePreNodeTraversal(const rclcpp::Time & rostime)
{
  if (publish_2d_map_) {
    // init projected 2D map:
    gridmap_.header.frame_id = map_frame_id;
    gridmap_.header.stamp = rostime;
    MapMetaData old_map_info = gridmap_.info;

    // TODO(someone): move most of this stuff into c'tor
    // and init map only once (adjust if size changes)
    double min_x{};
    double min_y{};
    double min_z{};
    double max_x{};
    double max_y{};
    double max_z{};
    octree_->getMetricMin(min_x, min_y, min_z);
    octree_->getMetricMax(max_x, max_y, max_z);

    octomap::point3d min_pt(min_x, min_y, min_z);
    octomap::point3d max_pt(max_x, max_y, max_z);
    octomap::OcTreeKey min_key = octree_->coordToKey(min_pt, max_tree_depth_);
    octomap::OcTreeKey max_key = octree_->coordToKey(max_pt, max_tree_depth_);

    // RCLCPP_DEBUG(
    //   get_logger(),
    //   "min_key: %d %d %d / max_key: %d %d %d", min_key[0], min_key[1], min_key[2], max_key[0],
    //   max_key[1], max_key[2]);

    // add padding if requested (= new min/max_pts in x&y):
    double half_padded_x = 0.5 * min_x_size_;
    double half_padded_y = 0.5 * min_y_size_;
    min_x = std::min(min_x, -half_padded_x);
    max_x = std::max(max_x, half_padded_x);
    min_y = std::min(min_y, -half_padded_y);
    max_y = std::max(max_y, half_padded_y);
    min_pt = octomap::point3d(min_x, min_y, min_z);
    max_pt = octomap::point3d(max_x, max_y, max_z);

    octomap::OcTreeKey padded_max_key;
    if (!octree_->coordToKeyChecked(min_pt, max_tree_depth_, padded_min_key_)) {
      // RCLCPP_ERROR(
      //   get_logger(),
      //   "Could not create padded min OcTree key at %f %f %f", min_pt.x(), min_pt.y(),
      //   min_pt.z());
      return;
    }
    if (!octree_->coordToKeyChecked(max_pt, max_tree_depth_, padded_max_key)) {
      // RCLCPP_ERROR(
      //   get_logger(),
      //   "Could not create padded max OcTree key at %f %f %f", max_pt.x(), max_pt.y(),
      //   max_pt.z());
      return;
    }

    // RCLCPP_DEBUG(
    //   get_logger(),
    //   "Padded MinKey: %d %d %d / padded MaxKey: %d %d %d", padded_min_key_[0],
    //   padded_min_key_[1], padded_min_key_[2], padded_max_key[0], padded_max_key[1],
    //   padded_max_key[2]);
    assert(padded_max_key[0] >= max_key[0] && padded_max_key[1] >= max_key[1]);

    multires_2d_scale_ = 1 << (tree_depth_ - max_tree_depth_);
    gridmap_.info.width = (padded_max_key[0] - padded_min_key_[0]) / multires_2d_scale_ + 1;
    gridmap_.info.height = (padded_max_key[1] - padded_min_key_[1]) / multires_2d_scale_ + 1;

    [[maybe_unused]] int map_origin_x = min_key[0] - padded_min_key_[0];
    [[maybe_unused]] int map_origin_y = min_key[1] - padded_min_key_[1];
    assert(map_origin_x >= 0 && map_origin_y >= 0);

    // might not exactly be min / max of octree:
    octomap::point3d origin = octree_->keyToCoord(padded_min_key_, tree_depth_);
    double grid_res = octree_->getNodeSize(max_tree_depth_);
    project_complete_map_ =
      (!incremental_2D_projection_ || (std::abs(grid_res - gridmap_.info.resolution) > 1e-6));
    gridmap_.info.resolution = grid_res;
    gridmap_.info.origin.position.x = origin.x() - grid_res * 0.5;
    gridmap_.info.origin.position.y = origin.y() - grid_res * 0.5;
    if (max_tree_depth_ != tree_depth_) {
      gridmap_.info.origin.position.x -= res_ / 2.0;
      gridmap_.info.origin.position.y -= res_ / 2.0;
    }

    // workaround for  multires. projection not working properly for inner nodes:
    // force re-building complete map
    if (max_tree_depth_ < tree_depth_) {
      project_complete_map_ = true;
    }

    if (project_complete_map_) {
      // RCLCPP_DEBUG(get_logger(), "Rebuilding complete 2D map");
      gridmap_.data.clear();
      // init to unknown:
      gridmap_.data.resize(gridmap_.info.width * gridmap_.info.height, -1);

    } else {
      if (mapChanged(old_map_info, gridmap_.info)) {
        // RCLCPP_DEBUG(
        //   get_logger(), "2D grid map size changed to %dx%d", gridmap_.info.width,
        //   gridmap_.info.height);
        adjustMapData(gridmap_, old_map_info);
      }
      OccupancyGrid::_data_type::iterator startIt;
      size_t mapUpdateBBXmin_x =
        std::max(
        0,
        (static_cast<int>(update_bbox_min_[0]) - static_cast<int>(padded_min_key_[0])) /
        static_cast<int>(multires_2d_scale_));
      size_t mapUpdateBBXmin_y =
        std::max(
        0,
        (static_cast<int>(update_bbox_min_[1]) - static_cast<int>(padded_min_key_[1])) /
        static_cast<int>(multires_2d_scale_));
      size_t mapUpdateBBXmax_x =
        std::min(
        static_cast<int>(gridmap_.info.width - 1),
        (static_cast<int>(update_bbox_max_[0]) - static_cast<int>(padded_min_key_[0])) /
        static_cast<int>(multires_2d_scale_));
      size_t mapUpdateBBXmax_y =
        std::min(
        static_cast<int>(gridmap_.info.height - 1),
        (static_cast<int>(update_bbox_max_[1]) - static_cast<int>(padded_min_key_[1])) /
        static_cast<int>(multires_2d_scale_));

      assert(mapUpdateBBXmax_x > mapUpdateBBXmin_x);
      assert(mapUpdateBBXmax_y > mapUpdateBBXmin_y);

      size_t numCols = mapUpdateBBXmax_x - mapUpdateBBXmin_x + 1;

      // test for max idx:
      uint max_idx = gridmap_.info.width * mapUpdateBBXmax_y + mapUpdateBBXmax_x;
      if (max_idx >= gridmap_.data.size()) {
        // RCLCPP_ERROR(
        //   get_logger(),
        //   "BBX index not valid: %d (max index %zu for size %d x %d) "
        //   "update-BBX is: [%zu %zu]-[%zu %zu]", max_idx,
        //   gridmap_.data.size(), gridmap_.info.width, gridmap_.info.height,
        //   mapUpdateBBXmin_x, mapUpdateBBXmin_y, mapUpdateBBXmax_x, mapUpdateBBXmax_y);
      }

      // reset proj. 2D map in bounding box:
      for (unsigned int j = mapUpdateBBXmin_y; j <= mapUpdateBBXmax_y; ++j) {
        std::fill_n(
          gridmap_.data.begin() + gridmap_.info.width * j + mapUpdateBBXmin_x,
          numCols, -1);
      }
    }
  }
}


void GlimOctomap::adjustMapData(OccupancyGrid & map, const MapMetaData & old_map_info) const
{
  if (map.info.resolution != old_map_info.resolution) {
    // RCLCPP_ERROR(get_logger(), "Resolution of map changed, cannot be adjusted");
    return;
  }

  int i_off =
    static_cast<int>((old_map_info.origin.position.x - map.info.origin.position.x) /
    map.info.resolution + 0.5);
  int j_off =
    static_cast<int>((old_map_info.origin.position.y - map.info.origin.position.y) /
    map.info.resolution + 0.5);

  if (i_off < 0 || j_off < 0 ||
    old_map_info.width + i_off > map.info.width ||
    old_map_info.height + j_off > map.info.height)
  {
    // RCLCPP_ERROR(
    //   get_logger(), "New 2D map does not contain old map area, this case is not implemented");
    return;
  }

  OccupancyGrid::_data_type old_map_data = map.data;

  map.data.clear();
  // init to unknown:
  map.data.resize(map.info.width * map.info.height, -1);

  OccupancyGrid::_data_type::iterator from_start, from_end, to_start;

  for (size_t j = 0; j < old_map_info.height; ++j) {
    // copy chunks, row by row:
    from_start = old_map_data.begin() + j * old_map_info.width;
    from_end = from_start + old_map_info.width;
    to_start = map.data.begin() + ((j + j_off) * gridmap_.info.width + i_off);
    copy(from_start, from_end, to_start);

//    for (int i =0; i < int(old_map_info.width); ++i){
//      map.data[gridmap_.info.width*(j+j_off) +i+i_off] = oldMapData[old_map_info.width*j +i];
//    }
  }
}


void GlimOctomap::globalmap_on_update_submaps(const std::vector<SubMap::Ptr>& submaps) {
  if (submaps.size() == 0)
    return;
  const SubMap::ConstPtr latest_submap = submaps.back();

  const double stamp_endpoint_R = latest_submap->odom_frames.back()->stamp;
  const Eigen::Isometry3d T_world_endpoint_R = latest_submap->T_world_origin * latest_submap->T_origin_endpoint_R;

  // Invoke a submap concatenation task in the GlimOctomap thread
  {
    std::lock_guard<std::mutex> lock(submaps_mutex);
    this->submap_pointclouds_.clear();
    this->submap_poses_.clear();

    for (auto &submap: submaps) {
      this->submap_pointclouds_.push_back((submap->frame));
      this->submap_poses_.push_back(submap->T_world_origin);
    }
  }

  invoke([this] {

    std::mt19937 mt;

    // Publish global map every 10 seconds
    const rclcpp::Time now = rclcpp::Clock(rcl_clock_type_t::RCL_ROS_TIME).now();
    if (now - last_globalmap_pub_time < std::chrono::seconds(10)) {
      return;
    }
    last_globalmap_pub_time = now;

    this->reset_octree();
    gtsam_points::PointCloudCPU::Ptr merged(new gtsam_points::PointCloudCPU);
    // render
    {
        std::lock_guard<std::mutex> lock(this->submaps_mutex);

      int total_num_points = 0;

      for (size_t i = 0; i < this->submap_pointclouds_.size(); ++i) {
        total_num_points += this->submap_pointclouds_[i]->size();
      }
    // Concatenate all the submap points

      merged->num_points = total_num_points;
      merged->points_storage.resize(total_num_points);
      merged->points = merged->points_storage.data();

      // auto &submap = latest_submap->frame;
      int begin = 0;

      for (size_t i = 0; i < this->submap_pointclouds_.size(); ++i) {
        auto &frame = this->submap_pointclouds_[i];
        auto &pose = this->submap_poses_[i];
        std::transform(frame->points, frame->points + frame->size(), merged->points + begin, [&](const Eigen::Vector4d& p) {
          return pose * p; });
        begin += frame->size();
      }
    }

    this->insert_pointcloud_to_octomap(
      merged,
      Eigen::Isometry3d::Identity());
    this->publish_2d_map(now);

  });
}

bool GlimOctomap::isSpeckleNode(const octomap::OcTreeKey & n_key) const
{
  octomap::OcTreeKey key;
  bool neighbor_found = false;
  for (key[2] = n_key[2] - 1; !neighbor_found && key[2] <= n_key[2] + 1; ++key[2]) {
    for (key[1] = n_key[1] - 1; !neighbor_found && key[1] <= n_key[1] + 1; ++key[1]) {
      for (key[0] = n_key[0] - 1; !neighbor_found && key[0] <= n_key[0] + 1; ++key[0]) {
        if (key != n_key) {
          octomap::OcTreeNode * node = octree_->search(key);
          if (node && octree_->isNodeOccupied(node)) {
            // we have a neighbor=> break!
            neighbor_found = true;
          }
        }
      }
    }
  }

  return neighbor_found;
}

void GlimOctomap::publish_2d_map(const rclcpp::Time & rostime) {
   handlePreNodeTraversal(rostime);

  // now, traverse all leafs in the tree:
  for (OcTreeT::iterator it = octree_->begin(max_tree_depth_),
    end = octree_->end(); it != end; ++it)
  {
    bool in_update_bbox = isInUpdateBBX(it);

    // call general hook:
    handleNode(it);
    if (in_update_bbox) {
      handleNodeInBBX(it);
    }

    if (octree_->isNodeOccupied(*it)) {
      double z = it.getZ();
      double half_size = it.getSize() / 2.0;
      if (z + half_size > occupancy_min_z_ && z - half_size < occupancy_max_z_) {
        double x = it.getX();
        double y = it.getY();
#ifdef COLOR_OCTOMAP_SERVER
        int r = it->getColor().r;
        int g = it->getColor().g;
        int b = it->getColor().b;
#endif

        // Ignore speckles in the map:
        if (filter_speckles_ && (it.getDepth() == tree_depth_ + 1) && isSpeckleNode(it.getKey())) {
          // RCLCPP_DEBUG(get_logger(), "Ignoring single speckle at (%f,%f,%f)", x, y, z);
          continue;
        }  // else: current octree node is no speckle, send it out

        handleOccupiedNode(it);
        if (in_update_bbox) {
          handleOccupiedNodeInBBX(it);
        }

      }
    } else {  // node not occupied => mark as free in 2D map if unknown so far
      double z = it.getZ();
      double half_size = it.getSize() / 2.0;
      if (z + half_size > occupancy_min_z_ && z - half_size < occupancy_max_z_) {
        handleFreeNode(it);
        if (in_update_bbox) {
          handleFreeNodeInBBX(it);
        }


      }
    }
  }

  // call post-traversal hook:
  handlePostNodeTraversal(rostime);
}

void GlimOctomap::reset_octree() {
    octree_->clear();
    // clear 2D map:
    gridmap_.data.clear();
    gridmap_.info.height = 0.0;
    gridmap_.info.width = 0.0;
    gridmap_.info.resolution = 0.0;
    gridmap_.info.origin.position.x = 0.0;
    gridmap_.info.origin.position.y = 0.0;
}

void GlimOctomap::invoke(const std::function<void()>& task) {
  std::lock_guard<std::mutex> lock(invoke_queue_mutex);
  invoke_queue.push_back(task);
}

void GlimOctomap::spin_once() {
  std::vector<std::function<void()>> invoke_queue;

  {
    std::lock_guard<std::mutex> lock(invoke_queue_mutex);
    invoke_queue.swap(this->invoke_queue);
  }

  for (const auto& task : invoke_queue) {
    task();
  }
}

}  // namespace glim

extern "C" glim::ExtensionModule* create_extension_module() {
  return new glim::GlimOctomap();
}