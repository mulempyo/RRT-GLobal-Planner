#include <rrt_global_planner/rrt.h>
#include <pluginlib/class_list_macros.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <cmath>
#include <random>
#include <limits>
#include <unordered_map>

PLUGINLIB_EXPORT_CLASS(rrt::RRTPlanner, nav_core::BaseGlobalPlanner)

namespace rrt {

RRTPlanner::RRTPlanner() : initialized_(false), goal_threshold_(0.5), step_size_(0.25), max_iterations_(10000) {}

RRTPlanner::RRTPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
    : initialized_(false), goal_threshold_(0.5), step_size_(0.25), max_iterations_(10000) {
  initialize(name, costmap_ros);
}

RRTPlanner::~RRTPlanner() {
  if(world_model_)
    delete[] world_model_;
}

void RRTPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros) {
  ros::NodeHandle n;

  if (!initialized_) {
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();
    origin_x_ = costmap_->getOriginX();
    origin_y_ = costmap_->getOriginY();
    resolution_ = costmap_->getResolution();
    width_ = costmap_->getSizeInCellsX();
    height_ = costmap_->getSizeInCellsY();
    world_model_ = new base_local_planner::CostmapModel(*costmap_);
    initialized_ = true;
  } else {
    ROS_WARN("RRTPlanner has already been initialized.");
  }
}

bool RRTPlanner::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                          std::vector<geometry_msgs::PoseStamped> &plan) {

  ros::NodeHandle n;
  if (!initialized_) {
    ROS_ERROR("RRTPlanner has not been initialized, please call initialize() before use.");
    return false;
  }

  plan.clear();

  // Convert world coordinates to map coordinates
  unsigned int start_x, start_y, goal_x, goal_y;
  if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y)) {
    ROS_WARN("The start is out of the map bounds.");
    return false;
  }

  if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y)) {
    ROS_WARN("The goal is out of the map bounds.");
    return false;
  }

  unsigned int start_index = start_y * width_ + start_x;
  unsigned int goal_index = goal_y * width_ + goal_x;

  std::unordered_map<unsigned int, unsigned int> parent_map;
  parent_map[start_index] = start_index;

  tree.push_back(start_index);
  unsigned int final_node_index = 0;

  for (unsigned int i = 0; i < max_iterations_; ++i) {
    // Generate a random valid pose
    double random_x, random_y, random_th;
    createRandomValidPose(random_x, random_y, random_th);

    unsigned int nearest_index = nearestNode(random_x, random_y);

    unsigned int nearest_x = nearest_index % width_;
    unsigned int nearest_y = nearest_index / width_;

    double theta = atan2(random_y - nearest_y, random_x - nearest_x);
    double new_x = nearest_x + step_size_ * cos(theta);
    double new_y = nearest_y + step_size_ * sin(theta);

    if (new_x >= width_ || new_y >= height_ || !isValidPose(new_x, new_y, theta)) {
      continue;
    }

    unsigned int new_index = new_y * width_ + new_x;
    tree.push_back(new_index);
    parent_map[new_index] = nearest_index;

    if (distance(new_x, new_y, goal_x, goal_y) < goal_threshold_) {
      final_node_index = goal_index;
      parent_map[goal_index] = new_index;
      tree.push_back(goal_index);
      break;
    }
  }

  // Reconstruct path
  if (final_node_index != 0) {
    unsigned int current_index = final_node_index;
    while (current_index != start_index) {
      unsigned int current_x = current_index % width_;
      unsigned int current_y = current_index / width_;
      double wx, wy;
      mapToWorld(current_x, current_y, wx, wy);
      geometry_msgs::PoseStamped pose = goal;
      pose.pose.position.x = wx;
      pose.pose.position.y = wy;
      pose.pose.position.z = 0;
      pose.pose.orientation.w = 1.0;
      plan.push_back(pose);
      current_index = parent_map[current_index];
    }
    std::reverse(plan.begin(), plan.end());
    return true;
  } else {
    ROS_WARN("Failed to find a valid plan.");
    return false;
  }
}

double RRTPlanner::footprintCost(double x, double y, double th) const {
  if (!initialized_) {
    ROS_ERROR("The RRT Planner has not been initialized, you must call initialize().");
    return -1.0;
  }

  std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();

  if (footprint.size() < 3) return -1.0;

  double footprint_cost = world_model_->footprintCost(x, y, th, footprint);

  return footprint_cost;
}

bool RRTPlanner::isValidPose(double x, double y, double th) const {
  double footprint_cost = footprintCost(x, y, th);
  if ((footprint_cost < 0) || (footprint_cost > 128)) return false;
  return true;
}

void RRTPlanner::createRandomValidPose(double &x, double &y, double &th) const {
  // get bounds of the costmap in world coordinates
  double wx_min, wy_min;
  costmap_->mapToWorld(0, 0, wx_min, wy_min);

  double wx_max, wy_max;
  unsigned int mx_max = costmap_->getSizeInCellsX();
  unsigned int my_max = costmap_->getSizeInCellsY();
  costmap_->mapToWorld(mx_max, my_max, wx_max, wy_max);

  bool found_pose = false;

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(0.0, 1.0);

  while (!found_pose) {
    double wx_rand = dis(gen);
    wx_rand = wx_min + wx_rand * (wx_max - wx_min);

    double wy_rand = dis(gen);
    wy_rand = wy_min + wy_rand * (wy_max - wy_min);

    double th_rand = dis(gen);
    th_rand = -M_PI + th_rand * (M_PI - -M_PI);

    if (isValidPose(wx_rand, wy_rand, th_rand)) {
      x = wx_rand;
      y = wy_rand;
      th = th_rand;
      found_pose = true;
    }
  }
}

unsigned int RRTPlanner::nearestNode(double random_x, double random_y) {
  unsigned int nearest_index = 0;
  double min_dist = std::numeric_limits<double>::max();
  for (unsigned int node_index : tree) {
    double node_x = static_cast<double>(node_index % width_);
    double node_y = static_cast<double>(node_index / width_);
    double dist = distance(node_x, node_y, random_x, random_y);
    if (dist < min_dist) {
      min_dist = dist;
      nearest_index = node_index;
    }
  }
  return nearest_index;
}

double RRTPlanner::distance(double x1, double y1, double x2, double y2) {
  return std::hypot(x2 - x1, y2 - y1);
}

void RRTPlanner::mapToWorld(unsigned int mx, unsigned int my, double &wx, double &wy) {
  wx = origin_x_ + mx * resolution_;
  wy = origin_y_ + my * resolution_;
}

}; // namespace rrt

