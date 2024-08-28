#include <rrt_planner_ros/rrt.h>
#include <pluginlib/class_list_macros.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <cmath>
#include <random>
#include <limits>

PLUGINLIB_EXPORT_CLASS(my_global_planner::RRTPlanner, nav_core::BaseGlobalPlanner)

namespace my_global_planner{

RRTPlanner::RRTPlanner() : costmap_(nullptr), initialized_(false), goal_threshold_(0.5), step_size_(0.5), max_iterations_(1000) {}

RRTPlanner::RRTPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    : costmap_(nullptr), initialized_(false), goal_threshold_(0.5), step_size_(0.5), max_iterations_(1000) {
    initialize(name, costmap_ros);
}

void RRTPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {

    ros::NodeHandle n;

    if (!initialized_) {
        costmap_ = costmap_ros->getCostmap();
        origin_x_ = costmap_->getOriginX();
        origin_y_ = costmap_->getOriginY();
        resolution_ = costmap_->getResolution();
        width_ = costmap_->getSizeInCellsX();
        height_ = costmap_->getSizeInCellsY();
        initialized_ = true;
    } else {
        ROS_WARN("RRTPlanner has already been initialized.");
    }
}

bool RRTPlanner::makePlan(const geometry_msgs::PoseStamped& start, 
                          const geometry_msgs::PoseStamped& goal, 
                          std::vector<geometry_msgs::PoseStamped>& plan) {

    ros::NodeHandle n;

    if (!initialized_) {
        ROS_ERROR("RRTPlanner has not been initialized, please call initialize() before use.");
        return false;
    }

    plan.clear();

    double start_x, start_y;
    worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y);
    Node* start_node = new Node(start_x, start_y);

    double goal_x, goal_y;
    worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y);
    Node* goal_node = new Node(goal_x, goal_y);

    tree.push_back(start_node);

    Node* final_node = nullptr;

    for (unsigned int i = 0; i < max_iterations_; ++i) {
        Node* random_node = getRandomNode();
        Node* nearest_node = nearestNode(random_node);

        double theta = atan2(random_node->y - nearest_node->y, random_node->x - nearest_node->x);
        Node* new_node = new Node(nearest_node->x + step_size_ * cos(theta), nearest_node->y + step_size_ * sin(theta), nearest_node);

        if (!isCollision(new_node->x, new_node->y)) {
            tree.push_back(new_node);

            if (distance(new_node->x, new_node->y, goal_x, goal_y) < goal_threshold_) {
                final_node = new Node(goal_x, goal_y, new_node);
                tree.push_back(final_node);
                break;
            }
        }
        delete random_node;
    }

    if (final_node) {
        Node* current = final_node;
        while (current != nullptr) {
            double wx, wy;
            mapToWorld(current->x, current->y, wx, wy);
            geometry_msgs::PoseStamped pose = goal;
            pose.pose.position.x = wx;
            pose.pose.position.y = wy;
            pose.pose.position.z = 0;
            pose.pose.orientation.w = 1.0;
            plan.push_back(pose);
            current = current->parent;
        }
        std::reverse(plan.begin(), plan.end());
        return true;
    } else {
        ROS_WARN("Failed to find a valid plan.");
        return false;
    }
}

bool RRTPlanner::isCollision(double x, double y) {
    unsigned int mx, my;
    mx = static_cast<unsigned int>(x);
    my = static_cast<unsigned int>(y);
    return (costmap_->getCost(mx, my) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
}

RRTPlanner::Node* RRTPlanner::getRandomNode() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis_x(0, width_ - 1);
    std::uniform_real_distribution<> dis_y(0, height_ - 1);
    return new Node(dis_x(gen), dis_y(gen));
}

RRTPlanner::Node* RRTPlanner::nearestNode(Node* random_node) {
    Node* nearest = nullptr;
    double min_dist = std::numeric_limits<double>::max();
    for (Node* node : tree) {
        double dist = distance(node->x, node->y, random_node->x, random_node->y);
        if (dist < min_dist) {
            min_dist = dist;
            nearest = node;
        }
    }
    return nearest;
}

double RRTPlanner::distance(double x1, double y1, double x2, double y2) {
    return std::hypot(x2 - x1, y2 - y1);
}

void RRTPlanner::mapToWorld(double mx, double my, double& wx, double& wy) {
    wx = origin_x_ + mx * resolution_;
    wy = origin_y_ + my * resolution_;
}

void RRTPlanner::worldToMap(double wx, double wy, double& mx, double& my) {
    mx = (wx - origin_x_) / resolution_;
    my = (wy - origin_y_) / resolution_;
}

}; // namespace my_global_planner

