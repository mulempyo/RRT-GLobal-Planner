#ifndef RRT_PLANNER_
#define RRT_PLANNER_

#include <nav_core/base_global_planner.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>

namespace my_global_planner {

class RRTPlanner : public nav_core::BaseGlobalPlanner {

struct Node {
        double x, y;
        Node* parent;
        Node(double _x, double _y, Node* _parent = nullptr) : x(_x), y(_y), parent(_parent) {}
    };

public:
    RRTPlanner();
    RRTPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;
    bool makePlan(const geometry_msgs::PoseStamped& start, 
                  const geometry_msgs::PoseStamped& goal, 
                  std::vector<geometry_msgs::PoseStamped>& plan) override;

private:
    std::vector<Node*> tree;
    bool isCollision(double x, double y);
    Node* getRandomNode();
    Node* nearestNode(Node* random_node);
    std::vector<Node*> getNeighbors(Node* node);
    double distance(double x1, double y1, double x2, double y2);
    void mapToWorld(double mx, double my, double& wx, double& wy);
    void worldToMap(double wx, double wy, double& mx, double& my);

    costmap_2d::Costmap2D* costmap_;
    double origin_x_, origin_y_, resolution_;
    unsigned int width_, height_;
    bool initialized_;
    double goal_threshold_;
    double step_size_;
    unsigned int max_iterations_;
 };
};
#endif
