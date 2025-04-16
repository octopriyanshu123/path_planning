#include "path_planning/rrt_star.hpp"
#include <iostream>



namespace rrt_star {

RRTStar::RRTStar() :
    step_size_(0.5),
    max_iterations_(0),
    search_radius_(1.0),
    min_x_(-10.0), max_x_(10.0),
    min_y_(-10.0), max_y_(10.0),
    goal_reached_(false)
{
    // Initialize random number generator
    std::srand(std::time(nullptr));
}

RRTStar::~RRTStar() {
    // Clean up if needed
}

bool RRTStar::set_goal(const geometry_msgs::msg::Pose& goal) {
    goal_pose_ = goal;
    return true;
}

bool RRTStar::set_start(const geometry_msgs::msg::Pose& start) {
    start_pose_ = start;
    // Clear previous tree and initialize with start node
    nodes_.clear();
    nodes_.push_back(std::make_shared<Node>(start_pose_));
    goal_reached_ = false;
    return true;
}

void RRTStar::set_step_size(double step) {
    step_size_ = step;
}

void RRTStar::set_max_iterations(int iter) {
    max_iterations_ = iter;
}

void RRTStar::set_search_radius(double radius) {
    search_radius_ = radius;
}

void RRTStar::set_area_bounds(double min_x, double max_x, double min_y, double max_y) {
    min_x_ = min_x;
    max_x_ = max_x;
    min_y_ = min_y;
    max_y_ = max_y;
}

std::vector<geometry_msgs::msg::Pose> RRTStar::path_finding() {
    for (int i = 0; i < max_iterations_ && !goal_reached_; ++i) {
        // Get random point
        geometry_msgs::msg::Pose rand_point = get_random_point();
        
        // Find nearest node
        std::shared_ptr<Node> nearest_node = find_nearest_node(rand_point);
        
        // Steer towards the random point
        geometry_msgs::msg::Pose new_pose = steer(nearest_node->pose, rand_point);
        
        // Check if path is collision-free
        if (is_collision_free(nearest_node->pose, new_pose)) {
            // Create new node
            auto new_node = std::make_shared<Node>(new_pose);
            new_node->parent = nearest_node;
            new_node->cost_ = calculate_cost(new_node);
            
            // Find nearby nodes for possible rewiring
            auto near_nodes = find_near_nodes(new_node);
            
            // Choose the best parent
            for (const auto& near_node : near_nodes) {
                if (is_collision_free(near_node->pose, new_pose)) {
                    double new_cost = near_node->cost_ + get_distance(near_node->pose, new_pose);
                    if (new_cost < new_node->cost_) {
                        new_node->parent = near_node;
                        new_node->cost_ = new_cost;
                    }
                }
            }
            
            // Add to tree
            nodes_.push_back(new_node);
            
            // Rewire the tree
            rewire(new_node, near_nodes);
            
            // Check if goal is reached
            if (get_distance(new_pose, goal_pose_) <= step_size_) {
                goal_reached_ = true;
                auto goal_node = std::make_shared<Node>(goal_pose_);
                goal_node->parent = new_node;
                goal_node->cost_ = calculate_cost(goal_node);
                nodes_.push_back(goal_node);
            }
        }
    }
    
    // Reconstruct path if goal was reached
    std::vector<geometry_msgs::msg::Pose> path;
    if (goal_reached_) {
        auto node = nodes_.back(); // Goal node
        while (node != nullptr) {
            path.push_back(node->pose);
            node = node->parent;
        }
        std::reverse(path.begin(), path.end());
    }
    
    return path;
}


void RRTStar::add_one_more_Node(){

        // Get random point
        geometry_msgs::msg::Pose rand_point = get_random_point();
        
        // Find nearest node
        std::shared_ptr<Node> nearest_node = find_nearest_node(rand_point);
        
        // Steer towards the random point
        geometry_msgs::msg::Pose new_pose = steer(nearest_node->pose, rand_point);
        
        // Check if path is collision-free
        if (is_collision_free(nearest_node->pose, new_pose)) {
            // Create new node
            auto new_node = std::make_shared<Node>(new_pose);
            new_node->parent = nearest_node;
            new_node->cost_ = calculate_cost(new_node);
            
            // Find nearby nodes for possible rewiring
            auto near_nodes = find_near_nodes(new_node);
            
            // Choose the best parent
            for (const auto& near_node : near_nodes) {
                if (is_collision_free(near_node->pose, new_pose)) {
                    double new_cost = near_node->cost_ + get_distance(near_node->pose, new_pose);
                    if (new_cost < new_node->cost_) {
                        new_node->parent = near_node;
                        new_node->cost_ = new_cost;
                    }
                }
            }
            
            // Add to tree
            nodes_.push_back(new_node);
            
            // Rewire the tree
            rewire(new_node, near_nodes);
            
            // Check if goal is reached
            if (get_distance(new_pose, goal_pose_) <= step_size_) {
                goal_reached_ = true;
                auto goal_node = std::make_shared<Node>(goal_pose_);
                goal_node->parent = new_node;
                goal_node->cost_ = calculate_cost(goal_node);
                nodes_.push_back(goal_node);
            }
        }
    
}

std::vector<geometry_msgs::msg::Pose> RRTStar::get_path(){
    std::vector<geometry_msgs::msg::Pose> path;
    if (goal_reached_) {
        auto node = nodes_.back(); // Goal node
        while (node != nullptr) {
            path.push_back(node->pose);
            node = node->parent;
        }
        std::reverse(path.begin(), path.end());
    }
    
    return path;
}

geometry_msgs::msg::Pose RRTStar::get_random_point() {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<> x_dist(min_x_, max_x_);
    std::uniform_real_distribution<> y_dist(min_y_, max_y_);
    
    geometry_msgs::msg::Pose point;
    point.position.x = x_dist(gen);
    point.position.y = y_dist(gen);
    return point;
}

std::shared_ptr<Node> RRTStar::find_nearest_node(const geometry_msgs::msg::Pose& point) {
    std::shared_ptr<Node> nearest = nullptr;
    double min_dist = std::numeric_limits<double>::max();
    
    for (const auto& node : nodes_) {
        double dist = get_distance(node->pose, point);
        if (dist < min_dist) {
            min_dist = dist;
            nearest = node;
        }
    }
    
    return nearest;
}

geometry_msgs::msg::Pose RRTStar::steer(const geometry_msgs::msg::Pose& from, const geometry_msgs::msg::Pose& to) {
    double dist = get_distance(from, to);
    
    geometry_msgs::msg::Pose new_pose;
    if (dist <= step_size_) {
        new_pose = to;
    } else {
        double ratio = step_size_ / dist;
        new_pose.position.x = from.position.x + (to.position.x - from.position.x) * ratio;
        new_pose.position.y = from.position.y + (to.position.y - from.position.y) * ratio;
    }
    
    return new_pose;
}

bool RRTStar::is_collision_free(const geometry_msgs::msg::Pose& from, const geometry_msgs::msg::Pose& to) {
    // This is a placeholder - implement actual collision checking for your environment
    // For now, just check if the points are within bounds
    return (to.position.x >= min_x_ && to.position.x <= max_x_ &&
            to.position.y >= min_y_ && to.position.y <= max_y_);
}

std::vector<std::shared_ptr<Node>> RRTStar::find_near_nodes(const std::shared_ptr<Node>& new_node) {
    std::vector<std::shared_ptr<Node>> near_nodes;
    double radius = search_radius_ * std::sqrt(std::log(nodes_.size()) / nodes_.size());
    
    for (const auto& node : nodes_) {
        if (get_distance(node->pose, new_node->pose) <= radius) {
            near_nodes.push_back(node);
        }
    }
    
    return near_nodes;
}

void RRTStar::rewire(std::shared_ptr<Node>& new_node, const std::vector<std::shared_ptr<Node>>& near_nodes) {
    for (const auto& near_node : near_nodes) {
        if (near_node != new_node->parent && is_collision_free(new_node->pose, near_node->pose)) {
            double new_cost = new_node->cost_ + get_distance(new_node->pose, near_node->pose);
            if (new_cost < near_node->cost_) {
                near_node->parent = new_node;
                near_node->cost_ = new_cost;
            }
        }
    }
}

double RRTStar::calculate_cost(const std::shared_ptr<Node>& node) {
    if (node->parent == nullptr) {
        return 0.0;
    }
    return node->parent->cost_ + get_distance(node->parent->pose, node->pose);
}

double RRTStar::get_distance(const geometry_msgs::msg::Pose& a, const geometry_msgs::msg::Pose& b) {
    double dx = a.position.x - b.position.x;
    double dy = a.position.y - b.position.y;
    return std::sqrt(dx*dx + dy*dy);
}

void RRTStar::set_status(Node node ,bool status){
    node.status_ = status;
}


// void RRTStar::set_obstacle(std::vector<std::vector<int>> obstacles){
//     if(obstacles.size() >0 ){
//         for (auto obstacle : obstacles) {    
//                 double x = obstacle[0], y = obstacle [1], z = 0,  length = 1,  width = 1, height =1 ;
//                 Obstacle new_obstacle(x, y, z,  length,  width, height);
//                 obstacle_.push_back(std::make_shared<Obstacle>(new_obstacle));
           
//         }
//     }
   
// }


void RRTStar::add_obstacle(const Obstacle& obstacle) {
    obstacles_.push_back(std::make_shared<Obstacle>(obstacle));
}

void RRTStar::clear_obstacles() {
    obstacles_.clear();
}


bool RRTStar::check_collision(double x1, double y1, double x2, double y2) const {
    // Simple line-segment to rectangle collision check
    // This is a simplified version - you might want a more robust collision checker
    
    // Check each obstacle
    for (const auto& obstacle : obstacles_) {
        // Check if either point is inside the obstacle
        if (obstacle->collides(x1, y1) || obstacle->collides(x2, y2)) {
            return true;
        }
        
        // TODO: Add proper line-rectangle intersection check
        // for the path between (x1,y1) and (x2,y2)
    }
    
    return false;
}



} // namespace rrt_star





