/**
 * @file rrt_star.cpp
 * @author vss2sn
 * @brief Contains the RRTStar class
 */

/*
NOTE:
Midway through refactor
Trying to make rewire more efficient as well as allow rewire to be iteratively
called on all nodes. SO if a node's value cahnges call reqire on all nodes
within the  threshold amd see whether they need to be rewired
Grids ill now need to maintain a list of neighbours that have been seen to
easily iterate over all neighbours to update them
*/

#include "path_planning/rrt_star.hpp"

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

// constants
constexpr double half_grid_unit = 0.5;
constexpr double precision_limit = 0.000001;

std::tuple<bool, Grid> RRTStar::FindNearestPoint(Grid& new_node) {
  bool found = false;
  Grid nearest_node;
  near_nodes_.insert({new_node, {}});
  new_node.cost_ = std::numeric_limits<double>::max();
  for (const auto node : point_list_) {
    if (auto new_dist = std::sqrt(std::pow(node.x_ - new_node.x_, 2) +
                                  std::pow(node.y_ - new_node.y_, 2));
        new_dist <= threshold_ && !IsAnyObstacleInPath(node, new_node) &&
        !CompareCoordinates(node, new_node) && node.pid_ != new_node.id_) {
      near_nodes_[new_node].push_back(node);
      near_nodes_[node].push_back(new_node);
      found = true;
      if (new_dist + node.cost_ < new_node.cost_) {
        nearest_node = node;
        new_node.pid_ = nearest_node.id_;
        new_node.cost_ = new_dist + node.cost_;
      }
    }
  }
  if (!found) {
    near_nodes_.erase(new_node);
  }
  return {found, nearest_node};
}

bool RRTStar::IsAnyObstacleInPath(const Grid& n_1, const Grid& n_2) const {
  if (n_2.y_ - n_1.y_ == 0) {
    const double c = n_2.y_;
    return std::any_of(
        std::begin(obstacle_list_), std::end(obstacle_list_),
        [&c, &n_1, &n_2](const auto& obs_node) {
          return obs_node.y_ == c &&
                 ((n_1.x_ >= obs_node.x_ && obs_node.x_ >= n_2.x_) ||
                  (n_1.x_ <= obs_node.x_ && obs_node.x_ <= n_2.x_));
        });
  }
  const double slope = static_cast<double>(n_2.x_ - n_1.x_) / (n_2.y_ - n_1.y_);
  const double c = n_2.x_ - slope * n_2.y_;

  // TODO: Cache this
  for (const auto& obs_node : obstacle_list_) {
    if (!(((n_1.y_ >= obs_node.y_) && (obs_node.y_ >= n_2.y_)) ||
          ((n_1.y_ <= obs_node.y_) && (obs_node.y_ <= n_2.y_)))) {
      continue;
    }
    if (!(((n_1.x_ >= obs_node.x_) && (obs_node.x_ >= n_2.x_)) ||
          ((n_1.x_ <= obs_node.x_) && (obs_node.x_ <= n_2.x_)))) {
      continue;
    }
    std::vector<double> arr(4);
    // Using properties of a point and a line here.
    // If the obtacle lies on one side of a line, substituting its edge points
    // (all obstacles are grid sqaures in this example) into the equation of
    // the line passing through the coordinated of the two nodes under
    // consideration will lead to all four resulting values having the same
    // sign. Hence if their sum of the value/abs(value) is 4 the obstacle is
    // not in the way. If a single point is touched ie the substitution leads
    // ot a value under 10^-7, it is set to 0. Hence the obstacle has
    // 1 point on side 1, 3 points on side 2, the sum is 2 (-1+3)
    // 2 point on side 1, 2 points on side 2, the sum is 0 (-2+2)
    // 0 point on side 1, 3 points on side 2, (1 point on the line, ie,
    // grazes the obstacle) the sum is 3 (0+3)
    // Hence the condition < 3
    arr[0] = obs_node.x_ + half_grid_unit -
             slope * (obs_node.y_ + half_grid_unit) - c;
    arr[1] = obs_node.x_ + half_grid_unit -
             slope * (obs_node.y_ - half_grid_unit) - c;
    arr[2] = obs_node.x_ - half_grid_unit -
             slope * (obs_node.y_ + half_grid_unit) - c;
    arr[3] = obs_node.x_ - half_grid_unit -
             slope * (obs_node.y_ - half_grid_unit) - c;
    double count = 0;
    for (auto& a : arr) {
      if (std::fabs(a) <= precision_limit) {
        a = 0;
      } else {
        count += a / std::fabs(a);
      }
    }
    if (std::abs(count) < 3) {
      return true;
    }
  }
  return false;
}

Grid RRTStar::GenerateRandomGrid() const {
  std::random_device rd;   // obtain a random number from hardware
  std::mt19937 eng(rd());  // seed the generator
  std::uniform_int_distribution<int> distr(0, n_ - 1);  // define the range
  const int x = distr(eng);
  const int y = distr(eng);
  return Grid(x, y, 0, 0, n_ * x + y, 0);
}

void RRTStar::Rewire(const Grid& new_node) {
  Grid new_node_in_point_list = *point_list_.find(new_node);
  for (const auto& neighbour_coords_node :
       near_nodes_[new_node_in_point_list]) {
    Grid neighbour = *point_list_.find(neighbour_coords_node);
    if (const auto dist =
            std::sqrt(std::pow(neighbour.x_ - new_node_in_point_list.x_, 2) +
                      std::pow(neighbour.y_ - new_node_in_point_list.y_, 2));
        neighbour.cost_ > dist + new_node_in_point_list.cost_ &&
        neighbour.id_ != new_node_in_point_list.id_) {
      neighbour.pid_ = new_node_in_point_list.id_;
      neighbour.cost_ = dist + new_node_in_point_list.cost_;
      point_list_.erase(neighbour);
      point_list_.insert(neighbour);
      Rewire(neighbour);
    }
  }
}

std::tuple<bool, std::vector<Grid>> RRTStar::Plan(const Grid& start,
                                                  const Grid& goal) {
  start_ = start;
  goal_ = goal;
  grid_ = original_grid_;
  int max_iterations = max_iter_x_factor_ * n_ * n_;
  CreateObstacleList();
  point_list_.insert(start_);
  grid_[start_.x_][start_.y_] = 3;
  int iteration = 0;
  Grid new_node = start_;
  bool found_goal = false;
  if (CheckGoalVisible(new_node)) {
    found_goal = true;
  }
  while (iteration < max_iterations) {
    iteration++;
    new_node = GenerateRandomGrid();

    // Go back to beginning of loop if point already considered
    if (grid_[new_node.x_][new_node.y_] != 0) {
      continue;
    }

    // Go back to beginning of loop if no near neighbour
    const auto [found_nearest_point, nearest_node] = FindNearestPoint(new_node);
    if (!found_nearest_point) {
      continue;
    }

    // Setting to 2 implies visited/considered
    grid_[new_node.x_][new_node.y_] = 2;
    point_list_.insert(new_node);
    Rewire(new_node);  // Rewire
    if (CheckGoalVisible(new_node)) {
      found_goal = true;
    }
  }
  if (!found_goal) {
    return {false, {}};
  }
  return {true, CreatePath()};
}

bool RRTStar::CheckGoalVisible(const Grid& new_node) {
  const auto dist = std::sqrt(std::pow(goal_.x_ - new_node.x_, 2) +
                              std::pow(goal_.y_ - new_node.y_, 2));
  if (dist > threshold_) {
    return false;
  }
  if (CompareCoordinates(goal_, new_node)) {
    point_list_.erase(new_node);
    point_list_.insert(new_node);
    return true;
  }
  if (!IsAnyObstacleInPath(new_node, goal_)) {
    if (auto it = point_list_.find(goal_);
        it != point_list_.end() && it->cost_ > dist + new_node.cost_) {
      auto goal_in_point_list = goal_;
      goal_in_point_list.cost_ = dist + new_node.cost_;
      goal_in_point_list.pid_ = new_node.id_;
      point_list_.erase(goal_in_point_list);
      point_list_.insert(goal_in_point_list);
    } else if (it == point_list_.end()) {
      auto goal_in_point_list = goal_;
      goal_in_point_list.cost_ = dist + new_node.cost_;
      goal_in_point_list.pid_ = new_node.id_;
      point_list_.insert(goal_in_point_list);
    }
    return true;
  }
  return false;
}

void RRTStar::CreateObstacleList() {
  for (int i = 0; i < n_; i++) {
    for (int j = 0; j < n_; j++) {
      if (grid_[i][j] == 1) {
        Grid obs(i, j, 0, 0, i * n_ + j, 0);
        obstacle_list_.push_back(obs);
      }
    }
  }
}

std::vector<Grid> RRTStar::CreatePath() {
  std::vector<Grid> path;
  Grid current = *point_list_.find(goal_);
  while (!CompareCoordinates(current, start_)) {
    path.push_back(current);
    current = *point_list_.find(
        Grid(current.pid_ / n_, current.pid_ % n_, 0, 0, current.pid_));
  }
  path.push_back(current);
  return path;
}

void RRTStar::SetParams(const int threshold, const int max_iter_x_factor) {
  threshold_ = threshold;
  max_iter_x_factor_ = max_iter_x_factor;
}

#ifdef BUILD_INDIVIDUAL
int main() {
  constexpr int n = 11;
  std::vector<std::vector<int>> grid(n, std::vector<int>(n, 0));
  MakeGrid(grid);

  std::random_device rd;   // obtain a random number from hardware
  std::mt19937 eng(rd());  // seed the generator
  std::uniform_int_distribution<int> distr(0, n - 1);  // define the range

  Grid start(distr(eng), distr(eng), 0, 0, 0, 0);
  Grid goal(distr(eng), distr(eng), 0, 0, 0, 0);

  start.id_ = start.x_ * n + start.y_;
  start.pid_ = start.x_ * n + start.y_;
  goal.id_ = goal.x_ * n + goal.y_;
  start.h_cost_ = abs(start.x_ - goal.x_) + abs(start.y_ - goal.y_);

  // Make sure start and goal are not obstacles and their ids are correctly
  // assigned.
  grid[start.x_][start.y_] = 0;
  grid[goal.x_][goal.y_] = 0;

  start.PrintStatus();
  goal.PrintStatus();

  PrintGrid(grid);

  constexpr double threshold = 2;
  constexpr int max_iter_x_factor = 20;

  RRTStar new_rrt_star(grid);
  new_rrt_star.SetParams(threshold, max_iter_x_factor);
  const auto [found_path, path_vector] = new_rrt_star.Plan(start, goal);
  PrintPath(path_vector, start, goal, grid);
  return 0;
}
#endif  // BUILD_INDIVIDUAL
