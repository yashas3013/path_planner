#include <chrono>
#include <cstdio>
#include <functional>
#include <iostream>
#include <math.h>
#include <memory>
#include <string>

#include "grid_map_core/grid_map_core.hpp"
#include "grid_map_cv/grid_map_cv.hpp"
#include "grid_map_ros/grid_map_ros.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include <opencv2/opencv.hpp>

#include "path_planning/a_star.hpp"

#include "../include/path_planning/lib/utils/src/utils.cpp"
#include "../include/path_planning/src/a_star.cpp"
#include "utils/utils.hpp"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

using namespace ::std::chrono_literals;
using std::placeholders::_1;
class planner : public rclcpp::Node {
private:
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_sub;

  std::shared_ptr<grid_map::GridMap> map_ptr;
  const int min_val = -1.0;
  const int max_val = 100.0;
  cv::Mat image;
  cv::Mat imgGray, Blur_img, aggregate;
  int val;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      goal_subscription;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr path_found_publisher;
  rclcpp::TimerBase::SharedPtr path_publish_timer;
  // grid_map::Matrix &res;
  std::vector<Grid> path;
  std::vector<std::vector<int>> grid;

  unsigned int counter_path_pub = 0;
  // the start and goal position for the thingi
  Grid start; //(0, 0, 0, 0, 0, 0);
  Grid goal;  //(0, 0, 0, 0, 0, 0);

  geometry_msgs::msg::TransformStamped odom_to_base;
  geometry_msgs::msg::TransformStamped odom_to_map;
  geometry_msgs::msg::TransformStamped map_to_base;
  geometry_msgs::msg::TransformStamped odom_to_path;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  // std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::TimerBase::SharedPtr timer_tf_publisher{nullptr};

  float goal_x = 4;
  float goal_y = 0;

public:
  planner() : Node("path_planner") {
    occ_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map", 10, std::bind(&planner::map_callback, this, _1));
    this->map_ptr = std::shared_ptr<grid_map::GridMap>(new grid_map::GridMap);
    goal_subscription =
        this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal_pose", 10, std::bind(&planner::goal_callback, this, _1));
    path_publisher =
        this->create_publisher<nav_msgs::msg::Path>("path_local", 10);
    // path_publish_timer=this->create_wall_timer(50ms,
    // std::bind(&PathPlanner::path_publisher_callback, this));
    path_found_publisher =
        this->create_publisher<std_msgs::msg::Bool>("path_found", 10);

    std::cout << "initializing tf stuff\n";
    // tf_listener

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    /* tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
     */
    // timer_tf_publisher =
    //     this->create_wall_timer(2ms, std::bind(&planner::tf_broadcaster,
    //     this));

    this->map_ptr->add("map", 0.0);
    this->map_ptr->add("aggregate", 0.0);
  }

public:
  void mat_add(grid_map::Matrix &m1, grid_map::Matrix &m2,
               grid_map::Matrix &m3) {
    for (int r = 0; r < m1.rows(); r++) {
      for (int c = 0; c < m1.cols(); c++) {
        val = m1(r, c) + m2(r, c);
        if (val > 100) {
          val = 100;
        }
        m3(r, c) = val;
      }
    }
  }

public:
  void replaceNan(grid_map::Matrix &m, const double newValue) {
    for (int r = 0; r < m.rows(); r++) {
      for (int c = 0; c < m.cols(); c++) {
        if (std::isnan(m(r, c))) {
          m(r, c) = newValue;
        }
      }
    }
  }
  void get_odom_to_base_tf() {
    // lookup the latest transform between odom and base_link
    this->odom_to_base = this->tf_buffer_->lookupTransform("odom", "base_link",
                                                           tf2::TimePointZero);
  }

  void get_odom_to_map_tf() {
    this->odom_to_map =
        this->tf_buffer_->lookupTransform("map", "odom", tf2::TimePointZero);
  }

  void get_map_to_base_tf() {
    this->map_to_base = this->tf_buffer_->lookupTransform("map", "base_link",
                                                          tf2::TimePointZero);
  }

  // void tf_broadcaster() {
  //   this->odom_to_path.header.stamp = this->get_clock()->now();
  //   this->tf_broadcaster_->sendTransform(this->odom_to_path);
  // }
  void calculate_start_and_goal() {
    try {
      this->get_map_to_base_tf();
      // this->get_odom_to_map_tf();
      // this->get_odom_to_base_tf();
    } catch (const tf2::TransformException &ex) {
      std::cout << "unable to look up tf.";
    }

    grid_map::Position pose(this->map_to_base.transform.translation.x,
                            this->map_to_base.transform.translation.y);
    grid_map::Index ind;
    this->map_ptr->getIndex(pose, ind);

    // TODO check if it is ind(0) ind(1) or the opposite
    // DONE, this is correct
    std::cout << "setting start to " << ind(0) << " " << ind(1) << std::endl;
    // Change the value to current size of map
    if (ind(0) < 0) {
      ind(0) = 0;
    } else if (ind(0) >= 320) {
      ind(0) = 319;
    }

    if (ind(1) < 0) {
      ind(1) = 0;
    } else if (ind(1) >= 320) {
      ind(1) = 319;
    }
    this->start = Grid(ind(0), ind(1), 0, 0, 0, 0);
    // std::cout << "setting the goal to " << n - 1 - goal_y << " " << goal_x <<
    // std::endl; Grid goal(0, 319, 0, 0, 0, 0);

    // we have OG stored in (this->goal_x, this->goal_y)
    // we have OM stored in this->odom_to_map
    // MG = OG - OM
    // we need to rotate it by -yaw to supply the goal to the path planner
    tf2::Quaternion q;
    double roll, pitch, yaw;
    q.setX(this->odom_to_map.transform.rotation.x);
    q.setY(this->odom_to_map.transform.rotation.y);
    q.setZ(this->odom_to_map.transform.rotation.z);
    q.setW(this->odom_to_map.transform.rotation.w);
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    // this returns yaw in radians

    // OG - OM = MG
    float mg_x = this->goal_x - this->odom_to_map.transform.translation.x;
    float mg_y = this->goal_y - this->odom_to_map.transform.translation.y;

    // (mogx, mogy) is MG but with M as a non rotated frame, we can convert this
    // to indices to provide as a goal
    float mogx = (mg_x * std::cos(-yaw)) + (mg_y * (-std::sin(-yaw)));
    float mogy = (mg_x * std::sin(-yaw)) + (mg_y * std::cos(-yaw));

    grid_map::Position mog_pose(mogx, mogy);
    grid_map::Index goal_ind;
    this->map_ptr->getIndex(mog_pose, goal_ind);
    // TODO Change to map to size
    if (goal_ind(0) < 0) {
      goal_ind(0) = 0;
    } else if (goal_ind(0) >= 320) {
      goal_ind(0) = 319;
    }
    if (goal_ind(1) < 0) {
      goal_ind(1) = 0;
    } else if (goal_ind(1) >= 320) {
      goal_ind(1) = 319;
    }

    this->goal = Grid(goal_ind(0), goal_ind(1), 0, 0, 0, 0);

    std::cout << "setting goal to " << goal_ind(0) << " " << goal_ind(1)
              << "\n";
    start.id_ = start.x_ + start.y_;
    start.pid_ = start.x_ + start.y_;
    goal.id_ = goal.x_ + goal.y_;
    start.h_cost_ =
        sqrt(powf(start.x_ - goal.x_, 2) + powf(start.y_ - goal.y_, 2));
  }
  void plan_path() {
    // this->get_odom_to_map_tf();
    // this->odom_to_path = this->odom_to_map;
    // this->odom_to_path.header.frame_id = "map";
    // this->odom_to_path.child_frame_id = "path_frame";
    AStar a_star(this->grid);
    {
      std::cout << "trying path planning\n";
      /* const auto [path_found, path_vector] = a_star.Plan(start, goal); */
      a_star.Plan(this->start, this->goal);
      // std::cout << "path planning done? " << path_found << std::endl;
      // std::cout << "size of the path found " << path_vector.size() <<
      // std::endl;
      // this->path = path_vector;
      // std::cout << "received path of length " << path_vector.size()
      // std_msgs::msg::Bool path_found_msg;
      // path_found_msg.data = path_found;
      // this->path_found_publisher->publish(std_msgs::msg::Bool(path_found_msg));

      // <<std::endl; std::cout << "resized the path now printing\n";
      // std::cout << "path printed now it needs to be published\n";
    }
  }

public:
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    // this->map_ptr->setPosition(grid_map::Position(4, 0));

    grid_map::GridMapRosConverter::fromOccupancyGrid(*msg, "map",
                                                     *this->map_ptr);
    // grid_map::replaceNan(this->map_ptr->get("map"), min_val);
    replaceNan(this->map_ptr->get("map"), 0);
    grid_map::GridMapCvConverter::toImage<unsigned char, 4>(
        *this->map_ptr, "map", CV_8UC4, min_val, max_val, image);
    cv::cvtColor(image, imgGray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(image, Blur_img, cv::Size(5, 5), 0, 0);
    // cv::addWeighted(image, 1, Blur_img, 1, 0.0, aggregate);
    // cv::imshow("Blured", Blur_img);
    // cv::imshow("Gray", imgGray);
    // cv::imshow("image", image);
    // cv::waitKey(0);
    grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 4>(
        Blur_img, "inflation", *this->map_ptr, min_val, max_val);
    // grid_map::GridMapCvConverter::toImage<unsigned char, 4>(
    // *this->map_ptr, "inflation", CV_8UC4, 0, 10, image);
    // cv::imshow("image", image);
    // cv::waitKey(0);
    // std::cout << this->map_ptr->get("inflation") << std::endl;
    std::cout << "-----------------------------------------" << std::endl;
    // std::cout << this->map_ptr->get("map") << std::endl;
    mat_add(this->map_ptr->get("map"), this->map_ptr->get("inflation"),
            this->map_ptr->get("aggregate"));
    // std::cout << this->map_ptr->get("aggregate") << std::endl;
    Eigen::MatrixXf processed_grid_map =
        this->map_ptr->get("aggregate").cast<float>();

    int rows = processed_grid_map.rows();
    int cols = processed_grid_map.cols();
    std::cout << rows << " " << cols << std::endl;

    std::cout << "lol" << typeid(processed_grid_map(13, 0)).name();
    std::cout << processed_grid_map.cols() << std::endl;
    Eigen::MatrixXi int_grid_map = processed_grid_map.cast<int>().transpose();
    std::cout << int_grid_map.cols() << std::endl;
    this->grid.clear();
    for (int i = 0; i < cols; i++) {
      const int *begin = &int_grid_map.col(i).data()[0];
      this->grid.push_back(std::vector<int>(begin, begin + rows));
    }

    this->counter_path_pub++;
    if (this->counter_path_pub == 1) {
      this->counter_path_pub = 0;
      this->calculate_start_and_goal();
      this->plan_path();
      // std::cout << "received path of length " << path_vector.size()

      // <<std::endl; std::cout << "resized the path now printing\n";
      // std::cout << "path printed now it needs to be published\n";
      this->path_publisher_callback();
    }
    // grid_map::GridMapCvConverter::toImage<unsigned char, 4>(
    // *this->map_ptr, "aggregate", CV_8UC4, min_val, max_val, image);
    // cv::imshow("aggregate", image);
    // cv::waitKey(0);
  }

public:
  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr goal) {
    this->goal_x = goal->pose.position.x;
    this->goal_y = goal->pose.position.y;

    tf2::Quaternion q;
    double roll, pitch, yaw;
    q.setX(this->odom_to_base.transform.rotation.x);
    q.setY(this->odom_to_base.transform.rotation.y);
    q.setZ(this->odom_to_base.transform.rotation.z);
    q.setW(this->odom_to_base.transform.rotation.w);
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    // this returns yaw in radians

    // now to get the vector BG (B = base_link, G = goal)
    // std trig functions take in radians as inputs
    this->goal_x = (goal->pose.position.x * std::cos(yaw)) +
                   (goal->pose.position.y * (-std::sin(yaw)));
    this->goal_y = (goal->pose.position.x * std::sin(yaw)) +
                   (goal->pose.position.y * std::cos(yaw));

    // We have OB (O = odom, B = base_link), from this->odom_to_base
    // we just have to add OB to BG to get OG, which we can store
    this->goal_x += this->odom_to_base.transform.translation.x;
    this->goal_y += this->odom_to_base.transform.translation.y;
    // (goal_x, goal_y) forms vector OG
    std::cout << goal_x << goal_y;
    this->calculate_start_and_goal();
    this->plan_path();
    this->path_publisher_callback();
  }
  void path_publisher_callback() {
    // std::cout << "publishing path : \n";
    auto path_local = nav_msgs::msg::Path();
    path_local.header.frame_id = "path_frame";

    // for(int i=path.size()-1; i>=0; i--){
    for (int i = path.size() - 1; i >= 0; i--) {
      geometry_msgs::msg::PoseStamped pose_stamped_msg;
      grid_map::Position position;
      grid_map::Index index(path[i].x_, path[i].y_);
      this->map_ptr->getPosition(index, position);
      pose_stamped_msg.pose.position.x = position.x();
      pose_stamped_msg.pose.position.y = position.y();
      path_local.poses.emplace_back(pose_stamped_msg);
      std::cout << "the path in the map is : " << path[i].x_ << " "
                << path[i].y_ << std::endl;
      std::cout << "the path is : " << position.x() << " " << position.y()
                << std::endl;
    }
    // std::cout << path_local << std::endl;
    path_publisher->publish(path_local);
  }
};

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<planner>());
  rclcpp::shutdown();

  return 0;
}
