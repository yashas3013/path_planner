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

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

#include <opencv2/opencv.hpp>

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
  // grid_map::Matrix &res;

public:
  planner() : Node("path_planner") {
    occ_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map", 10, std::bind(&planner::map_callback, this, _1));
    this->map_ptr = std::shared_ptr<grid_map::GridMap>(new grid_map::GridMap);
    // this->map_ptr->setGeometry(grid_map::Length(8, 8), 0.025);
    // this->map_ptr->setPosition(grid_map::Position(4, 0));
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

public:
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    this->map_ptr->setPosition(grid_map::Position(4, 0));

    grid_map::GridMapRosConverter::fromOccupancyGrid(*msg, "map",
                                                     *this->map_ptr);
    // grid_map::replaceNan(this->map_ptr->get("map"), min_val);
    replaceNan(this->map_ptr->get("map"), 0);
    grid_map::GridMapCvConverter::toImage<unsigned char, 4>(
        *this->map_ptr, "map", CV_8UC4, min_val, max_val, image);
    cv::cvtColor(image, imgGray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(image, Blur_img, cv::Size(5, 5), 0, 0);
    // cv::addWeighted(image, 1, Blur_img, 1, 0.0, aggregate);
    cv::imshow("Blured", Blur_img);
    // cv::imshow("Gray", imgGray);
    cv::imshow("image", image);
    // cv::waitKey(0);
    grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 4>(
        Blur_img, "inflation", *this->map_ptr, min_val, max_val);
    // grid_map::GridMapCvConverter::toImage<unsigned char, 4>(
    // *this->map_ptr, "inflation", CV_8UC4, 0, 10, image);
    // cv::imshow("image", image);
    // cv::waitKey(0);
    std::cout << this->map_ptr->get("inflation") << std::endl;
    std::cout << "-----------------------------------------" << std::endl;
    std::cout << this->map_ptr->get("map") << std::endl;
    mat_add(this->map_ptr->get("map"), this->map_ptr->get("inflation"),
            this->map_ptr->get("aggregate"));
    std::cout << this->map_ptr->get("aggregate") << std::endl;
    grid_map::GridMapCvConverter::toImage<unsigned char, 4>(
        *this->map_ptr, "aggregate", CV_8UC4, min_val, max_val, image);
    cv::imshow("aggregate", image);
    cv::waitKey(0);
  }
};

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<planner>());
  rclcpp::shutdown();

  return 0;
}
