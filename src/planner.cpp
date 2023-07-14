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
  const int max_val = 1.0;
  cv::Mat image;
  cv::Mat imgGray, Blur_img;

public:
  planner() : Node("path_planner") {
    occ_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map", 10, std::bind(&planner::map_callback, this, _1));
    this->map_ptr = std::shared_ptr<grid_map::GridMap>(new grid_map::GridMap);
    // this->map_ptr->setGeometry(grid_map::Length(8, 8), 0.025);
    // this->map_ptr->setPosition(grid_map::Position(4, 0));
    this->map_ptr->add("map", 0.0);
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
    cv::GaussianBlur(image, Blur_img, cv::Size(5, 5), 10, 0);
    cv::imshow("Blured", Blur_img);
    cv::imshow("Gray", imgGray);
    cv::imshow("image", image);
    cv::waitKey(0);
    grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 4>(
        Blur_img, "inflation", *this->map_ptr, minValue, maxValue);
    // std::cout << this->map_ptr->get("map") << std::endl;
  }
};

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<planner>());
  rclcpp::shutdown();

  return 0;
}
