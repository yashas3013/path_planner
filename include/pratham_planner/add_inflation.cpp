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

grid_map::Matrix mat_add(grid_map::Matrix &m1, grid_map::Matrix &m2) {
  grid_map::Matrix res;
  int val;
  for (int r = 0; r < m1.rows(); r++) {
    for (int c = 0; c < m1.cols(); c++) {
      val = m1(r, c) + m2(r, c);
      if (val > 100) {
        val = 100;
      }
      res(r, c) = val;
    }
  }
  return res;
}
grid_map::Matrix replaceNan(grid_map::Matrix &m, const double newValue) {
  for (int r = 0; r < m.rows(); r++) {
    for (int c = 0; c < m.cols(); c++) {
      if (std::isnan(m(r, c))) {
        m(r, c) = newValue;
      }
    }
  }
  return m;
}

grid_map::Matrix add_inf(grid_map::Matrix occ_grid) {
  cv::Mat image;
  cv::Mat Blur_img;
  cv::Mat imgGray;
  occ_grid = replaceNan(occ_grid, 0);
  grid_map::GridMapCvConverter::toImage<unsigned char, 4>(
      occ_grid, "map", CV_8UC4, min_val, max_val, image);
  cv::cvtColor(image, imgGray, cv::COLOR_BGR2GRAY);
  cv::GaussianBlur(image, Blur_img, cv::Size(5, 5), 0, 0);
}
