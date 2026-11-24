#ifndef CALIBRATION__PATTERN_HPP
#define CALIBRATION__PATTERN_HPP

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

namespace calibration
{

struct PatternConfig
{
  std::string type = "circles";   // "chessboard" 或 "circles"
  cv::Size size{10, 7};           // 列、行（棋盘格为内角点数）
  double spacing_mm = 40.0;       // 棋盘格=方格边长；圆点阵=圆心间距
};

inline PatternConfig load_pattern(const std::string & yaml_path)
{
  auto yaml = YAML::LoadFile(yaml_path);
  PatternConfig pc;
  if (yaml["pattern_type"]) pc.type = yaml["pattern_type"].as<std::string>();
  pc.size = cv::Size(yaml["pattern_cols"].as<int>(), yaml["pattern_rows"].as<int>());
  if (pc.type == "chessboard")
    pc.spacing_mm = yaml["square_size_mm"] ? yaml["square_size_mm"].as<double>() : 20.0;
  else
    pc.spacing_mm = yaml["center_distance_mm"] ? yaml["center_distance_mm"].as<double>() : 40.0;
  return pc;
}

inline bool detect_pattern(
  const cv::Mat & img, const PatternConfig & pc, std::vector<cv::Point2f> & points)
{
  if (pc.type == "chessboard") {
    bool ok = cv::findChessboardCorners(
      img, pc.size, points,
      cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);
    if (ok) {
      cv::Mat gray;
      cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
      cv::cornerSubPix(
        gray, points, {5, 5}, {-1, -1},
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.01));
    }
    return ok;
  }
  return cv::findCirclesGrid(img, pc.size, points, cv::CALIB_CB_SYMMETRIC_GRID);
}

inline std::vector<cv::Point3f> object_points(const PatternConfig & pc)
{
  std::vector<cv::Point3f> pts;
  for (int i = 0; i < pc.size.height; ++i)
    for (int j = 0; j < pc.size.width; ++j)
      pts.push_back({float(j * pc.spacing_mm), float(i * pc.spacing_mm), 0.0f});
  return pts;
}

}  // namespace calibration

#endif  // CALIBRATION__PATTERN_HPP
