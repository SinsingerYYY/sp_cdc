#ifndef AUTO_AIM__ARMOR_HPP
#define AUTO_AIM__ARMOR_HPP

#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace auto_aim
{
enum Color
{
  red,
  blue,
  extinguish,
  purple
};
const std::vector<std::string> COLORS = {"red", "blue", "extinguish", "purple"};

enum ArmorType
{
  big,
  small
};
const std::vector<std::string> ARMOR_TYPES = {"big", "small"};

enum ArmorName
{
  one,
  two,
  three,
  four,
  five,
  sentry,
  outpost,
  base,
  not_armor
};
const std::vector<std::string> ARMOR_NAMES = {"one",    "two",     "three", "four",     "five",
                                              "sentry", "outpost", "base",  "not_armor"};

enum ArmorPriority
{
  first = 1,
  second,
  third,
  forth,
  fifth
};

// 赛制相关的装甲属性表集中在这里，下赛季规则变化时只需修改此处和 armor_type_allowed()
// 2026 赛制：仅英雄（one）使用大装甲，其余（含基地）均为小装甲，无平衡步兵
// clang-format off
const std::vector<std::tuple<Color, ArmorName, ArmorType>> armor_properties = {
  {blue, sentry, small},     {red, sentry, small},     {extinguish, sentry, small},
  {blue, one, big},          {red, one, big},          {extinguish, one, big},
  {blue, two, small},        {red, two, small},        {extinguish, two, small},
  {blue, three, small},      {red, three, small},      {extinguish, three, small},
  {blue, four, small},       {red, four, small},       {extinguish, four, small},
  {blue, five, small},       {red, five, small},       {extinguish, five, small},
  {blue, outpost, small},    {red, outpost, small},    {extinguish, outpost, small},
  {blue, base, small},       {red, base, small},       {extinguish, base, small},
  {purple, base, small},     {blue, base, small},      {red, base, small},
  {extinguish, base, small}, {purple, base, small},
};
// clang-format on

// 兵种与装甲大小是否一致：英雄必须是大装甲，其余必须是小装甲，未知兵种不合法
inline bool armor_type_allowed(ArmorName name, ArmorType type)
{
  switch (name) {
    case ArmorName::one:
      return type == ArmorType::big;
    case ArmorName::two:
    case ArmorName::three:
    case ArmorName::four:
    case ArmorName::five:
    case ArmorName::sentry:
    case ArmorName::outpost:
    case ArmorName::base:
      return type == ArmorType::small;
    default:
      return false;
  }
}

// 按兵种决定大小装甲：英雄为大装甲，其余为小装甲（未知兵种按小装甲，后续由 armor_type_allowed() 过滤）
inline ArmorType armor_type_of(ArmorName name)
{
  return name == ArmorName::one ? ArmorType::big : ArmorType::small;
}

// 小装甲在图像中的长宽比上限（灯条间距 / 灯条长度，无量纲）
// 小装甲正视约 135 mm / 56 mm = 2.4，透视只会让它变小；超过 3.0 说明分类或角点有误，
// 按小装甲尺寸做 PnP 会得到错误距离，因此丢弃。大装甲斜视时比值可降到 2.5 以下，不做下限校验
constexpr double SMALL_ARMOR_MAX_RATIO = 3.0;

// 长宽比与装甲大小是否一致；ratio 非有限时视为不一致
inline bool armor_ratio_consistent(ArmorType type, double ratio)
{
  if (!std::isfinite(ratio) || ratio <= 0) return false;
  if (type == ArmorType::small) return ratio <= SMALL_ARMOR_MAX_RATIO;
  return true;
}

// YOLOv5 模型输出的 color id（下标）到 Color 的映射，必须与所用模型一致
using ColorOrder = std::array<Color, 4>;
inline const ColorOrder DEFAULT_YOLOV5_COLOR_ORDER = {blue, red, extinguish, purple};

// 解析形如 "blue_red_gray_purple" 的配置，必须是 red/blue/gray/purple 的一个排列，否则抛异常
ColorOrder parse_yolov5_color_order(const std::string & order);

// YOLOv5 的 color id / num id 到颜色、兵种、大小装甲的统一映射，未知类别归为 not_armor
void set_yolov5_classification(
  int color_id, int num_id, const ColorOrder & color_order, Color & color, ArmorName & name,
  ArmorType & type);

struct Lightbar
{
  std::size_t id;
  Color color;
  cv::Point2f center, top, bottom, top2bottom;
  std::vector<cv::Point2f> points;
  double angle, angle_error, length, width, ratio;
  cv::RotatedRect rotated_rect;

  Lightbar(const cv::RotatedRect & rotated_rect, std::size_t id);
  Lightbar() {};
};

struct Armor
{
  Color color;
  Lightbar left, right;     //used to be const
  cv::Point2f center;       // 不是对角线交点，不能作为实际中心！
  cv::Point2f center_norm;  // 归一化坐标
  std::vector<cv::Point2f> points;

  double ratio;              // 两灯条的中点连线与长灯条的长度之比
  double side_ratio;         // 长灯条与短灯条的长度之比
  double rectangular_error;  // 灯条和中点连线所成夹角与π/2的差值

  ArmorType type;
  ArmorName name;
  ArmorPriority priority;
  int class_id;
  cv::Rect box;
  cv::Mat pattern;
  double confidence;
  bool duplicated;

  Eigen::Vector3d xyz_in_camera;  // 单位：m（相机光学系：x右 y下 z前）
  Eigen::Vector3d xyz_in_gimbal;  // 单位：m
  Eigen::Vector3d xyz_in_world;   // 单位：m
  Eigen::Vector3d ypr_in_gimbal;  // 单位：rad
  Eigen::Vector3d ypr_in_world;   // 单位：rad
  Eigen::Vector3d ypd_in_world;   // 球坐标系

  double yaw_raw;  // rad

  Armor(const Lightbar & left, const Lightbar & right);
  Armor(
    int class_id, float confidence, const cv::Rect & box, std::vector<cv::Point2f> armor_keypoints);
  Armor(
    int class_id, float confidence, const cv::Rect & box, std::vector<cv::Point2f> armor_keypoints,
    cv::Point2f offset);
  Armor(
    int color_id, int num_id, float confidence, const cv::Rect & box,
    std::vector<cv::Point2f> armor_keypoints,
    const ColorOrder & color_order = DEFAULT_YOLOV5_COLOR_ORDER);
  Armor(
    int color_id, int num_id, float confidence, const cv::Rect & box,
    std::vector<cv::Point2f> armor_keypoints, cv::Point2f offset,
    const ColorOrder & color_order = DEFAULT_YOLOV5_COLOR_ORDER);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__ARMOR_HPP