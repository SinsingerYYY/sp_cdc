#ifndef AUTO_AIM__TARGET_HPP
#define AUTO_AIM__TARGET_HPP

#include <Eigen/Dense>
#include <array>
#include <chrono>
#include <optional>
#include <queue>
#include <string>
#include <vector>

#include "armor.hpp"
#include "tools/extended_kalman_filter.hpp"

namespace auto_aim
{

// 前哨站旋转半径，单位 m，数值以实测为准
inline constexpr double OUTPOST_RADIUS = 0.2765;
// 前哨站相邻装甲的高度间距，单位 m，布局识别假设三块装甲高度差为此值的整数倍
inline constexpr double OUTPOST_HEIGHT_STEP = 0.102;
// 前哨站高度平均的有效窗口（次）：前若干次为算术平均，之后为指数滑动平均，早期错误观测的影响逐步衰减
inline constexpr int OUTPOST_HEIGHT_WINDOW = 10;
// 每块前哨站装甲至少观测此次数后才认为布局可信；未完成前只打当前观测到的装甲
inline constexpr int OUTPOST_MIN_HEIGHT_OBSERVATIONS = 3;

class Target
{
public:
  ArmorName name;
  ArmorType armor_type;
  ArmorPriority priority;
  bool jumped;
  int last_id;  // debug only

  Target() = default;
  Target(
    const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num,
    Eigen::VectorXd P0_dig);
  Target(double x, double vyaw, double radius, double h);

  void predict(std::chrono::steady_clock::time_point t);
  void predict(double dt);
  void update(const Armor & armor);

  Eigen::VectorXd ekf_x() const;
  const tools::ExtendedKalmanFilter & ekf() const;
  Eigen::Vector4d armor_xyza(int id) const;
  std::vector<Eigen::Vector4d> armor_xyza_list() const;
  // 前哨站三块装甲各观测满 OUTPOST_MIN_HEIGHT_OBSERVATIONS 次后为 true；非前哨站目标恒为 true
  bool outpost_layout_complete() const;

  bool diverged() const;

  bool convergened();

  bool isinit = false;

  bool checkinit();

private:
  int armor_num_;
  int switch_count_;
  int update_count_;

  bool is_switch_, is_converged_;

  tools::ExtendedKalmanFilter ekf_;
  std::chrono::steady_clock::time_point t_;

  std::array<bool, 3> outpost_observed_{{false, false, false}};
  std::array<double, 3> outpost_measured_heights_{{0, 0, 0}};
  std::array<int, 3> outpost_height_observation_counts_{{0, 0, 0}};
  int outpost_layout_index_{0};

  void update_ypda(const Armor & armor, int id);  // yaw pitch distance angle
  void update_outpost_layout(const Armor & armor, int id);
  double outpost_height_offset(int id) const;

  Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd & x, int id) const;
  Eigen::MatrixXd h_jacobian(const Eigen::VectorXd & x, int id) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TARGET_HPP