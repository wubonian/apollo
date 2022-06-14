/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 **/
#include "modules/planning/tasks/optimizers/path_time_heuristic/dp_st_cost.h"

#include <algorithm>
#include <limits>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/speed/st_point.h"
#include "modules/planning/tasks/utils/st_gap_estimator.h"

namespace apollo {
namespace planning {
namespace {
constexpr double kInf = std::numeric_limits<double>::infinity();
}

/* DpStCost初始化函数 */
DpStCost::DpStCost(const DpStSpeedOptimizerConfig& config, const double total_t,
                   const double total_s,
                   const std::vector<const Obstacle*>& obstacles,
                   const STDrivableBoundary& st_drivable_boundary,
                   const common::TrajectoryPoint& init_point)
    : config_(config),
      obstacles_(obstacles),
      st_drivable_boundary_(st_drivable_boundary),
      init_point_(init_point),
      unit_t_(config.unit_t()),
      total_s_(total_s) {
  int index = 0;
  // 设置每个obstacle id对应的index到boundary_map_中
  for (const auto& obstacle : obstacles) {
    boundary_map_[obstacle->path_st_boundary().id()] = index++;
  }

  AddToKeepClearRange(obstacles);

  // dimension_t为t的采样个数
  const auto dimension_t =
      static_cast<uint32_t>(std::ceil(total_t / static_cast<double>(unit_t_))) +
      1;
  // 初始化boundary_cost_与obstacles_的size相同
  boundary_cost_.resize(obstacles_.size());
  // 初始化每个obstacle的boundary_cost_为维数为dimension_t的<-1.0, 1.0>
  // 即对每个obstacle, 在每个时间t采样点, 对应的boundary_cost_值为<-1.0, 1.0>
  for (auto& vec : boundary_cost_) {
    vec.resize(dimension_t, std::make_pair(-1.0, -1.0));
  }
  // 初始化accel_cost_, jerk_cost_
  accel_cost_.fill(-1.0);         // 初始化所有的accel_cost_ && jerk_cost_为-1.0
  jerk_cost_.fill(-1.0);
}

/* 将所有KEEP_CLEAR obstacle的<start_s, end_s>存放在keep_clear_range_内 */
void DpStCost::AddToKeepClearRange(
    const std::vector<const Obstacle*>& obstacles) {
  for (const auto& obstacle : obstacles) {
    if (obstacle->path_st_boundary().IsEmpty()) {
      continue;
    }
    if (obstacle->path_st_boundary().boundary_type() !=
        STBoundary::BoundaryType::KEEP_CLEAR) {
      continue;
    }

    double start_s = obstacle->path_st_boundary().min_s();
    double end_s = obstacle->path_st_boundary().max_s();
    keep_clear_range_.emplace_back(start_s, end_s);
  }
  SortAndMergeRange(&keep_clear_range_);
}

/*  */
void DpStCost::SortAndMergeRange(
    std::vector<std::pair<double, double>>* keep_clear_range) {
  if (!keep_clear_range || keep_clear_range->empty()) {
    return;
  }
  std::sort(keep_clear_range->begin(), keep_clear_range->end());
  size_t i = 0;
  size_t j = i + 1;
  while (j < keep_clear_range->size()) {
    if (keep_clear_range->at(i).second < keep_clear_range->at(j).first) {
      ++i;
      ++j;
    } else {
      keep_clear_range->at(i).second = std::max(keep_clear_range->at(i).second,
                                                keep_clear_range->at(j).second);
      ++j;
    }
  }
  keep_clear_range->resize(i + 1);
}

bool DpStCost::InKeepClearRange(double s) const {
  for (const auto& p : keep_clear_range_) {
    if (p.first <= s && p.second >= s) {
      return true;
    }
  }
  return false;
}

/* 计算采样点st_graph_point在ST图中的距离cost
   -> 与obstacle重叠 或 不在StBoundsDecider计算的ST可通行区域内, 直接设置cost为无限大
   -> 在可通行区域内时, 如果follow或者overtake的过近, 则会根据与follow/overtake target distance的偏差, 计算对应的cost
   -> 满足该target distance阈值要求, 则没有cost */
double DpStCost::GetObstacleCost(const StGraphPoint& st_graph_point) {
  const double s = st_graph_point.point().s();    // 提取st_graph_point的s
  const double t = st_graph_point.point().t();    // 提取st_graph_point的t

  double cost = 0.0;

  // FLAGS_use_st_drivable_boundary = true: 直接使用StBoundsDecider计算得到的st_driverable_boundary_
  // (经过obstacle纵向决策后, 得到的ST图的可通行区域)来筛除位于可通行区域以外的采样点 (返回cost为无穷大)
  if (FLAGS_use_st_drivable_boundary) {
    // TODO(Jiancheng): move to configs
    static constexpr double boundary_resolution = 0.1;
    int index = static_cast<int>(t / boundary_resolution);
    const double lower_bound =
        st_drivable_boundary_.st_boundary(index).s_lower();
    const double upper_bound =
        st_drivable_boundary_.st_boundary(index).s_upper();

    // 如果当前采样点的s位于st_driverable_boundary_以外, 则设置cost为无穷大
    if (s > upper_bound || s < lower_bound) {
      return kInf;
    }
  }

  // 遍历所有的obstacle
  for (const auto* obstacle : obstacles_) {
    // Not applying obstacle approaching cost to virtual obstacle like created
    // stop fences
    if (obstacle->IsVirtual()) {
      continue;
    }

    // Stop obstacles are assumed to have a safety margin when mapping them out,
    // so repelling force in dp st is not needed as it is designed to have adc
    // stop right at the stop distance we design in prior mapping process
    // 跳过stop obstacle, 只对动态目标进行处理
    if (obstacle->LongitudinalDecision().has_stop()) {
      continue;
    }

    // 提取目标的st_boundary (在StBoundsDecider/SpeedBoundsDecider里计算得到)
    auto boundary = obstacle->path_st_boundary();

    // 如果目标的纵向距离过远, 略过
    if (boundary.min_s() > FLAGS_speed_lon_decision_horizon) {
      continue;
    }
    // 如果当前采样点的时间t在obstacle STBoundary的时间范围以外, 略过
    if (t < boundary.min_t() || t > boundary.max_t()) {
      continue;
    }
    // 如果当前采样点位于obstacle的STBoundary以内, 则返回cost为无穷大
    if (boundary.IsPointInBoundary(st_graph_point.point())) {
      return kInf;
    }
    double s_upper = 0.0;
    double s_lower = 0.0;
    // 得到obstacle在dp_st_cost中存储的index
    int boundary_index = boundary_map_[boundary.id()];
    // 如果boundary_index对应的目标在第index_t个时间采样点的boundary_cost_值小于0
    // 表示该目标的对应时间处的boundary_cost_还没有初始化
    if (boundary_cost_[boundary_index][st_graph_point.index_t()].first < 0.0) {
      // 计算该时间处的S的上下边界<s_lower, s_upper>
      boundary.GetBoundarySRange(t, &s_upper, &s_lower);
      // 设置对应的boundary_cost_值为<s_upper, s_lower>
      boundary_cost_[boundary_index][st_graph_point.index_t()] =
          std::make_pair(s_upper, s_lower);
    } 
    // 如果该目标对应t位置的boundary_cost_ <s_upper, s_lower>已经初始化, 则直接提取对应的上下边界值
    else {
      s_upper = boundary_cost_[boundary_index][st_graph_point.index_t()].first;
      s_lower = boundary_cost_[boundary_index][st_graph_point.index_t()].second;
    }
    // 如果采样点的s < s_lower
    // 如果s位于s_lower以下follow distance以内 (跟的过近), 则设置对应的距离cost为
    // obstacle_weight * default_obstacle_cost * (s_diff)^2
    // s_diff为超出follow_distance的距离
    if (s < s_lower) {
      const double follow_distance_s = config_.safe_distance();
      if (s + follow_distance_s < s_lower) {
        continue;
      } else {
        auto s_diff = follow_distance_s - s_lower + s;
        cost += config_.obstacle_weight() * config_.default_obstacle_cost() *
                s_diff * s_diff;
      }
    }
    // 如果采样点的s > s_upper
    // 如果s的overtake距离过近, 类似的计算对应的距离cost
    else if (s > s_upper) {
      const double overtake_distance_s =
          StGapEstimator::EstimateSafeOvertakingGap();
      if (s > s_upper + overtake_distance_s) {  // or calculated from velocity
        continue;
      } else {
        auto s_diff = overtake_distance_s + s_upper - s;
        cost += config_.obstacle_weight() * config_.default_obstacle_cost() *
                s_diff * s_diff;
      }
    }
  }
  // 最终计算得到的cost为与目标距离cost * unit_t_
  // 包含的信息为: 当前采样点的s在t时, 与所有目标的距离cost (follow的过近/overtake的过近都会增加cost)
  // 计算cost的距离threshold为target follow distance与overtake distance
  return cost * unit_t_;
}

/* 计算空间potential的cost: s越大, cost越小 */
double DpStCost::GetSpatialPotentialCost(const StGraphPoint& point) {
  return (total_s_ - point.point().s()) * config_.spatial_potential_penalty();
}

double DpStCost::GetReferenceCost(const STPoint& point,
                                  const STPoint& reference_point) const {
  return config_.reference_weight() * (point.s() - reference_point.s()) *
         (point.s() - reference_point.s()) * unit_t_;
}

/* 计算first->second对应的速度, 以及产生的速度cost:
   -> 停止在Keep-Clear范围内
   -> speed_limit: 超过speed_limit产生较大的cost, 小于speed_limit产生较小的cost
   -> cruise_speed: 不满足巡航速度, 产生cost */
double DpStCost::GetSpeedCost(const STPoint& first, const STPoint& second,
                              const double speed_limit,
                              const double cruise_speed) const {
  double cost = 0.0;
  // 计算first -> second的速度
  const double speed = (second.s() - first.s()) / unit_t_;
  // 如果速度为负, 则返回速度cost为无限大
  if (speed < 0) {
    return kInf;
  }

  const double max_adc_stop_speed = common::VehicleConfigHelper::Instance()
                                        ->GetConfig()
                                        .vehicle_param()
                                        .max_abs_speed_when_stopped();
  // 如果速度为0且处于KeepClear范围 -> 停车在了keep clear范围内
  // 叠加对应的cost
  if (speed < max_adc_stop_speed && InKeepClearRange(second.s())) {
    // first.s in range
    cost += config_.keep_clear_low_speed_penalty() * unit_t_ *
            config_.default_speed_cost();
  }

  double det_speed = (speed - speed_limit) / speed_limit;
  // 超过speed_limit, 计算对应的cost
  if (det_speed > 0) {
    cost += config_.exceed_speed_penalty() * config_.default_speed_cost() *
            (det_speed * det_speed) * unit_t_;
  } 
  // 达不到speed_limit, 也会增加一些cost -> 防止一直维持在低速, 提高通行效率
  else if (det_speed < 0) {
    cost += config_.low_speed_penalty() * config_.default_speed_cost() *
            -det_speed * unit_t_;
  }

  // 对与reference_speed的偏差进行惩罚
  if (FLAGS_enable_dp_reference_speed) {
    double diff_speed = speed - cruise_speed;
    cost += config_.reference_speed_penalty() * config_.default_speed_cost() *
            fabs(diff_speed) * unit_t_;
  }

  return cost;
}

/* 计算给定accel对应的cost: accel幅值的cost + 与accel_limit偏差的cost */
double DpStCost::GetAccelCost(const double accel) {
  double cost = 0.0;
  static constexpr double kEpsilon = 0.1;
  static constexpr size_t kShift = 100;
  // 计算当前accel在accel_cost_的vector中对应的index: accel_key
  const size_t accel_key = static_cast<size_t>(accel / kEpsilon + 0.5 + kShift);
  DCHECK_LT(accel_key, accel_cost_.size());
  if (accel_key >= accel_cost_.size()) {
    return kInf;
  }

  // 如果accel_cost_在accel_key处还没有初始化
  if (accel_cost_.at(accel_key) < 0.0) {
    const double accel_sq = accel * accel;
    double max_acc = config_.max_acceleration();
    double max_dec = config_.max_deceleration();
    double accel_penalty = config_.accel_penalty();
    double decel_penalty = config_.decel_penalty();

    // accel绝对值对应的cost + accel相对于max_acc/max_dec的cost
    if (accel > 0.0) {
      cost = accel_penalty * accel_sq;
    } else {
      cost = decel_penalty * accel_sq;
    }
    cost += accel_sq * decel_penalty * decel_penalty /
                (1 + std::exp(1.0 * (accel - max_dec))) +
            accel_sq * accel_penalty * accel_penalty /
                (1 + std::exp(-1.0 * (accel - max_acc)));
    accel_cost_.at(accel_key) = cost;
  } 
  // 如果该accel的cost已经更新过, 则直接返回对应的cost值
  else {
    cost = accel_cost_.at(accel_key);
  }
  return cost * unit_t_;
}

/* 基于三个点first -> second -> third, 计算对应的加速度以及accel_cost */
double DpStCost::GetAccelCostByThreePoints(const STPoint& first,
                                           const STPoint& second,
                                           const STPoint& third) {
  double accel = (first.s() + third.s() - 2 * second.s()) / (unit_t_ * unit_t_);
  return GetAccelCost(accel);
}

/* 计算pre_point->curr_point对应的加速度, 并更新accel cost */
double DpStCost::GetAccelCostByTwoPoints(const double pre_speed,
                                         const STPoint& pre_point,
                                         const STPoint& curr_point) {
  double current_speed = (curr_point.s() - pre_point.s()) / unit_t_;
  double accel = (current_speed - pre_speed) / unit_t_;
  return GetAccelCost(accel);
}

/* 计算jerk的cost (只考虑cost的绝对值) */
double DpStCost::JerkCost(const double jerk) {
  double cost = 0.0;
  static constexpr double kEpsilon = 0.1;
  static constexpr size_t kShift = 200;
  const size_t jerk_key = static_cast<size_t>(jerk / kEpsilon + 0.5 + kShift);
  if (jerk_key >= jerk_cost_.size()) {
    return kInf;
  }

  // 如果对应jerk值的cost没有更新, 则计算对应的cost值
  // cost = k * jerk^2
  if (jerk_cost_.at(jerk_key) < 0.0) {
    double jerk_sq = jerk * jerk;
    if (jerk > 0) {
      cost = config_.positive_jerk_coeff() * jerk_sq * unit_t_;
    } else {
      cost = config_.negative_jerk_coeff() * jerk_sq * unit_t_;
    }
    jerk_cost_.at(jerk_key) = cost;
  } else {
    cost = jerk_cost_.at(jerk_key);
  }

  // TODO(All): normalize to unit_t_
  return cost;
}

/* 基于4个点计算jerk与对应的cost */
double DpStCost::GetJerkCostByFourPoints(const STPoint& first,
                                         const STPoint& second,
                                         const STPoint& third,
                                         const STPoint& fourth) {
  double jerk = (fourth.s() - 3 * third.s() + 3 * second.s() - first.s()) /
                (unit_t_ * unit_t_ * unit_t_);
  return JerkCost(jerk);
}

/* 基于两个点计算jerk, 随后计算jerk对应的cost */
double DpStCost::GetJerkCostByTwoPoints(const double pre_speed,
                                        const double pre_acc,
                                        const STPoint& pre_point,
                                        const STPoint& curr_point) {
  const double curr_speed = (curr_point.s() - pre_point.s()) / unit_t_;
  const double curr_accel = (curr_speed - pre_speed) / unit_t_;
  const double jerk = (curr_accel - pre_acc) / unit_t_;
  return JerkCost(jerk);
}

/* 基于三个点计算jerk以及对应的cost */
double DpStCost::GetJerkCostByThreePoints(const double first_speed,
                                          const STPoint& first,
                                          const STPoint& second,
                                          const STPoint& third) {
  const double pre_speed = (second.s() - first.s()) / unit_t_;
  const double pre_acc = (pre_speed - first_speed) / unit_t_;
  const double curr_speed = (third.s() - second.s()) / unit_t_;
  const double curr_acc = (curr_speed - pre_speed) / unit_t_;
  const double jerk = (curr_acc - pre_acc) / unit_t_;
  return JerkCost(jerk);
}

}  // namespace planning
}  // namespace apollo
