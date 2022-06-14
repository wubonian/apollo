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
 * @file gridded_path_time_graph.cc
 **/

#include "modules/planning/tasks/optimizers/path_time_heuristic/gridded_path_time_graph.h"

#include <algorithm>
#include <limits>
#include <string>

#include "modules/common/proto/pnc_point.pb.h"

#include "cyber/common/log.h"
#include "cyber/task/task.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/util/point_factory.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::SpeedPoint;
using apollo::common::Status;
using apollo::common::util::PointFactory;

namespace {

static constexpr double kDoubleEpsilon = 1.0e-6;

// Continuous-time collision check using linear interpolation as closed-loop
// dynamics
/* 检查<p1, p2>与obstacles的STBoundary是否有重叠 */
bool CheckOverlapOnDpStGraph(const std::vector<const STBoundary*>& boundaries,
                             const StGraphPoint& p1, const StGraphPoint& p2) {
  // 如果使用StBoundsDecider生成的St_Driverable_Boundary, 则直接返回false
  // 因为执行该函数时, 已经处于可通行区域内
  if (FLAGS_use_st_drivable_boundary) {
    return false;
  }
  // 检查<p1, p2>的连线与boundaries是否有可能碰撞
  for (const auto* boundary : boundaries) {
    if (boundary->boundary_type() == STBoundary::BoundaryType::KEEP_CLEAR) {
      continue;
    }
    // Check collision between a polygon and a line segment
    if (boundary->HasOverlap({p1.point(), p2.point()})) {
      return true;
    }
  }
  return false;
}
}  // namespace

/* GriddedPathTimeGraph初始化函数*/
GriddedPathTimeGraph::GriddedPathTimeGraph(
    const StGraphData& st_graph_data, const DpStSpeedOptimizerConfig& dp_config,
    const std::vector<const Obstacle*>& obstacles,
    const common::TrajectoryPoint& init_point)
    : st_graph_data_(st_graph_data),
      gridded_path_time_graph_config_(dp_config),
      obstacles_(obstacles),
      init_point_(init_point),
      dp_st_cost_(dp_config, st_graph_data_.total_time_by_conf(),
                  st_graph_data_.path_length(), obstacles,
                  st_graph_data_.st_drivable_boundary(), init_point_) {
  total_length_t_ = st_graph_data_.total_time_by_conf();
  unit_t_ = gridded_path_time_graph_config_.unit_t();
  total_length_s_ = st_graph_data_.path_length();
  dense_unit_s_ = gridded_path_time_graph_config_.dense_unit_s();
  sparse_unit_s_ = gridded_path_time_graph_config_.sparse_unit_s();
  dense_dimension_s_ = gridded_path_time_graph_config_.dense_dimension_s();
  // Safety approach preventing unreachable acceleration/deceleration
  max_acceleration_ =
      std::min(std::abs(vehicle_param_.max_acceleration()),
               std::abs(gridded_path_time_graph_config_.max_acceleration()));
  max_deceleration_ =
      -1.0 *
      std::min(std::abs(vehicle_param_.max_deceleration()),
               std::abs(gridded_path_time_graph_config_.max_deceleration()));
}

/* 进行ST图的DP优化, 返回DP结果speed_data: vector<t,s,v,a,da>:
   -> 对ST图按照采样间隔离散化cost_table_
   -> 更新每个位置点处的speed_limit信息
   -> DP搜索离散化的cost_table_, 更新每个节点的cost(位置, 速度, 加速度, jerk), 以及最优pre-point等信息
   -> 基于更新后的cost_table_, 寻找最优end_point, 并反向递推得到整条链路, 生成最终的DP结果speed_data */
Status GriddedPathTimeGraph::Search(SpeedData* const speed_data) {
  static constexpr double kBounadryEpsilon = 1e-2;
  // 遍历每个obstacle的STBoundary, 如果发现init point位于obstacle的STBoundary内
  // 则返回speed_data为(s=0, t, v=0, a=0, da=0) -> t = [0, t_end]
  for (const auto& boundary : st_graph_data_.st_boundaries()) {
    // KeepClear obstacles not considered in Dp St decision
    // 跳过KEEP_CLEAR obstacle的STBoundary
    if (boundary->boundary_type() == STBoundary::BoundaryType::KEEP_CLEAR) {
      continue;
    }
    // If init point in collision with obstacle, return speed fallback
    // 如果init point位于STBoundary内, 则设置speed_data为(s=0, t, v=0, a=0, da=0) -> t = [0, t_end]
    if (boundary->IsPointInBoundary({0.0, 0.0}) ||
        (std::fabs(boundary->min_t()) < kBounadryEpsilon &&
         std::fabs(boundary->min_s()) < kBounadryEpsilon)) {
      dimension_t_ = static_cast<uint32_t>(std::ceil(
                         total_length_t_ / static_cast<double>(unit_t_))) +
                     1;
      std::vector<SpeedPoint> speed_profile;
      double t = 0.0;
      for (uint32_t i = 0; i < dimension_t_; ++i, t += unit_t_) {
        // 将speed_profile设置为(s=0, t, v=0, a=0, da=0)
        speed_profile.push_back(PointFactory::ToSpeedPoint(0, t));
      }
      *speed_data = SpeedData(speed_profile);
      return Status::OK();
    }
  }

  // 对ST图进行离散化采样, 初始化dimension_t_ * dimension_s_ StGraphPoint的cost_table_
  if (!InitCostTable().ok()) {
    const std::string msg = "Initialize cost table failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // 基于SpeedBoundsDecider计算得到的speed_limit信息, 计算每个采样位置点处的限速信息
  if (!InitSpeedLimitLookUp().ok()) {
    const std::string msg = "Initialize speed limit lookup table failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // 更新cost_table_中每个采样点的以下属性
  // -> 最小total_cost_(与obstacle的位置, speed, accel, jerk)
  // -> 该最小cost对应的前一时刻采样点
  // -> 基于前一时刻采样点计算得到的当前采样点速度
  // 下一时刻筛选的s范围:
  // -> 基于当前时刻的位置与速度, 按照最大/最小加速度, 计算下一时刻需要筛选的位置范围
  if (!CalculateTotalCost().ok()) {
    const std::string msg = "Calculate total cost failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // 基于更新的cost_table_中的结果, 选择最优end-point并递推出整条链路, 返回DP结果speed_data
  if (!RetrieveSpeedProfile(speed_data).ok()) {
    const std::string msg = "Retrieve best speed profile failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  return Status::OK();
}

/* 对ST图进行t与s的采样(s包括粗采样与细采样)
   初始化dimension_t_ * dimension_s_ StGraphPoint的cost_table_ */
Status GriddedPathTimeGraph::InitCostTable() {
  // Time dimension is homogeneous while Spatial dimension has two resolutions,
  // dense and sparse with dense resolution coming first in the spatial horizon

  // Sanity check for numerical stability
  if (unit_t_ < kDoubleEpsilon) {
    const std::string msg = "unit_t is smaller than the kDoubleEpsilon.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // Sanity check on s dimension setting
  if (dense_dimension_s_ < 1) {
    const std::string msg = "dense_dimension_s is at least 1.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // 计算t维度的采样点数, 以unit_t_ (1.0) 为t的采样间隔
  dimension_t_ = static_cast<uint32_t>(std::ceil(
                     total_length_t_ / static_cast<double>(unit_t_))) +
                 1;
  // dense_unit_s_(0.1)为s的较密的采样间隔
  // dense_dimension_s_(101)为以较密间隔(dense_unit_s_: 0.1)采样s的点的个数
  // sparse_length_s为剩余的需要以较稀疏距离采样的s的长度
  double sparse_length_s =
      total_length_s_ -
      static_cast<double>(dense_dimension_s_ - 1) * dense_unit_s_;
  // sparse_unit_s_(1.0)为s的较稀疏的采样间隔
  // sparse_dimension_s_为计算的需要以较稀疏的距离采样s的点的个数
  sparse_dimension_s_ =
      sparse_length_s > std::numeric_limits<double>::epsilon()
          ? static_cast<uint32_t>(std::ceil(sparse_length_s / sparse_unit_s_))
          : 0;
  // 如果总长度过短, 都以较密的点采样, 则更新实际需要以较密间隔采样s的点的个数
  dense_dimension_s_ =
      sparse_length_s > std::numeric_limits<double>::epsilon()
          ? dense_dimension_s_
          : static_cast<uint32_t>(std::ceil(total_length_s_ / dense_unit_s_)) +
                1;
  // 计算s的总的采样点个数
  dimension_s_ = dense_dimension_s_ + sparse_dimension_s_;

  // Sanity Check
  if (dimension_t_ < 1 || dimension_s_ < 1) {
    const std::string msg = "Dp st cost table size incorrect.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // 初始化cost_table_为dimension_s_ * dimension_t_的StGraphPoint()矩阵
  cost_table_ = std::vector<std::vector<StGraphPoint>>(
      dimension_t_, std::vector<StGraphPoint>(dimension_s_, StGraphPoint()));

  double curr_t = 0.0;
  // 遍历所有的采样时间t && s, 初始化每个StGraphPoint的(s,t)点以及对应的s/t index
  for (uint32_t i = 0; i < cost_table_.size(); ++i, curr_t += unit_t_) {
    auto& cost_table_i = cost_table_[i];
    double curr_s = 0.0;
    // 初始化所有的StGraphPoint的index_t_, index_s_, point_
    // 遍历所有的细采样距离s
    for (uint32_t j = 0; j < dense_dimension_s_; ++j, curr_s += dense_unit_s_) {
      cost_table_i[j].Init(i, j, STPoint(curr_s, curr_t));
    }
    curr_s = static_cast<double>(dense_dimension_s_ - 1) * dense_unit_s_ +
             sparse_unit_s_;
    // 遍历所有的稀疏采样距离s
    for (uint32_t j = dense_dimension_s_; j < cost_table_i.size();
         ++j, curr_s += sparse_unit_s_) {
      cost_table_i[j].Init(i, j, STPoint(curr_s, curr_t));
    }
  }

  const auto& cost_table_0 = cost_table_[0];  // 对应t=0时对应的s vector
  spatial_distance_by_index_ = std::vector<double>(cost_table_0.size(), 0.0);
  // 更新s的所有采样点到spatial_distance_by_index_
  for (uint32_t i = 0; i < cost_table_0.size(); ++i) {
    spatial_distance_by_index_[i] = cost_table_0[i].point().s();
  }
  return Status::OK();
}

/* 基于SpeedBoundsDecider计算得到的speed_limit信息, 计算每个采样位置点处的限速信息:
   -> speed_limit包含由map, 横向加速度, 静态障碍物nudge产生的限速 */
Status GriddedPathTimeGraph::InitSpeedLimitLookUp() {
  speed_limit_by_index_.clear();

  // 初始化speed_limit_by_index_的size为dimension_s_
  speed_limit_by_index_.resize(dimension_s_);
  const auto& speed_limit = st_graph_data_.speed_limit();

  // 基于SpeedBoundsDecider计算得到的speed_limit(map, 横向加速度, 静态障碍物nudge限速)
  // 更新每个位置点对应的限速信息到speed_limit_by_index_
  for (uint32_t i = 0; i < dimension_s_; ++i) {
    speed_limit_by_index_[i] =
        speed_limit.GetSpeedLimitByS(cost_table_[0][i].point().s());
  }
  return Status::OK();
}

/* 更新total_cost_中每个采样点的以下属性
   -> 最小total_cost_(与obstacle的位置, speed, accel, jerk),
   -> 该最小cost对应的前一时刻采样点
   -> 基于前一时刻采样点计算得到的当前采样点速度
   下一时刻筛选的s范围:
   -> 基于当前时刻的位置与速度, 按照最大/最小加速度, 计算下一时刻需要筛选的位置范围 */
Status GriddedPathTimeGraph::CalculateTotalCost() {
  // col and row are for STGraph
  // t corresponding to col
  // s corresponding to row
  size_t next_highest_row = 0;
  size_t next_lowest_row = 0;
  // cost_table_.size()对应t的采样个数
  for (size_t c = 0; c < cost_table_.size(); ++c) {
    size_t highest_row = 0;
    size_t lowest_row = cost_table_.back().size() - 1;

    int count = static_cast<int>(next_highest_row) -
                static_cast<int>(next_lowest_row) + 1;
    // 如果当前<next_lowest_row, next_highest_row>之间有效采样点个数大于0
    if (count > 0) {
      std::vector<std::future<void>> results;
      // 遍历该采样时间下, next_lowest_row与next_highest_row之间的采样点
      for (size_t r = next_lowest_row; r <= next_highest_row; ++r) {
        auto msg = std::make_shared<StGraphMessage>(c, r);
        // 允许对该时刻下的采样点并行计算
        // 更新采样点对应cost_table_[c][r]的total_cost, pre-point(连接的前一时刻采样点), 
        // optimal_speed_(基于选取的前一采样点计算得到的当前速度, 起始点速度为当前车速)
        if (FLAGS_enable_multi_thread_in_dp_st_graph) {
          results.push_back(
              cyber::Async(&GriddedPathTimeGraph::CalculateCostAt, this, msg));
        } 
        // 否则不进行并行计算, 按序计算
        else {
          CalculateCostAt(msg);
        }
      }
      if (FLAGS_enable_multi_thread_in_dp_st_graph) {
        for (auto& result : results) {
          result.get();
        }
      }
    }
    
    // 遍历有效row range范围内的所有采样点, 基于每个采样点在最大最小加速度时, 下一时刻位置的预测
    // 计算得到下一时刻的最大s range范围对应的row index 范围: <lowest_row, highest_row>
    for (size_t r = next_lowest_row; r <= next_highest_row; ++r) {
      const auto& cost_cr = cost_table_[c][r];
      // 如果采样点的total_cost_非无穷大, 计算当前采样点在最大最小加速度下, 下一时刻可能的s位置, 对应的row index: h_r, l_r
      if (cost_cr.total_cost() < std::numeric_limits<double>::infinity()) {
        size_t h_r = 0;
        size_t l_r = 0;
        GetRowRange(cost_cr, &h_r, &l_r);
        // 更新highest_row, lowest_row
        highest_row = std::max(highest_row, h_r);
        lowest_row = std::min(lowest_row, l_r);
      }
    }
    next_highest_row = highest_row;
    next_lowest_row = lowest_row;
  }

  return Status::OK();
}

/* 基于给定采样点point的位置/速度, 以及自车最大&最小加速度, 计算下个采样时刻可能的s范围, 以及对应的row index:
   -> next_highest_row && next_lowest_row */ 
void GriddedPathTimeGraph::GetRowRange(const StGraphPoint& point,
                                       size_t* next_highest_row,
                                       size_t* next_lowest_row) {
  double v0 = 0.0;
  // TODO(all): Record speed information in StGraphPoint and deprecate this.
  // A scaling parameter for DP range search due to the lack of accurate
  // information of the current velocity (set to 1 by default since we use
  // past 1 second's average v as approximation)
  double acc_coeff = 0.5;
  // 提取当前采样点的期望速度(基于选取的最优前一时刻点)
  if (!point.pre_point()) {
    v0 = init_point_.v();
  } else {
    v0 = point.GetOptimalSpeed();
  }

  // 基于给定采样点的速度, 与最大可能加速度, 计算下一时刻可能的最大位置s为s_upper_bound
  const auto max_s_size = dimension_s_ - 1;
  const double t_squared = unit_t_ * unit_t_;
  const double s_upper_bound = v0 * unit_t_ +
                               acc_coeff * max_acceleration_ * t_squared +
                               point.point().s();
  // 求取s_upper_bound对应的row的index
  const auto next_highest_itr =
      std::lower_bound(spatial_distance_by_index_.begin(),
                       spatial_distance_by_index_.end(), s_upper_bound);
  if (next_highest_itr == spatial_distance_by_index_.end()) {
    *next_highest_row = max_s_size;
  } else {
    *next_highest_row =
        std::distance(spatial_distance_by_index_.begin(), next_highest_itr);
  }

  // 基于给定采样点的速度, 与最大可能减速度, 计算下一时刻可能的最小位置s为s_lower_bound
  const double s_lower_bound =
      std::fmax(0.0, v0 * unit_t_ + acc_coeff * max_deceleration_ * t_squared) +
      point.point().s();
  // 求取s_lower_bound对应的row的index
  const auto next_lowest_itr =
      std::lower_bound(spatial_distance_by_index_.begin(),
                       spatial_distance_by_index_.end(), s_lower_bound);
  if (next_lowest_itr == spatial_distance_by_index_.end()) {
    *next_lowest_row = max_s_size;
  } else {
    *next_lowest_row =
        std::distance(spatial_distance_by_index_.begin(), next_lowest_itr);
  }
}

/* 计算给定坐标cost_table_[c][r]处的total_cost_, 
   cost最低的前一采样点pre_point_, 基于前一采样点计算的当前速度optimal_speed_ */
void GriddedPathTimeGraph::CalculateCostAt(
    const std::shared_ptr<StGraphMessage>& msg) {
  const uint32_t c = msg->c;      // column: time
  const uint32_t r = msg->r;      // row: s
  auto& cost_cr = cost_table_[c][r];    // 提取对应cost_table_的元素

  // 对采样点cost_cr, 计算并设置由于follow/overtake目标过近导致的距离cost
  cost_cr.SetObstacleCost(dp_st_cost_.GetObstacleCost(cost_cr));
  // 如果采样点与obstacle的STBoundary重合, 则直接返回, 因为cost为无限大
  if (cost_cr.obstacle_cost() > std::numeric_limits<double>::max()) {
    return;
  }

  // 计算采样点的空间potential cost, 采样点的s越大, cost越小; s越小, cost越大
  cost_cr.SetSpatialPotentialCost(dp_st_cost_.GetSpatialPotentialCost(cost_cr));

  const auto& cost_init = cost_table_[0][0];
  // 对于时间0, 设置total_cost_为0.0, optimal_speed_为当前速度
  if (c == 0) {
    DCHECK_EQ(r, 0U) << "Incorrect. Row should be 0 with col = 0. row: " << r;
    cost_cr.SetTotalCost(0.0);
    cost_cr.SetOptimalSpeed(init_point_.v());
    return;
  }

  const double speed_limit = speed_limit_by_index_[r];      // 提取当前位置的速度限值
  const double cruise_speed = st_graph_data_.cruise_speed();      // 提取目标巡航速度
  // The mininal s to model as constant acceleration formula
  // default: 0.25 * 7 = 1.75 m
  const double min_s_consider_speed = dense_unit_s_ * dimension_t_;

  // 对于第一个采样时间
  if (c == 1) {
    // 计算第一个采样点的加速度
    const double acc =
        2 * (cost_cr.point().s() / unit_t_ - init_point_.v()) / unit_t_;
    // 如果加速度超过上下限值, 则返回
    if (acc < max_deceleration_ || acc > max_acceleration_) {
      return;
    }

    if (init_point_.v() + acc * unit_t_ < -kDoubleEpsilon &&
        cost_cr.point().s() > min_s_consider_speed) {
      return;
    }

    // 检查从起始点cost_init到当前点cost_cr的连接线是否会与obstacles的STBoundary发生碰撞
    if (CheckOverlapOnDpStGraph(st_graph_data_.st_boundaries(), cost_cr,
                                cost_init)) {
      return;
    }

    // 计算total_cost
    cost_cr.SetTotalCost(
        cost_cr.obstacle_cost() + cost_cr.spatial_potential_cost() +
        cost_init.total_cost() +
        CalculateEdgeCostForSecondCol(r, speed_limit, cruise_speed));
    cost_cr.SetPrePoint(cost_init);
    cost_cr.SetOptimalSpeed(init_point_.v() + acc * unit_t_);
    return;
  }

  static constexpr double kSpeedRangeBuffer = 0.20;
  // 假设按照最大车速, 计算前一时刻可能的最小s位置值pre_lowest_s (通过计算按照最大可能车速, 自车在单位时间内能走过的距离)
  const double pre_lowest_s =
      cost_cr.point().s() -
      FLAGS_planning_upper_speed_limit * (1 + kSpeedRangeBuffer) * unit_t_;
  // 计算pre_lowest_s对应在s采样点的index值
  const auto pre_lowest_itr =
      std::lower_bound(spatial_distance_by_index_.begin(),
                       spatial_distance_by_index_.end(), pre_lowest_s);
  
  uint32_t r_low = 0;
  if (pre_lowest_itr == spatial_distance_by_index_.end()) {
    r_low = dimension_s_ - 1;
  }
  // 计算pre_lowest_s对应的row number
  else {
    r_low = static_cast<uint32_t>(
        std::distance(spatial_distance_by_index_.begin(), pre_lowest_itr));
  }
  // 计算pre_lowest_s到当前采样点的行数r_pre_size
  const uint32_t r_pre_size = r - r_low + 1;
  const auto& pre_col = cost_table_[c - 1];
  double curr_speed_limit = speed_limit;

  // 如果当前点位于第三列(c==2)
  if (c == 2) {
    // 依次取当前采样点前一列中[pre_lowest_s, s]的采样点作为当前采样点的前一个点
    for (uint32_t i = 0; i < r_pre_size; ++i) {
      uint32_t r_pre = r - i;
      if (std::isinf(pre_col[r_pre].total_cost()) ||
          pre_col[r_pre].pre_point() == nullptr) {
        continue;
      }
      // TODO(Jiaxuan): Calculate accurate acceleration by recording speed
      // data in ST point.
      // Use curr_v = (point.s - pre_point.s) / unit_t as current v
      // Use pre_v = (pre_point.s - prepre_point.s) / unit_t as previous v
      // Current acc estimate: curr_a = (curr_v - pre_v) / unit_t
      // = (point.s + prepre_point.s - 2 * pre_point.s) / (unit_t * unit_t)
      // 计算根据当前点与前一采样时刻的点, 计算加速度curr_a
      const double curr_a =
          2 *
          ((cost_cr.point().s() - pre_col[r_pre].point().s()) / unit_t_ -
           pre_col[r_pre].GetOptimalSpeed()) /
          unit_t_;
      if (curr_a < max_deceleration_ || curr_a > max_acceleration_) {
        continue;
      }

      if (pre_col[r_pre].GetOptimalSpeed() + curr_a * unit_t_ <
              -kDoubleEpsilon &&
          cost_cr.point().s() > min_s_consider_speed) {
        continue;
      }

      // Filter out continuous-time node connection which is in collision with
      // obstacle
      // 计算与所选前一时刻点的连线是否会与obstacle的STBoundary碰撞, 如果碰撞, 则剔除
      if (CheckOverlapOnDpStGraph(st_graph_data_.st_boundaries(), cost_cr,
                                  pre_col[r_pre])) {
        continue;
      }
      // 计算当前的speed_limit
      curr_speed_limit =
          std::fmin(curr_speed_limit, speed_limit_by_index_[r_pre]);
      const double cost = cost_cr.obstacle_cost() +
                          cost_cr.spatial_potential_cost() +
                          pre_col[r_pre].total_cost() +
                          CalculateEdgeCostForThirdCol(
                              r, r_pre, curr_speed_limit, cruise_speed);
      // 选择cost最小的, 
      // 计算并更新设置total_cost, 当前点的PreCost, OptimalSpeed (估算出的当前速度)
      if (cost < cost_cr.total_cost()) {
        cost_cr.SetTotalCost(cost);
        cost_cr.SetPrePoint(pre_col[r_pre]);
        cost_cr.SetOptimalSpeed(pre_col[r_pre].GetOptimalSpeed() +
                                curr_a * unit_t_);
      }
    }
    return;
  }

  // 如果当前点位于第四列及以后各列
  // 依次遍历依照最大速度计算出的 前一时刻可能的位置范围内的 采样点
  for (uint32_t i = 0; i < r_pre_size; ++i) {
    uint32_t r_pre = r - i;
    if (std::isinf(pre_col[r_pre].total_cost()) ||
        pre_col[r_pre].pre_point() == nullptr) {
      continue;
    }
    // Use curr_v = (point.s - pre_point.s) / unit_t as current v
    // Use pre_v = (pre_point.s - prepre_point.s) / unit_t as previous v
    // Current acc estimate: curr_a = (curr_v - pre_v) / unit_t
    // = (point.s + prepre_point.s - 2 * pre_point.s) / (unit_t * unit_t)
    // 依照当前与前一时刻的采样点位置, 以及预估的前一时刻点的速度, 预估当前两点之间的加速度
    const double curr_a =
        2 *
        ((cost_cr.point().s() - pre_col[r_pre].point().s()) / unit_t_ -
         pre_col[r_pre].GetOptimalSpeed()) /
        unit_t_;
    // 如果加速度超过最大最小加速度范围
    if (curr_a > max_acceleration_ || curr_a < max_deceleration_) {
      continue;
    }

    // 如果估算出的当前速度为负, 且自车停在min_s_consider_speed以外(正常情况下, ST图不考虑负的车速), 则跳过
    if (pre_col[r_pre].GetOptimalSpeed() + curr_a * unit_t_ < -kDoubleEpsilon &&
        cost_cr.point().s() > min_s_consider_speed) {
      continue;
    }

    // 如果当前与前一时刻采样点的连线与obstacle的STBoundary有碰撞, 则跳过
    if (CheckOverlapOnDpStGraph(st_graph_data_.st_boundaries(), cost_cr,
                                pre_col[r_pre])) {
      continue;
    }

    // 找到pre-point对应的pre-pre-point
    uint32_t r_prepre = pre_col[r_pre].pre_point()->index_s();
    const StGraphPoint& prepre_graph_point = cost_table_[c - 2][r_prepre];
    if (std::isinf(prepre_graph_point.total_cost())) {
      continue;
    }

    if (!prepre_graph_point.pre_point()) {
      continue;
    }
    // 寻找pre-pre-pre point
    const STPoint& triple_pre_point = prepre_graph_point.pre_point()->point();
    const STPoint& prepre_point = prepre_graph_point.point();
    const STPoint& pre_point = pre_col[r_pre].point();
    const STPoint& curr_point = cost_cr.point();
    curr_speed_limit =
        std::fmin(curr_speed_limit, speed_limit_by_index_[r_pre]);
    // 基于四个点<pre_pre_pre_point, pre_pre_point, pre_point, point>计算总的cost
    double cost = cost_cr.obstacle_cost() + cost_cr.spatial_potential_cost() +
                  pre_col[r_pre].total_cost() +
                  CalculateEdgeCost(triple_pre_point, prepre_point, pre_point,
                                    curr_point, curr_speed_limit, cruise_speed);

    // 寻找最终cost最小的pre-point, 并设置当前采样点的total_cost, pre-point, optimal_speed
    if (cost < cost_cr.total_cost()) {
      cost_cr.SetTotalCost(cost);
      cost_cr.SetPrePoint(pre_col[r_pre]);
      cost_cr.SetOptimalSpeed(pre_col[r_pre].GetOptimalSpeed() +
                              curr_a * unit_t_);
    }
  }
}

/* 返回DP的结果speed_data: vector<<t, s, v, a, da>>
   -> 基于cost_table_中每个采样点的cost以及对应的pre-point, 寻找最佳end_point, 反向递推出整条链路
   -> 将链路中的每个点的<t, s, v>信息推入到speed_data中 */
Status GriddedPathTimeGraph::RetrieveSpeedProfile(SpeedData* const speed_data) {
  double min_cost = std::numeric_limits<double>::infinity();
  const StGraphPoint* best_end_point = nullptr;
  // 搜索cost_table_最后一行的每一列, 寻找cost最低的点, 作为最后一行的best_end_point, 同时记录该最小的min_cost
  for (const StGraphPoint& cur_point : cost_table_.back()) {
    if (!std::isinf(cur_point.total_cost()) &&
        cur_point.total_cost() < min_cost) {
      best_end_point = &cur_point;
      min_cost = cur_point.total_cost();
    }
  }

  // 搜索最后一列的每一行, 更新cost最低的点, 作为最终的终点best_end_point, 同时记录该最小的min_cost
  for (const auto& row : cost_table_) {
    const StGraphPoint& cur_point = row.back();
    if (!std::isinf(cur_point.total_cost()) &&
        cur_point.total_cost() < min_cost) {
      best_end_point = &cur_point;
      min_cost = cur_point.total_cost();
    }
  }

  // 如果搜索不到best_end_point, 则DP生成有问题, 返回错误
  if (best_end_point == nullptr) {
    const std::string msg = "Fail to find the best feasible trajectory.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // 以best_end_point为反向搜索起点, 通过更新cost_table_时得到的每个点的pre-point
  // 递归搜索整条链路直到起点
  // 将该条链路上的每一个点的(t, s)更新到speed_profile中
  std::vector<SpeedPoint> speed_profile;
  const StGraphPoint* cur_point = best_end_point;
  while (cur_point != nullptr) {
    ADEBUG << "Time: " << cur_point->point().t();
    ADEBUG << "S: " << cur_point->point().s();
    ADEBUG << "V: " << cur_point->GetOptimalSpeed();
    SpeedPoint speed_point;
    speed_point.set_s(cur_point->point().s());
    speed_point.set_t(cur_point->point().t());
    speed_profile.push_back(speed_point);
    cur_point = cur_point->pre_point();
  }
  // 将speed_profile反向, 则得到正向的speed_profile
  std::reverse(speed_profile.begin(), speed_profile.end());

  static constexpr double kEpsilon = std::numeric_limits<double>::epsilon();
  // 如果起点不在原点(0, 0), 则DP生成有问题, 返回错误
  if (speed_profile.front().t() > kEpsilon ||
      speed_profile.front().s() > kEpsilon) {
    const std::string msg = "Fail to retrieve speed profile.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // 基于speed_profile每个点的(t, s), 计算并设置对应的v
  for (size_t i = 0; i + 1 < speed_profile.size(); ++i) {
    const double v = (speed_profile[i + 1].s() - speed_profile[i].s()) /
                     (speed_profile[i + 1].t() - speed_profile[i].t() + 1e-3);
    speed_profile[i].set_v(v);
  }

  // 将speed_profile设置到speed_data中, 返回
  *speed_data = SpeedData(speed_profile);
  return Status::OK();
}

/* 计算其他列的cost: 计算其他列的speed, accel, jerk, 并计算对应的cost */
double GriddedPathTimeGraph::CalculateEdgeCost(
    const STPoint& first, const STPoint& second, const STPoint& third,
    const STPoint& forth, const double speed_limit, const double cruise_speed) {
  return dp_st_cost_.GetSpeedCost(third, forth, speed_limit, cruise_speed) +
         dp_st_cost_.GetAccelCostByThreePoints(second, third, forth) +
         dp_st_cost_.GetJerkCostByFourPoints(first, second, third, forth);
}

/* 计算第2列的cost: 计算第2列的speed, accel, jerk, 并计算对应的cost */
double GriddedPathTimeGraph::CalculateEdgeCostForSecondCol(
    const uint32_t row, const double speed_limit, const double cruise_speed) {
  double init_speed = init_point_.v();
  double init_acc = init_point_.a();
  const STPoint& pre_point = cost_table_[0][0].point();
  const STPoint& curr_point = cost_table_[1][row].point();
  // 计算从起点到当前点的speed cost, accel cost, jerk cost
  return dp_st_cost_.GetSpeedCost(pre_point, curr_point, speed_limit,
                                  cruise_speed) +
         dp_st_cost_.GetAccelCostByTwoPoints(init_speed, pre_point,
                                             curr_point) +
         dp_st_cost_.GetJerkCostByTwoPoints(init_speed, init_acc, pre_point,
                                            curr_point);
}

/* 计算第3列的cost: 计算第3列的speed, accel, jerk, 并计算对应的cost */
double GriddedPathTimeGraph::CalculateEdgeCostForThirdCol(
    const uint32_t curr_row, const uint32_t pre_row, const double speed_limit,
    const double cruise_speed) {
  double init_speed = init_point_.v();
  const STPoint& first = cost_table_[0][0].point();
  const STPoint& second = cost_table_[1][pre_row].point();
  const STPoint& third = cost_table_[2][curr_row].point();
  return dp_st_cost_.GetSpeedCost(second, third, speed_limit, cruise_speed) +
         dp_st_cost_.GetAccelCostByThreePoints(first, second, third) +
         dp_st_cost_.GetJerkCostByThreePoints(init_speed, first, second, third);
}

}  // namespace planning
}  // namespace apollo
