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

#include "modules/planning/tasks/deciders/speed_bounds_decider/st_boundary_mapper.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/proto/decision.pb.h"

#include "cyber/common/log.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/line_segment2d.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/util/string_util.h"
#include "modules/common/util/util.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::PathPoint;
using apollo::common::Status;
using apollo::common::math::Box2d;
using apollo::common::math::Vec2d;

/* 初始化STBoundaryMapper各内部变量 */
STBoundaryMapper::STBoundaryMapper(
    const SpeedBoundsDeciderConfig& config, const ReferenceLine& reference_line,
    const PathData& path_data, const double planning_distance,
    const double planning_time,
    const std::shared_ptr<DependencyInjector>& injector)
    : speed_bounds_config_(config),
      reference_line_(reference_line),
      path_data_(path_data),
      vehicle_param_(common::VehicleConfigHelper::GetConfig().vehicle_param()),
      planning_max_distance_(planning_distance),
      planning_max_time_(planning_time),
      injector_(injector) {}

/* 遍历path_decision中每个obstacle, 更新它的STBoundary:
   -> use_st_drivable_boundary = true: 直接沿用StBoundsDecider决策时生成的STBoundary
   -> 基于决策结果, 重新生成每个obstacle的STBoundary */
Status STBoundaryMapper::ComputeSTBoundary(PathDecision* path_decision) const {
  // Sanity checks.
  CHECK_GT(planning_max_time_, 0.0);
  if (path_data_.discretized_path().size() < 2) {
    AERROR << "Fail to get params because of too few path points. path points "
              "size: "
           << path_data_.discretized_path().size() << ".";
    return Status(ErrorCode::PLANNING_ERROR,
                  "Fail to get params because of too few path points");
  }

  // Go through every obstacle.
  Obstacle* stop_obstacle = nullptr;
  ObjectDecisionType stop_decision;
  double min_stop_s = std::numeric_limits<double>::max();
  // 遍历每一个障碍物obstacle
  for (const auto* ptr_obstacle_item : path_decision->obstacles().Items()) {
    Obstacle* ptr_obstacle = path_decision->Find(ptr_obstacle_item->Id());
    ACHECK(ptr_obstacle != nullptr);

    // If no longitudinal decision has been made, then plot it onto ST-graph.
    
    // 将没有longitudinal decision的目标映射到ST图上, 但没有BoundaryType
    // 如果FLAGS_use_st_drivable_boundary置位, 则使用STBoundsDecider的结果, 这里调用的ComputeSTBoundary不做任何处理
    if (!ptr_obstacle->HasLongitudinalDecision()) {
      ComputeSTBoundary(ptr_obstacle);
      continue;
    }

    // If there is a longitudinal decision, then fine-tune boundary.
    const auto& decision = ptr_obstacle->LongitudinalDecision();
    // 如果目标有stop decision, 递归筛选出最近的stop decision以及对应的obstacle
    if (decision.has_stop()) {
      // 1. Store the closest stop fence info.
      // TODO(all): store ref. s value in stop decision; refine the code then.
      common::SLPoint stop_sl_point;
      // 计算stop point在SL坐标
      reference_line_.XYToSL(decision.stop().stop_point(), &stop_sl_point);
      const double stop_s = stop_sl_point.s();
      // 寻找距离自车最近的stop_decision
      if (stop_s < min_stop_s) {
        stop_obstacle = ptr_obstacle;   // 最近有stop decision的obstacle
        min_stop_s = stop_s;      // 最近的stop decision的距离
        stop_decision = decision;     // 最近的stop decision的状态
      }
    }
     
    // 如果目标已经有follow, overtake, yield这三种decision
    else if (decision.has_follow() || decision.has_overtake() ||
               decision.has_yield()) {
      // 2. Depending on the longitudinal overtake/yield decision,
      //    fine-tune the upper/lower st-boundary of related obstacles.
      // 更新path_decision中每个obstacle的STBoundary
      ComputeSTBoundaryWithDecision(ptr_obstacle, decision);
    } else if (!decision.has_ignore()) {
      // 3. Ignore those unrelated obstacles.
      AWARN << "No mapping for decision: " << decision.DebugString();
    }
  }
  // 如果找到最近的stop decision对应的静止obstacle, 将其设置在ST图中, 对应的BoundaryType设置为STOP
  if (stop_obstacle) {
    bool success = MapStopDecision(stop_obstacle, stop_decision);
    if (!success) {
      const std::string msg = "Fail to MapStopDecision.";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
  }

  return Status::OK();
}

/* 将stop decision设置到ST Boundary中, BoundaryType为STOP */
bool STBoundaryMapper::MapStopDecision(
    Obstacle* stop_obstacle, const ObjectDecisionType& stop_decision) const {
  DCHECK(stop_decision.has_stop()) << "Must have stop decision";
  // 提取stop decision的stop point
  common::SLPoint stop_sl_point;
  reference_line_.XYToSL(stop_decision.stop().stop_point(), &stop_sl_point);

  double st_stop_s = 0.0;     // 
  const double stop_ref_s =
      stop_sl_point.s() - vehicle_param_.front_edge_to_center();    // stop point在reference-line下距离自车的纵向距离

  if (stop_ref_s > path_data_.frenet_frame_path().back().s()) {
    st_stop_s = path_data_.discretized_path().back().s() +
                (stop_ref_s - path_data_.frenet_frame_path().back().s());
  } else {
    PathPoint stop_point;
    if (!path_data_.GetPathPointWithRefS(stop_ref_s, &stop_point)) {
      return false;
    }
    st_stop_s = stop_point.s();
  }

  const double s_min = std::fmax(0.0, st_stop_s);
  const double s_max = std::fmax(
      s_min, std::fmax(planning_max_distance_, reference_line_.Length()));

  // 基于stop_s, 设置ST坐标系下的边界, boundary type为STOP
  std::vector<std::pair<STPoint, STPoint>> point_pairs;
  point_pairs.emplace_back(STPoint(s_min, 0.0), STPoint(s_max, 0.0));
  point_pairs.emplace_back(
      STPoint(s_min, planning_max_time_),
      STPoint(s_max + speed_bounds_config_.boundary_buffer(),
              planning_max_time_));
  auto boundary = STBoundary(point_pairs);
  boundary.SetBoundaryType(STBoundary::BoundaryType::STOP);
  boundary.SetCharacteristicLength(speed_bounds_config_.boundary_buffer());
  boundary.set_id(stop_obstacle->Id());
  stop_obstacle->set_path_st_boundary(boundary);
  return true;
}

/* 对于没有做过纵向决策的obstacle, 将其投影到ST平面上, 生成对应的boundary
   -> 对于新生成的Boundary, 没有对应的BoundaryType */
void STBoundaryMapper::ComputeSTBoundary(Obstacle* obstacle) const {
  // 如果使用STBoundsDecider的决策和生成的Boundary, 则直接返回
  if (FLAGS_use_st_drivable_boundary) {
    return;
  }
  std::vector<STPoint> lower_points;
  std::vector<STPoint> upper_points;

  // 计算自车SL轨迹与目标预测轨迹的overlap部分, 并将该部分投影到ST平面上, 生成ST平面的上下边界点lower_points & upper_points
  if (!GetOverlapBoundaryPoints(path_data_.discretized_path(), *obstacle,
                                &upper_points, &lower_points)) {
    return;
  }

  // 对目标生成对应的STBoundary边界, 没有BoundaryType
  auto boundary = STBoundary::CreateInstance(lower_points, upper_points);
  boundary.set_id(obstacle->Id());

  // TODO(all): potential bug here.
  const auto& prev_st_boundary = obstacle->path_st_boundary();
  const auto& ref_line_st_boundary = obstacle->reference_line_st_boundary();
  // 如果之前存在path_st_boundary, 或者ref_line_st_boundary, 设置对应的BoundaryType
  if (!prev_st_boundary.IsEmpty()) {
    boundary.SetBoundaryType(prev_st_boundary.boundary_type());
  } else if (!ref_line_st_boundary.IsEmpty()) {
    boundary.SetBoundaryType(ref_line_st_boundary.boundary_type());
  }

  obstacle->set_path_st_boundary(boundary);
}

/* 检查obstacle的预测轨迹与path_points的overlap, 并产生ST图上重合部分的上下边界lower/upper_points
   -> 在计算过程中, 对静态障碍物与动态障碍物区分处理 */
bool STBoundaryMapper::GetOverlapBoundaryPoints(
    const std::vector<PathPoint>& path_points, const Obstacle& obstacle,
    std::vector<STPoint>* upper_points,
    std::vector<STPoint>* lower_points) const {
  // Sanity checks.
  DCHECK(upper_points->empty());
  DCHECK(lower_points->empty());
  if (path_points.empty()) {
    AERROR << "No points in path_data_.discretized_path().";
    return false;
  }

  const auto* planning_status = injector_->planning_context()
                                    ->mutable_planning_status()
                                    ->mutable_change_lane();

  // 在lane change的时候, 设定较小的l_buffer
  double l_buffer =
      planning_status->status() == ChangeLaneStatus::IN_CHANGE_LANE
          ? FLAGS_lane_change_obstacle_nudge_l_buffer
          : FLAGS_nonstatic_obstacle_nudge_l_buffer;

  // Draw the given obstacle on the ST-graph.
  // 提取目标的预测轨迹
  const auto& trajectory = obstacle.Trajectory();
  // 如果目标为静止障碍物:
  // 如果obstacle的目标轨迹为空(静态障碍物), 则寻找SL坐标系下会碰撞的点, 
  // 在ST图的所有时间范围内, 对该位置设置lower与upper边界
  if (trajectory.trajectory_point().empty()) {
    // For those with no predicted trajectories, just map the obstacle's
    // current position to ST-graph and always assume it's static.
    if (!obstacle.IsStatic()) {
      AWARN << "Non-static obstacle[" << obstacle.Id()
            << "] has NO prediction trajectory."
            << obstacle.Perception().ShortDebugString();
    }
    for (const auto& curr_point_on_path : path_points) {
      if (curr_point_on_path.s() > planning_max_distance_) {
        break;
      }

      const Box2d& obs_box = obstacle.PerceptionBoundingBox();
      // 检查自车在该path_point处是否会与该静止障碍物有overlap
      // 如果存在overlap, 则设置ST-Boundary为<low_s, 0~planning_max_time_> & <high_s, 0~planning_max_time_>
      if (CheckOverlap(curr_point_on_path, obs_box, l_buffer)) {
        // If there is overlapping, then plot it on ST-graph.
        const double backward_distance = -vehicle_param_.front_edge_to_center();
        const double forward_distance = obs_box.length();
        double low_s =
            std::fmax(0.0, curr_point_on_path.s() + backward_distance);
        double high_s = std::fmin(planning_max_distance_,
                                  curr_point_on_path.s() + forward_distance);
        // It is an unrotated rectangle appearing on the ST-graph.
        // TODO(jiacheng): reconsider the backward_distance, it might be
        // unnecessary, but forward_distance is indeed meaningful though.
        lower_points->emplace_back(low_s, 0.0);
        lower_points->emplace_back(low_s, planning_max_time_);
        upper_points->emplace_back(high_s, 0.0);
        upper_points->emplace_back(high_s, planning_max_time_);
        break;
      }
    }
  } 
  // 如果目标为动态障碍物:
  // 查找目标预测轨迹与规划轨迹在SL坐标系内重合的部分
  // 对于重合的轨迹点, 在轨迹点对应时间处, 将产生重合的纵向距离上下边界映射在ST图上
  else {
    // For those with predicted trajectories (moving obstacles):
    // 1. Subsample to reduce computation time.
    // 对自车的SL Path重采样, 最多100个path_point
    const int default_num_point = 50;
    DiscretizedPath discretized_path;
    if (path_points.size() > 2 * default_num_point) {
      const auto ratio = path_points.size() / default_num_point;
      std::vector<PathPoint> sampled_path_points;
      for (size_t i = 0; i < path_points.size(); ++i) {
        if (i % ratio == 0) {
          sampled_path_points.push_back(path_points[i]);
        }
      }
      discretized_path = DiscretizedPath(std::move(sampled_path_points));
    } else {
      discretized_path = DiscretizedPath(path_points);
    }
    // 2. Go through every point of the predicted obstacle trajectory.
    // 遍历目标预测轨迹的每一个轨迹点, 如果其在SL平面上与自车轨迹存在overlap, 则在ST图的对应时间, 设置overlap的上下边界(low_s, high_s)
    for (int i = 0; i < trajectory.trajectory_point_size(); ++i) {
      const auto& trajectory_point = trajectory.trajectory_point(i);
      const Box2d obs_box = obstacle.GetBoundingBox(trajectory_point);

      double trajectory_point_time = trajectory_point.relative_time();
      static constexpr double kNegtiveTimeThreshold = -1.0;
      if (trajectory_point_time < kNegtiveTimeThreshold) {
        continue;
      }

      const double step_length = vehicle_param_.front_edge_to_center();
      auto path_len =
          std::min(FLAGS_max_trajectory_len, discretized_path.Length());
      // Go through every point of the ADC's path.
      // 对给定的目标轨迹点, 遍历自车所有的轨迹点, 检查是否会在SL平面上发生overlap
      for (double path_s = 0.0; path_s < path_len; path_s += step_length) {
        const auto curr_adc_path_point =
            discretized_path.Evaluate(path_s + discretized_path.front().s());
        // 如果检查到自车的SL轨迹点与obstacle的当前trajectory point有重叠, 
        // 则在object trajectory对应时刻, 设置s的lower/higher bound
        if (CheckOverlap(curr_adc_path_point, obs_box, l_buffer)) {
          // Found overlap, start searching with higher resolution
          const double backward_distance = -step_length;
          const double forward_distance = vehicle_param_.length() +
                                          vehicle_param_.width() +
                                          obs_box.length() + obs_box.width();
          const double default_min_step = 0.1;  // in meters
          const double fine_tuning_step_length = std::fmin(
              default_min_step, discretized_path.Length() / default_num_point);

          bool find_low = false;
          bool find_high = false;
          // 粗略给出对应时刻点的low_s与high_s, 以这两个距离为搜索起点, 在后面的逻辑中, 查询准确的overlap边界位置
          double low_s = std::fmax(0.0, path_s + backward_distance);
          double high_s =
              std::fmin(discretized_path.Length(), path_s + forward_distance);

          // Keep shrinking by the resolution bidirectionally until finally
          // locating the tight upper and lower bounds.
          // 从low_s往上, 按照最小步长fine_tuning_step_length, 查找实际的下限碰撞距离low_s, 如果找到, 设置find_low = true
          // 从high_s往下, 按照最小步长fine_tuning_step_length, 查找实际的上限碰撞距离high_s, 如果找到, 设置find_high = true
          while (low_s < high_s) {
            if (find_low && find_high) {
              break;
            }
            if (!find_low) {
              const auto& point_low = discretized_path.Evaluate(
                  low_s + discretized_path.front().s());
              if (!CheckOverlap(point_low, obs_box, l_buffer)) {
                low_s += fine_tuning_step_length;
              } else {
                find_low = true;
              }
            }
            if (!find_high) {
              const auto& point_high = discretized_path.Evaluate(
                  high_s + discretized_path.front().s());
              if (!CheckOverlap(point_high, obs_box, l_buffer)) {
                high_s -= fine_tuning_step_length;
              } else {
                find_high = true;
              }
            }
          }

          // 如果找到实际的上下碰撞距离low_s, high_s, 则在ST map的对应时间, 
          // 设置其上下边界点[lower_points, upper_points] @ trajectory_point_time
          if (find_high && find_low) {
            lower_points->emplace_back(
                low_s - speed_bounds_config_.point_extension(),
                trajectory_point_time);
            upper_points->emplace_back(
                high_s + speed_bounds_config_.point_extension(),
                trajectory_point_time);
          }
          break;
        }
      }
    }
  }

  // Sanity checks and return.
  DCHECK_EQ(lower_points->size(), upper_points->size());
  return (lower_points->size() > 1 && upper_points->size() > 1);
}

/* 在obstacle已经有follow/yield/overtake的decision时, 更新path_decision中每个obstacle的STBoundary:
   -> option 1: 直接沿用StBoundsDecider中计算得到的每个obstacle的STBoundary
   -> option 2: 重新进行obstacle trajectory到ST图的映射, 得到每个obstacle的STBoundary */
void STBoundaryMapper::ComputeSTBoundaryWithDecision(
    Obstacle* obstacle, const ObjectDecisionType& decision) const {
  DCHECK(decision.has_follow() || decision.has_yield() ||
         decision.has_overtake())
      << "decision is " << decision.DebugString()
      << ", but it must be follow or yield or overtake.";

  std::vector<STPoint> lower_points;
  std::vector<STPoint> upper_points;

  // 该flag默认为true
  // 如果该目标已经生成了ST Bboundary, 直接使用对应的ST boundary
  if (FLAGS_use_st_drivable_boundary &&
      obstacle->is_path_st_boundary_initialized()) {
    const auto& path_st_boundary = obstacle->path_st_boundary();
    lower_points = path_st_boundary.lower_points();
    upper_points = path_st_boundary.upper_points();
  } 
  // 如果没有生成过STBoundary
  // 寻找目标的轨迹与自车的轨迹重叠的部分, 投影到ST图上, 得到对应的lower_points & upper_points
  else {
    if (!GetOverlapBoundaryPoints(path_data_.discretized_path(), *obstacle,
                                  &upper_points, &lower_points)) {
      return;
    }
  }

  // 使用lower_points & upper_points生成对应的STBoundary
  auto boundary = STBoundary::CreateInstance(lower_points, upper_points);

  // get characteristic_length and boundary_type.
  // 初始化BoundaryType为UNKNOWN
  STBoundary::BoundaryType b_type = STBoundary::BoundaryType::UNKNOWN;
  double characteristic_length = 0.0;
  // 如果之前有follow决策, 则设置BoundaryType为FOLLOW, 设置characteristic_length
  if (decision.has_follow()) {
    characteristic_length = std::fabs(decision.follow().distance_s());
    b_type = STBoundary::BoundaryType::FOLLOW;
  } 
  // 如果之前有Yield决策(减速避让), 将障碍物轨迹映射的ST图上下边界分别扩展characteristic_length距离
  // 设置BoundaryType为YIELD
  else if (decision.has_yield()) {
    characteristic_length = std::fabs(decision.yield().distance_s());
    boundary = STBoundary::CreateInstance(lower_points, upper_points)
                   .ExpandByS(characteristic_length);
    b_type = STBoundary::BoundaryType::YIELD;
  } 
  // 如果之前有Overtake决策, 则设置BoundaryType为OVERTAKE, 设置characteristic_length
  else if (decision.has_overtake()) {
    characteristic_length = std::fabs(decision.overtake().distance_s());
    b_type = STBoundary::BoundaryType::OVERTAKE;
  } else {
    DCHECK(false) << "Obj decision should be either yield or overtake: "
                  << decision.DebugString();
  }
  boundary.SetBoundaryType(b_type);
  boundary.set_id(obstacle->Id());
  boundary.SetCharacteristicLength(characteristic_length);
  obstacle->set_path_st_boundary(boundary);
}

/* 检查自车在给定path_point时, 是否会与obstacle有overlap, 自车宽度两侧各加了l_buffer的安全阈值 */
bool STBoundaryMapper::CheckOverlap(const PathPoint& path_point,
                                    const Box2d& obs_box,
                                    const double l_buffer) const {
  // Convert reference point from center of rear axis to center of ADC.
  Vec2d ego_center_map_frame((vehicle_param_.front_edge_to_center() -
                              vehicle_param_.back_edge_to_center()) *
                                 0.5,
                             (vehicle_param_.left_edge_to_center() -
                              vehicle_param_.right_edge_to_center()) *
                                 0.5);
  ego_center_map_frame.SelfRotate(path_point.theta());
  ego_center_map_frame.set_x(ego_center_map_frame.x() + path_point.x());
  ego_center_map_frame.set_y(ego_center_map_frame.y() + path_point.y());

  // Compute the ADC bounding box.
  // 生成自车在给定path_point处的SL Bounding Box
  Box2d adc_box(ego_center_map_frame, path_point.theta(),
                vehicle_param_.length(), vehicle_param_.width() + l_buffer * 2);

  // Check whether ADC bounding box overlaps with obstacle bounding box.
  // 检查自车与目标的bounding box是否存在overlap
  return obs_box.HasOverlap(adc_box);
}

}  // namespace planning
}  // namespace apollo
