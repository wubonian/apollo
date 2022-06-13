/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/tasks/deciders/speed_bounds_decider/speed_bounds_decider.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/path/path_data.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/st_graph_data.h"
#include "modules/planning/common/util/common.h"
#include "modules/planning/tasks/deciders/speed_bounds_decider/speed_limit_decider.h"
#include "modules/planning/tasks/deciders/speed_bounds_decider/st_boundary_mapper.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::planning_internal::StGraphBoundaryDebug;
using apollo::planning_internal::STGraphDebug;

SpeedBoundsDecider::SpeedBoundsDecider(
    const TaskConfig &config,
    const std::shared_ptr<DependencyInjector> &injector)
    : Decider(config, injector) {
  ACHECK(config.has_speed_bounds_decider_config());
  speed_bounds_config_ = config.speed_bounds_decider_config();
}

/* SpeedBoundsDecider主执行函数:
   -> 根据path_decision的决策, 将每个obstacle的STBoundary返回到boundaries中
   -> 计算ST图中, 每个位置点s处的速度限制speed_limit
   -> 将STBoundary与speed_limit设置到reference_lint_info的StGraphData中 */
Status SpeedBoundsDecider::Process(
    Frame *const frame, ReferenceLineInfo *const reference_line_info) {
  // retrieve data from frame and reference_line_info
  const PathData &path_data = reference_line_info->path_data();
  const TrajectoryPoint &init_point = frame->PlanningStartPoint();
  const ReferenceLine &reference_line = reference_line_info->reference_line();
  PathDecision *const path_decision = reference_line_info->path_decision();

  // 1. Map obstacles into st graph
  auto time1 = std::chrono::system_clock::now();
  STBoundaryMapper boundary_mapper(
      speed_bounds_config_, reference_line, path_data,
      path_data.discretized_path().Length(), speed_bounds_config_.total_time(),
      injector_);
  // 如果该值为false, 则不使用STBoundsDecider的结果, 清除之前计算的st_boundary
  if (!FLAGS_use_st_drivable_boundary) {
    path_decision->EraseStBoundaries();
  }

  // 更新计算每个obstacle的STBoundary以及对应的BoundaryType
  // 基于path_decision的结果, 生成每个obstacle的ST Boundary以及对应的BoundaryType, 
  // 将结果提取到boundaries, 并更新到reference_line_info中的StGraphData中
  // option 1: 直接使用StBoundsDecider决策时生成的每个obstacle的ST Boundary
  // option 2: 基于决策, 重新计算每个obstacle的STBoundary
  if (boundary_mapper.ComputeSTBoundary(path_decision).code() ==
      ErrorCode::PLANNING_ERROR) {
    const std::string msg = "Mapping obstacle failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  auto time2 = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = time2 - time1;
  ADEBUG << "Time for ST Boundary Mapping = " << diff.count() * 1000
         << " msec.";

  std::vector<const STBoundary *> boundaries;
  // 剔除掉BoundaryType为KEEP_CLEAR的obstacle, 以及对应的boundary
  // 将path_decision中的ST boundary导出到boundaries中
  for (auto *obstacle : path_decision->obstacles().Items()) {
    const auto &id = obstacle->Id();
    const auto &st_boundary = obstacle->path_st_boundary();
    if (!st_boundary.IsEmpty()) {
      if (st_boundary.boundary_type() == STBoundary::BoundaryType::KEEP_CLEAR) {
        path_decision->Find(id)->SetBlockingObstacle(false);
      } else {
        path_decision->Find(id)->SetBlockingObstacle(true);
      }
      boundaries.push_back(&st_boundary);
    }
  }

  // 返回当前时刻最近障碍物的s距离作为fall-back distance (安全距离)
  const double min_s_on_st_boundaries = SetSpeedFallbackDistance(path_decision);

  // 2. Create speed limit along path
  SpeedLimitDecider speed_limit_decider(speed_bounds_config_, reference_line,
                                        path_data);

  SpeedLimit speed_limit;
  // 基于地图, 道路曲率, nudge, 计算path上每个点的speed limit信息
  // 后续将计算得到的speed limit信息更新到reference_line_info的StGraphData中
  if (!speed_limit_decider
           .GetSpeedLimits(path_decision->obstacles(), &speed_limit)
           .ok()) {
    const std::string msg = "Getting speed limits failed!";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // 3. Get path_length as s axis search bound in st graph
  const double path_data_length = path_data.discretized_path().Length();

  // 4. Get time duration as t axis search bound in st graph
  const double total_time_by_conf = speed_bounds_config_.total_time();

  // Load generated st graph data back to frame
  StGraphData *st_graph_data = reference_line_info_->mutable_st_graph_data();

  // Add a st_graph debug info and save the pointer to st_graph_data for
  // optimizer logging
  auto *debug = reference_line_info_->mutable_debug();
  STGraphDebug *st_graph_debug = debug->mutable_planning_data()->add_st_graph();

  // 将结果设置到st_graph_data的内部变量中
  st_graph_data->LoadData(boundaries, min_s_on_st_boundaries, init_point,
                          speed_limit, reference_line_info->GetCruiseSpeed(),
                          path_data_length, total_time_by_conf, st_graph_debug);

  // Create and record st_graph debug info
  RecordSTGraphDebug(*st_graph_data, st_graph_debug);

  return Status::OK();
}

/* 返回当前时刻, 最近的障碍物距离作为fall-back distance (自车预留的安全区间) */
double SpeedBoundsDecider::SetSpeedFallbackDistance(
    PathDecision *const path_decision) {
  // Set min_s_on_st_boundaries to guide speed fallback.
  static constexpr double kEpsilon = 1.0e-6;
  double min_s_non_reverse = std::numeric_limits<double>::infinity();     // 表示同向dynamic obstacle在ST图投影的最近点
  double min_s_reverse = std::numeric_limits<double>::infinity();       // 表示对向dynamic obstacle在ST图投影的最近点

  // 遍历每一个path_decision中的obstacle
  for (auto *obstacle : path_decision->obstacles().Items()) {
    const auto &st_boundary = obstacle->path_st_boundary();

    if (st_boundary.IsEmpty()) {
      continue;
    }

    const auto left_bottom_point_s = st_boundary.bottom_left_point().s();
    const auto right_bottom_point_s = st_boundary.bottom_right_point().s();
    const auto lowest_s = std::min(left_bottom_point_s, right_bottom_point_s);

    if (left_bottom_point_s - right_bottom_point_s > kEpsilon) {
      if (min_s_reverse > lowest_s) {
        min_s_reverse = lowest_s;
      }
    } else if (min_s_non_reverse > lowest_s) {
      min_s_non_reverse = lowest_s;
    }
  }

  min_s_reverse = std::max(min_s_reverse, 0.0);
  min_s_non_reverse = std::max(min_s_non_reverse, 0.0);

  // 如果同向的最小s 大于 对向的最小s, 则返回0.0, 否则返回同向的最小s
  return min_s_non_reverse > min_s_reverse ? 0.0 : min_s_non_reverse;
}

/* 导入debug信息 */
void SpeedBoundsDecider::RecordSTGraphDebug(
    const StGraphData &st_graph_data, STGraphDebug *st_graph_debug) const {
  if (!FLAGS_enable_record_debug || !st_graph_debug) {
    ADEBUG << "Skip record debug info";
    return;
  }

  for (const auto &boundary : st_graph_data.st_boundaries()) {
    auto boundary_debug = st_graph_debug->add_boundary();
    boundary_debug->set_name(boundary->id());
    switch (boundary->boundary_type()) {
      case STBoundary::BoundaryType::FOLLOW:
        boundary_debug->set_type(StGraphBoundaryDebug::ST_BOUNDARY_TYPE_FOLLOW);
        break;
      case STBoundary::BoundaryType::OVERTAKE:
        boundary_debug->set_type(
            StGraphBoundaryDebug::ST_BOUNDARY_TYPE_OVERTAKE);
        break;
      case STBoundary::BoundaryType::STOP:
        boundary_debug->set_type(StGraphBoundaryDebug::ST_BOUNDARY_TYPE_STOP);
        break;
      case STBoundary::BoundaryType::UNKNOWN:
        boundary_debug->set_type(
            StGraphBoundaryDebug::ST_BOUNDARY_TYPE_UNKNOWN);
        break;
      case STBoundary::BoundaryType::YIELD:
        boundary_debug->set_type(StGraphBoundaryDebug::ST_BOUNDARY_TYPE_YIELD);
        break;
      case STBoundary::BoundaryType::KEEP_CLEAR:
        boundary_debug->set_type(
            StGraphBoundaryDebug::ST_BOUNDARY_TYPE_KEEP_CLEAR);
        break;
    }

    for (const auto &point : boundary->points()) {
      auto point_debug = boundary_debug->add_point();
      point_debug->set_t(point.x());
      point_debug->set_s(point.y());
    }
  }

  for (const auto &point : st_graph_data.speed_limit().speed_limit_points()) {
    common::SpeedPoint *speed_point = st_graph_debug->add_speed_limit();
    speed_point->set_s(point.first);
    speed_point->set_v(point.second);
  }
}

}  // namespace planning
}  // namespace apollo
