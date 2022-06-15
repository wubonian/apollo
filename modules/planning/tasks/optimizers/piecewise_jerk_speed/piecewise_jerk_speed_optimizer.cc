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

/**
 * @file piecewise_jerk_fallback_speed.cc
 **/

#include "modules/planning/tasks/optimizers/piecewise_jerk_speed/piecewise_jerk_speed_optimizer.h"

#include <string>
#include <utility>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/speed_profile_generator.h"
#include "modules/planning/common/st_graph_data.h"
#include "modules/planning/math/piecewise_jerk/piecewise_jerk_speed_problem.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::PathPoint;
using apollo::common::SpeedPoint;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;

PiecewiseJerkSpeedOptimizer::PiecewiseJerkSpeedOptimizer(
    const TaskConfig& config)
    : SpeedOptimizer(config) {
  ACHECK(config_.has_piecewise_jerk_speed_optimizer_config());
}

/* 速度QP规划主函数:
   -> 1/2*X*P*X + q*X = weight_x_ref_*(x-x_ref)^2 + weight_dx_ref*(dx-dx_ref)^2 + 
                        weight_ddx_*ddx^2 + weight_dddx_*(ddx[i+1]^2-ddx[i]^2)/dt^2
   -> 位置边界由各obstacle的STBoundary以及decision得到
   -> 速度边界由[0, speed_limit]得到
   -> acceleration/jerk边界由常数值设定 */
Status PiecewiseJerkSpeedOptimizer::Process(const PathData& path_data,
                                            const TrajectoryPoint& init_point,
                                            SpeedData* const speed_data) {
  // 计算当前自车是否抵达终点
  if (reference_line_info_->ReachedDestination()) {
    return Status::OK();
  }

  ACHECK(speed_data != nullptr);
  // 提取PathTimeHeuristicOptimizer中速度DP规划的结果 (time采样时间1s)
  SpeedData reference_speed_data = *speed_data;

  if (path_data.discretized_path().empty()) {
    const std::string msg = "Empty path data";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // 提取SpeedBoundsDecider中更新的ST图数据StGraphData st_graph_data
  StGraphData& st_graph_data = *reference_line_info_->mutable_st_graph_data();

  const auto& veh_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();

  std::array<double, 3> init_s = {0.0, st_graph_data.init_point().v(),
                                  st_graph_data.init_point().a()};
  // 时间采样的间隔
  double delta_t = 0.1;
  double total_length = st_graph_data.path_length();
  double total_time = st_graph_data.total_time_by_conf();
  // 时间采样点的个数
  int num_of_knots = static_cast<int>(total_time / delta_t) + 1;

  PiecewiseJerkSpeedProblem piecewise_jerk_problem(num_of_knots, delta_t,
                                                   init_s);

  const auto& config = config_.piecewise_jerk_speed_optimizer_config();
  // 设置piecewise_jerk_problem的weight, x/dx/ddx/dddx的bound, dx_ref
  piecewise_jerk_problem.set_weight_ddx(config.acc_weight());
  piecewise_jerk_problem.set_weight_dddx(config.jerk_weight());

  piecewise_jerk_problem.set_x_bounds(0.0, total_length);
  piecewise_jerk_problem.set_dx_bounds(
      0.0, std::fmax(FLAGS_planning_upper_speed_limit,
                     st_graph_data.init_point().v()));
  piecewise_jerk_problem.set_ddx_bounds(veh_param.max_deceleration(),
                                        veh_param.max_acceleration());
  piecewise_jerk_problem.set_dddx_bound(FLAGS_longitudinal_jerk_lower_bound,
                                        FLAGS_longitudinal_jerk_upper_bound);
  // 设置参考速度与权重
  piecewise_jerk_problem.set_dx_ref(config.ref_v_weight(),
                                    reference_line_info_->GetCruiseSpeed());

  // Update STBoundary
  // 基于obstacle的STBoundary, 计算并提取每个采样时刻下, 自车的可通行区域s_bounds
  // 将s_bounds设置为QP问题的x_bounds_
  std::vector<std::pair<double, double>> s_bounds;
  for (int i = 0; i < num_of_knots; ++i) {
    double curr_t = i * delta_t;    // 当前采样时间
    double s_lower_bound = 0.0;
    double s_upper_bound = total_length;
    // 遍历st_graph_data中所有的目标, 计算当前采样时刻下, 自车可通行区域范围<s_lower_bound, s_upper_bound>
    for (const STBoundary* boundary : st_graph_data.st_boundaries()) {
      double s_lower = 0.0;
      double s_upper = 0.0;
      // 计算当前obstacle的决策下, 自车可通行的区域<s_lower, s_upper>
      if (!boundary->GetUnblockSRange(curr_t, &s_upper, &s_lower)) {
        continue;
      }
      switch (boundary->boundary_type()) {
        case STBoundary::BoundaryType::STOP:
        case STBoundary::BoundaryType::YIELD:
          s_upper_bound = std::fmin(s_upper_bound, s_upper);
          break;
        case STBoundary::BoundaryType::FOLLOW:
          // TODO(Hongyi): unify follow buffer on decision side
          s_upper_bound = std::fmin(s_upper_bound, s_upper - 8.0);
          break;
        case STBoundary::BoundaryType::OVERTAKE:
          s_lower_bound = std::fmax(s_lower_bound, s_lower);
          break;
        default:
          break;
      }
    }
    
    if (s_lower_bound > s_upper_bound) {
      const std::string msg =
          "s_lower_bound larger than s_upper_bound on STGraph";
      AERROR << msg;
      speed_data->clear();
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }

    s_bounds.emplace_back(s_lower_bound, s_upper_bound);
  }
  // 设置piecewise_jerk_problem的x_bounds
  piecewise_jerk_problem.set_x_bounds(std::move(s_bounds));

  // Update SpeedBoundary and ref_s
  // 以速度DP规划结果reference_speed_data为参考线
  std::vector<double> x_ref;      // 提取参考线每个采样时刻的s位置x_ref
  std::vector<double> penalty_dx;           // 速度惩罚, 该点处的曲率越大, QP中对应的P矩阵速度权重越大, curvature * weight
  std::vector<std::pair<double, double>> s_dot_bounds;      // 参考线每个点的speed_limit, 作为spd_bound
  // 提取st_graph_data中(map, lat_accel, nudge)产生的speed_limit
  const SpeedLimit& speed_limit = st_graph_data.speed_limit();
  for (int i = 0; i < num_of_knots; ++i) {
    double curr_t = i * delta_t;
    // get path_s
    SpeedPoint sp;
    reference_speed_data.EvaluateByTime(curr_t, &sp);
    const double path_s = sp.s();
    x_ref.emplace_back(path_s);   // 参考线reference_speed_data的s位置
    // get curvature
    PathPoint path_point = path_data.GetPathPointWithPathS(path_s);
    penalty_dx.push_back(std::fabs(path_point.kappa()) *
                         config.kappa_penalty_weight());    // curvature * weight
    // get v_upper_bound
    const double v_lower_bound = 0.0;
    double v_upper_bound = FLAGS_planning_upper_speed_limit;
    v_upper_bound = speed_limit.GetSpeedLimitByS(path_s);     // 提取speed_limit上限
    s_dot_bounds.emplace_back(v_lower_bound, std::fmax(v_upper_bound, 0.0));
  }
  // 设置piecewise_jerk_problem的x_ref, dx_bounds, penalty_dx
  piecewise_jerk_problem.set_x_ref(config.ref_s_weight(), std::move(x_ref));
  piecewise_jerk_problem.set_penalty_dx(penalty_dx);
  piecewise_jerk_problem.set_dx_bounds(std::move(s_dot_bounds));

  // Solve the problem
  // 构建速度QP规划问题, 并求解
  if (!piecewise_jerk_problem.Optimize()) {
    const std::string msg = "Piecewise jerk speed optimizer failed!";
    AERROR << msg;
    speed_data->clear();
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // Extract output
  // 提取速度规划的结果为目标s, ds, dds
  const std::vector<double>& s = piecewise_jerk_problem.opt_x();
  const std::vector<double>& ds = piecewise_jerk_problem.opt_dx();
  const std::vector<double>& dds = piecewise_jerk_problem.opt_ddx();
  for (int i = 0; i < num_of_knots; ++i) {
    ADEBUG << "For t[" << i * delta_t << "], s = " << s[i] << ", v = " << ds[i]
           << ", a = " << dds[i];
  }
  // 将速度规划的QP结果设置到speed_data中
  speed_data->clear();
  speed_data->AppendSpeedPoint(s[0], 0.0, ds[0], dds[0], 0.0);
  for (int i = 1; i < num_of_knots; ++i) {
    // Avoid the very last points when already stopped
    if (ds[i] <= 0.0) {
      break;
    }
    speed_data->AppendSpeedPoint(s[i], delta_t * i, ds[i], dds[i],
                                 (dds[i] - dds[i - 1]) / delta_t);
  }
  SpeedProfileGenerator::FillEnoughSpeedPoints(speed_data);
  RecordDebugInfo(*speed_data, st_graph_data.mutable_st_graph_debug());
  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
