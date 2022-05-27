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
 * @file piecewise_jerk_path_optimizer.cc
 **/

#include "modules/planning/tasks/optimizers/piecewise_jerk_path/piecewise_jerk_path_optimizer.h"

#include <memory>
#include <string>

#include "modules/common/math/math_utils.h"
#include "modules/common/util/point_factory.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/speed/speed_data.h"
#include "modules/planning/common/trajectory1d/piecewise_jerk_trajectory1d.h"
#include "modules/planning/math/piecewise_jerk/piecewise_jerk_path_problem.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::VehicleConfigHelper;
using apollo::common::math::Gaussian;

PiecewiseJerkPathOptimizer::PiecewiseJerkPathOptimizer(
    const TaskConfig& config,
    const std::shared_ptr<DependencyInjector>& injector)
    : PathOptimizer(config, injector) {
  ACHECK(config_.has_piecewise_jerk_path_optimizer_config());
}

/* 对于给定的每个path_boundary, 基于分段3次多项式与QP, 生成最优化的离散frenet轨迹, 并更新在reference_line_info中:
   final_path_data为对应reference_line中的path_data的指针 */
common::Status PiecewiseJerkPathOptimizer::Process(
    const SpeedData& speed_data, const ReferenceLine& reference_line,
    const common::TrajectoryPoint& init_point, const bool path_reusable,
    PathData* const final_path_data) {
  // skip piecewise_jerk_path_optimizer if reused path
  // 如果path_reusable, 则跳过横向轨迹规划的部分
  if (FLAGS_enable_skip_path_tasks && path_reusable) {
    return Status::OK();
  }

  ADEBUG << "Plan at the starting point: x = " << init_point.path_point().x()
         << ", y = " << init_point.path_point().y()
         << ", and angle = " << init_point.path_point().theta();
  
  common::TrajectoryPoint planning_start_point = init_point;
  
  if (FLAGS_use_front_axe_center_in_path_planning) {
    planning_start_point =
        InferFrontAxeCenterFromRearAxeCenter(planning_start_point);
  }

  // init_frenet_state为planning_start_point在frenet坐标系的初始坐标
  // QP优化的起始点
  const auto init_frenet_state =
      reference_line.ToFrenetFrame(planning_start_point);

  // Choose lane_change_path_config for lane-change cases
  // Otherwise, choose default_path_config for normal path planning
  const auto& config = reference_line_info_->IsChangeLanePath()
                           ? config_.piecewise_jerk_path_optimizer_config()
                                 .lane_change_path_config()
                           : config_.piecewise_jerk_path_optimizer_config()
                                 .default_path_config();

  // QP优化中, 各状态向量的权重{x, x', x'', x''', x''''}
  std::array<double, 5> w = {
      config.l_weight(),
      config.dl_weight() *
          std::fmax(init_frenet_state.first[1] * init_frenet_state.first[1],
                    5.0),
      config.ddl_weight(), config.dddl_weight(), 0.0};

  // QP优化的横向位置边界
  const auto& path_boundaries =
      reference_line_info_->GetCandidatePathBoundaries();
  ADEBUG << "There are " << path_boundaries.size() << " path boundaries.";
  const auto& reference_path_data = reference_line_info_->path_data();

  std::vector<PathData> candidate_path_data;
  // 对每一个path_boundary进行QP优化, 求出期望的横向位置状态: l, l', l''
  // 并将其转换为对应的candidate_path_data
  for (const auto& path_boundary : path_boundaries) {
    size_t path_boundary_size = path_boundary.boundary().size();

    // if the path_boundary is normal, it is possible to have less than 2 points
    // skip path boundary of this kind
    // 跳过path_boundary_size较少的regular path (非fallback path bound)
    if (path_boundary.label().find("regular") != std::string::npos &&
        path_boundary_size < 2) {
      continue;
    }

    int max_iter = 4000;
    // lower max_iter for regular/self/
    if (path_boundary.label().find("self") != std::string::npos) {
      max_iter = 4000;
    }

    CHECK_GT(path_boundary_size, 1U);

    std::vector<double> opt_l;
    std::vector<double> opt_dl;
    std::vector<double> opt_ddl;

    std::array<double, 3> end_state = {0.0, 0.0, 0.0};

    if (!FLAGS_enable_force_pull_over_open_space_parking_test) {
      // pull over scenario
      // set end lateral to be at the desired pull over destination
      const auto& pull_over_status =
          injector_->planning_context()->planning_status().pull_over();
      if (pull_over_status.has_position() &&
          pull_over_status.position().has_x() &&
          pull_over_status.position().has_y() &&
          path_boundary.label().find("pullover") != std::string::npos) {
        common::SLPoint pull_over_sl;
        reference_line.XYToSL(pull_over_status.position(), &pull_over_sl);
        end_state[0] = pull_over_sl.l();
      }
    }

    // TODO(all): double-check this;
    // final_path_data might carry info from upper stream
    // path_data对应reference_line_info中的Path_Data
    PathData path_data = *final_path_data;

    // updated cost function for path reference
    // 初始化path_reference_l为0, 每个纵向位置处的参考横向位置
    std::vector<double> path_reference_l(path_boundary_size, 0.0);
    bool is_valid_path_reference = false;
    size_t path_reference_size = reference_path_data.path_reference().size();

    // 对于regular path (非fallback path), 更新path_reference_l为上一次规划结果对应的横向位置
    if (path_boundary.label().find("regular") != std::string::npos &&
        reference_path_data.is_valid_path_reference()) {
      ADEBUG << "path label is: " << path_boundary.label();
      // when path reference is ready
      for (size_t i = 0; i < path_reference_size; ++i) {
        common::SLPoint path_reference_sl;
        reference_line.XYToSL(
            common::util::PointFactory::ToPointENU(
                reference_path_data.path_reference().at(i).x(),
                reference_path_data.path_reference().at(i).y()),
            &path_reference_sl);
        path_reference_l[i] = path_reference_sl.l();
      }
      // 将end_state[0]更新为reference path的最后一个值
      end_state[0] = path_reference_l.back();
      path_data.set_is_optimized_towards_trajectory_reference(true);
      is_valid_path_reference = true;
    }

    // ddl_bounds: 对应曲率的limit, 与自车最大方向盘转角与reference_line曲率相关
    // 曲率limit(s_dis) = reference_line曲率(s_dis) +/- 自车最大转角对应的曲率
    const auto& veh_param =
        common::VehicleConfigHelper::GetConfig().vehicle_param();
    const double lat_acc_bound =
        std::tan(veh_param.max_steer_angle() / veh_param.steer_ratio()) /
        veh_param.wheel_base();
    std::vector<std::pair<double, double>> ddl_bounds;
    for (size_t i = 0; i < path_boundary_size; ++i) {
      double s = static_cast<double>(i) * path_boundary.delta_s() +
                 path_boundary.start_s();
      double kappa = reference_line.GetNearestReferencePoint(s).kappa();
      ddl_bounds.emplace_back(-lat_acc_bound - kappa, lat_acc_bound - kappa);
    }

    // 进行path optimization: 分段三次多项式 + OSQP
    // 输入: init_state, end_state, path_reference_l, path_boundary, ddl_bounds, w, delta_s
    // 输出: opt_l, opt_dl, opt_ddl
    bool res_opt = OptimizePath(
        init_frenet_state, end_state, std::move(path_reference_l),
        path_reference_size, path_boundary.delta_s(), is_valid_path_reference,
        path_boundary.boundary(), ddl_bounds, w, max_iter, &opt_l, &opt_dl,
        &opt_ddl);

    if (res_opt) {
      // 打印debug信息
      for (size_t i = 0; i < path_boundary_size; i += 4) {
        ADEBUG << "for s[" << static_cast<double>(i) * path_boundary.delta_s()
               << "], l = " << opt_l[i] << ", dl = " << opt_dl[i];
      }
      
      // convert to piece wise jerk path
      // 将QP优化得到的各端点状态(l, l', l'')与delta_s, 生成分段三次多项式, 并以固定间隔采样得到frenet坐标系离散轨迹
      // {s, (l, l', l'')}
      auto frenet_frame_path =
          ToPiecewiseJerkPath(opt_l, opt_dl, opt_ddl, path_boundary.delta_s(),
                              path_boundary.start_s());

      path_data.SetReferenceLine(&reference_line);
      path_data.SetFrenetPath(std::move(frenet_frame_path));
      if (FLAGS_use_front_axe_center_in_path_planning) {
        auto discretized_path = DiscretizedPath(
            ConvertPathPointRefFromFrontAxeToRearAxe(path_data));
        path_data.SetDiscretizedPath(discretized_path);
      }
      path_data.set_path_label(path_boundary.label());
      path_data.set_blocking_obstacle_id(path_boundary.blocking_obstacle_id());
      // candidate_path_data包括: 离散frenet trajectory: frenet_frame_path, 
      // reference_line, label, blocking_obstacle_id (在当前reference_line中无法绕过的静态障碍物)
      candidate_path_data.push_back(std::move(path_data));
    }
  }
  if (candidate_path_data.empty()) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "Path Optimizer failed to generate path");
  }
  // 每个path_boundary对应的candidate_path_data会更新到reference_line_info中
  reference_line_info_->SetCandidatePathData(std::move(candidate_path_data));
  return Status::OK();
}

common::TrajectoryPoint
PiecewiseJerkPathOptimizer::InferFrontAxeCenterFromRearAxeCenter(
    const common::TrajectoryPoint& traj_point) {
  double front_to_rear_axe_distance =
      VehicleConfigHelper::GetConfig().vehicle_param().wheel_base();
  common::TrajectoryPoint ret = traj_point;
  ret.mutable_path_point()->set_x(
      traj_point.path_point().x() +
      front_to_rear_axe_distance * std::cos(traj_point.path_point().theta()));
  ret.mutable_path_point()->set_y(
      traj_point.path_point().y() +
      front_to_rear_axe_distance * std::sin(traj_point.path_point().theta()));
  return ret;
}

std::vector<common::PathPoint>
PiecewiseJerkPathOptimizer::ConvertPathPointRefFromFrontAxeToRearAxe(
    const PathData& path_data) {
  std::vector<common::PathPoint> ret;
  double front_to_rear_axe_distance =
      VehicleConfigHelper::GetConfig().vehicle_param().wheel_base();
  for (auto path_point : path_data.discretized_path()) {
    common::PathPoint new_path_point = path_point;
    new_path_point.set_x(path_point.x() - front_to_rear_axe_distance *
                                              std::cos(path_point.theta()));
    new_path_point.set_y(path_point.y() - front_to_rear_axe_distance *
                                              std::sin(path_point.theta()));
    ret.push_back(new_path_point);
  }
  return ret;
}

/* 基于给定的输入, 进行基于分段三次多项式的QP优化:
   -> 初始/终点状态(init_state, end_state)
   -> 横向参考位置(path_reference_l_ref)
   -> 纵向分段距离(delta_s)
   -> 横向0/2阶导数边界(lat_boundaries, ddl_bounds)
   -> x的0/1/2/3/4阶导数权重 */
bool PiecewiseJerkPathOptimizer::OptimizePath(
    const std::pair<std::array<double, 3>, std::array<double, 3>>& init_state,
    const std::array<double, 3>& end_state,
    std::vector<double> path_reference_l_ref, const size_t path_reference_size,
    const double delta_s, const bool is_valid_path_reference,
    const std::vector<std::pair<double, double>>& lat_boundaries,
    const std::vector<std::pair<double, double>>& ddl_bounds,
    const std::array<double, 5>& w, const int max_iter, std::vector<double>* x,
    std::vector<double>* dx, std::vector<double>* ddx) {
  // num of knots
  const size_t kNumKnots = lat_boundaries.size();
  PiecewiseJerkPathProblem piecewise_jerk_problem(kNumKnots, delta_s,
                                                  init_state.second);

  // TODO(Hongyi): update end_state settings
  // 设置end_state_ref的权重与值
  piecewise_jerk_problem.set_end_state_ref({1000.0, 0.0, 0.0}, end_state);
  // pull over scenarios
  // Because path reference might also make the end_state != 0
  // we have to exclude this condition here
  
  // 设置参考横向位置x_ref以及对应的跟踪weight
  // 对于Pull-Over Scenario, 设置点的参考横向位置均为终点的横向位置, 且根据Pull-Over Type不同, 设置不同的weight
  if (end_state[0] != 0 && !is_valid_path_reference) {
    std::vector<double> x_ref(kNumKnots, end_state[0]);
    const auto& pull_over_type = injector_->planning_context()
                                     ->planning_status()
                                     .pull_over()
                                     .pull_over_type();
    const double weight_x_ref =
        pull_over_type == PullOverStatus::EMERGENCY_PULL_OVER ? 200.0 : 10.0;
    piecewise_jerk_problem.set_x_ref(weight_x_ref, std::move(x_ref));
  }
  
  // use path reference as a optimization cost function
  if (is_valid_path_reference) {
    // for non-path-reference part
    // weight_x_ref is set to default value, where
    // l weight = weight_x_ + weight_x_ref_ = (1.0 + 0.0)
    std::vector<double> weight_x_ref_vec(kNumKnots, 0.0);
    // increase l weight for path reference part only

    // 按照高斯分布, 设置每个位置处的weight_x_ref_vec
    const double peak_value = config_.piecewise_jerk_path_optimizer_config()
                                  .path_reference_l_weight();
    const double peak_value_x =
        0.5 * static_cast<double>(path_reference_size) * delta_s;
    for (size_t i = 0; i < path_reference_size; ++i) {
      // Gaussian weighting
      const double x = static_cast<double>(i) * delta_s;
      weight_x_ref_vec.at(i) = GaussianWeighting(x, peak_value, peak_value_x);
      ADEBUG << "i: " << i << ", weight: " << weight_x_ref_vec.at(i);
    }
    piecewise_jerk_problem.set_x_ref(std::move(weight_x_ref_vec),
                                     std::move(path_reference_l_ref));
  }

  // 设置x, x', x'', x'''的weight
  piecewise_jerk_problem.set_weight_x(w[0]);
  piecewise_jerk_problem.set_weight_dx(w[1]);
  piecewise_jerk_problem.set_weight_ddx(w[2]);
  piecewise_jerk_problem.set_weight_dddx(w[3]);

  // scale_factor: 在优化时, 使用{1.0*x, 10.0*x', 100.0*x''}作为优化的X状态向量
  // {x, x', x''} = result/{scale_factor[0], scale_factor[1], scale_factor[2]}
  piecewise_jerk_problem.set_scale_factor({1.0, 10.0, 100.0});

  auto start_time = std::chrono::system_clock::now();

  // 设置x, dx, ddx的上下边界
  piecewise_jerk_problem.set_x_bounds(lat_boundaries);
  piecewise_jerk_problem.set_dx_bounds(-FLAGS_lateral_derivative_bound_default,
                                       FLAGS_lateral_derivative_bound_default);
  piecewise_jerk_problem.set_ddx_bounds(ddl_bounds);

  // 计算并设置dddx(lateral jerk)边界
  // Estimate lat_acc and jerk boundary from vehicle_params
  const auto& veh_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  const double axis_distance = veh_param.wheel_base();
  // max achievable yaw rate, calculated from max steer angle rate
  const double max_yaw_rate =
      veh_param.max_steer_angle_rate() / veh_param.steer_ratio() / 2.0;
  const double jerk_bound = EstimateJerkBoundary(
      std::fmax(init_state.first[1], 1.0), axis_distance, max_yaw_rate);
  piecewise_jerk_problem.set_dddx_bound(jerk_bound);

  // 构建OSQP问题(设置lower/upper边界, 生成P, q, A矩阵), 并进行最优化求解, 提取最优化的结果{x, dx, ddx}
  // QP问题形式: min(1/2*X*P*X+q*X), with l <= A*X <= u
  // 假设每个knot点之间用三次曲线连接, 在QP问题的A矩阵中体现两段间的0/1/2阶导连续性
  bool success = piecewise_jerk_problem.Optimize(max_iter);

  auto end_time = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = end_time - start_time;
  ADEBUG << "Path Optimizer used time: " << diff.count() * 1000 << " ms.";

  if (!success) {
    AERROR << "piecewise jerk path optimizer failed";
    return false;
  }

  *x = piecewise_jerk_problem.opt_x();
  *dx = piecewise_jerk_problem.opt_dx();
  *ddx = piecewise_jerk_problem.opt_ddx();

  return true;
}

/* 基于计算得到的分段三次多项式各端点值(x, dx, ddx)以及delta_s, 生成分段三次多项式, 并以固定间隔采样得到frenet坐标系下的轨迹
   frenet_frame_path {s, (l, l', l'')} */
FrenetFramePath PiecewiseJerkPathOptimizer::ToPiecewiseJerkPath(
    const std::vector<double>& x, const std::vector<double>& dx,
    const std::vector<double>& ddx, const double delta_s,
    const double start_s) const {
  ACHECK(!x.empty());
  ACHECK(!dx.empty());
  ACHECK(!ddx.empty());

  PiecewiseJerkTrajectory1d piecewise_jerk_traj(x.front(), dx.front(),
                                                ddx.front());
  // 基于输入, 迭代新增各段三次多项式, 并计算三次多项式的系数, 更新每个三次多项式的end_state{x1, x1', x1''}作为下一段三次多项式的起始
  for (std::size_t i = 1; i < x.size(); ++i) {
    const auto dddl = (ddx[i] - ddx[i - 1]) / delta_s;
    piecewise_jerk_traj.AppendSegment(dddl, delta_s);
  }

  std::vector<common::FrenetFramePoint> frenet_frame_path;
  double accumulated_s = 0.0;
  // 基于设定纵向间隔, 对三次多项式进行采样, 输出frenet坐标系下的轨迹frenet_frame_path -> {s, (l, l', l'')}
  while (accumulated_s < piecewise_jerk_traj.ParamLength()) {
    double l = piecewise_jerk_traj.Evaluate(0, accumulated_s);
    double dl = piecewise_jerk_traj.Evaluate(1, accumulated_s);
    double ddl = piecewise_jerk_traj.Evaluate(2, accumulated_s);

    common::FrenetFramePoint frenet_frame_point;
    frenet_frame_point.set_s(accumulated_s + start_s);
    frenet_frame_point.set_l(l);
    frenet_frame_point.set_dl(dl);
    frenet_frame_point.set_ddl(ddl);
    frenet_frame_path.push_back(std::move(frenet_frame_point));

    accumulated_s += FLAGS_trajectory_space_resolution;
  }

  return FrenetFramePath(std::move(frenet_frame_path));
}

/* 生成预期的最大lateral jerk limit, 用于框定dddx的值 */
double PiecewiseJerkPathOptimizer::EstimateJerkBoundary(
    const double vehicle_speed, const double axis_distance,
    const double max_yaw_rate) const {
  return max_yaw_rate / axis_distance / vehicle_speed;
}

/* 返回x基于高斯分布的weight: peak_weighting * exp(-0.5 * (x - peak_weighting_x) * (x - peak_weighting_x)) */
double PiecewiseJerkPathOptimizer::GaussianWeighting(
    const double x, const double peak_weighting,
    const double peak_weighting_x) const {
  double std = 1 / (std::sqrt(2 * M_PI) * peak_weighting);
  double u = peak_weighting_x * std;
  double x_updated = x * std;
  // 实际返回值与下式相同
  ADEBUG << peak_weighting *
                exp(-0.5 * (x - peak_weighting_x) * (x - peak_weighting_x));
  ADEBUG << Gaussian(u, std, x_updated);
  return Gaussian(u, std, x_updated);
}

}  // namespace planning
}  // namespace apollo
