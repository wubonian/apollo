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

#include "modules/planning/math/piecewise_jerk/piecewise_jerk_speed_problem.h"

#include "cyber/common/log.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

/* 1/2*X*P*X + q*X = weight_x_ref_*(x-x_ref)^2 + weight_dx_ref*(dx-dx_ref)^2 + 
                     weight_ddx_*ddx^2 + weight_dddx_*(ddx[i+1]^2-ddx[i]^2)/dt^2 */

/* 构建PieceWiseJerkSpeedProblem */
PiecewiseJerkSpeedProblem::PiecewiseJerkSpeedProblem(
    const size_t num_of_knots, const double delta_s,
    const std::array<double, 3>& x_init)
    : PiecewiseJerkProblem(num_of_knots, delta_s, x_init) {
  penalty_dx_.resize(num_of_knots_, 0.0);
}

/* 设置速度参考与权重 */
void PiecewiseJerkSpeedProblem::set_dx_ref(const double weight_dx_ref,
                                           const double dx_ref) {
  weight_dx_ref_ = weight_dx_ref;
  dx_ref_ = dx_ref;
  has_dx_ref_ = true;
}

/* set penalty_dx_ */
void PiecewiseJerkSpeedProblem::set_penalty_dx(std::vector<double> penalty_dx) {
  CHECK_EQ(penalty_dx.size(), num_of_knots_);
  penalty_dx_ = std::move(penalty_dx);
}

/* 构建QP问题的P矩阵:
   -> (1/2*X*P*X)*2 = (1/2*weight_x_ref_*x[i]^2 + 1/2*(weight_dx_ref_ + penalty_dx_)*dx[i]^2 +
                      1/2*weight_ddx_*ddx[i]^2 + 1/2*2*weight_dddx_*(ddx[i+1]^2 - ddx[i]^2)/dt^2) *2 (乘以2是为了与q矩阵共同构建)
   -> P_data: P矩阵的非零系数, 按列->行
   -> P_indices: 每个系数对应的行号
   -> P_indptr: 每个非零系数的序号, 按照列->行*/
void PiecewiseJerkSpeedProblem::CalculateKernel(std::vector<c_float>* P_data,
                                                std::vector<c_int>* P_indices,
                                                std::vector<c_int>* P_indptr) {
  const int n = static_cast<int>(num_of_knots_);
  const int kNumParam = 3 * n;
  const int kNumValue = 4 * n - 1;
  std::vector<std::vector<std::pair<c_int, c_float>>> columns;
  columns.resize(kNumParam);
  int value_index = 0;

  // x(i)^2 * w_x_ref
  // 位置s的cost: 1/2*X*P*X = 1/2 * weight_x_ref_ * x[i]^2
  for (int i = 0; i < n - 1; ++i) {
    // 在row = i, column = i, 添加对应的P矩阵的位置x权重
    columns[i].emplace_back(
        i, weight_x_ref_ / (scale_factor_[0] * scale_factor_[0]));
    ++value_index;
  }
  // x(n-1)^2 * (w_x_ref + w_end_x)
  columns[n - 1].emplace_back(n - 1, (weight_x_ref_ + weight_end_state_[0]) /
                                         (scale_factor_[0] * scale_factor_[0]));
  ++value_index;

  // x(i)'^2 * (w_dx_ref + penalty_dx)
  // 速度ds的cost: 1/2*X*P*X = 1/2 * (weight_dx_ref_ + penalty_dx_) * dx[i]^2
  for (int i = 0; i < n - 1; ++i) {
    // 在row = n+i, column = n+i, 添加对应的P矩阵的速度dx权重
    columns[n + i].emplace_back(n + i,
                                (weight_dx_ref_ + penalty_dx_[i]) /
                                    (scale_factor_[1] * scale_factor_[1]));
    ++value_index;
  }
  
  // x(n-1)'^2 * (w_dx_ref + penalty_dx + w_end_dx)
  columns[2 * n - 1].emplace_back(
      2 * n - 1, (weight_dx_ref_ + penalty_dx_[n - 1] + weight_end_state_[1]) /
                     (scale_factor_[1] * scale_factor_[1]));
  ++value_index;

  auto delta_s_square = delta_s_ * delta_s_;
  // x(i)''^2 * (w_ddx + 2 * w_dddx / delta_s^2)
  columns[2 * n].emplace_back(2 * n,
                              (weight_ddx_ + weight_dddx_ / delta_s_square) /
                                  (scale_factor_[2] * scale_factor_[2]));
  ++value_index;

  // 加速度dds的cost: 1/2*X*P*X = 1/2 * (weight_ddx_) * ddx[i]^2
  // 纵向jerk ddds的cost: 1/2*X*P*X = 1/2 * 2 * (weight_dddx_) * (ddx[i+1]^2 - ddx[i]^2)/dt^2
  // jerk的cost由该for循环与下面的for循环中weight_dddx_部分共同构建
  for (int i = 1; i < n - 1; ++i) {
    // 在row = 2n+i, column = 2n+i, 添加对应的P矩阵的加速度ddx权重和部分dddx的权重
    columns[2 * n + i].emplace_back(
        2 * n + i, (weight_ddx_ + 2.0 * weight_dddx_ / delta_s_square) /
                       (scale_factor_[2] * scale_factor_[2]));
    ++value_index;
  }

  columns[3 * n - 1].emplace_back(
      3 * n - 1,
      (weight_ddx_ + weight_dddx_ / delta_s_square + weight_end_state_[2]) /
          (scale_factor_[2] * scale_factor_[2]));
  ++value_index;

  // -2 * w_dddx / delta_s^2 * x(i)'' * x(i + 1)''
  for (int i = 0; i < n - 1; ++i) {
    columns[2 * n + i].emplace_back(2 * n + i + 1,
                                    -2.0 * weight_dddx_ / delta_s_square /
                                        (scale_factor_[2] * scale_factor_[2]));
    ++value_index;
  }

  CHECK_EQ(value_index, kNumValue);

  int ind_p = 0;
  for (int i = 0; i < kNumParam; ++i) {
    P_indptr->push_back(ind_p);
    for (const auto& row_data_pair : columns[i]) {
      P_data->push_back(row_data_pair.second * 2.0);    // "x*2.0": 1/2*X*P*X -> X*P*X
      P_indices->push_back(row_data_pair.first);
      ++ind_p;
    }
  }
  P_indptr->push_back(ind_p);
}

/* 构建QP问题的q矩阵:
   -> q*X = - 2*weight_x_ref_*x_ref_[i]*x[i] - 2*weight_dx_ref_*dx_ref_*dx[i] */
void PiecewiseJerkSpeedProblem::CalculateOffset(std::vector<c_float>* q) {
  CHECK_NOTNULL(q);
  const int n = static_cast<int>(num_of_knots_);
  const int kNumParam = 3 * n;
  q->resize(kNumParam);
  for (int i = 0; i < n; ++i) {
    // q*X = -2*weight_x_ref_*x_ref_[i]*x[i]
    if (has_x_ref_) {
      q->at(i) += -2.0 * weight_x_ref_ * x_ref_[i] / scale_factor_[0];
    }
    // q*x = -2*weight_dx_ref_*dx_ref_*dx[i]
    if (has_dx_ref_) {
      q->at(n + i) += -2.0 * weight_dx_ref_ * dx_ref_ / scale_factor_[1];
    }
  }
  // set end state reference part
  if (has_end_state_ref_) {
    q->at(n - 1) +=
        -2.0 * weight_end_state_[0] * end_state_ref_[0] / scale_factor_[0];
    q->at(2 * n - 1) +=
        -2.0 * weight_end_state_[1] * end_state_ref_[1] / scale_factor_[1];
    q->at(3 * n - 1) +=
        -2.0 * weight_end_state_[2] * end_state_ref_[2] / scale_factor_[2];
  }
}

OSQPSettings* PiecewiseJerkSpeedProblem::SolverDefaultSettings() {
  // Define Solver default settings
  OSQPSettings* settings =
      reinterpret_cast<OSQPSettings*>(c_malloc(sizeof(OSQPSettings)));
  osqp_set_default_settings(settings);
  settings->eps_abs = 1e-4;
  settings->eps_rel = 1e-4;
  settings->eps_prim_inf = 1e-5;
  settings->eps_dual_inf = 1e-5;
  settings->polish = true;
  settings->verbose = FLAGS_enable_osqp_debug;
  settings->scaled_termination = true;

  return settings;
}

}  // namespace planning
}  // namespace apollo
