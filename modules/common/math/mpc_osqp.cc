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

#include "modules/common/math/mpc_osqp.h"

namespace apollo {
namespace common {
namespace math {
MpcOsqp::MpcOsqp(const Eigen::MatrixXd &matrix_a,
                 const Eigen::MatrixXd &matrix_b,
                 const Eigen::MatrixXd &matrix_q,
                 const Eigen::MatrixXd &matrix_r,
                 const Eigen::MatrixXd &matrix_initial_x,
                 const Eigen::MatrixXd &matrix_u_lower,
                 const Eigen::MatrixXd &matrix_u_upper,
                 const Eigen::MatrixXd &matrix_x_lower,
                 const Eigen::MatrixXd &matrix_x_upper,
                 const Eigen::MatrixXd &matrix_x_ref, const int max_iter,
                 const int horizon, const double eps_abs)
    : matrix_a_(matrix_a),
      matrix_b_(matrix_b),
      matrix_q_(matrix_q),
      matrix_r_(matrix_r),
      matrix_initial_x_(matrix_initial_x),
      matrix_u_lower_(matrix_u_lower),
      matrix_u_upper_(matrix_u_upper),
      matrix_x_lower_(matrix_x_lower),
      matrix_x_upper_(matrix_x_upper),
      matrix_x_ref_(matrix_x_ref),
      max_iteration_(max_iter),
      horizon_(horizon),
      eps_abs_(eps_abs) {
  state_dim_ = matrix_b.rows();
  control_dim_ = matrix_b.cols();
  ADEBUG << "state_dim" << state_dim_;
  ADEBUG << "control_dim_" << control_dim_;
  num_param_ = state_dim_ * (horizon_ + 1) + control_dim_ * horizon_;   // total number of x for QP solver
}

/* 计算QP优化的P矩阵:
   -> P由Q与R组成
   -> Q对应状态向量X, R对应控制向量U */
void MpcOsqp::CalculateKernel(std::vector<c_float> *P_data,
                              std::vector<c_int> *P_indices,
                              std::vector<c_int> *P_indptr) {
  // col1:(row,val),...; col2:(row,val),....; ...
  std::vector<std::vector<std::pair<c_int, c_float>>> columns;
  columns.resize(num_param_);
  size_t value_index = 0;
  // state and terminal state
  // add state x part for P matrix
  for (size_t i = 0; i <= horizon_; ++i) {
    for (size_t j = 0; j < state_dim_; ++j) {
      // (row, val)
      columns[i * state_dim_ + j].emplace_back(i * state_dim_ + j,
                                               matrix_q_(j, j));
      ++value_index;
    }
  }
  // control
  const size_t state_total_dim = state_dim_ * (horizon_ + 1);
  // add control u part for matrix
  for (size_t i = 0; i < horizon_; ++i) {
    for (size_t j = 0; j < control_dim_; ++j) {
      // (row, val)
      columns[i * control_dim_ + j + state_total_dim].emplace_back(
          state_total_dim + i * control_dim_ + j, matrix_r_(j, j));
      ++value_index;
    }
  }
  CHECK_EQ(value_index, num_param_);

  int ind_p = 0;
  for (size_t i = 0; i < num_param_; ++i) {
    // TODO(SHU) Check this
    P_indptr->emplace_back(ind_p);
    for (const auto &row_data_pair : columns[i]) {
      P_data->emplace_back(row_data_pair.second);    // val
      P_indices->emplace_back(row_data_pair.first);  // row
      ++ind_p;
    }
  }
  P_indptr->emplace_back(ind_p);
}

// reference is always zero
// gradient_ = -matrix_q_ * matrix_x_ref_
void MpcOsqp::CalculateGradient() {
  // populate the gradient vector
  gradient_ = Eigen::VectorXd::Zero(
      state_dim_ * (horizon_ + 1) + control_dim_ * horizon_, 1);
  for (size_t i = 0; i < horizon_ + 1; i++) {
    // 从第i*state_dim_个元素开始, 提取后续state_dim_的部分
    gradient_.block(i * state_dim_, 0, state_dim_, 1) =
        -1.0 * matrix_q_ * matrix_x_ref_;
  }
  ADEBUG << "Gradient_mat";
  ADEBUG << gradient_;
}

// equality constraints x(k+1) = A*x(k)
/* 计算QP优化的A矩阵[m, n] = [2*X+U, 2*X+U] */
void MpcOsqp::CalculateEqualityConstraint(std::vector<c_float> *A_data,
                                          std::vector<c_int> *A_indices,
                                          std::vector<c_int> *A_indptr) {
  static constexpr double kEpsilon = 1e-6;
  // block matrix
  Eigen::MatrixXd matrix_constraint = Eigen::MatrixXd::Zero(
      state_dim_ * (horizon_ + 1) + state_dim_ * (horizon_ + 1) +
          control_dim_ * horizon_,
      state_dim_ * (horizon_ + 1) + control_dim_ * horizon_);
  Eigen::MatrixXd state_identity_mat = Eigen::MatrixXd::Identity(
      state_dim_ * (horizon_ + 1), state_dim_ * (horizon_ + 1));
  ADEBUG << "state_identity_mat" << state_identity_mat;

  // matrix_constraint的[state_dim_ * (horizon_ + 1), state_dim_ * (horizon_ + 1)]部分设置为负单位矩阵
  matrix_constraint.block(0, 0, state_dim_ * (horizon_ + 1),
                          state_dim_ * (horizon_ + 1)) =
      -1 * state_identity_mat;
  ADEBUG << "matrix_constraint";
  ADEBUG << matrix_constraint;

  Eigen::MatrixXd control_identity_mat =
      Eigen::MatrixXd::Identity(control_dim_, control_dim_);

  // 设置[state_dim_, state_dim_]部分为matrix_a_
  for (size_t i = 0; i < horizon_; i++) {
    matrix_constraint.block((i + 1) * state_dim_, i * state_dim_, state_dim_,
                            state_dim_) = matrix_a_;
  }
  ADEBUG << "matrix_constraint with A";
  ADEBUG << matrix_constraint;

  // 设置matrix_b_矩阵
  for (size_t i = 0; i < horizon_; i++) {
    matrix_constraint.block((i + 1) * state_dim_,
                            i * control_dim_ + (horizon_ + 1) * state_dim_,
                            state_dim_, control_dim_) = matrix_b_;
  }
  ADEBUG << "matrix_constraint with B";
  ADEBUG << matrix_constraint;

  Eigen::MatrixXd all_identity_mat =
      Eigen::MatrixXd::Identity(num_param_, num_param_);

  matrix_constraint.block(state_dim_ * (horizon_ + 1), 0, num_param_,
                          num_param_) = all_identity_mat;
  ADEBUG << "matrix_constraint with I";
  ADEBUG << matrix_constraint;

  std::vector<std::vector<std::pair<c_int, c_float>>> columns;
  columns.resize(num_param_ + 1);
  int value_index = 0;
  // state and terminal state
  // 将matrix_constraint设置到columns中
  for (size_t i = 0; i < num_param_; ++i) {  // col
    for (size_t j = 0; j < num_param_ + state_dim_ * (horizon_ + 1);
         ++j)  // row
      if (std::fabs(matrix_constraint(j, i)) > kEpsilon) {
        // (row, val)
        columns[i].emplace_back(j, matrix_constraint(j, i));
        ++value_index;
      }
  }
  ADEBUG << "value_index";
  ADEBUG << value_index;
  int ind_A = 0;
  for (size_t i = 0; i < num_param_; ++i) {
    A_indptr->emplace_back(ind_A);
    for (const auto &row_data_pair : columns[i]) {
      A_data->emplace_back(row_data_pair.second);    // value
      A_indices->emplace_back(row_data_pair.first);  // row
      ++ind_A;
    }
  }
  A_indptr->emplace_back(ind_A);
}

/* 为<X, X, u>计算生成上下边界lowerBound_, upperBound_ */
void MpcOsqp::CalculateConstraintVectors() {
  // evaluate the lower and the upper inequality vectors
  Eigen::VectorXd lowerInequality = Eigen::MatrixXd::Zero(
      state_dim_ * (horizon_ + 1) + control_dim_ * horizon_, 1);
  Eigen::VectorXd upperInequality = Eigen::MatrixXd::Zero(
      state_dim_ * (horizon_ + 1) + control_dim_ * horizon_, 1);
  // 更新控制的上下边界
  for (size_t i = 0; i < horizon_; i++) {
    lowerInequality.block(control_dim_ * i + state_dim_ * (horizon_ + 1), 0,
                          control_dim_, 1) = matrix_u_lower_;
    upperInequality.block(control_dim_ * i + state_dim_ * (horizon_ + 1), 0,
                          control_dim_, 1) = matrix_u_upper_;
  }
  ADEBUG << " matrix_u_lower_";

  // 更新状态变量的上下边界
  for (size_t i = 0; i < horizon_ + 1; i++) {
    lowerInequality.block(state_dim_ * i, 0, state_dim_, 1) = matrix_x_lower_;
    upperInequality.block(state_dim_ * i, 0, state_dim_, 1) = matrix_x_upper_;
  }
  ADEBUG << " matrix_x_lower_";

  // evaluate the lower and the upper equality vectors
  // 将初始状态向量塞进lowerEquality与upperEquality中
  Eigen::VectorXd lowerEquality =
      Eigen::MatrixXd::Zero(state_dim_ * (horizon_ + 1), 1);
  Eigen::VectorXd upperEquality;
  lowerEquality.block(0, 0, state_dim_, 1) = -1 * matrix_initial_x_;
  upperEquality = lowerEquality;
  lowerEquality = lowerEquality;
  ADEBUG << " matrix_initial_x_";

  // merge inequality and equality vectors
  // 用<lowerInequality, upperInequality>与<lowerEquality, upperEquality>生成上下边界<lowerBound_, upperBound_>
  lowerBound_ = Eigen::MatrixXd::Zero(
      2 * state_dim_ * (horizon_ + 1) + control_dim_ * horizon_, 1);
  lowerBound_ << lowerEquality, lowerInequality;
  ADEBUG << " lowerBound_ ";
  upperBound_ = Eigen::MatrixXd::Zero(
      2 * state_dim_ * (horizon_ + 1) + control_dim_ * horizon_, 1);
  upperBound_ << upperEquality, upperInequality;
  ADEBUG << " upperBound_";
}

/* 设置OSQP求解器 */
OSQPSettings *MpcOsqp::Settings() {
  // default setting
  OSQPSettings *settings =
      reinterpret_cast<OSQPSettings *>(c_malloc(sizeof(OSQPSettings)));
  if (settings == nullptr) {
    return nullptr;
  } else {
    osqp_set_default_settings(settings);
    settings->polish = true;
    settings->scaled_termination = true;
    settings->verbose = false;
    settings->max_iter = max_iteration_;
    settings->eps_abs = eps_abs_;
    return settings;
  }
}

/* setup OSQP problem:
   -> Q, P, A, l, u */
OSQPData *MpcOsqp::Data() {
  OSQPData *data = reinterpret_cast<OSQPData *>(c_malloc(sizeof(OSQPData)));
  size_t kernel_dim = state_dim_ * (horizon_ + 1) + control_dim_ * horizon_;
  size_t num_affine_constraint =
      2 * state_dim_ * (horizon_ + 1) + control_dim_ * horizon_;
  if (data == nullptr) {
    return nullptr;
  } else {
    data->n = kernel_dim;
    data->m = num_affine_constraint;
    std::vector<c_float> P_data;
    std::vector<c_int> P_indices;
    std::vector<c_int> P_indptr;
    ADEBUG << "before CalculateKernel";
    // 计算QP优化的P矩阵
    CalculateKernel(&P_data, &P_indices, &P_indptr);
    ADEBUG << "CalculateKernel done";
    // 由CalculateKernel()计算结果设置P矩阵: data->P
    data->P =
        csc_matrix(kernel_dim, kernel_dim, P_data.size(), CopyData(P_data),
                   CopyData(P_indices), CopyData(P_indptr));
    ADEBUG << "Get P matrix";
    // 由gradient_设置Q矩阵: data->q
    data->q = gradient_.data();
    ADEBUG << "before CalculateEqualityConstraint";
    std::vector<c_float> A_data;
    std::vector<c_int> A_indices;
    std::vector<c_int> A_indptr;
    // 计算QP优化的A矩阵, l <= A*X <= u
    // 设置A, l, u
    CalculateEqualityConstraint(&A_data, &A_indices, &A_indptr);
    ADEBUG << "CalculateEqualityConstraint done";
    data->A =
        csc_matrix(state_dim_ * (horizon_ + 1) + state_dim_ * (horizon_ + 1) +
                       control_dim_ * horizon_,
                   kernel_dim, A_data.size(), CopyData(A_data),
                   CopyData(A_indices), CopyData(A_indptr));
    ADEBUG << "Get A matrix";
    data->l = lowerBound_.data();
    data->u = upperBound_.data();
    return data;
  }
}

void MpcOsqp::FreeData(OSQPData *data) {
  c_free(data->A);
  c_free(data->P);
  c_free(data);
}

bool MpcOsqp::Solve(std::vector<double> *control_cmd) {
  // MPC QP问题的构建 - equation list:
  // 以下等式与不等式会构成QP问题的A矩阵
  // -> matrix_a_ * X[i] + matrix_b_ * U[i] - X[i+1] = 0
  // -> X[0] = X_init
  // -> l_x <= X[i] <= u_x
  // -> l_u <= U[i] <= u_u
  // 以下评价函数会构成QP问题的P与q矩阵:
  // J = 1/2*X*matrix_q_*X - matrix_q_*matrix_x_ref_ + 1/2*matrix_x_ref_*matrix_x_ref_ + 1/2*U*matrix_r_*U
  //   = 1/2*matrix_q_*(X - matrix_x_ref_)^2 + 1/2*U*matrix_r_*U
  ADEBUG << "Before Calc Gradient";
  // 计算生成梯度矩阵gradient_
  CalculateGradient();
  ADEBUG << "After Calc Gradient";
  // 计算生成上下边界<lowerBound_, upperBound_>
  CalculateConstraintVectors();
  ADEBUG << "MPC2Matrix";

  // 设置QP求解的Q, P, A, l, u矩阵
  OSQPData *data = Data();
  ADEBUG << "OSQP data done";
  ADEBUG << "OSQP data n" << data->n;
  ADEBUG << "OSQP data m" << data->m;
  for (int i = 0; i < data->n; ++i) {
    ADEBUG << "OSQP data q" << i << ":" << (data->q)[i];
  }
  ADEBUG << "OSQP data l" << data->l;
  for (int i = 0; i < data->m; ++i) {
    ADEBUG << "OSQP data l" << i << ":" << (data->l)[i];
  }
  ADEBUG << "OSQP data u" << data->u;
  for (int i = 0; i < data->m; ++i) {
    ADEBUG << "OSQP data u" << i << ":" << (data->u)[i];
  }

  OSQPSettings *settings = Settings();
  ADEBUG << "OSQP setting done";
  OSQPWorkspace *osqp_workspace = nullptr;
  // osqp_setup(&osqp_workspace, data, settings);
  // setup osqp问题
  osqp_workspace = osqp_setup(data, settings);
  ADEBUG << "OSQP workspace ready";
  // osqp求解
  osqp_solve(osqp_workspace);

  auto status = osqp_workspace->info->status_val;
  ADEBUG << "status:" << status;
  // check status
  if (status < 0 || (status != 1 && status != 2)) {
    AERROR << "failed optimization status:\t" << osqp_workspace->info->status;
    osqp_cleanup(osqp_workspace);
    FreeData(data);
    c_free(settings);
    return false;
  } else if (osqp_workspace->solution == nullptr) {
    AERROR << "The solution from OSQP is nullptr";
    osqp_cleanup(osqp_workspace);
    FreeData(data);
    c_free(settings);
    return false;
  }

  size_t first_control = state_dim_ * (horizon_ + 1);
  for (size_t i = 0; i < control_dim_; ++i) {
    control_cmd->at(i) = osqp_workspace->solution->x[i + first_control];
    ADEBUG << "control_cmd:" << i << ":" << control_cmd->at(i);
  }

  // Cleanup
  osqp_cleanup(osqp_workspace);
  FreeData(data);
  c_free(settings);
  return true;
}

}  // namespace math
}  // namespace common
}  // namespace apollo
