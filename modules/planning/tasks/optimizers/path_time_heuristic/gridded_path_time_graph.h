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
 * @file gridded_path_time_graph.h
 **/

#pragma once

#include <memory>
#include <vector>

#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/status/status.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/path_decision.h"
#include "modules/planning/common/speed/speed_data.h"
#include "modules/planning/common/speed/st_point.h"
#include "modules/planning/common/st_graph_data.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/proto/task_config.pb.h"
#include "modules/planning/tasks/optimizers/path_time_heuristic/dp_st_cost.h"
#include "modules/planning/tasks/optimizers/path_time_heuristic/st_graph_point.h"

namespace apollo {
namespace planning {

/* 完成速度的动态规划DP过程 */
class GriddedPathTimeGraph {
 public:
  GriddedPathTimeGraph(const StGraphData& st_graph_data,
                       const DpStSpeedOptimizerConfig& dp_config,
                       const std::vector<const Obstacle*>& obstacles,
                       const common::TrajectoryPoint& init_point);

  common::Status Search(SpeedData* const speed_data);

 private:
  common::Status InitCostTable();

  common::Status InitSpeedLimitLookUp();

  common::Status RetrieveSpeedProfile(SpeedData* const speed_data);

  common::Status CalculateTotalCost();

  // defined for cyber task
  struct StGraphMessage {
    StGraphMessage(const uint32_t c_, const int32_t r_) : c(c_), r(r_) {}
    uint32_t c;
    uint32_t r;
  };
  void CalculateCostAt(const std::shared_ptr<StGraphMessage>& msg);

  double CalculateEdgeCost(const STPoint& first, const STPoint& second,
                           const STPoint& third, const STPoint& forth,
                           const double speed_limit, const double cruise_speed);
  double CalculateEdgeCostForSecondCol(const uint32_t row,
                                       const double speed_limit,
                                       const double cruise_speed);
  double CalculateEdgeCostForThirdCol(const uint32_t curr_row,
                                      const uint32_t pre_row,
                                      const double speed_limit,
                                      const double cruise_speed);

  // get the row-range of next time step
  void GetRowRange(const StGraphPoint& point, size_t* next_highest_row,
                   size_t* next_lowest_row);

 private:
  const StGraphData& st_graph_data_;

  std::vector<double> speed_limit_by_index_;      // 每个s采样点对应的限速信息(map, 横向加速度, 静态障碍物nudge限速)

  std::vector<double> spatial_distance_by_index_;   // 包含s的所有采样点

  // dp st configuration
  DpStSpeedOptimizerConfig gridded_path_time_graph_config_;

  // obstacles based on the current reference line
  const std::vector<const Obstacle*>& obstacles_;

  // vehicle configuration parameter
  const common::VehicleParam& vehicle_param_ =
      common::VehicleConfigHelper::GetConfig().vehicle_param();

  // initial status
  common::TrajectoryPoint init_point_;

  // cost utility with configuration;
  DpStCost dp_st_cost_;

  double total_length_t_ = 0.0;     // t轴的总采样长度
  double unit_t_ = 0.0;      // t轴的采样间隔
  uint32_t dimension_t_ = 0;      // t轴的采样数

  double total_length_s_ = 0.0;     // s轴的总采样长度
  double dense_unit_s_ = 0.0;     // s轴的细采样间隔
  double sparse_unit_s_ = 0.0;      // s轴的粗采样间隔
  uint32_t dense_dimension_s_ = 0;      // s轴的细采样点个数
  uint32_t sparse_dimension_s_ = 0;     // s轴的粗采样点个数
  uint32_t dimension_s_ = 0;      // s轴的总采样点个数

  double max_acceleration_ = 0.0;     // 最大纵向加速度
  double max_deceleration_ = 0.0;     // 最大纵向减速度

  // cost_table_[t][s]
  // row: s, col: t --- NOTICE: Please do NOT change.
  std::vector<std::vector<StGraphPoint>> cost_table_;     // dimension_t_ * dimension_s_ 的 StGraphPoint
};

}  // namespace planning
}  // namespace apollo
