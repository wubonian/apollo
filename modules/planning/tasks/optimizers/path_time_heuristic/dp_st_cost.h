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

#pragma once

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/speed/st_boundary.h"
#include "modules/planning/common/speed/st_point.h"
#include "modules/planning/proto/st_drivable_boundary.pb.h"
#include "modules/planning/proto/task_config.pb.h"
#include "modules/planning/tasks/optimizers/path_time_heuristic/st_graph_point.h"

namespace apollo {
namespace planning {

/* 该类用于每个采样点的cost计算 */
class DpStCost {
 public:
  DpStCost(const DpStSpeedOptimizerConfig& config, const double total_t,
           const double total_s, const std::vector<const Obstacle*>& obstacles,
           const STDrivableBoundary& st_drivable_boundary,
           const common::TrajectoryPoint& init_point);

  double GetObstacleCost(const StGraphPoint& point);

  double GetSpatialPotentialCost(const StGraphPoint& point);

  double GetReferenceCost(const STPoint& point,
                          const STPoint& reference_point) const;

  double GetSpeedCost(const STPoint& first, const STPoint& second,
                      const double speed_limit,
                      const double cruise_speed) const;

  double GetAccelCostByTwoPoints(const double pre_speed, const STPoint& first,
                                 const STPoint& second);
  double GetAccelCostByThreePoints(const STPoint& first, const STPoint& second,
                                   const STPoint& third);

  double GetJerkCostByTwoPoints(const double pre_speed, const double pre_acc,
                                const STPoint& pre_point,
                                const STPoint& curr_point);
  double GetJerkCostByThreePoints(const double first_speed,
                                  const STPoint& first_point,
                                  const STPoint& second_point,
                                  const STPoint& third_point);

  double GetJerkCostByFourPoints(const STPoint& first, const STPoint& second,
                                 const STPoint& third, const STPoint& fourth);

 private:
  double GetAccelCost(const double accel);
  double JerkCost(const double jerk);

  void AddToKeepClearRange(const std::vector<const Obstacle*>& obstacles);
  static void SortAndMergeRange(
      std::vector<std::pair<double, double>>* keep_clear_range_);
  bool InKeepClearRange(double s) const;

  const DpStSpeedOptimizerConfig& config_;
  const std::vector<const Obstacle*>& obstacles_;

  STDrivableBoundary st_drivable_boundary_;

  const common::TrajectoryPoint& init_point_;

  double unit_t_ = 0.0;
  double total_s_ = 0.0;

  std::unordered_map<std::string, int> boundary_map_;     // <id, index>: 每个obstacle id对应的index
  std::vector<std::vector<std::pair<double, double>>> boundary_cost_;   // 每个obstacle的每个采样时间t的boundary_cost_ <s_upper, s_lower>
                                                                        // 默认为<-1.0, 1.0>

  std::vector<std::pair<double, double>> keep_clear_range_;     // 所有KEEP_CLEAR obstacle的<start_s, end_s> list

  std::array<double, 200> accel_cost_;    // 以0.1 m/s^2为间隔, 存储每个acceleration对应的cost, 可覆盖[-10.0, 10.0]m/s^2的范围
  std::array<double, 400> jerk_cost_;     // 以0.1 m/s^3为间隔, 存储每个jerk对应的cost, 可覆盖[-20.0, 20.0]m/s^3的范围
};

}  // namespace planning
}  // namespace apollo
