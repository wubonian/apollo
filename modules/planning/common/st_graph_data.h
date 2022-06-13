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
 * @file: st_graph_data.h
 * @brief: data with map info and obstacle info
 **/

#pragma once

#include <tuple>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/common/speed/st_boundary.h"
#include "modules/planning/common/speed_limit.h"
#include "modules/planning/proto/st_drivable_boundary.pb.h"

namespace apollo {
namespace planning {

constexpr double kObsSpeedIgnoreThreshold = 100.0;

class StGraphData {
 public:
  StGraphData() = default;

  void LoadData(const std::vector<const STBoundary*>& st_boundaries,
                const double min_s_on_st_boundaries,
                const apollo::common::TrajectoryPoint& init_point,
                const SpeedLimit& speed_limit, const double cruise_speed,
                const double path_data_length, const double total_time_by_conf,
                planning_internal::STGraphDebug* st_graph_debug);

  bool is_initialized() const { return init_; }

  const std::vector<const STBoundary*>& st_boundaries() const;

  double min_s_on_st_boundaries() const;

  const apollo::common::TrajectoryPoint& init_point() const;

  const SpeedLimit& speed_limit() const;

  double cruise_speed() const;

  double path_length() const;

  double total_time_by_conf() const;

  planning_internal::STGraphDebug* mutable_st_graph_debug();

  bool SetSTDrivableBoundary(
      const std::vector<std::tuple<double, double, double>>& s_boundary,
      const std::vector<std::tuple<double, double, double>>& v_obs_info);

  const STDrivableBoundary& st_drivable_boundary() const;

 private:
  bool init_ = false;
  std::vector<const STBoundary*> st_boundaries_;      // 在SpeedBoundsDecider中更新, 包含每个obstacle的ST Boundary
                                                      // <id_, BoundaryType, upper_points_, lower_points_, other>
  
  double min_s_on_st_boundaries_ = 0.0;           // ST图上当前最近的障碍物距离(fallback distance), 作为安全距离

  apollo::common::TrajectoryPoint init_point_;        // 在SpeedBoundsDecider中更新, 包含规划的起点

  SpeedLimit speed_limit_;          // 在SpeedBoundsDecider中更新, 包含每个位置s对应的speed_limit
                                    // speed limit包含道路, 曲率, nudge对应的限速信息

  double cruise_speed_ = 0.0;       // 在SpeedBoundsDecider中更新, 包含cruise_speed信息
  double path_data_length_ = 0.0;
  double path_length_by_conf_ = 0.0;
  double total_time_by_conf_ = 0.0;
  planning_internal::STGraphDebug* st_graph_debug_ = nullptr;

  STDrivableBoundary st_drivable_boundary_;   // 在STBoundsDecider中更新, 包含决策后自车在ST图上可通行的位置, 速度区间
                                              // <t, s_lower, s_upper, v_obs_lower, v_obs_upper>
};

}  // namespace planning
}  // namespace apollo
