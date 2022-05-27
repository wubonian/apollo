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

#include "modules/planning/tasks/deciders/path_bounds_decider/path_bounds_decider.h"

#include <algorithm>
#include <functional>
#include <limits>
#include <memory>
#include <set>

#include "absl/strings/str_cat.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/util/point_factory.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/common/path_boundary.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/tasks/deciders/utils/path_decider_obstacle_utils.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::VehicleConfigHelper;
using apollo::hdmap::HDMapUtil;
using apollo::hdmap::JunctionInfo;

namespace {
// PathBoundPoint contains: (s, l_min, l_max).
using PathBoundPoint = std::tuple<double, double, double>;
// PathBound contains a vector of PathBoundPoints.
using PathBound = std::vector<PathBoundPoint>;
// ObstacleEdge contains: (is_start_s, s, l_min, l_max, obstacle_id).
using ObstacleEdge = std::tuple<int, double, double, double, std::string>;
}  // namespace

PathBoundsDecider::PathBoundsDecider(
    const TaskConfig& config,
    const std::shared_ptr<DependencyInjector>& injector)
    : Decider(config, injector) {}

/* 基于任务, 决策后续横向规划的path_bounds
   -> fallback path_bound生成: 只考虑自车道lane boundary, 不考虑static obstacle
   -> 以下path_bound生成三选一:
      -> pullover path_bound生成: <left_lane, right_road_boundary>, 考虑static obstacle, 截止到pullover position
      -> laneChange path_bound生成: neighbor_left/right lane boundary, 考虑static obstacle, 以及lane_change_start_position
      -> regular path_bound生成: 基于lane_borrow_info考虑neighbor/ego lane boundary, 考虑static obstacle */
Status PathBoundsDecider::Process(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  // Sanity checks.
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  // Skip the path boundary decision if reusing the path.
  if (FLAGS_enable_skip_path_tasks && reference_line_info->path_reusable()) {
    return Status::OK();
  }

  std::vector<PathBoundary> candidate_path_boundaries;
  // const TaskConfig& config = Decider::config_;

  // Initialization.
  InitPathBoundsDecider(*frame, *reference_line_info);

  // Generate the fallback path boundary.
  PathBound fallback_path_bound;
  Status ret =
      GenerateFallbackPathBound(*reference_line_info, &fallback_path_bound);
  
  if (!ret.ok()) {
    ADEBUG << "Cannot generate a fallback path bound.";
    return Status(ErrorCode::PLANNING_ERROR, ret.error_message());
  }

  if (fallback_path_bound.empty()) {
    const std::string msg = "Failed to get a valid fallback path boundary";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  if (!fallback_path_bound.empty()) {
    CHECK_LE(adc_frenet_l_, std::get<2>(fallback_path_bound[0]));
    CHECK_GE(adc_frenet_l_, std::get<1>(fallback_path_bound[0]));
  }

  // Update the fallback path boundary into the reference_line_info.
  // 将fallback path加入到candidate_path_boundaries中, 标签为"fallback"
  std::vector<std::pair<double, double>> fallback_path_bound_pair;
  for (size_t i = 0; i < fallback_path_bound.size(); ++i) {
    fallback_path_bound_pair.emplace_back(std::get<1>(fallback_path_bound[i]),
                                          std::get<2>(fallback_path_bound[i]));
  }
  candidate_path_boundaries.emplace_back(std::get<0>(fallback_path_bound[0]),
                                         kPathBoundsDeciderResolution,
                                         fallback_path_bound_pair);
  candidate_path_boundaries.back().set_label("fallback");

  // If pull-over is requested, generate pull-over path boundary.
  auto* pull_over_status = injector_->planning_context()
                               ->mutable_planning_status()
                               ->mutable_pull_over();
  const bool plan_pull_over_path = pull_over_status->plan_pull_over_path();
  
  // 如果进入(EMERGENCY_)PULL_OVER scenario, 需要计算pull_over_path, 则计算pull over对应的path_bound, 计算完后退出
  // 不在计算后续的regular path_bound, 二者属于二选一的关系
  if (plan_pull_over_path) {
    PathBound pull_over_path_bound;
    // 计算PullOver PathBound <left_lane, right_road_boundary> + static obstacle limit
    // 裁剪到pull_over position位置处
    Status ret = GeneratePullOverPathBound(*frame, *reference_line_info,
                                           &pull_over_path_bound);
    if (!ret.ok()) {
      AWARN << "Cannot generate a pullover path bound, do regular planning.";
    } else {
      ACHECK(!pull_over_path_bound.empty());
      CHECK_LE(adc_frenet_l_, std::get<2>(pull_over_path_bound[0]));
      CHECK_GE(adc_frenet_l_, std::get<1>(pull_over_path_bound[0]));

      // Update the fallback path boundary into the reference_line_info.
      std::vector<std::pair<double, double>> pull_over_path_bound_pair;
      for (size_t i = 0; i < pull_over_path_bound.size(); ++i) {
        pull_over_path_bound_pair.emplace_back(
            std::get<1>(pull_over_path_bound[i]),
            std::get<2>(pull_over_path_bound[i]));
      }
      // 将pullover path加入到candidate_path_boundaries中, 标签为"regular/pullover"
      candidate_path_boundaries.emplace_back(
          std::get<0>(pull_over_path_bound[0]), kPathBoundsDeciderResolution,
          pull_over_path_bound_pair);
      candidate_path_boundaries.back().set_label("regular/pullover");

      reference_line_info->SetCandidatePathBoundaries(
          std::move(candidate_path_boundaries));
      ADEBUG << "Completed pullover and fallback path boundaries generation.";

      // set debug info in planning_data
      auto* pull_over_debug = reference_line_info->mutable_debug()
                                  ->mutable_planning_data()
                                  ->mutable_pull_over();
      pull_over_debug->mutable_position()->CopyFrom(
          pull_over_status->position());
      pull_over_debug->set_theta(pull_over_status->theta());
      pull_over_debug->set_length_front(pull_over_status->length_front());
      pull_over_debug->set_length_back(pull_over_status->length_back());
      pull_over_debug->set_width_left(pull_over_status->width_left());
      pull_over_debug->set_width_right(pull_over_status->width_right());

      return Status::OK();
    }
  }

  // If it's a lane-change reference-line, generate lane-change path boundary.
  // FLAGS_enable_smarter_lane_change默认为true
  // 对于router目标的lane, 计算LaneChangePathBound: lane_change_start_position前的基于自车位置的修正; 叠加static_obstacle limit信息
  // 不再计算后续的regular path_bound, 二者属于二选一的关系
  if (FLAGS_enable_smarter_lane_change &&
      reference_line_info->IsChangeLanePath()) {
    PathBound lanechange_path_bound;
    Status ret = GenerateLaneChangePathBound(*reference_line_info,
                                             &lanechange_path_bound);
    // error report
    if (!ret.ok()) {
      ADEBUG << "Cannot generate a lane-change path bound.";
      return Status(ErrorCode::PLANNING_ERROR, ret.error_message());
    }
    // error report
    if (lanechange_path_bound.empty()) {
      const std::string msg = "Failed to get a valid fallback path boundary";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }

    // disable this change when not extending lane bounds to include adc
    if (config_.path_bounds_decider_config()
            .is_extend_lane_bounds_to_include_adc()) {
      CHECK_LE(adc_frenet_l_, std::get<2>(lanechange_path_bound[0]));
      CHECK_GE(adc_frenet_l_, std::get<1>(lanechange_path_bound[0]));
    }
    // Update the fallback path boundary into the reference_line_info.
    // 将lane change path加入到candidate_path_boundaries中, 标签为"regular/lanechange"
    std::vector<std::pair<double, double>> lanechange_path_bound_pair;
    for (size_t i = 0; i < lanechange_path_bound.size(); ++i) {
      lanechange_path_bound_pair.emplace_back(
          std::get<1>(lanechange_path_bound[i]),
          std::get<2>(lanechange_path_bound[i]));
    }
    candidate_path_boundaries.emplace_back(
        std::get<0>(lanechange_path_bound[0]), kPathBoundsDeciderResolution,
        lanechange_path_bound_pair);
    candidate_path_boundaries.back().set_label("regular/lanechange");
    RecordDebugInfo(lanechange_path_bound, "", reference_line_info);
    reference_line_info->SetCandidatePathBoundaries(
        std::move(candidate_path_boundaries));
    ADEBUG << "Completed lanechange and fallback path boundaries generation.";
    return Status::OK();
  }

  // Generate regular path boundaries.
  std::vector<LaneBorrowInfo> lane_borrow_info_list;
  lane_borrow_info_list.push_back(LaneBorrowInfo::NO_BORROW);

  // 如果决策进行lane borrow, 将lane_borrow_decider的结果导出到lane_borrow_info_list中
  if (reference_line_info->is_path_lane_borrow()) {
    const auto& path_decider_status =
        injector_->planning_context()->planning_status().path_decider();
    for (const auto& lane_borrow_direction :
         path_decider_status.decided_side_pass_direction()) {
      if (lane_borrow_direction == PathDeciderStatus::LEFT_BORROW) {
        lane_borrow_info_list.push_back(LaneBorrowInfo::LEFT_BORROW);
      } else if (lane_borrow_direction == PathDeciderStatus::RIGHT_BORROW) {
        lane_borrow_info_list.push_back(LaneBorrowInfo::RIGHT_BORROW);
      }
    }
  }

  // Try every possible lane-borrow option:
  // PathBound regular_self_path_bound;
  // bool exist_self_path_bound = false;
  // 为决策出的每一个lane_borrow_info, 生成对应的regular_path_bound:
  // -> 基于lane_borrow_info考虑ego/neighbor lane boundary
  // -> 考虑static obstacle以及基于blocking_obstacle的裁剪
  for (const auto& lane_borrow_info : lane_borrow_info_list) {
    PathBound regular_path_bound;
    std::string blocking_obstacle_id = "";
    std::string borrow_lane_type = "";
    Status ret = GenerateRegularPathBound(
        *reference_line_info, lane_borrow_info, &regular_path_bound,
        &blocking_obstacle_id, &borrow_lane_type);
    
    // error handle
    if (!ret.ok()) {
      continue;
    }
    // error handle
    if (regular_path_bound.empty()) {
      continue;
    }
    // disable this change when not extending lane bounds to include adc
    if (config_.path_bounds_decider_config()
            .is_extend_lane_bounds_to_include_adc()) {
      CHECK_LE(adc_frenet_l_, std::get<2>(regular_path_bound[0]));
      CHECK_GE(adc_frenet_l_, std::get<1>(regular_path_bound[0]));
    }

    // Update the path boundary into the reference_line_info.
    // 将regular path加入到candidate_path_boundaries中, 标签为"regular/(left, right, self)"
    std::vector<std::pair<double, double>> regular_path_bound_pair;
    for (size_t i = 0; i < regular_path_bound.size(); ++i) {
      regular_path_bound_pair.emplace_back(std::get<1>(regular_path_bound[i]),
                                           std::get<2>(regular_path_bound[i]));
    }
    candidate_path_boundaries.emplace_back(std::get<0>(regular_path_bound[0]),
                                           kPathBoundsDeciderResolution,
                                           regular_path_bound_pair);
    std::string path_label = "";
    switch (lane_borrow_info) {
      case LaneBorrowInfo::LEFT_BORROW:
        path_label = "left";
        break;
      case LaneBorrowInfo::RIGHT_BORROW:
        path_label = "right";
        break;
      default:
        path_label = "self";
        // exist_self_path_bound = true;
        // regular_self_path_bound = regular_path_bound;
        break;
    }
    // RecordDebugInfo(regular_path_bound, "", reference_line_info);
    candidate_path_boundaries.back().set_label(
        absl::StrCat("regular/", path_label, "/", borrow_lane_type));
    candidate_path_boundaries.back().set_blocking_obstacle_id(
        blocking_obstacle_id);
  }

  // Remove redundant boundaries.
  // RemoveRedundantPathBoundaries(&candidate_path_boundaries);

  // Success
  // 将candidate_path_boundaries更新到reference_line_info中
  reference_line_info->SetCandidatePathBoundaries(
      std::move(candidate_path_boundaries));
  ADEBUG << "Completed regular and fallback path boundaries generation.";
  return Status::OK();
}

/* 初始化自车在frenet坐标系下的位置&速度信息, 以及车道宽度信息 */
void PathBoundsDecider::InitPathBoundsDecider(
    const Frame& frame, const ReferenceLineInfo& reference_line_info) {
  const ReferenceLine& reference_line = reference_line_info.reference_line();
  common::TrajectoryPoint planning_start_point = frame.PlanningStartPoint();
  if (FLAGS_use_front_axe_center_in_path_planning) {
    planning_start_point =
        InferFrontAxeCenterFromRearAxeCenter(planning_start_point);
  }
  ADEBUG << "Plan at the starting point: x = "
         << planning_start_point.path_point().x()
         << ", y = " << planning_start_point.path_point().y()
         << ", and angle = " << planning_start_point.path_point().theta();

  // Initialize some private variables.
  // ADC s/l info.
  auto adc_sl_info = reference_line.ToFrenetFrame(planning_start_point);
  adc_frenet_s_ = adc_sl_info.first[0];
  adc_frenet_l_ = adc_sl_info.second[0];
  adc_frenet_sd_ = adc_sl_info.first[1];
  adc_frenet_ld_ = adc_sl_info.second[1] * adc_frenet_sd_;
  double offset_to_map = 0.0;
  reference_line.GetOffsetToMap(adc_frenet_s_, &offset_to_map);
  adc_l_to_lane_center_ = adc_frenet_l_ + offset_to_map;

  // ADC's lane width.
  double lane_left_width = 0.0;
  double lane_right_width = 0.0;
  if (!reference_line.GetLaneWidth(adc_frenet_s_, &lane_left_width,
                                   &lane_right_width)) {
    AWARN << "Failed to get lane width at planning start point.";
    adc_lane_width_ = kDefaultLaneWidth;
  } else {
    adc_lane_width_ = lane_left_width + lane_right_width;
  }
}

common::TrajectoryPoint PathBoundsDecider::InferFrontAxeCenterFromRearAxeCenter(
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

/* 为正常工况生成regular path_bound:
   -> 考虑lane_borrow_info, 基于lane boundary生成path_bound
   -> 在path bound中考虑static_obstacle, 并基于blocking_obstacle进行裁剪 */
Status PathBoundsDecider::GenerateRegularPathBound(
    const ReferenceLineInfo& reference_line_info,
    const LaneBorrowInfo& lane_borrow_info, PathBound* const path_bound,
    std::string* const blocking_obstacle_id,
    std::string* const borrow_lane_type) {
  // 1. Initialize the path boundaries to be an indefinitely large area.
  // 初始化path boundary
  if (!InitPathBoundary(reference_line_info, path_bound)) {
    const std::string msg = "Failed to initialize path boundaries.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  // PathBoundsDebugString(*path_bound);

  // 2. Decide a rough boundary based on lane info and ADC's position
  // 基于左右lane boundary生成初步的path bound, 考虑lane_borrow_info, 加入相邻车道的lane boundary
  if (!GetBoundaryFromLanesAndADC(reference_line_info, lane_borrow_info, 0.1,
                                  path_bound, borrow_lane_type)) {
    const std::string msg =
        "Failed to decide a rough boundary based on "
        "road information.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  // PathBoundsDebugString(*path_bound);

  // TODO(jiacheng): once ready, limit the path boundary based on the
  //                 actual road boundary to avoid getting off-road.

  // 3. Fine-tune the boundary based on static obstacles
  PathBound temp_path_bound = *path_bound;
  // 在生成的path boundary中增加static obstacle信息, 并对blocking_obstacle后的path_bound进行裁剪
  if (!GetBoundaryFromStaticObstacles(reference_line_info.path_decision(),
                                      path_bound, blocking_obstacle_id)) {
    const std::string msg =
        "Failed to decide fine tune the boundaries after "
        "taking into consideration all static obstacles.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  // Append some extra path bound points to avoid zero-length path data.
  int counter = 0;
  // 在被blocking_obstacle截取的path_bound之后, 延长一段距离
  while (!blocking_obstacle_id->empty() &&
         path_bound->size() < temp_path_bound.size() &&
         counter < kNumExtraTailBoundPoint) {
    path_bound->push_back(temp_path_bound[path_bound->size()]);
    counter++;
  }
  // PathBoundsDebugString(*path_bound);

  // 4. Adjust the boundary considering dynamic obstacles
  // TODO(all): may need to implement this in the future.

  ADEBUG << "Completed generating path boundaries.";
  return Status::OK();
}

/* 生成LaneChange PathBound:
   -> 基于Lane Boundary与自车状态生成对应的path_bound
   -> 对lane_change_start_position前的path_bound, 基于自车横向位置进行修正
   -> 在path_bound中加入static_obstacle的limit信息 */
Status PathBoundsDecider::GenerateLaneChangePathBound(
    const ReferenceLineInfo& reference_line_info,
    std::vector<std::tuple<double, double, double>>* const path_bound) {
  // 1. Initialize the path boundaries to be an indefinitely large area.
  // 初始化path boundary, 左右limit设置为最大最小值
  if (!InitPathBoundary(reference_line_info, path_bound)) {
    const std::string msg = "Failed to initialize path boundaries.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  // PathBoundsDebugString(*path_bound);

  // 2. Decide a rough boundary based on lane info and ADC's position
  // 基于目标变道车道的左右lane Marker, 生成path_bound, 考虑自车运动极限 (最大横向加速度, 制动横向速度到0)
  std::string dummy_borrow_lane_type;
  if (!GetBoundaryFromLanesAndADC(reference_line_info,
                                  LaneBorrowInfo::NO_BORROW, 0.1, path_bound,
                                  &dummy_borrow_lane_type, true)) {
    const std::string msg =
        "Failed to decide a rough boundary based on "
        "road information.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  // PathBoundsDebugString(*path_bound);

  // 3. Remove the S-length of target lane out of the path-bound.
  // 对path_bound中lane_change_start_position前的部分进行基于自车横向位置进行修正
  GetBoundaryFromLaneChangeForbiddenZone(reference_line_info, path_bound);

  PathBound temp_path_bound = *path_bound;
  std::string blocking_obstacle_id;
  // 在path boundary加入static obstacle信息, 根据obstacle相对于中心的位置, 决定向左/向右绕障, 并根据该决策更新path bound
  // 遇到blocking_obstacle, 将path_bound裁剪到blocking_obstacle的位置
  if (!GetBoundaryFromStaticObstacles(reference_line_info.path_decision(),
                                      path_bound, &blocking_obstacle_id)) {
    const std::string msg =
        "Failed to decide fine tune the boundaries after "
        "taking into consideration all static obstacles.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  
  // Append some extra path bound points to avoid zero-length path data.
  // 在blocking_obstacle后, 延续一定长度的path_bound
  int counter = 0;
  while (!blocking_obstacle_id.empty() &&
         path_bound->size() < temp_path_bound.size() &&
         counter < kNumExtraTailBoundPoint) {
    path_bound->push_back(temp_path_bound[path_bound->size()]);
    counter++;
  }

  ADEBUG << "Completed generating path boundaries.";
  return Status::OK();
}

/* 生成PullOver PathBound:
   -> PullOver PathBound由两部分组成<left_lane, right_roadBoundary> + static_obstacle limit
   -> (如果没有) 计算目标的pull_over位置: 
      -> PULL_OVER: Router Destination的右侧路边缘附近
      -> EMERGENCY_PULL_OVER: 自车向前一段距离后的右边路边缘附近
   -> 将path_bound裁剪到pull_over位置处 */
Status PathBoundsDecider::GeneratePullOverPathBound(
    const Frame& frame, const ReferenceLineInfo& reference_line_info,
    PathBound* const path_bound) {
  // 1. Initialize the path boundaries to be an indefinitely large area.
  if (!InitPathBoundary(reference_line_info, path_bound)) {
    const std::string msg = "Failed to initialize path boundaries.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  // PathBoundsDebugString(*path_bound);

  // 2. Decide a rough boundary based on road boundary
  // 将path_bound更新为<left_road_edge, right_road_edge>
  if (!GetBoundaryFromRoads(reference_line_info, path_bound)) {
    const std::string msg =
        "Failed to decide a rough boundary based on road boundary.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  // PathBoundsDebugString(*path_bound);

  // 将path_bound的左右极限转换为相对于RefLine
  ConvertBoundarySAxisFromLaneCenterToRefLine(reference_line_info, path_bound);
  if (adc_frenet_l_ < std::get<1>(path_bound->front()) ||
      adc_frenet_l_ > std::get<2>(path_bound->front())) {
    const std::string msg =
        "ADC is outside road boundary already. Cannot generate pull-over path";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // 2. Update boundary by lane boundary for pull_over
  // 将path_bound的边界更新为<left_ego_lane, right_road_edge>
  UpdatePullOverBoundaryByLaneBoundary(reference_line_info, path_bound);
  // PathBoundsDebugString(*path_bound);

  // 3. Fine-tune the boundary based on static obstacles
  PathBound temp_path_bound = *path_bound;
  std::string blocking_obstacle_id;
  // 在path_bound中, 加入static_obstacle的边界信息:
  // -> 对每个静态障碍物, 基于预测的自车中心位置, 做left_nudge/right_nudge决策, 并用新的path_bound生成新的center_line
  // -> blocking_obstacle_id对应在当前path_bound中无法绕开的static_obstacle (挡在路中间)
  if (!GetBoundaryFromStaticObstacles(reference_line_info.path_decision(),
                                      path_bound, &blocking_obstacle_id)) {
    const std::string msg =
        "Failed to decide fine tune the boundaries after "
        "taking into consideration all static obstacles.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  // PathBoundsDebugString(*path_bound);

  auto* pull_over_status = injector_->planning_context()
                               ->mutable_planning_status()
                               ->mutable_pull_over();
  // If already found a pull-over position, simply check if it's valid.
  int curr_idx = -1;

  // 如果有pull_over position, 并且位于path_bound内, 则返回path_bound对应的idx
  if (pull_over_status->has_position()) {
    curr_idx = IsPointWithinPathBound(
        reference_line_info, pull_over_status->position().x(),
        pull_over_status->position().y(), *path_bound);
  }

  // If haven't found a pull-over position, search for one.
  if (curr_idx < 0) {
    auto pull_over_type = pull_over_status->pull_over_type();
    pull_over_status->Clear();
    pull_over_status->set_pull_over_type(pull_over_type);
    pull_over_status->set_plan_pull_over_path(true);

    std::tuple<double, double, double, int> pull_over_configuration;
    // 基于pull_over type与path bound寻找pull-over目标停车位置属性 pull_over_configuration
    // PULL_OVER目标停止在router destination附近
    // EMERGENCY_PULL_OVER目标停止在自车以最小半径运动估算的一个纵向距离处
    if (!SearchPullOverPosition(frame, reference_line_info, *path_bound,
                                &pull_over_configuration)) {
      const std::string msg = "Failed to find a proper pull-over position.";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }

    curr_idx = std::get<3>(pull_over_configuration);

    // If have found a pull-over position, update planning-context
    pull_over_status->mutable_position()->set_x(
        std::get<0>(pull_over_configuration));
    pull_over_status->mutable_position()->set_y(
        std::get<1>(pull_over_configuration));
    pull_over_status->mutable_position()->set_z(0.0);
    pull_over_status->set_theta(std::get<2>(pull_over_configuration));
    pull_over_status->set_length_front(FLAGS_obstacle_lon_start_buffer);
    pull_over_status->set_length_back(FLAGS_obstacle_lon_end_buffer);
    pull_over_status->set_width_left(
        VehicleConfigHelper::GetConfig().vehicle_param().width() / 2.0);
    pull_over_status->set_width_right(
        VehicleConfigHelper::GetConfig().vehicle_param().width() / 2.0);

    ADEBUG << "Pull Over: x[" << pull_over_status->position().x() << "] y["
           << pull_over_status->position().y() << "] theta["
           << pull_over_status->theta() << "]";
  }

  // Trim path-bound properly
  // 裁剪掉path bound中, pull_over_position之后的部分
  while (static_cast<int>(path_bound->size()) - 1 >
         curr_idx + kNumExtraTailBoundPoint) {
    path_bound->pop_back();
  }
  // 将pull_over_position后剩余的path_bound边界设置成与pull_over_position处一致
  for (size_t idx = curr_idx + 1; idx < path_bound->size(); ++idx) {
    std::get<1>((*path_bound)[idx]) = std::get<1>((*path_bound)[curr_idx]);
    std::get<2>((*path_bound)[idx]) = std::get<2>((*path_bound)[curr_idx]);
  }

  return Status::OK();
}

/* 生成fallback path_bound:
   仅考虑自车道(LaneBorrowInfo::NO_BORROW), 考虑自车motion limit信息 (在支持的横向加速度下横向速度为0) */
Status PathBoundsDecider::GenerateFallbackPathBound(
    const ReferenceLineInfo& reference_line_info, PathBound* const path_bound) {
  // 1. Initialize the path boundaries to be an indefinitely large area.
  if (!InitPathBoundary(reference_line_info, path_bound)) {
    const std::string msg = "Failed to initialize fallback path boundaries.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  // PathBoundsDebugString(*path_bound);

  // 2. Decide a rough boundary based on lane info and ADC's position
  std::string dummy_borrow_lane_type;
  if (!GetBoundaryFromLanesAndADC(reference_line_info,
                                  LaneBorrowInfo::NO_BORROW, 0.5, path_bound,
                                  &dummy_borrow_lane_type, true)) {
    const std::string msg =
        "Failed to decide a rough fallback boundary based on "
        "road information.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  // PathBoundsDebugString(*path_bound);

  ADEBUG << "Completed generating fallback path boundaries.";
  return Status::OK();
}

/* 检查输入的(x, y)坐标位于path_bound之内 */
int PathBoundsDecider::IsPointWithinPathBound(
    const ReferenceLineInfo& reference_line_info, const double x,
    const double y,
    const std::vector<std::tuple<double, double, double>>& path_bound) {
  common::SLPoint point_sl;
  reference_line_info.reference_line().XYToSL({x, y}, &point_sl);
  
  if (point_sl.s() > std::get<0>(path_bound.back()) ||
      point_sl.s() <
          std::get<0>(path_bound.front()) - kPathBoundsDeciderResolution * 2) {
    ADEBUG << "Longitudinally outside the boundary.";
    return -1;
  }

  int idx_after = 0;
  while (idx_after < static_cast<int>(path_bound.size()) &&
         std::get<0>(path_bound[idx_after]) < point_sl.s()) {
    ++idx_after;
  }

  ADEBUG << "The idx_after = " << idx_after;
  ADEBUG << "The boundary is: "
         << "[" << std::get<1>(path_bound[idx_after]) << ", "
         << std::get<2>(path_bound[idx_after]) << "].";
  ADEBUG << "The point is at: " << point_sl.l();
  int idx_before = idx_after - 1;
  if (std::get<1>(path_bound[idx_before]) <= point_sl.l() &&
      std::get<2>(path_bound[idx_before]) >= point_sl.l() &&
      std::get<1>(path_bound[idx_after]) <= point_sl.l() &&
      std::get<2>(path_bound[idx_after]) >= point_sl.l()) {
    return idx_after;
  }
  ADEBUG << "Laterally outside the boundary.";
  return -1;
}

/* 以Router Destination纵向位置为pull_over_s:
   -> 在自车距离destination过近, 或者path_bound长度达不到destination时返回false */
bool PathBoundsDecider::FindDestinationPullOverS(
    const Frame& frame, const ReferenceLineInfo& reference_line_info,
    const std::vector<std::tuple<double, double, double>>& path_bound,
    double* pull_over_s) {
  // destination_s based on routing_end
  const auto& reference_line = reference_line_info_->reference_line();
  common::SLPoint destination_sl;
  const auto& routing = frame.local_view().routing;
  const auto& routing_end = *(routing->routing_request().waypoint().rbegin());
  reference_line.XYToSL(routing_end.pose(), &destination_sl);
  const double destination_s = destination_sl.s();
  const double adc_end_s = reference_line_info.AdcSlBoundary().end_s();

  // Check if destination is some distance away from ADC.
  // 如果destination离自车位置小于25m, 则返回false
  ADEBUG << "Destination s[" << destination_s << "] adc_end_s[" << adc_end_s
         << "]";
  if (destination_s - adc_end_s < config_.path_bounds_decider_config()
                                      .pull_over_destination_to_adc_buffer())  // 25m
  {
    AERROR << "Destination is too close to ADC. distance["
           << destination_s - adc_end_s << "]";
    return false;
  }

  // Check if destination is within path-bounds searching scope.
  // 如果path_bound length < destination + 4, 返回false
  const double destination_to_pathend_buffer =
      config_.path_bounds_decider_config()
          .pull_over_destination_to_pathend_buffer();   // 4.0m
  if (destination_s + destination_to_pathend_buffer >=
      std::get<0>(path_bound.back())) {
    AERROR << "Destination is not within path_bounds search scope";
    return false;
  }

  *pull_over_s = destination_s;
  return true;
}

/* 以自车在最小半径下运动的假设, 返回对应紧急靠边停车终点的纵向距离: pull_over_s = min_turn_radius * 2 * 1.5 */
bool PathBoundsDecider::FindEmergencyPullOverS(
    const ReferenceLineInfo& reference_line_info, double* pull_over_s) {
  const double adc_end_s = reference_line_info.AdcSlBoundary().end_s();
  const double min_turn_radius = common::VehicleConfigHelper::Instance()
                                     ->GetConfig()
                                     .vehicle_param()
                                     .min_turn_radius();
  const double adjust_factor =
      config_.path_bounds_decider_config()
          .pull_over_approach_lon_distance_adjust_factor();  // 1.5
  const double pull_over_distance = min_turn_radius * 2 * adjust_factor;
  *pull_over_s = adc_end_s + pull_over_distance;

  return true;
}

/* 基于pull_over type (是否Emergency), 计算对应的pull_over目标点位置属性pull_over_configuration
   -> PULL_OVER目标位置点在router终点处, EMERGENCY_PULL_OVER目标位置点在自车最小运动半径估算出的一个纵向距离之后
   -> 从高该目标点, 在path_bound内向前/向后搜索一个适合停车的feasible_window (固定长宽, 靠近路边, 合理的横向buffer)
   -> 如果找到feasible window, 则基于它得到最终目标的pull_over position */
bool PathBoundsDecider::SearchPullOverPosition(
    const Frame& frame, const ReferenceLineInfo& reference_line_info,
    const std::vector<std::tuple<double, double, double>>& path_bound,
    std::tuple<double, double, double, int>* const pull_over_configuration) {
  const auto& pull_over_status =
      injector_->planning_context()->planning_status().pull_over();

  // search direction
  bool search_backward = false;  // search FORWARD by default

  double pull_over_s = 0.0;
  // 搜索自车(紧急)靠边停车目标终点的纵向距离
  // 当前为EMERGENCY_PULL_OVER时, 假设自车以最小转向半径运动, 计算pull_over_s
  if (pull_over_status.pull_over_type() ==
      PullOverStatus::EMERGENCY_PULL_OVER) {
    if (!FindEmergencyPullOverS(reference_line_info, &pull_over_s)) {
      AERROR << "Failed to find emergency_pull_over s";
      return false;
    }
    search_backward = false;  // search FORWARD from target position
  } 
  // 当前为PULL_OVER时, 以route终点对应纵向距离为目标, 计算pull_over_s
  else if (pull_over_status.pull_over_type() == PullOverStatus::PULL_OVER) {
    if (!FindDestinationPullOverS(frame, reference_line_info, path_bound, &pull_over_s)) {
      AERROR << "Failed to find pull_over s upon destination arrival";
      return false;
    }
    search_backward = true;  // search BACKWARD from target position
  } else {
    return false;
  }

  int idx = 0;
  // 从后向前/从前向后, 搜索pull_over_s对应的path_bound的index
  if (search_backward) {
    // 1. Locate the first point before destination.
    idx = static_cast<int>(path_bound.size()) - 1;
    while (idx >= 0 && std::get<0>(path_bound[idx]) > pull_over_s) {
      --idx;
    }
  } else {
    // 1. Locate the first point after emergency_pull_over s.
    while (idx < static_cast<int>(path_bound.size()) &&
           std::get<0>(path_bound[idx]) < pull_over_s) {
      ++idx;
    }
  }

  // 如果搜索不到对应的index, 返回false
  if (idx < 0 || idx >= static_cast<int>(path_bound.size())) {
    AERROR << "Failed to find path_bound index for pull over s";
    return false;
  }

  // Search for a feasible location for pull-over.
  // pull_over空间的长度: 1.5 * length
  const double pull_over_space_length =
      kPulloverLonSearchCoeff *
          VehicleConfigHelper::GetConfig().vehicle_param().length() -
      FLAGS_obstacle_lon_start_buffer - FLAGS_obstacle_lon_end_buffer;
  // pull_over空间的宽度: 0.25 * width
  const double pull_over_space_width =
      (kPulloverLatSearchCoeff - 1.0) *
      VehicleConfigHelper::GetConfig().vehicle_param().width();
  const double adc_half_width =
      VehicleConfigHelper::GetConfig().vehicle_param().width() / 2.0;

  // 2. Find a window that is close to road-edge.
  // (not in any intersection)
  // 从pull_over_s对应的path_bound位置向前/向后搜索, 看是否可以找到一个feasible_range
  // idx对应feasible_range的边框起点, 在下面的循环中, 会先检查idx对应点是否ok, 如果ok, 
  // 再检查从该点扩展的feasible_range矩形框是否ok (内部嵌套的while循环实现)
  // 如果检测到feasible_range, 基于其生成pull_over destination属性pull_over_configuration
  bool has_a_feasible_window = false;
  while ((search_backward && idx >= 0 &&
          std::get<0>(path_bound[idx]) - std::get<0>(path_bound.front()) >
              pull_over_space_length) ||
         (!search_backward && idx < static_cast<int>(path_bound.size()) &&
          std::get<0>(path_bound.back()) - std::get<0>(path_bound[idx]) >
              pull_over_space_length)) {
    int j = idx;
    bool is_feasible_window = true;

    // Check if the point of idx is within intersection.
    double pt_ref_line_s = std::get<0>(path_bound[idx]);
    double pt_ref_line_l = 0.0;
    common::SLPoint pt_sl;
    pt_sl.set_s(pt_ref_line_s);
    pt_sl.set_l(pt_ref_line_l);
    common::math::Vec2d pt_xy;
    reference_line_info.reference_line().SLToXY(pt_sl, &pt_xy);
    common::PointENU hdmap_point;
    hdmap_point.set_x(pt_xy.x());
    hdmap_point.set_y(pt_xy.y());
    ADEBUG << "Pull-over position might be around (" << pt_xy.x() << ", "
           << pt_xy.y() << ")";
    std::vector<std::shared_ptr<const JunctionInfo>> junctions;
    HDMapUtil::BaseMap().GetJunctions(hdmap_point, 1.0, &junctions);

    // 如果idx对应的点位于intersection, 则不满足条件, 继续检查下一个点
    if (!junctions.empty()) {
      AWARN << "Point is in PNC-junction.";
      idx = search_backward ? idx - 1 : idx + 1;
      continue;
    }

    // 以path_bound[idx].s为起点, 向前/向后搜索一个固定长宽的feasible range: 
    // [length: pull_over_space_length, width: pull_over_space_width]
    // 如果搜索到, 则基于当前位置计算pull_over_position, 否则idx向前/向后运动, 继续搜索
    while ((search_backward && j >= 0 &&
            std::get<0>(path_bound[idx]) - std::get<0>(path_bound[j]) <
                pull_over_space_length) ||
           (!search_backward && j < static_cast<int>(path_bound.size()) &&
            std::get<0>(path_bound[j]) - std::get<0>(path_bound[idx]) <
                pull_over_space_length)) {
      double curr_s = std::get<0>(path_bound[j]);
      double curr_right_bound = std::fabs(std::get<1>(path_bound[j]));
      double curr_road_left_width = 0;
      double curr_road_right_width = 0;
      reference_line_info.reference_line().GetRoadWidth(
          curr_s, &curr_road_left_width, &curr_road_right_width);
      ADEBUG << "s[" << curr_s << "] curr_road_left_width["
             << curr_road_left_width << "] curr_road_right_width["
             << curr_road_right_width << "]";
      
      // 如果发现path_bound的右边界 距离 road_bound较远, 则退出向前/向后搜索下一个可行区间
      if (curr_road_right_width - (curr_right_bound + adc_half_width) >
          config_.path_bounds_decider_config().pull_over_road_edge_buffer()) {
        AERROR << "Not close enough to road-edge. Not feasible for pull-over.";
        is_feasible_window = false;
        break;
      }
      const double right_bound = std::get<1>(path_bound[j]);
      const double left_bound = std::get<2>(path_bound[j]);
      ADEBUG << "left_bound[" << left_bound << "] right_bound[" << right_bound
             << "]";
      
      // 如果不满足: 左边界-右边界 < pull_over_space_width (最少在pull_over位置要预留0.25*width的buffer)
      // 则退出向前/向后搜索下一个可行区间
      if (left_bound - right_bound < pull_over_space_width) {
        AERROR << "Not wide enough to fit ADC. Not feasible for pull-over.";
        is_feasible_window = false;
        break;
      }

      j = search_backward ? j - 1 : j + 1;
    }
    
    if (j < 0) {
      return false;
    }
    
    // 如果我们找到一个feasible_window [length: pull_over_space_length, width: pull_over_space_width]
    // 首先基于该feasible_window计算pull_over destination point属性 -> pull_over_configuration
    // 然后退出结束循环搜索
    if (is_feasible_window) {
      has_a_feasible_window = true;
      const auto& reference_line = reference_line_info.reference_line();
      // estimate pull over point to have the vehicle keep same safety distance
      // to front and back
      const auto& vehicle_param =
          VehicleConfigHelper::GetConfig().vehicle_param();
      const double back_clear_to_total_length_ratio =
          (0.5 * (kPulloverLonSearchCoeff - 1.0) * vehicle_param.length() +
           vehicle_param.back_edge_to_center()) /
          vehicle_param.length() / kPulloverLonSearchCoeff;

      int start_idx = j;
      int end_idx = idx;
      if (!search_backward) {
        start_idx = idx;
        end_idx = j;
      }
      auto pull_over_idx = static_cast<size_t>(
          back_clear_to_total_length_ratio * static_cast<double>(end_idx) +
          (1.0 - back_clear_to_total_length_ratio) *
              static_cast<double>(start_idx));

      const auto& pull_over_point = path_bound[pull_over_idx];
      const double pull_over_s = std::get<0>(pull_over_point);
      const double pull_over_l =
          std::get<1>(pull_over_point) + pull_over_space_width / 2.0;
      common::SLPoint pull_over_sl_point;
      pull_over_sl_point.set_s(pull_over_s);
      pull_over_sl_point.set_l(pull_over_l);

      common::math::Vec2d pull_over_xy_point;
      reference_line.SLToXY(pull_over_sl_point, &pull_over_xy_point);
      const double pull_over_x = pull_over_xy_point.x();
      const double pull_over_y = pull_over_xy_point.y();

      // set the pull over theta to be the nearest lane theta rather than
      // reference line theta in case of reference line theta not aligned with
      // the lane
      const auto& reference_point =
          reference_line.GetReferencePoint(pull_over_s);
      double pull_over_theta = reference_point.heading();
      hdmap::LaneInfoConstPtr lane;
      double s = 0.0;
      double l = 0.0;
      auto point =
          common::util::PointFactory::ToPointENU(pull_over_x, pull_over_y);
      if (HDMapUtil::BaseMap().GetNearestLaneWithHeading(
              point, 5.0, pull_over_theta, M_PI_2, &lane, &s, &l) == 0) {
        pull_over_theta = lane->Heading(s);
      }
      *pull_over_configuration =
          std::make_tuple(pull_over_x, pull_over_y, pull_over_theta,
                          static_cast<int>(pull_over_idx));
      break;
    }

    idx = search_backward ? idx - 1 : idx + 1;
  }

  // 返回当前是否成功找到feasible_window
  return has_a_feasible_window;
}

void PathBoundsDecider::RemoveRedundantPathBoundaries(
    std::vector<PathBoundary>* const candidate_path_boundaries) {
  // 1. Check to see if both "left" and "right" exist.
  bool is_left_exist = false;
  std::vector<std::pair<double, double>> left_boundary;
  bool is_right_exist = false;
  std::vector<std::pair<double, double>> right_boundary;
  for (const auto& path_boundary : *candidate_path_boundaries) {
    if (path_boundary.label().find("left") != std::string::npos) {
      is_left_exist = true;
      left_boundary = path_boundary.boundary();
    }
    if (path_boundary.label().find("right") != std::string::npos) {
      is_right_exist = true;
      right_boundary = path_boundary.boundary();
    }
  }
  // 2. Check if "left" is contained by "right", and vice versa.
  if (!is_left_exist || !is_right_exist) {
    return;
  }
  bool is_left_redundant = false;
  bool is_right_redundant = false;
  if (IsContained(left_boundary, right_boundary)) {
    is_left_redundant = true;
  }
  if (IsContained(right_boundary, left_boundary)) {
    is_right_redundant = true;
  }

  // 3. If one contains the other, then remove the redundant one.
  for (size_t i = 0; i < candidate_path_boundaries->size(); ++i) {
    const auto& path_boundary = (*candidate_path_boundaries)[i];
    if (path_boundary.label().find("right") != std::string::npos &&
        is_right_redundant) {
      (*candidate_path_boundaries)[i] = candidate_path_boundaries->back();
      candidate_path_boundaries->pop_back();
      break;
    }
    if (path_boundary.label().find("left") != std::string::npos &&
        is_left_redundant) {
      (*candidate_path_boundaries)[i] = candidate_path_boundaries->back();
      candidate_path_boundaries->pop_back();
      break;
    }
  }
}

bool PathBoundsDecider::IsContained(
    const std::vector<std::pair<double, double>>& lhs,
    const std::vector<std::pair<double, double>>& rhs) {
  if (lhs.size() > rhs.size()) {
    return false;
  }
  for (size_t i = 0; i < lhs.size(); ++i) {
    if (lhs[i].first < rhs[i].first) {
      return false;
    }
    if (lhs[i].second > rhs[i].second) {
      return false;
    }
  }
  return true;
}

/* 初始化path_bound (adc_position -> decision horizon):
   -> 左右边界初始化为<最小值, 最大值> */
bool PathBoundsDecider::InitPathBoundary(
    const ReferenceLineInfo& reference_line_info, PathBound* const path_bound) {
  // Sanity checks.
  CHECK_NOTNULL(path_bound);
  path_bound->clear();
  const auto& reference_line = reference_line_info.reference_line();

  // Starting from ADC's current position, increment until the horizon, and
  // set lateral bounds to be infinite at every spot.
  for (double curr_s = adc_frenet_s_;
       curr_s < std::fmin(adc_frenet_s_ +
                              std::fmax(kPathBoundsDeciderHorizon,
                                        reference_line_info.GetCruiseSpeed() *
                                            FLAGS_trajectory_time_length),
                          reference_line.Length());
       curr_s += kPathBoundsDeciderResolution) {
    path_bound->emplace_back(curr_s, std::numeric_limits<double>::lowest(),
                             std::numeric_limits<double>::max());
  }

  // Return.
  if (path_bound->empty()) {
    ADEBUG << "Empty path boundary in InitPathBoundary";
    return false;
  }
  return true;
}

/* 基于当前Road Width, 生成PathBound (每个纵向位置对应的左右极限) */
bool PathBoundsDecider::GetBoundaryFromRoads(
    const ReferenceLineInfo& reference_line_info, PathBound* const path_bound) {
  // Sanity checks.
  CHECK_NOTNULL(path_bound);
  ACHECK(!path_bound->empty());
  const ReferenceLine& reference_line = reference_line_info.reference_line();

  // Go through every point, update the boudnary based on the road boundary.
  double past_road_left_width = adc_lane_width_ / 2.0;
  double past_road_right_width = adc_lane_width_ / 2.0;
  int path_blocked_idx = -1;
  for (size_t i = 0; i < path_bound->size(); ++i) {
    // 1. Get road boundary.
    double curr_s = std::get<0>((*path_bound)[i]);
    double curr_road_left_width = 0.0;
    double curr_road_right_width = 0.0;
    double refline_offset_to_lane_center = 0.0;
    reference_line.GetOffsetToMap(curr_s, &refline_offset_to_lane_center);
    if (!reference_line.GetRoadWidth(curr_s, &curr_road_left_width,
                                     &curr_road_right_width)) {
      AWARN << "Failed to get lane width at s = " << curr_s;
      curr_road_left_width = past_road_left_width;
      curr_road_right_width = past_road_right_width;
    } else {
      curr_road_left_width += refline_offset_to_lane_center;
      curr_road_right_width -= refline_offset_to_lane_center;
      past_road_left_width = curr_road_left_width;
      past_road_right_width = curr_road_right_width;
    }
    double curr_left_bound = curr_road_left_width;
    double curr_right_bound = -curr_road_right_width;
    ADEBUG << "At s = " << curr_s
           << ", left road bound = " << curr_road_left_width
           << ", right road bound = " << curr_road_right_width
           << ", offset from refline to lane-center = "
           << refline_offset_to_lane_center;

    // 2. Update into path_bound.
    if (!UpdatePathBoundaryWithBuffer(i, curr_left_bound, curr_right_bound,
                                      path_bound)) {
      path_blocked_idx = static_cast<int>(i);
    }
    if (path_blocked_idx != -1) {
      break;
    }
  }

  TrimPathBounds(path_blocked_idx, path_bound);
  return true;
}

/* 考虑lane_borrow_info, 基于当前EgoLane/NeighborLane的边界, 生成PathBound (每个纵向位置对应的左右极限) */
bool PathBoundsDecider::GetBoundaryFromLanes(
    const ReferenceLineInfo& reference_line_info,
    const LaneBorrowInfo& lane_borrow_info, PathBound* const path_bound,
    std::string* const borrow_lane_type) {
  // Sanity checks.
  CHECK_NOTNULL(path_bound);
  ACHECK(!path_bound->empty());
  const ReferenceLine& reference_line = reference_line_info.reference_line();

  // Go through every point, update the boundary based on lane-info.
  double past_lane_left_width = adc_lane_width_ / 2.0;
  double past_lane_right_width = adc_lane_width_ / 2.0;
  int path_blocked_idx = -1;
  bool borrowing_reverse_lane = false;
  for (size_t i = 0; i < path_bound->size(); ++i) {
    double curr_s = std::get<0>((*path_bound)[i]);

    // 1. Get the current lane width at current point.
    double curr_lane_left_width = 0.0;
    double curr_lane_right_width = 0.0;
    if (!reference_line.GetLaneWidth(curr_s, &curr_lane_left_width,
                                     &curr_lane_right_width)) {
      AWARN << "Failed to get lane width at s = " << curr_s;
      curr_lane_left_width = past_lane_left_width;
      curr_lane_right_width = past_lane_right_width;
    } else {
      double offset_to_lane_center = 0.0;
      reference_line.GetOffsetToMap(curr_s, &offset_to_lane_center);
      // The left-width and right-width are w.r.t. lane-center, not ref-line.
      curr_lane_left_width += offset_to_lane_center;
      curr_lane_right_width -= offset_to_lane_center;
      past_lane_left_width = curr_lane_left_width;
      past_lane_right_width = curr_lane_right_width;
    }

    // 2. Get the neighbor lane widths at the current point.
    double curr_neighbor_lane_width = 0.0;
    if (CheckLaneBoundaryType(reference_line_info, curr_s, lane_borrow_info)) {
      hdmap::Id neighbor_lane_id;
      if (lane_borrow_info == LaneBorrowInfo::LEFT_BORROW) {
        // Borrowing left neighbor lane.
        if (reference_line_info.GetNeighborLaneInfo(
                ReferenceLineInfo::LaneType::LeftForward, curr_s,
                &neighbor_lane_id, &curr_neighbor_lane_width)) {
          ADEBUG << "Borrow left forward neighbor lane.";
        } else if (reference_line_info.GetNeighborLaneInfo(
                       ReferenceLineInfo::LaneType::LeftReverse, curr_s,
                       &neighbor_lane_id, &curr_neighbor_lane_width)) {
          borrowing_reverse_lane = true;
          ADEBUG << "Borrow left reverse neighbor lane.";
        } else {
          ADEBUG << "There is no left neighbor lane.";
        }
      } else if (lane_borrow_info == LaneBorrowInfo::RIGHT_BORROW) {
        // Borrowing right neighbor lane.
        if (reference_line_info.GetNeighborLaneInfo(
                ReferenceLineInfo::LaneType::RightForward, curr_s,
                &neighbor_lane_id, &curr_neighbor_lane_width)) {
          ADEBUG << "Borrow right forward neighbor lane.";
        } else if (reference_line_info.GetNeighborLaneInfo(
                       ReferenceLineInfo::LaneType::RightReverse, curr_s,
                       &neighbor_lane_id, &curr_neighbor_lane_width)) {
          borrowing_reverse_lane = true;
          ADEBUG << "Borrow right reverse neighbor lane.";
        } else {
          ADEBUG << "There is no right neighbor lane.";
        }
      }
    }

    // 3. Get the proper boundary
    double curr_left_bound =
        curr_lane_left_width + (lane_borrow_info == LaneBorrowInfo::LEFT_BORROW
                                    ? curr_neighbor_lane_width
                                    : 0.0);
    double curr_right_bound = -curr_lane_right_width -
                              (lane_borrow_info == LaneBorrowInfo::RIGHT_BORROW
                                   ? curr_neighbor_lane_width
                                   : 0.0);
    ADEBUG << "At s = " << curr_s << ", left_lane_bound = " << curr_left_bound
           << ", right_lane_bound = " << curr_right_bound;

    // 4. Update the boundary.
    if (!UpdatePathBoundary(i, curr_left_bound, curr_right_bound, path_bound)) {
      path_blocked_idx = static_cast<int>(i);
    }
    if (path_blocked_idx != -1) {
      break;
    }
  }
  TrimPathBounds(path_blocked_idx, path_bound);

  if (lane_borrow_info == LaneBorrowInfo::NO_BORROW) {
    *borrow_lane_type = "";
  } else {
    *borrow_lane_type = borrowing_reverse_lane ? "reverse" : "forward";
  }
  return true;
}

bool PathBoundsDecider::GetBoundaryFromADC(
    const ReferenceLineInfo& reference_line_info, double ADC_extra_buffer,
    PathBound* const path_bound) {
  // Sanity checks.
  CHECK_NOTNULL(path_bound);
  ACHECK(!path_bound->empty());

  // Calculate the ADC's lateral boundary.
  static constexpr double kMaxLateralAccelerations = 1.5;
  double ADC_lat_decel_buffer = (adc_frenet_ld_ > 0 ? 1.0 : -1.0) *
                                adc_frenet_ld_ * adc_frenet_ld_ /
                                kMaxLateralAccelerations / 2.0;
  double curr_left_bound_adc =
      GetBufferBetweenADCCenterAndEdge() + ADC_extra_buffer +
      std::fmax(adc_l_to_lane_center_,
                adc_l_to_lane_center_ + ADC_lat_decel_buffer);
  double curr_right_bound_adc =
      -GetBufferBetweenADCCenterAndEdge() - ADC_extra_buffer +
      std::fmin(adc_l_to_lane_center_,
                adc_l_to_lane_center_ + ADC_lat_decel_buffer);

  // Expand the boundary in case ADC falls outside.
  for (size_t i = 0; i < path_bound->size(); ++i) {
    double curr_left_bound = std::get<2>((*path_bound)[i]);
    curr_left_bound = std::fmax(curr_left_bound_adc, curr_left_bound);
    double curr_right_bound = std::get<1>((*path_bound)[i]);
    curr_right_bound = std::fmin(curr_right_bound_adc, curr_right_bound);
    UpdatePathBoundary(i, curr_left_bound, curr_right_bound, path_bound);
  }
  return true;
}

// TODO(jiacheng): this function is to be retired soon.
// 考虑车道信息与自车横向运动极限, 生成path_bound
// 在输入的lane_borrow_info为LEFT_BORROW/RIGHT_BORROW时, 将path_bound扩展到相邻车道
// is_fallback_lanechange: 表示该path_bound对应fallback或lane_change场景, 在对应场景下, path_bound要考虑自车运动极限
bool PathBoundsDecider::GetBoundaryFromLanesAndADC(
    const ReferenceLineInfo& reference_line_info,
    const LaneBorrowInfo& lane_borrow_info, double ADC_buffer,
    PathBound* const path_bound, std::string* const borrow_lane_type,
    bool is_fallback_lanechange) {
  // Sanity checks.
  CHECK_NOTNULL(path_bound);
  ACHECK(!path_bound->empty());
  const ReferenceLine& reference_line = reference_line_info.reference_line();
  bool is_left_lane_boundary = true;
  bool is_right_lane_boundary = true;
  const double boundary_buffer = 0.05;  // meter

  // Go through every point, update the boundary based on lane info and
  // ADC's position.
  double past_lane_left_width = adc_lane_width_ / 2.0;
  double past_lane_right_width = adc_lane_width_ / 2.0;
  int path_blocked_idx = -1;
  bool borrowing_reverse_lane = false;
  // loop over all path bounds point, and generate path bounds for each s distance
  // 考虑lane_borrow_info (LEFT_BORROW/RIGHT_BORROW)
  for (size_t i = 0; i < path_bound->size(); ++i) {
    double curr_s = std::get<0>((*path_bound)[i]);
    // 1. Get the current lane width at current point.
    // curr_lane_left_width: 自车左车道线到车道中心的距离
    // curr_lane_right_width: 自车右车道线到车道中心的距离
    double curr_lane_left_width = 0.0;
    double curr_lane_right_width = 0.0;
    double offset_to_lane_center = 0.0;
    if (!reference_line.GetLaneWidth(curr_s, &curr_lane_left_width,
                                     &curr_lane_right_width)) {
      AWARN << "Failed to get lane width at s = " << curr_s;
      curr_lane_left_width = past_lane_left_width;
      curr_lane_right_width = past_lane_right_width;
    } else {
      // check if lane boundary is also road boundary
      double curr_road_left_width = 0.0;
      double curr_road_right_width = 0.0;
      // is_left/right_lane_boundary表示当前bound信息是产生于lane_boundary还是road_boundary
      if (reference_line.GetRoadWidth(curr_s, &curr_road_left_width,
                                      &curr_road_right_width)) {
        is_left_lane_boundary =
            (std::abs(curr_road_left_width - curr_lane_left_width) >
             boundary_buffer);
        is_right_lane_boundary =
            (std::abs(curr_road_right_width - curr_lane_right_width) >
             boundary_buffer);
      }
      reference_line.GetOffsetToMap(curr_s, &offset_to_lane_center);
      curr_lane_left_width += offset_to_lane_center;
      curr_lane_right_width -= offset_to_lane_center;
      past_lane_left_width = curr_lane_left_width;
      past_lane_right_width = curr_lane_right_width;
    }

    // 2. Get the neighbor lane widths at the current point.
    double curr_neighbor_lane_width = 0.0;
    // 基于输入的lane_borrow_info, 检查对应的lane boundary type是否符合条件 (非实线)
    if (CheckLaneBoundaryType(reference_line_info, curr_s, lane_borrow_info)) {
      hdmap::Id neighbor_lane_id;
      // 对于LEFT_BORROW:
      // curr_neighbor_lane_width: 左侧车道的宽度
      if (lane_borrow_info == LaneBorrowInfo::LEFT_BORROW) {
        // Borrowing left neighbor lane.
        if (reference_line_info.GetNeighborLaneInfo(
                ReferenceLineInfo::LaneType::LeftForward, curr_s,
                &neighbor_lane_id, &curr_neighbor_lane_width)) {
          ADEBUG << "Borrow left forward neighbor lane.";
        } else if (reference_line_info.GetNeighborLaneInfo(
                       ReferenceLineInfo::LaneType::LeftReverse, curr_s,
                       &neighbor_lane_id, &curr_neighbor_lane_width)) {
          borrowing_reverse_lane = true;
          ADEBUG << "Borrow left reverse neighbor lane.";
        } else {
          ADEBUG << "There is no left neighbor lane.";
        }
      }
      // 对于RIGHT_BORROW:
      // curr_neighbor_lane_width: 右侧车道的宽度
      else if (lane_borrow_info == LaneBorrowInfo::RIGHT_BORROW) {
        // Borrowing right neighbor lane.
        if (reference_line_info.GetNeighborLaneInfo(
                ReferenceLineInfo::LaneType::RightForward, curr_s,
                &neighbor_lane_id, &curr_neighbor_lane_width)) {
          ADEBUG << "Borrow right forward neighbor lane.";
        } else if (reference_line_info.GetNeighborLaneInfo(
                       ReferenceLineInfo::LaneType::RightReverse, curr_s,
                       &neighbor_lane_id, &curr_neighbor_lane_width)) {
          borrowing_reverse_lane = true;
          ADEBUG << "Borrow right reverse neighbor lane.";
        } else {
          ADEBUG << "There is no right neighbor lane.";
        }
      }
    }

    // 3. Calculate the proper boundary based on lane-width, ADC's position,
    //    and ADC's velocity.
    static constexpr double kMaxLateralAccelerations = 1.5;
    double offset_to_map = 0.0;
    reference_line.GetOffsetToMap(curr_s, &offset_to_map);

    // ADC_speed_buffer: 自车在最大横向加速度下达到横向速度为0, 对应的横向移动距离
    double ADC_speed_buffer = (adc_frenet_ld_ > 0 ? 1.0 : -1.0) *
                              adc_frenet_ld_ * adc_frenet_ld_ /
                              kMaxLateralAccelerations / 2.0;

    // 当前s距离下, 基于车道线产生的左右边界curr_left_bound_lane/curr_right_bound_lane
    double curr_left_bound_lane =
        curr_lane_left_width + (lane_borrow_info == LaneBorrowInfo::LEFT_BORROW
                                    ? curr_neighbor_lane_width
                                    : 0.0);

    double curr_right_bound_lane =
        -curr_lane_right_width -
        (lane_borrow_info == LaneBorrowInfo::RIGHT_BORROW
             ? curr_neighbor_lane_width
             : 0.0);

    double curr_left_bound = 0.0;
    double curr_right_bound = 0.0;

    // 决策left/right boundary是否需要考虑自车横向运动的产生的边界 (默认 is_extend_lane_bounds_to_include_adc = false)
    // curr_left_bound_adc / curr_right_bound_adc分别为自车在最大横向加速度下将横向速度降为0 对应的边界
    if (config_.path_bounds_decider_config().is_extend_lane_bounds_to_include_adc() || is_fallback_lanechange) {
      // extend path bounds to include ADC in fallback or change lane path
      // bounds.
      double curr_left_bound_adc =
          std::fmax(adc_l_to_lane_center_,
                    adc_l_to_lane_center_ + ADC_speed_buffer) +
          GetBufferBetweenADCCenterAndEdge() + ADC_buffer;
      curr_left_bound =
          std::fmax(curr_left_bound_lane, curr_left_bound_adc) - offset_to_map;

      double curr_right_bound_adc =
          std::fmin(adc_l_to_lane_center_,
                    adc_l_to_lane_center_ + ADC_speed_buffer) -
          GetBufferBetweenADCCenterAndEdge() - ADC_buffer;
      curr_right_bound =
          std::fmin(curr_right_bound_lane, curr_right_bound_adc) -
          offset_to_map;
    }
    // 否则只考虑左右车道线产生的boundary
    else
    {
      curr_left_bound = curr_left_bound_lane - offset_to_map;
      curr_right_bound = curr_right_bound_lane - offset_to_map;
    }

    ADEBUG << "At s = " << curr_s
           << ", left_lane_bound = " << curr_lane_left_width
           << ", right_lane_bound = " << curr_lane_right_width
           << ", offset = " << offset_to_map;

    // 4. Update the boundary.
    if (!UpdatePathBoundaryWithBuffer(i, curr_left_bound, curr_right_bound,
                                      path_bound, is_left_lane_boundary,
                                      is_right_lane_boundary)) {
      path_blocked_idx = static_cast<int>(i);
    }
    if (path_blocked_idx != -1) {
      break;
    }
  }

  // 在生成的path_bound中, 剔除掉path_block的部分 (l_right > l_left)
  TrimPathBounds(path_blocked_idx, path_bound);

  if (lane_borrow_info == LaneBorrowInfo::NO_BORROW) {
    *borrow_lane_type = "";
  } else {
    *borrow_lane_type = borrowing_reverse_lane ? "reverse" : "forward";
  }

  return true;
}

// update boundaries with corresponding one-side lane boundary for pull over
// (1) use left lane boundary for normal PULL_OVER type
// (2) use left/right(which is opposite to pull over direction
//     (pull over at closer road side) lane boundary for EMERGENCY_PULL_OVER
/* 对于PullOver/EmergencyPullOver场景, 将path_bound更新为<left_ego_lane, right_road_edge> */
void PathBoundsDecider::UpdatePullOverBoundaryByLaneBoundary(
    const ReferenceLineInfo& reference_line_info, PathBound* const path_bound) {
  const ReferenceLine& reference_line = reference_line_info.reference_line();
  const auto& pull_over_status =
      injector_->planning_context()->planning_status().pull_over();
  const auto pull_over_type = pull_over_status.pull_over_type();
  if (pull_over_type != PullOverStatus::PULL_OVER &&
      pull_over_type != PullOverStatus::EMERGENCY_PULL_OVER) {
    return;
  }

  for (size_t i = 0; i < path_bound->size(); ++i) {
    const double curr_s = std::get<0>((*path_bound)[i]);
    double left_bound = 3.0;
    double right_bound = 3.0;
    double curr_lane_left_width = 0.0;
    double curr_lane_right_width = 0.0;
    if (reference_line.GetLaneWidth(curr_s, &curr_lane_left_width,
                                    &curr_lane_right_width)) {
      double offset_to_lane_center = 0.0;
      reference_line.GetOffsetToMap(curr_s, &offset_to_lane_center);
      // 将left/right_bound从相对于车道线中心转换到相对于reference_line
      left_bound = curr_lane_left_width + offset_to_lane_center;
      right_bound = curr_lane_right_width + offset_to_lane_center;
    }
    ADEBUG << "left_bound[" << left_bound << "] right_bound[" << right_bound
           << "]";
    // 将path_bound的左边界更新为当前ego lane left marker, 右边界保持为road edge
    if (pull_over_type == PullOverStatus::PULL_OVER) {
      std::get<2>((*path_bound)[i]) = left_bound;
    } else if (pull_over_type == PullOverStatus::EMERGENCY_PULL_OVER) {
      // TODO(all): use left/right lane boundary accordingly
      std::get<2>((*path_bound)[i]) = left_bound;
    }
  }
}

/* 对每一个纵向位置s, 将Refline相对于laneCenter的横向距离补偿到PathBound的左右极限 */
void PathBoundsDecider::ConvertBoundarySAxisFromLaneCenterToRefLine(
    const ReferenceLineInfo& reference_line_info, PathBound* const path_bound) {
  const ReferenceLine& reference_line = reference_line_info.reference_line();
  for (size_t i = 0; i < path_bound->size(); ++i) {
    // 1. Get road boundary.
    double curr_s = std::get<0>((*path_bound)[i]);
    double refline_offset_to_lane_center = 0.0;
    reference_line.GetOffsetToMap(curr_s, &refline_offset_to_lane_center);
    std::get<1>((*path_bound)[i]) -= refline_offset_to_lane_center;
    std::get<2>((*path_bound)[i]) -= refline_offset_to_lane_center;
  }
}

/* 计算lane_change_start_position, 并对其前的path_bound进行更新 */
void PathBoundsDecider::GetBoundaryFromLaneChangeForbiddenZone(
    const ReferenceLineInfo& reference_line_info, PathBound* const path_bound) {
  // Sanity checks.
  CHECK_NOTNULL(path_bound);
  const ReferenceLine& reference_line = reference_line_info.reference_line();

  // If there is a pre-determined lane-change starting position, then use it;
  // otherwise, decide one.
  auto* lane_change_status = injector_->planning_context()
                                 ->mutable_planning_status()
                                 ->mutable_change_lane();
  // 当前lane change ok
  // 如果当前obstacle的time-gap均满足要求, 表示比较空旷, 可以直接变道, 不需要对path_bound进行修改
  if (lane_change_status->is_clear_to_change_lane()) {
    ADEBUG << "Current position is clear to change lane. No need prep s.";
    lane_change_status->set_exist_lane_change_start_position(false);
    return;
  }
  
  double lane_change_start_s = 0.0;
  // 如果有lane_change_start_position, 则计算lane_change_start_position在当前时刻的s distance, 基于其对path bound进行更新
  // 如果没有lane_change_start_position, 则将其固定设为自车当前向前80m
  if (lane_change_status->exist_lane_change_start_position()) {
    common::SLPoint point_sl;
    reference_line.XYToSL(lane_change_status->lane_change_start_position(),
                          &point_sl);
    lane_change_start_s = point_sl.s();
  } else {
    // TODO(jiacheng): train ML model to learn this.
    lane_change_start_s = FLAGS_lane_change_prepare_length + adc_frenet_s_;

    // Update the decided lane_change_start_s into planning-context.
    common::SLPoint lane_change_start_sl;
    lane_change_start_sl.set_s(lane_change_start_s);
    lane_change_start_sl.set_l(0.0);
    common::math::Vec2d lane_change_start_xy;
    reference_line.SLToXY(lane_change_start_sl, &lane_change_start_xy);
    lane_change_status->set_exist_lane_change_start_position(true);
    lane_change_status->mutable_lane_change_start_position()->set_x(
        lane_change_start_xy.x());
    lane_change_status->mutable_lane_change_start_position()->set_y(
        lane_change_start_xy.y());
  }

  // Remove the target lane out of the path-boundary, up to the decided S.
  // 如果自车已经越过lane_change_start_s, 则直接返回, 不对path_bound进行更新
  if (lane_change_start_s < adc_frenet_s_) {
    // If already passed the decided S, then return.
    // lane_change_status->set_exist_lane_change_start_position(false);
    return;
  }

  // 对于lane_change_start_s之前的每一个path_bound
  // 分别用自车位置, 对其左右边界进行限制; 同时考虑自车完成变道, 对其边界进行限制
  for (size_t i = 0; i < path_bound->size(); ++i) {
    double curr_s = std::get<0>((*path_bound)[i]);
    // 只对lane_change_start_s之前的path_bound进行修正
    if (curr_s > lane_change_start_s) {
      break;
    }
    double curr_lane_left_width = 0.0;
    double curr_lane_right_width = 0.0;
    double offset_to_map = 0.0;
    reference_line.GetOffsetToMap(curr_s, &offset_to_map);
    if (reference_line.GetLaneWidth(curr_s, &curr_lane_left_width,
                                    &curr_lane_right_width)) {
      double offset_to_lane_center = 0.0;
      reference_line.GetOffsetToMap(curr_s, &offset_to_lane_center);
      curr_lane_left_width += offset_to_lane_center;
      curr_lane_right_width -= offset_to_lane_center;
    }
    curr_lane_left_width -= offset_to_map;
    curr_lane_right_width += offset_to_map;

    // get<1>(path_bound) is l_min, get<2>(path_bound) is l_max
    std::get<1>((*path_bound)[i]) =
        adc_frenet_l_ > curr_lane_left_width
            ? curr_lane_left_width + GetBufferBetweenADCCenterAndEdge()
            : std::get<1>((*path_bound)[i]);
    std::get<1>((*path_bound)[i]) =
        std::fmin(std::get<1>((*path_bound)[i]), adc_frenet_l_ - 0.1);  // 将右边界限制在自车 adc_frenet_l_ - 0.1
    std::get<2>((*path_bound)[i]) =
        adc_frenet_l_ < -curr_lane_right_width
            ? -curr_lane_right_width - GetBufferBetweenADCCenterAndEdge()
            : std::get<2>((*path_bound)[i]);
    std::get<2>((*path_bound)[i]) =
        std::fmax(std::get<2>((*path_bound)[i]), adc_frenet_l_ + 0.1); // 将左边界限制在自车 adc_frenet_l_ + 0.1
  }
}

// Currently, it processes each obstacle based on its frenet-frame
// projection. Therefore, it might be overly conservative when processing
// obstacles whose headings differ from road-headings a lot.
// TODO(all): (future work) this can be improved in the future.
/* 在path_bound中, 加入static_obstacle的边界信息 (They have to be avoided in any case)
   -> 对每个obstacle, 基于相对于自车当前centerline位置(偏左/偏右), 决策从左绕障还是从右绕障
   -> 基于每一个纵向位置更新后的left/right bound, 取其终点作为新的centerline
   -> blocking_obstacle_id: 在当前path_bound限制内, 无法向左/向右绕开的障碍物, 对应产生path_block的障碍物 */
bool PathBoundsDecider::GetBoundaryFromStaticObstacles(
    const PathDecision& path_decision, PathBound* const path_boundaries,
    std::string* const blocking_obstacle_id) {
  // Preprocessing.
  auto indexed_obstacles = path_decision.obstacles();
  auto sorted_obstacles = SortObstaclesForSweepLine(indexed_obstacles);
  ADEBUG << "There are " << sorted_obstacles.size() << " obstacles.";
  double center_line = adc_frenet_l_;
  size_t obs_idx = 0;
  int path_blocked_idx = -1;
  std::multiset<double, std::greater<double>> right_bounds;
  right_bounds.insert(std::numeric_limits<double>::lowest());
  std::multiset<double> left_bounds;
  left_bounds.insert(std::numeric_limits<double>::max());
  // Maps obstacle ID's to the decided ADC pass direction, if ADC should
  // pass from left, then true; otherwise, false.
  std::unordered_map<std::string, bool> obs_id_to_direction;
  // Maps obstacle ID's to the decision of whether side-pass on this obstacle
  // is allowed. If allowed, then true; otherwise, false.
  std::unordered_map<std::string, bool> obs_id_to_sidepass_decision;

  // Step through every path point.
  // 对每个obstacle, 基于相对于自车当前centerline位置(偏左/偏右), 决策从左绕障还是从右绕障
  // 基于每一个纵向位置更新后的left/right bound, 取其终点作为新的centerline
  for (size_t i = 1; i < path_boundaries->size(); ++i) {
    double curr_s = std::get<0>((*path_boundaries)[i]);
    // Check and see if there is any obstacle change:
    if (obs_idx < sorted_obstacles.size() &&
        std::get<1>(sorted_obstacles[obs_idx]) < curr_s) {
      while (obs_idx < sorted_obstacles.size() &&
             std::get<1>(sorted_obstacles[obs_idx]) < curr_s) {
        const auto& curr_obstacle = sorted_obstacles[obs_idx];
        const double curr_obstacle_s = std::get<1>(curr_obstacle);
        const double curr_obstacle_l_min = std::get<2>(curr_obstacle);
        const double curr_obstacle_l_max = std::get<3>(curr_obstacle);
        const std::string curr_obstacle_id = std::get<4>(curr_obstacle);
        ADEBUG << "id[" << curr_obstacle_id << "] s[" << curr_obstacle_s
               << "] curr_obstacle_l_min[" << curr_obstacle_l_min
               << "] curr_obstacle_l_max[" << curr_obstacle_l_max
               << "] center_line[" << center_line << "]";
        if (std::get<0>(curr_obstacle) == 1) {
          // A new obstacle enters into our scope:
          //   - Decide which direction for the ADC to pass.
          //   - Update the left/right bound accordingly.
          //   - If boundaries blocked, then decide whether can side-pass.
          //   - If yes, then borrow neighbor lane to side-pass.
          if (curr_obstacle_l_min + curr_obstacle_l_max < center_line * 2) {
            // Obstacle is to the right of center-line, should pass from left.
            obs_id_to_direction[curr_obstacle_id] = true;
            right_bounds.insert(curr_obstacle_l_max);
          } else {
            // Obstacle is to the left of center-line, should pass from right.
            obs_id_to_direction[curr_obstacle_id] = false;
            left_bounds.insert(curr_obstacle_l_min);
          }
          if (!UpdatePathBoundaryAndCenterLineWithBuffer(
                  i, *left_bounds.begin(), *right_bounds.begin(),
                  path_boundaries, &center_line)) {
            path_blocked_idx = static_cast<int>(i);
            *blocking_obstacle_id = curr_obstacle_id;
            break;
          }
        } else {
          // An existing obstacle exits our scope.
          if (obs_id_to_direction[curr_obstacle_id]) {
            right_bounds.erase(right_bounds.find(curr_obstacle_l_max));
          } else {
            left_bounds.erase(left_bounds.find(curr_obstacle_l_min));
          }
          obs_id_to_direction.erase(curr_obstacle_id);
        }
        // Update the bounds and center_line.
        std::get<1>((*path_boundaries)[i]) = std::fmax(
            std::get<1>((*path_boundaries)[i]),
            *right_bounds.begin() + GetBufferBetweenADCCenterAndEdge());
        std::get<2>((*path_boundaries)[i]) = std::fmin(
            std::get<2>((*path_boundaries)[i]),
            *left_bounds.begin() - GetBufferBetweenADCCenterAndEdge());
        if (std::get<1>((*path_boundaries)[i]) >
            std::get<2>((*path_boundaries)[i])) {
          ADEBUG << "Path is blocked at s = " << curr_s;
          path_blocked_idx = static_cast<int>(i);
          if (!obs_id_to_direction.empty()) {
            *blocking_obstacle_id = obs_id_to_direction.begin()->first;
          }
          break;
        } else {
          center_line = (std::get<1>((*path_boundaries)[i]) +
                         std::get<2>((*path_boundaries)[i])) /
                        2.0;
        }

        ++obs_idx;
      }
    } else {
      // If no obstacle change, update the bounds and center_line.
      std::get<1>((*path_boundaries)[i]) =
          std::fmax(std::get<1>((*path_boundaries)[i]),
                    *right_bounds.begin() + GetBufferBetweenADCCenterAndEdge());
      std::get<2>((*path_boundaries)[i]) =
          std::fmin(std::get<2>((*path_boundaries)[i]),
                    *left_bounds.begin() - GetBufferBetweenADCCenterAndEdge());
      if (std::get<1>((*path_boundaries)[i]) >
          std::get<2>((*path_boundaries)[i])) {
        ADEBUG << "Path is blocked at s = " << curr_s;
        path_blocked_idx = static_cast<int>(i);
        if (!obs_id_to_direction.empty()) {
          *blocking_obstacle_id = obs_id_to_direction.begin()->first;
        }
      } else {
        center_line = (std::get<1>((*path_boundaries)[i]) +
                       std::get<2>((*path_boundaries)[i])) /
                      2.0;
      }
    }

    // Early exit if path is blocked.
    // 如果遇到block (当前左边界 < 有边界), 则终止
    if (path_blocked_idx != -1) {
      break;
    }
  }

  // 基于block的位置, 裁剪path_boundaries
  TrimPathBounds(path_blocked_idx, path_boundaries);

  return true;
}

// The tuple contains (is_start_s, s, l_min, l_max, obstacle_id)
/* 从obstacle中筛选出静态障碍物, 并按left_edge从小到大, s_dis从大到小排列 */
std::vector<ObstacleEdge> PathBoundsDecider::SortObstaclesForSweepLine(
    const IndexedList<std::string, Obstacle>& indexed_obstacles) {
  std::vector<ObstacleEdge> sorted_obstacles;

  // Go through every obstacle and preprocess it.
  // 将障碍物转换为<is_start_s, s, l_min, l_max, id>, 塞入到sorted_obstacles
  for (const auto* obstacle : indexed_obstacles.Items()) {
    // Only focus on those within-scope obstacles.
    // 筛选出静止障碍物
    if (!IsWithinPathDeciderScopeObstacle(*obstacle)) {
      continue;
    }
    // Only focus on obstacles that are ahead of ADC.
    // 筛选出前方障碍物
    if (obstacle->PerceptionSLBoundary().end_s() < adc_frenet_s_) {
      continue;
    }
    // Decompose each obstacle's rectangle into two edges: one at
    // start_s; the other at end_s.
    const auto obstacle_sl = obstacle->PerceptionSLBoundary();
    sorted_obstacles.emplace_back(
        1, obstacle_sl.start_s() - FLAGS_obstacle_lon_start_buffer,
        obstacle_sl.start_l() - FLAGS_obstacle_lat_buffer,
        obstacle_sl.end_l() + FLAGS_obstacle_lat_buffer, obstacle->Id());
    sorted_obstacles.emplace_back(
        0, obstacle_sl.end_s() + FLAGS_obstacle_lon_end_buffer,
        obstacle_sl.start_l() - FLAGS_obstacle_lat_buffer,
        obstacle_sl.end_l() + FLAGS_obstacle_lat_buffer, obstacle->Id());
  }

  // Sort.
  // 对sorted_obstacles按照s从小到大, 相同s下is_start_s优先
  std::sort(sorted_obstacles.begin(), sorted_obstacles.end(),
            [](const ObstacleEdge& lhs, const ObstacleEdge& rhs) {
              if (std::get<1>(lhs) != std::get<1>(rhs)) {
                return std::get<1>(lhs) < std::get<1>(rhs);
              } else {
                return std::get<0>(lhs) > std::get<0>(rhs);
              }
            });

  return sorted_obstacles;
}

std::vector<PathBound> PathBoundsDecider::ConstructSubsequentPathBounds(
    const std::vector<ObstacleEdge>& sorted_obstacles, size_t path_idx,
    size_t obs_idx,
    std::unordered_map<std::string, std::tuple<bool, double>>* const
        obs_id_to_details,
    PathBound* const curr_path_bounds) {
  double left_bounds_from_obstacles = std::numeric_limits<double>::max();
  double right_bounds_from_obstacles = std::numeric_limits<double>::lowest();
  double curr_s = std::get<0>((*curr_path_bounds)[path_idx]);
  //==============================================================
  // If searched through all available s and found a path, return.
  if (path_idx >= curr_path_bounds->size()) {
    ADEBUG << "Completed path bounds search ending at path_idx = " << path_idx;
    return {*curr_path_bounds};
  }

  //==============================================================
  // If there is no obstacle updates at this path_idx.
  if (obs_idx >= sorted_obstacles.size() ||
      std::get<1>(sorted_obstacles[obs_idx]) > curr_s) {
    // 0. Backup the old memory.
    auto old_path_boundary = *curr_path_bounds;
    // 1. Get the boundary from obstacles.
    for (auto it = obs_id_to_details->begin(); it != obs_id_to_details->end();
         ++it) {
      if (std::get<0>(it->second)) {
        // Pass from left.
        right_bounds_from_obstacles =
            std::max(right_bounds_from_obstacles, std::get<1>(it->second));
      } else {
        // Pass from right.
        left_bounds_from_obstacles =
            std::min(left_bounds_from_obstacles, std::get<1>(it->second));
      }
    }
    // 2. Update the path boundary
    bool is_able_to_update = UpdatePathBoundaryWithBuffer(
        path_idx, left_bounds_from_obstacles, right_bounds_from_obstacles,
        curr_path_bounds);
    // 3. Return proper values.
    std::vector<PathBound> ret;
    if (is_able_to_update) {
      ret =
          ConstructSubsequentPathBounds(sorted_obstacles, path_idx + 1, obs_idx,
                                        obs_id_to_details, curr_path_bounds);
    } else {
      TrimPathBounds(static_cast<int>(path_idx), curr_path_bounds);
      ret.push_back(*curr_path_bounds);
    }
    *curr_path_bounds = old_path_boundary;
    return ret;
  }

  //==============================================================
  // If there are obstacle changes
  // 0. Backup the old memory.
  std::unordered_map<std::string, std::tuple<bool, double>>
      old_obs_id_to_details = *obs_id_to_details;
  auto old_path_boundary = *curr_path_bounds;

  // 1. Go through all obstacle changes.
  //    - For exiting obstacle, remove from our memory.
  //    - For entering obstacle, save it to a vector.
  std::vector<ObstacleEdge> new_entering_obstacles;
  size_t new_obs_idx = obs_idx;
  while (new_obs_idx < sorted_obstacles.size() &&
         std::get<1>(sorted_obstacles[new_obs_idx]) <= curr_s) {
    if (!std::get<0>(sorted_obstacles[new_obs_idx])) {
      // For exiting obstacle.
      obs_id_to_details->erase(std::get<4>(sorted_obstacles[new_obs_idx]));
    } else {
      // For entering obstacle.
      new_entering_obstacles.push_back(sorted_obstacles[new_obs_idx]);
    }
    ++new_obs_idx;
  }
  // 2. For new entering obstacles, decide possible pass directions.
  //    (ranked in terms of optimality)
  auto pass_direction_decisions =
      DecidePassDirections(0.0, 0.0, new_entering_obstacles);
  // 3. Try constructing subsequent path-bounds for all possible directions.
  std::vector<PathBound> ret;
  for (size_t i = 0; i < pass_direction_decisions.size(); ++i) {
    // For each possible direction:
    // a. Update the obs_id_to_details
    for (size_t j = 0; j < pass_direction_decisions[i].size(); ++j) {
      if (pass_direction_decisions[i][j]) {
        // Pass from left.
        (*obs_id_to_details)[std::get<4>(new_entering_obstacles[j])] =
            std::make_tuple(true, std::get<3>(new_entering_obstacles[j]));
      } else {
        // Pass from right.
        (*obs_id_to_details)[std::get<4>(new_entering_obstacles[j])] =
            std::make_tuple(false, std::get<2>(new_entering_obstacles[j]));
      }
    }
    // b. Figure out left/right bounds after the updates.
    for (auto it = obs_id_to_details->begin(); it != obs_id_to_details->end();
         ++it) {
      if (std::get<0>(it->second)) {
        // Pass from left.
        right_bounds_from_obstacles =
            std::max(right_bounds_from_obstacles, std::get<1>(it->second));
      } else {
        // Pass from right.
        left_bounds_from_obstacles =
            std::min(left_bounds_from_obstacles, std::get<1>(it->second));
      }
    }
    // c. Update for this path_idx, and construct the subsequent path bounds.
    std::vector<PathBound> curr_dir_path_boundaries;
    bool is_able_to_update = UpdatePathBoundaryWithBuffer(
        path_idx, left_bounds_from_obstacles, right_bounds_from_obstacles,
        curr_path_bounds);
    if (is_able_to_update) {
      curr_dir_path_boundaries = ConstructSubsequentPathBounds(
          sorted_obstacles, path_idx + 1, new_obs_idx, obs_id_to_details,
          curr_path_bounds);
    } else {
      TrimPathBounds(static_cast<int>(path_idx), curr_path_bounds);
      curr_dir_path_boundaries.push_back(*curr_path_bounds);
    }
    // d. Update the path_bounds into the vector, and revert changes
    //    to curr_path_bounds for next cycle.
    ret.insert(ret.end(), curr_dir_path_boundaries.begin(),
               curr_dir_path_boundaries.end());
    *curr_path_bounds = old_path_boundary;
  }
  // 4. Select the best path_bounds in ret.
  *obs_id_to_details = old_obs_id_to_details;
  *curr_path_bounds = old_path_boundary;
  std::sort(ret.begin(), ret.end(),
            [](const PathBound& lhs, const PathBound& rhs) {
              return lhs.size() > rhs.size();
            });
  while (ret.size() > 3) {
    ret.pop_back();
  }
  return ret;
}

std::vector<std::vector<bool>> PathBoundsDecider::DecidePassDirections(
    double l_min, double l_max,
    const std::vector<ObstacleEdge>& new_entering_obstacles) {
  std::vector<std::vector<bool>> decisions;

  // Convert into lateral edges.
  std::vector<ObstacleEdge> lateral_edges;
  lateral_edges.emplace_back(1, std::numeric_limits<double>::lowest(), 0.0, 0.0,
                             "l_min");
  lateral_edges.emplace_back(0, l_min, 0.0, 0.0, "l_min");
  lateral_edges.emplace_back(1, l_max, 0.0, 0.0, "l_max");
  lateral_edges.emplace_back(0, std::numeric_limits<double>::max(), 0.0, 0.0,
                             "l_max");
  for (size_t i = 0; i < new_entering_obstacles.size(); ++i) {
    if (std::get<3>(new_entering_obstacles[i]) < l_min ||
        std::get<2>(new_entering_obstacles[i]) > l_max) {
      continue;
    }
    lateral_edges.emplace_back(1, std::get<2>(new_entering_obstacles[i]), 0.0,
                               0.0, std::get<4>(new_entering_obstacles[i]));
    lateral_edges.emplace_back(0, std::get<3>(new_entering_obstacles[i]), 0.0,
                               0.0, std::get<4>(new_entering_obstacles[i]));
  }
  // Sort the lateral edges for lateral sweep-line algorithm.
  std::sort(lateral_edges.begin(), lateral_edges.end(),
            [](const ObstacleEdge& lhs, const ObstacleEdge& rhs) {
              if (std::get<1>(lhs) != std::get<1>(rhs)) {
                return std::get<1>(lhs) < std::get<1>(rhs);
              } else {
                return std::get<0>(lhs) > std::get<0>(rhs);
              }
            });

  // Go through the lateral edges and find any possible slot.
  std::vector<double> empty_slot;
  int num_obs = 0;
  for (size_t i = 0; i < lateral_edges.size(); ++i) {
    // Update obstacle overlapping info.
    if (std::get<0>(lateral_edges[i])) {
      ++num_obs;
    } else {
      --num_obs;
    }
    // If there is an empty slot within lane boundary.
    if (num_obs == 0 && i != lateral_edges.size() - 1) {
      empty_slot.push_back(
          (std::get<1>(lateral_edges[i]) + std::get<1>(lateral_edges[i + 1])) /
          2.0);
    }
  }
  // For each empty slot, update a corresponding pass direction
  for (size_t i = 0; i < empty_slot.size(); ++i) {
    double pass_position = empty_slot[i];
    std::vector<bool> pass_direction;
    for (size_t j = 0; j < new_entering_obstacles.size(); ++j) {
      if (std::get<2>(new_entering_obstacles[j]) > pass_position) {
        pass_direction.push_back(false);
      } else {
        pass_direction.push_back(true);
      }
    }
    decisions.push_back(pass_direction);
  }
  // TODO(jiacheng): sort the decisions based on the feasibility.

  return decisions;
}

/* 返回 自车宽度/2 + Buffer */
double PathBoundsDecider::GetBufferBetweenADCCenterAndEdge() {
  double adc_half_width =
      VehicleConfigHelper::GetConfig().vehicle_param().width() / 2.0;
  // TODO(all): currently it's a fixed number. But it can take into account many
  // factors such as: ADC length, possible turning angle, speed, etc.
  static constexpr double kAdcEdgeBuffer = 0.0;

  return (adc_half_width + kAdcEdgeBuffer);
}

/* 将输入的左右车道边界减去半个车身宽度, 作为最终的PathBounds */
bool PathBoundsDecider::UpdatePathBoundaryWithBuffer(
    size_t idx, double left_bound, double right_bound,
    PathBound* const path_boundaries, bool is_left_lane_bound,
    bool is_right_lane_bound) {
  // substract vehicle width when bound does not come from the lane boundary
  const double default_adc_buffer_coeff = 1.0;
  double left_adc_buffer_coeff =
      (is_left_lane_bound
           ? config_.path_bounds_decider_config().adc_buffer_coeff()
           : default_adc_buffer_coeff);
  double right_adc_buffer_coeff =
      (is_right_lane_bound
           ? config_.path_bounds_decider_config().adc_buffer_coeff()
           : default_adc_buffer_coeff);

  // Update the right bound (l_min):
  // 在lane_boundary基础上减去半个车身宽度
  double new_l_min =
      std::fmax(std::get<1>((*path_boundaries)[idx]),
                right_bound + right_adc_buffer_coeff *
                                  GetBufferBetweenADCCenterAndEdge());
  // Update the left bound (l_max):
  double new_l_max = std::fmin(
      std::get<2>((*path_boundaries)[idx]),
      left_bound - left_adc_buffer_coeff * GetBufferBetweenADCCenterAndEdge());

  // Check if ADC is blocked.
  // If blocked, don't update anything, return false.
  if (new_l_min > new_l_max) {
    ADEBUG << "Path is blocked at idx = " << idx;
    return false;
  }
  // Otherwise, update path_boundaries and center_line; then return true.
  std::get<1>((*path_boundaries)[idx]) = new_l_min;
  std::get<2>((*path_boundaries)[idx]) = new_l_max;
  return true;
}

/* 将left_bound / right_bound更新到path_boundaries中, 并根据(left + right) / 2计算新的center_line */
bool PathBoundsDecider::UpdatePathBoundaryAndCenterLineWithBuffer(
    size_t idx, double left_bound, double right_bound,
    PathBound* const path_boundaries, double* const center_line) {
  UpdatePathBoundaryWithBuffer(idx, left_bound, right_bound, path_boundaries);
  *center_line = (std::get<1>((*path_boundaries)[idx]) +
                  std::get<2>((*path_boundaries)[idx])) /
                 2.0;
  return true;
}

bool PathBoundsDecider::UpdatePathBoundary(size_t idx, double left_bound,
                                           double right_bound,
                                           PathBound* const path_boundaries) {
  // Update the right bound (l_min):
  double new_l_min =
      std::fmax(std::get<1>((*path_boundaries)[idx]), right_bound);
  // Update the left bound (l_max):
  double new_l_max =
      std::fmin(std::get<2>((*path_boundaries)[idx]), left_bound);

  // Check if ADC is blocked.
  // If blocked, don't update anything, return false.
  if (new_l_min > new_l_max) {
    ADEBUG << "Path is blocked at idx = " << idx;
    return false;
  }
  // Otherwise, update path_boundaries and center_line; then return true.
  std::get<1>((*path_boundaries)[idx]) = new_l_min;
  std::get<2>((*path_boundaries)[idx]) = new_l_max;
  return true;
}

/* 裁剪掉path_boundaries中, block的部分 (将path_blocked_idx后面的部分移除) */
void PathBoundsDecider::TrimPathBounds(const int path_blocked_idx,
                                       PathBound* const path_boundaries) {
  if (path_blocked_idx != -1) {
    if (path_blocked_idx == 0) {
      ADEBUG << "Completely blocked. Cannot move at all.";
    }
    int range = static_cast<int>(path_boundaries->size()) - path_blocked_idx;
    for (int i = 0; i < range; ++i) {
      path_boundaries->pop_back();
    }
  }
}

void PathBoundsDecider::PathBoundsDebugString(
    const PathBound& path_boundaries) {
  for (size_t i = 0; i < path_boundaries.size(); ++i) {
    AWARN << "idx " << i << "; s = " << std::get<0>(path_boundaries[i])
          << "; l_min = " << std::get<1>(path_boundaries[i])
          << "; l_max = " << std::get<2>(path_boundaries[i]);
  }
}

/* 检查check_s处的LaneBoundaryType与输入的LaneBorrowInfo是否一致
   LaneBorrowInfo为Left_Borrow, 对应的LaneBoundaryType需要为非实线(SOLID_YELLOW/SOLID_WHITE) */
bool PathBoundsDecider::CheckLaneBoundaryType(
    const ReferenceLineInfo& reference_line_info, const double check_s,
    const LaneBorrowInfo& lane_borrow_info) {SOLID_WHITE
  if (lane_borrow_info == LaneBorrowInfo::NO_BORROW) {
    return false;
  }

  const ReferenceLine& reference_line = reference_line_info.reference_line();
  auto ref_point = reference_line.GetNearestReferencePoint(check_s);
  if (ref_point.lane_waypoints().empty()) {
    return false;
  }

  const auto waypoint = ref_point.lane_waypoints().front();
  hdmap::LaneBoundaryType::Type lane_boundary_type =
      hdmap::LaneBoundaryType::UNKNOWN;
  if (lane_borrow_info == LaneBorrowInfo::LEFT_BORROW) {
    lane_boundary_type = hdmap::LeftBoundaryType(waypoint);
  } else if (lane_borrow_info == LaneBorrowInfo::RIGHT_BORROW) {
    lane_boundary_type = hdmap::RightBoundaryType(waypoint);
  }
  if (lane_boundary_type == hdmap::LaneBoundaryType::SOLID_YELLOW ||
      lane_boundary_type == hdmap::LaneBoundaryType::SOLID_WHITE) {
    return false;
  }
  return true;
}

void PathBoundsDecider::RecordDebugInfo(
    const PathBound& path_boundaries, const std::string& debug_name,
    ReferenceLineInfo* const reference_line_info) {
  // Sanity checks.
  ACHECK(!path_boundaries.empty());
  CHECK_NOTNULL(reference_line_info);

  // Take the left and right path boundaries, and transform them into two
  // PathData so that they can be displayed in simulator.
  std::vector<common::FrenetFramePoint> frenet_frame_left_boundaries;
  std::vector<common::FrenetFramePoint> frenet_frame_right_boundaries;
  for (const PathBoundPoint& path_bound_point : path_boundaries) {
    common::FrenetFramePoint frenet_frame_point;
    frenet_frame_point.set_s(std::get<0>(path_bound_point));
    frenet_frame_point.set_dl(0.0);
    frenet_frame_point.set_ddl(0.0);

    frenet_frame_point.set_l(std::get<1>(path_bound_point));
    frenet_frame_right_boundaries.push_back(frenet_frame_point);
    frenet_frame_point.set_l(std::get<2>(path_bound_point));
    frenet_frame_left_boundaries.push_back(frenet_frame_point);
  }

  auto frenet_frame_left_path =
      FrenetFramePath(std::move(frenet_frame_left_boundaries));
  auto frenet_frame_right_path =
      FrenetFramePath(std::move(frenet_frame_right_boundaries));

  PathData left_path_data;
  left_path_data.SetReferenceLine(&(reference_line_info->reference_line()));
  left_path_data.SetFrenetPath(std::move(frenet_frame_left_path));
  PathData right_path_data;
  right_path_data.SetReferenceLine(&(reference_line_info->reference_line()));
  right_path_data.SetFrenetPath(std::move(frenet_frame_right_path));

  // Insert the transformed PathData into the simulator display.
  auto* ptr_display_path_1 =
      reference_line_info->mutable_debug()->mutable_planning_data()->add_path();
  ptr_display_path_1->set_name("planning_path_boundary_1");
  ptr_display_path_1->mutable_path_point()->CopyFrom(
      {left_path_data.discretized_path().begin(),
       left_path_data.discretized_path().end()});
  auto* ptr_display_path_2 =
      reference_line_info->mutable_debug()->mutable_planning_data()->add_path();
  ptr_display_path_2->set_name("planning_path_boundary_2");
  ptr_display_path_2->mutable_path_point()->CopyFrom(
      {right_path_data.discretized_path().begin(),
       right_path_data.discretized_path().end()});
}

}  // namespace planning
}  // namespace apollo
