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

#include "modules/planning/tasks/deciders/speed_decider/speed_decider.h"

#include <algorithm>
#include <memory>

#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/util/util.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/proto/decision.pb.h"
#include "modules/planning/tasks/utils/st_gap_estimator.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::VehicleConfigHelper;
using apollo::common::math::Vec2d;
using apollo::cyber::Clock;
using apollo::perception::PerceptionObstacle;

SpeedDecider::SpeedDecider(const TaskConfig& config,
                           const std::shared_ptr<DependencyInjector>& injector)
    : Task(config, injector) {}

common::Status SpeedDecider::Execute(Frame* frame,
                                     ReferenceLineInfo* reference_line_info) {
  Task::Execute(frame, reference_line_info);
  init_point_ = frame_->PlanningStartPoint();
  adc_sl_boundary_ = reference_line_info_->AdcSlBoundary();
  reference_line_ = &reference_line_info_->reference_line();
  if (!MakeObjectDecision(reference_line_info->speed_data(),
                          reference_line_info->path_decision())
           .ok()) {
    const std::string msg = "Get object decision by speed profile failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  return Status::OK();
}

/* 决策PathTimeHeuristicOptimizer计算得到的ST轨迹与obstacle的STBoundary的位置关系:
   -> ABOVE: 位于obstacle STBoundary的上方
   -> BELOW: 位于obstacle STBoundary的下方
   -> CROSS: 与obstacle STBoundary相交 (只在不使用StBoundsDecider决策的DriverableBoundary时计算) */
SpeedDecider::STLocation SpeedDecider::GetSTLocation(
    const PathDecision* const path_decision, const SpeedData& speed_profile,
    const STBoundary& st_boundary) const {
  if (st_boundary.IsEmpty()) {
    return BELOW;
  }

  STLocation st_location = BELOW;
  bool st_position_set = false;
  const double start_t = st_boundary.min_t();
  const double end_t = st_boundary.max_t();
  // 遍历PathTimeHeuristicOptimizer输出的speed_profile的每一个离散点
  for (size_t i = 0; i + 1 < speed_profile.size(); ++i) {
    const STPoint curr_st(speed_profile[i].s(), speed_profile[i].t());
    const STPoint next_st(speed_profile[i + 1].s(), speed_profile[i + 1].t());
    if (curr_st.t() < start_t && next_st.t() < start_t) {
      continue;
    }

    if (curr_st.t() > end_t) {
      break;
    }

    // 如果不使用StBoundsDecider生成的结果, 则检测生成的ST轨迹是否会与obstacle相交
    if (!FLAGS_use_st_drivable_boundary) {
      common::math::LineSegment2d speed_line(curr_st, next_st);
      // 如果检查到生成的ST轨迹会和obstacle相交, 则设置st_location为CROSS, 同时停止检测
      if (st_boundary.HasOverlap(speed_line)) {
        ADEBUG << "speed profile cross st_boundaries.";
        st_location = CROSS;

        if (!FLAGS_use_st_drivable_boundary) {
          // 如果为KEEP_CLEAR目标, 如果规划的ST轨迹没法穿过KEEP_CLEAR obstacle的STBoundary, 则设置st_location为below
          if (st_boundary.boundary_type() ==
              STBoundary::BoundaryType::KEEP_CLEAR) {
            if (!CheckKeepClearCrossable(path_decision, speed_profile,
                                         st_boundary)) {
              st_location = BELOW;
            }
          }
        }
        break;
      }
    }

    // note: st_position can be calculated by checking two st points once
    //       but we need iterate all st points to make sure there is no CROSS
    // 
    if (!st_position_set) {
      // 如果第一次搜索到<curr_st.t(), next_st.t()>与<start_t, end_t>相交
      // 首先判断规划的ST轨迹位于目标的STBoundary的上方还是下方
      if (start_t < next_st.t() && curr_st.t() < end_t) {
        // bd_point_front为obstacle STBoundary的左上角的点
        STPoint bd_point_front = st_boundary.upper_points().front();
        double side = common::math::CrossProd(bd_point_front, curr_st, next_st);
        st_location = side < 0.0 ? ABOVE : BELOW;   // ST轨迹位于目标的STBoundary的上方还是下方
        st_position_set = true;
      }
    }
  }
  return st_location;
}

/* 计算规划的ST轨迹是否可以穿过给定的Keep_Clear obstacle */
bool SpeedDecider::CheckKeepClearCrossable(
    const PathDecision* const path_decision, const SpeedData& speed_profile,
    const STBoundary& keep_clear_st_boundary) const {
  bool keep_clear_crossable = true;   // flag表征规划的ST轨迹是否可以穿过keep_clear obstacle

  const auto& last_speed_point = speed_profile.back();
  // 计算speed_profile中最后一个点的速度last_speed_point_v
  double last_speed_point_v = 0.0;
  if (last_speed_point.has_v()) {
    last_speed_point_v = last_speed_point.v();
  } else {
    const size_t len = speed_profile.size();
    if (len > 1) {
      const auto& last_2nd_speed_point = speed_profile[len - 2];
      last_speed_point_v = (last_speed_point.s() - last_2nd_speed_point.s()) /
                           (last_speed_point.t() - last_2nd_speed_point.t());
    }
  }
  static constexpr double kKeepClearSlowSpeed = 2.5;  // m/s
  ADEBUG << "last_speed_point_s[" << last_speed_point.s()
         << "] st_boundary.max_s[" << keep_clear_st_boundary.max_s()
         << "] last_speed_point_v[" << last_speed_point_v << "]";
  // 如果规划的轨迹的最后点落在KEEP_CLEAR范围内, 且速度小于设定阈值2.5m/s
  // 则返回keep_clear_crossable = false
  if (last_speed_point.s() <= keep_clear_st_boundary.max_s() &&
      last_speed_point_v < kKeepClearSlowSpeed) {
    keep_clear_crossable = false;
  }
  return keep_clear_crossable;
}

/* 检查KEEP_CLEAR obstacle是否被其他Blocking obstacle遮挡住 */
bool SpeedDecider::CheckKeepClearBlocked(
    const PathDecision* const path_decision,
    const Obstacle& keep_clear_obstacle) const {
  bool keep_clear_blocked = false;

  // check if overlap with other stop wall
  // 检查其他Blocking Obstacle是否会遮挡住KEEP_CLEAR obstacle (位于KEEP_CLEAR obstacle后面, 但是没有留出足够的空间给自车停车)
  for (const auto* obstacle : path_decision->obstacles().Items()) {
    if (obstacle->Id() == keep_clear_obstacle.Id()) {
      continue;
    }
    const double obstacle_start_s = obstacle->PerceptionSLBoundary().start_s();
    const double adc_length =
        VehicleConfigHelper::GetConfig().vehicle_param().length();
    const double distance =
        obstacle_start_s - keep_clear_obstacle.PerceptionSLBoundary().end_s();

    // 如果obstacle最近点与自车相交
    if (obstacle->IsBlockingObstacle() && distance > 0 &&
        distance < (adc_length / 2)) {
      keep_clear_blocked = true;
      break;
    }
  }
  return keep_clear_blocked;
}

/* 决策是否对静止目标follow的距离过近: 以follow的最大减速度, 无法在给定距离内, 将自车降速到与目标速度一致 */
bool SpeedDecider::IsFollowTooClose(const Obstacle& obstacle) const {
  // 如果不是Blocking Obstacle (静止目标), 则返回false
  if (!obstacle.IsBlockingObstacle()) {
    return false;
  }

  if (obstacle.path_st_boundary().min_t() > 0.0) {
    return false;
  }
  const double obs_speed = obstacle.speed();
  const double ego_speed = init_point_.v();
  // 目标比自车快
  if (obs_speed > ego_speed) {
    return false;
  }
  const double distance =
      obstacle.path_st_boundary().min_s() - FLAGS_min_stop_distance_obstacle;
  static constexpr double lane_follow_max_decel = 3.0;
  static constexpr double lane_change_max_decel = 3.0;
  auto* planning_status = injector_->planning_context()
                              ->mutable_planning_status()
                              ->mutable_change_lane();
  double distance_numerator = std::pow((ego_speed - obs_speed), 2) * 0.5;
  double distance_denominator = lane_follow_max_decel;
  if (planning_status->has_status() &&
      planning_status->status() == ChangeLaneStatus::IN_CHANGE_LANE) {
    distance_denominator = lane_change_max_decel;
  }
  // 如果距离目标的距离小于在最大减速度下, 自车速度降到与目标相同时走过的距离, 则返回true
  return distance < distance_numerator / distance_denominator;
}

/* 对之前没有longitudinal decision的obstacle(主要是KEEP_CLEAR_OBSTACLE, 以及非最近的静态障碍物):
   -> 对静止行人, 设置stop decision (10m内的目标最大4s内设置stop decision)
   -> 检查自车轨迹与obstacle的相对位置(上/下/CROSS), 基于检查结果, 给目标设置对应的follow/yield/overtake/stop decision */
Status SpeedDecider::MakeObjectDecision(
    const SpeedData& speed_profile, PathDecision* const path_decision) const {
  if (speed_profile.size() < 2) {
    const std::string msg = "dp_st_graph failed to get speed profile.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // 遍历path_decision中的obstacle
  for (const auto* obstacle : path_decision->obstacles().Items()) {
    auto* mutable_obstacle = path_decision->Find(obstacle->Id());
    const auto& boundary = mutable_obstacle->path_st_boundary();    // obstacle对应的STBoundary

    // 如果obstacle的STBoundary位于ST图的边界以外
    // 如果没有横向 或 纵向决策, 则对应添加横向 或 纵向的Ignore Decision
    // 随后check下一个obstacle
    if (boundary.IsEmpty() || boundary.max_s() < 0.0 ||
        boundary.max_t() < 0.0 ||
        boundary.min_t() >= speed_profile.back().t()) {
      AppendIgnoreDecision(mutable_obstacle);
      continue;
    }

    // 如果obstacle已经有纵向决策
    // 如果没有横向决策, 则对应添加横向的Ignore Decision
    // 随后check下一个obstacle
    if (obstacle->HasLongitudinalDecision()) {
      AppendIgnoreDecision(mutable_obstacle);
      continue;
    }

    // for Virtual obstacle, skip if center point NOT "on lane"
    if (obstacle->IsVirtual()) {
      const auto& obstacle_box = obstacle->PerceptionBoundingBox();
      if (!reference_line_->IsOnLane(obstacle_box.center())) {
        continue;
      }
    }

    // always STOP for pedestrian
    // 检查是否需要对静止Pedestrian添加Stop decision (10m内的静止Pedestrian 4s内做出stop decision)
    // 做完决策后, check下一个目标
    if (CheckStopForPedestrian(*mutable_obstacle)) {
      ObjectDecisionType stop_decision;
      if (CreateStopDecision(*mutable_obstacle, &stop_decision,
                             -FLAGS_min_stop_distance_obstacle)) {
        mutable_obstacle->AddLongitudinalDecision("dp_st_graph/pedestrian",
                                                  stop_decision);
      }
      continue;
    }

    // 对于没有longitudinal decision的obstacle, 检查自车轨迹相对于obstacle STBoundary的上/下/Cross, 返回在location中
    // 基于location的结果, 给目标设置follow/yield/stop/overtake决策

    // 决策计算的ST轨迹与obstacle的STBoundary的位置关系: 位于obstacle的上方/下方/CROSS
    auto location = GetSTLocation(path_decision, speed_profile, boundary);

    // 如果不使用StBoundsDecider的结果, 检查KEEP_CLEAR obstacle是否会被其他blocking_obstacle遮挡住
    // 如果是, 则返回location为BELOW
    if (!FLAGS_use_st_drivable_boundary) {
      if (boundary.boundary_type() == STBoundary::BoundaryType::KEEP_CLEAR) {
        if (CheckKeepClearBlocked(path_decision, *obstacle)) {
          location = BELOW;
        }
      }
    }

    switch (location) {
      case BELOW:
        // 如果obstacle为KEEP_CLEAR类型, 且决策结果为BELOW, 则给目标加上stop decision
        if (boundary.boundary_type() == STBoundary::BoundaryType::KEEP_CLEAR) {
          ObjectDecisionType stop_decision;
          if (CreateStopDecision(*mutable_obstacle, &stop_decision, 0.0)) {
            mutable_obstacle->AddLongitudinalDecision("dp_st_graph/keep_clear",
                                                      stop_decision);
          }
        } 
        // 检查是否需要给出Follow决策
        else if (CheckIsFollow(*obstacle, boundary)) {
          // stop for low_speed decelerating
          // 检查是否距离blocking_obstacle(静止目标)过近: 在最大的follow减速度下(-3.0), 无法在间距内, 将自车速度降到小于目标速度
          // 此时给出stop decision
          if (IsFollowTooClose(*mutable_obstacle)) {
            ObjectDecisionType stop_decision;
            if (CreateStopDecision(*mutable_obstacle, &stop_decision,
                                   -FLAGS_min_stop_distance_obstacle)) {
              mutable_obstacle->AddLongitudinalDecision("dp_st_graph/too_close",
                                                        stop_decision);
            }
          } 
          // 否则给目标设置follow decision
          else {  // high speed or low speed accelerating
            // FOLLOW decision
            ObjectDecisionType follow_decision;
            if (CreateFollowDecision(*mutable_obstacle, &follow_decision)) {
              mutable_obstacle->AddLongitudinalDecision("dp_st_graph",
                                                        follow_decision);
            }
          }
        } 
        // 否则给出YIELD决策
        else {
          // YIELD decision
          ObjectDecisionType yield_decision;
          if (CreateYieldDecision(*mutable_obstacle, &yield_decision)) {
            mutable_obstacle->AddLongitudinalDecision("dp_st_graph",
                                                      yield_decision);
          }
        }
        break;
      // 如果ST轨迹在目标的STBoundary以上
      case ABOVE:
        // 如果obstacle为KEEP_CLEAR类型, 则设置Ignore decision
        if (boundary.boundary_type() == STBoundary::BoundaryType::KEEP_CLEAR) {
          ObjectDecisionType ignore;
          ignore.mutable_ignore();
          mutable_obstacle->AddLongitudinalDecision("dp_st_graph", ignore);
        } 
        // 对于非KEEP_CLEAR obstacle, 设置Overtake decision
        else {
          // OVERTAKE decision
          ObjectDecisionType overtake_decision;
          if (CreateOvertakeDecision(*mutable_obstacle, &overtake_decision)) {
            mutable_obstacle->AddLongitudinalDecision("dp_st_graph/overtake",
                                                      overtake_decision);
          }
        }
        break;
      // 对于CROSS状态:
      // 如果目标为blocking obstacle, 则设置stop decision
      case CROSS:
        if (mutable_obstacle->IsBlockingObstacle()) {
          ObjectDecisionType stop_decision;
          if (CreateStopDecision(*mutable_obstacle, &stop_decision,
                                 -FLAGS_min_stop_distance_obstacle)) {
            mutable_obstacle->AddLongitudinalDecision("dp_st_graph/cross",
                                                      stop_decision);
          }
          const std::string msg = absl::StrCat(
              "Failed to find a solution for crossing obstacle: ",
              mutable_obstacle->Id());
          AERROR << msg;
          return Status(ErrorCode::PLANNING_ERROR, msg);
        }
        break;
      default:
        AERROR << "Unknown position:" << location;
    }
    // 对没有决策的部分, 补足为Ignore Decision
    AppendIgnoreDecision(mutable_obstacle);
  }

  return Status::OK();
}

/* 如果obstacle没有横向 或 纵向决策, 则添加对应的Ignore decision */
void SpeedDecider::AppendIgnoreDecision(Obstacle* obstacle) const {
  ObjectDecisionType ignore_decision;
  ignore_decision.mutable_ignore();
  // obstacle没有纵向决策, 则添加纵向的ignore decisiton, tag为"dp_st_graph"
  if (!obstacle->HasLongitudinalDecision()) {
    obstacle->AddLongitudinalDecision("dp_st_graph", ignore_decision);
  }
  // obstacle没有横向决策, 则添加横向的ignore decision, tag为"dp_st_graph"
  if (!obstacle->HasLateralDecision()) {
    obstacle->AddLateralDecision("dp_st_graph", ignore_decision);
  }
}

/* 给目标创建stop decision, stop_fence设置为固定distance */
bool SpeedDecider::CreateStopDecision(const Obstacle& obstacle,
                                      ObjectDecisionType* const stop_decision,
                                      double stop_distance) const {
  const auto& boundary = obstacle.path_st_boundary();

  // TODO(all): this is a bug! Cannot mix reference s and path s!
  // Replace boundary.min_s() with computed reference line s
  // fence is set according to reference line s.
  // 计算fence_s, 即stop_decision对应的s
  double fence_s = adc_sl_boundary_.end_s() + boundary.min_s() + stop_distance;
  // 如果为KEEP_CLEAR obstacle, 则fence_s直接设置为obstacle的start_s
  if (boundary.boundary_type() == STBoundary::BoundaryType::KEEP_CLEAR) {
    fence_s = obstacle.PerceptionSLBoundary().start_s();
  }

  // 如果之前决策的main_stop_s < fence_s, 则返回
  // 否则基于fence_s设置stop_decision
  const double main_stop_s =
      reference_line_info_->path_decision()->stop_reference_line_s();
  if (main_stop_s < fence_s) {
    ADEBUG << "Stop fence is further away, ignore.";
    return false;
  }

  const auto fence_point = reference_line_->GetReferencePoint(fence_s);

  // set STOP decision
  auto* stop = stop_decision->mutable_stop();
  stop->set_distance_s(stop_distance);
  auto* stop_point = stop->mutable_stop_point();
  stop_point->set_x(fence_point.x());
  stop_point->set_y(fence_point.y());
  stop_point->set_z(0.0);
  stop->set_stop_heading(fence_point.heading());

  // 如果obstacle为KEEP_CLEAR, 则设置stop reason为CLEAR_ZONE
  if (boundary.boundary_type() == STBoundary::BoundaryType::KEEP_CLEAR) {
    stop->set_reason_code(StopReasonCode::STOP_REASON_CLEAR_ZONE);
  }

  PerceptionObstacle::Type obstacle_type = obstacle.Perception().type();
  ADEBUG << "STOP: obstacle_id[" << obstacle.Id() << "] obstacle_type["
         << PerceptionObstacle_Type_Name(obstacle_type) << "]";

  return true;
}

/* 更新follow_decision决策, 设置fence_point
   -> follow_distance = speed * time_gap */
bool SpeedDecider::CreateFollowDecision(
    const Obstacle& obstacle, ObjectDecisionType* const follow_decision) const {
  const double follow_speed = init_point_.v();
  const double follow_distance_s =
      -StGapEstimator::EstimateProperFollowingGap(follow_speed);

  const auto& boundary = obstacle.path_st_boundary();
  // follow scenario的reference_s
  const double reference_s =
      adc_sl_boundary_.end_s() + boundary.min_s() + follow_distance_s;
  const double main_stop_s =
      reference_line_info_->path_decision()->stop_reference_line_s();
  // 检查与main_stop_s的关系
  if (main_stop_s < reference_s) {
    ADEBUG << "Follow reference_s is further away, ignore.";
    return false;
  }

  auto ref_point = reference_line_->GetReferencePoint(reference_s);

  // set FOLLOW decision
  // 设置follow decision的fence_point
  auto* follow = follow_decision->mutable_follow();
  follow->set_distance_s(follow_distance_s);
  auto* fence_point = follow->mutable_fence_point();
  fence_point->set_x(ref_point.x());
  fence_point->set_y(ref_point.y());
  fence_point->set_z(0.0);
  follow->set_fence_heading(ref_point.heading());

  PerceptionObstacle::Type obstacle_type = obstacle.Perception().type();
  ADEBUG << "FOLLOW: obstacle_id[" << obstacle.Id() << "] obstacle_type["
         << PerceptionObstacle_Type_Name(obstacle_type) << "]";

  return true;
}

/* 更新yield_decision决策, 设置fence_point
   -> yield_distance = constant */
bool SpeedDecider::CreateYieldDecision(
    const Obstacle& obstacle, ObjectDecisionType* const yield_decision) const {
  PerceptionObstacle::Type obstacle_type = obstacle.Perception().type();
  double yield_distance = StGapEstimator::EstimateProperYieldingGap();

  const auto& obstacle_boundary = obstacle.path_st_boundary();
  const double yield_distance_s =
      std::max(-obstacle_boundary.min_s(), -yield_distance);

  const double reference_line_fence_s =
      adc_sl_boundary_.end_s() + obstacle_boundary.min_s() + yield_distance_s;
  const double main_stop_s =
      reference_line_info_->path_decision()->stop_reference_line_s();
  if (main_stop_s < reference_line_fence_s) {
    ADEBUG << "Yield reference_s is further away, ignore.";
    return false;
  }

  auto ref_point = reference_line_->GetReferencePoint(reference_line_fence_s);

  // set YIELD decision
  auto* yield = yield_decision->mutable_yield();
  yield->set_distance_s(yield_distance_s);
  yield->mutable_fence_point()->set_x(ref_point.x());
  yield->mutable_fence_point()->set_y(ref_point.y());
  yield->mutable_fence_point()->set_z(0.0);
  yield->set_fence_heading(ref_point.heading());

  ADEBUG << "YIELD: obstacle_id[" << obstacle.Id() << "] obstacle_type["
         << PerceptionObstacle_Type_Name(obstacle_type) << "]";

  return true;
}

/* 更新overtake_decision, 设置fence_point
   -> overtake_distance = speed * time_gap */
bool SpeedDecider::CreateOvertakeDecision(
    const Obstacle& obstacle,
    ObjectDecisionType* const overtake_decision) const {
  const auto& velocity = obstacle.Perception().velocity();
  const double obstacle_speed =
      common::math::Vec2d::CreateUnitVec2d(init_point_.path_point().theta())
          .InnerProd(Vec2d(velocity.x(), velocity.y()));

  // 计算overtake_distance
  const double overtake_distance_s =
      StGapEstimator::EstimateProperOvertakingGap(obstacle_speed,
                                                  init_point_.v());

  const auto& boundary = obstacle.path_st_boundary();
  const double reference_line_fence_s =
      adc_sl_boundary_.end_s() + boundary.min_s() + overtake_distance_s;
  const double main_stop_s =
      reference_line_info_->path_decision()->stop_reference_line_s();
  if (main_stop_s < reference_line_fence_s) {
    ADEBUG << "Overtake reference_s is further away, ignore.";
    return false;
  }

  auto ref_point = reference_line_->GetReferencePoint(reference_line_fence_s);

  // set OVERTAKE decision
  auto* overtake = overtake_decision->mutable_overtake();
  overtake->set_distance_s(overtake_distance_s);
  overtake->mutable_fence_point()->set_x(ref_point.x());
  overtake->mutable_fence_point()->set_y(ref_point.y());
  overtake->mutable_fence_point()->set_z(0.0);
  overtake->set_fence_heading(ref_point.heading());

  PerceptionObstacle::Type obstacle_type = obstacle.Perception().type();
  ADEBUG << "OVERTAKE: obstacle_id[" << obstacle.Id() << "] obstacle_type["
         << PerceptionObstacle_Type_Name(obstacle_type) << "]";

  return true;
}

/* 决策是否为需要Follow的目标 */
bool SpeedDecider::CheckIsFollow(const Obstacle& obstacle,
                                 const STBoundary& boundary) const {
  const double obstacle_l_distance =
      std::min(std::fabs(obstacle.PerceptionSLBoundary().start_l()),
               std::fabs(obstacle.PerceptionSLBoundary().end_l()));
  // 如果目标的最小横向距离>2.5m (目标明显偏离车道线), 则返回false
  if (obstacle_l_distance > FLAGS_follow_min_obs_lateral_distance) {
    return false;
  }

  // move towards adc
  // 如果目标和自车方向相反, 则返回false
  if (boundary.bottom_left_point().s() > boundary.bottom_right_point().s()) {
    return false;
  }

  static constexpr double kFollowTimeEpsilon = 1e-3;
  static constexpr double kFollowCutOffTime = 0.5;
  // 如果目标不是一直都在自车目标轨迹上, 或者快要离开自车轨迹, 返回false
  if (boundary.min_t() > kFollowCutOffTime ||
      boundary.max_t() < kFollowTimeEpsilon) {
    return false;
  }

  // cross lane but be moving to different direction
  // 如果目标的STBoundary最大最小时间很短, 则返回false
  if (boundary.max_t() - boundary.min_t() < FLAGS_follow_min_time_sec) {
    return false;
  }

  return true;
}

/* 决策是否对Pedestrian添加stop decision
   -> 10m内的静态Pedestrian有效
   -> 在发现静态Pedestrian 4s内返回true(添加stop decision), 否则返回false, 不做stop decison, 可能会nudge */
bool SpeedDecider::CheckStopForPedestrian(const Obstacle& obstacle) const {
  const auto& perception_obstacle = obstacle.Perception();
  // 如果目标类型部位Pedestrian, 则跳过
  if (perception_obstacle.type() != PerceptionObstacle::PEDESTRIAN) {
    return false;
  }

  // 如果目标位于自车后面, 则跳过
  const auto& obstacle_sl_boundary = obstacle.PerceptionSLBoundary();
  if (obstacle_sl_boundary.end_s() < adc_sl_boundary_.start_s()) {
    return false;
  }

  // read pedestrian stop time from PlanningContext
  auto* mutable_speed_decider_status = injector_->planning_context()
                                           ->mutable_planning_status()
                                           ->mutable_speed_decider();
  std::unordered_map<std::string, double> stop_time_map;    
  // 临时存放当前speed_decider_status.pedestrian_stop_time, 在后面逻辑更新后
  // 将存放的信息重新写入到speed_decider_status.pedestrian_stop_time
  
  // 将speed_decider_status中的pedestrian_stop_time信息导入到stop_time_map
  for (const auto& pedestrian_stop_time :
       mutable_speed_decider_status->pedestrian_stop_time()) {
    stop_time_map[pedestrian_stop_time.obstacle_id()] =
        pedestrian_stop_time.stop_timestamp_sec();
  }

  const std::string& obstacle_id = obstacle.Id();

  // update stop timestamp on static pedestrian for watch timer
  // check on stop timer for static pedestrians
  static constexpr double kSDistanceStartTimer = 10.0;
  static constexpr double kMaxStopSpeed = 0.3;
  static constexpr double kPedestrianStopTimeout = 4.0;

  bool result = true;
  // 如果pedestrian的纵向距离 < 10.0m
  if (obstacle.path_st_boundary().min_s() < kSDistanceStartTimer) {
    // 计算obstacle的绝对速度
    const auto obstacle_speed = std::hypot(perception_obstacle.velocity().x(),
                                           perception_obstacle.velocity().y());
    // 如果目标非静止, 则清除stop_time_map里对应id的信息, 对应Pedestrian从静止走开
    if (obstacle_speed > kMaxStopSpeed) {
      stop_time_map.erase(obstacle_id);
    } 
    else {
      // 如果该静止Pedestrian不在stop_time_map中, 则将<obs_id, current_time>加入到stop_time_map中
      if (stop_time_map.count(obstacle_id) == 0) {
        // add timestamp
        stop_time_map[obstacle_id] = Clock::NowInSeconds();
        ADEBUG << "add timestamp: obstacle_id[" << obstacle_id << "] timestamp["
               << Clock::NowInSeconds() << "]";
      } 
      // 如果该静止Pedestrian已经存在于stop_time_map中
      // 如果超过4s, 则返回false跳过, 即不在做stop decision, 可能会触发nudge decision
      else {
        // check timeout
        double stop_timer = Clock::NowInSeconds() - stop_time_map[obstacle_id];
        ADEBUG << "stop_timer: obstacle_id[" << obstacle_id << "] stop_timer["
               << stop_timer << "]";
        if (stop_timer >= kPedestrianStopTimeout) {
          result = false;
        }
      }
    }
  }

  // write pedestrian stop time to PlanningContext
  mutable_speed_decider_status->mutable_pedestrian_stop_time()->Clear();
  // 将stop_time_map中的信息写入到speed_decider_status的pedestrian_stop_time中
  for (const auto& stop_time : stop_time_map) {
    auto pedestrian_stop_time =
        mutable_speed_decider_status->add_pedestrian_stop_time();
    pedestrian_stop_time->set_obstacle_id(stop_time.first);
    pedestrian_stop_time->set_stop_timestamp_sec(stop_time.second);
  }
  return result;
}

}  // namespace planning
}  // namespace apollo
