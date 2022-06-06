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

#include "modules/planning/tasks/deciders/rule_based_stop_decider/rule_based_stop_decider.h"

#include <string>
#include <tuple>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/util/common.h"
#include "modules/planning/tasks/deciders/lane_change_decider/lane_change_decider.h"

namespace apollo {
namespace planning {

using apollo::common::SLPoint;
using apollo::common::Status;
using apollo::common::math::Vec2d;

namespace {
// TODO(ALL): temporarily copy the value from lane_follow_stage.cc, will extract
// as a common value for planning later
constexpr double kStraightForwardLineCost = 10.0;
}  // namespace

RuleBasedStopDecider::RuleBasedStopDecider(
    const TaskConfig &config,
    const std::shared_ptr<DependencyInjector> &injector)
    : Decider(config, injector) {
  ACHECK(config.has_rule_based_stop_decider_config());
  rule_based_stop_decider_config_ = config.rule_based_stop_decider_config();
}

/* RuleBasedStopDecider主执行函数, 覆盖以下几种基于规则的停车决策
   -> 借道旁边对向车道, 在障碍物不满足time-gap需求时, 在变道切换点设置虚拟障碍物, 停车等待
   -> 导航变道工况, 如果距离变道终点过近, 则在变道终点设置虚拟障碍物, 停车等待变道
   -> 导航终点工况, 如果距离导航终点过近, 则在导航终点设置虚拟障碍物, 停车 */
apollo::common::Status RuleBasedStopDecider::Process(
    Frame *const frame, ReferenceLineInfo *const reference_line_info) {
  // 1. Rule_based stop for side pass onto reverse lane
  // 在借道进入旁边对向车道的场景, 如果障碍物不满足变道time-gap需求, 则在变道切换点, 
  // 设定虚拟障碍物, 并对该障碍物给定stop decision
  StopOnSidePass(frame, reference_line_info);

  // 2. Rule_based stop for urgent lane change
  // 对于routing触发的变道工况, 如果离变道终点过近, 则进行停车等待变道决策:
  // 在变道终点设置虚拟障碍物, 并给该障碍物给定的stop decision
  if (FLAGS_enable_lane_change_urgency_checking) {
    CheckLaneChangeUrgency(frame);
  }

  // 3. Rule_based stop at path end position
  // 对导航的终点, 设置停车决策: 在终点设置虚拟障碍物, 并给定stop decision
  AddPathEndStop(frame, reference_line_info);

  return Status::OK();
}

/* 对于routing触发变道的工况, 如果距离当前lane的routing-end-point过近, 则做对应的停车等待变道的决策
   -> 基于当前lane的routing-end-point生成虚拟obstacle, 并对该obstacle生成stop decision */
void RuleBasedStopDecider::CheckLaneChangeUrgency(Frame *const frame) {
  for (auto &reference_line_info : *frame->mutable_reference_line_info()) {
    // Check if the target lane is blocked or not
    // 如果是变道目标reference_line, 并且变道轨迹规划成功, 目标车道障碍物满足lane change的time-gap条件
    if (reference_line_info.IsChangeLanePath()) {
      is_clear_to_change_lane_ =
          LaneChangeDecider::IsClearToChangeLane(&reference_line_info);
      is_change_lane_planning_succeed_ =
          reference_line_info.Cost() < kStraightForwardLineCost;
      continue;
    }
    // If it's not in lane-change scenario || (target lane is not blocked &&
    // change lane planning succeed), skip
    if (frame->reference_line_info().size() <= 1 ||
        (is_clear_to_change_lane_ && is_change_lane_planning_succeed_)) {
      continue;
    }
    // When the target lane is blocked in change-lane case, check the urgency
    // Get the end point of current routing
    const auto &route_end_waypoint =
        reference_line_info.Lanes().RouteEndWaypoint();
    // If can't get lane from the route's end waypoint, then skip
    if (!route_end_waypoint.lane) {
      continue;
    }
    // 寻找当前lane的routing end point
    auto point = route_end_waypoint.lane->GetSmoothPoint(route_end_waypoint.s);
    auto *reference_line = reference_line_info.mutable_reference_line();
    common::SLPoint sl_point;
    // Project the end point to sl_point on current reference lane
    if (reference_line->XYToSL(point, &sl_point) &&
        reference_line->IsOnLane(sl_point)) {
      // Check the distance from ADC to the end point of current routing
      double distance_to_passage_end =
          sl_point.s() - reference_line_info.AdcSlBoundary().end_s();
      // If ADC is still far from the end of routing, no need to stop, skip
      // 如果自车离当前lane的routing终点较远, 则不做停车等待变道的决策
      if (distance_to_passage_end >
          rule_based_stop_decider_config_.approach_distance_for_lane_change()) {
        continue;
      }
      // In urgent case, set a temporary stop fence and wait to change lane
      // TODO(Jiaxuan Xu): replace the stop fence to more intelligent actions
      // 如果离routing终点较近, 则基于routing的end-point, 做停车等待变道的决策
      // 基于routing的终点, 生成虚拟的obstacle, 并对该obstacle做stop decision
      const std::string stop_wall_id = "lane_change_stop";
      std::vector<std::string> wait_for_obstacles;
      util::BuildStopDecision(
          stop_wall_id, sl_point.s(),
          rule_based_stop_decider_config_.urgent_distance_for_lane_change(),
          StopReasonCode::STOP_REASON_LANE_CHANGE_URGENCY, wait_for_obstacles,
          "RuleBasedStopDecider", frame, &reference_line_info);
    }
  }
}

/* 在终点前5m构建虚拟的obstacle, 并对该obstacle生成对应的stop decision */
void RuleBasedStopDecider::AddPathEndStop(
    Frame *const frame, ReferenceLineInfo *const reference_line_info) {
  // 检查是否遇到终点工况(path length < short_path_length_threshold(20m))
  if (!reference_line_info->path_data().path_label().empty() &&
      reference_line_info->path_data().frenet_frame_path().back().s() -
              reference_line_info->path_data().frenet_frame_path().front().s() <
          FLAGS_short_path_length_threshold) {
    const std::string stop_wall_id =
        PATH_END_VO_ID_PREFIX + reference_line_info->path_data().path_label();
    std::vector<std::string> wait_for_obstacles;
    // 在终点前5m构建虚拟的obstacle, 并对该obstacle生成stop decision
    util::BuildStopDecision(
        stop_wall_id,
        reference_line_info->path_data().frenet_frame_path().back().s() - 5.0,
        0.0, StopReasonCode::STOP_REASON_REFERENCE_END, wait_for_obstacles,
        "RuleBasedStopDecider", frame, reference_line_info);
  }
}

/* 决策变道到对向旁边车道时的停车工况:
   -> 遇到该工况时, 如果旁边车道障碍物time-gap满足变道需求, 则不做处理
   -> 如果不满足障碍物, 则在到旁边车道的切换点, 设置虚拟障碍物, 并对其给定停车决策(stop decision) */
void RuleBasedStopDecider::StopOnSidePass(
    Frame *const frame, ReferenceLineInfo *const reference_line_info) {
  static bool check_clear;
  static common::PathPoint change_lane_stop_path_point;   // 变道到对向旁边车道停车工况, 设置的虚拟障碍物点

  const PathData &path_data = reference_line_info->path_data();
  double stop_s_on_pathdata = 0.0;

  // 如果SL轨迹对应保持当前车道的轨迹
  if (path_data.path_label().find("self") != std::string::npos) {
    check_clear = false;
    change_lane_stop_path_point.Clear();
    return;
  }

  if (check_clear &&
      CheckClearDone(*reference_line_info, change_lane_stop_path_point)) {
    check_clear = false;
  }

  // 检查规划的轨迹是否会进入到对向旁边车道, 如果是, 寻找对应的切换点(Path InLane->OutOnReverse), 
  // 并计算该点在轨迹上的纵向距离
  if (!check_clear &&
      CheckSidePassStop(path_data, *reference_line_info, &stop_s_on_pathdata)) {
    // 如果检查到perception没有被障碍物遮挡, 并且满足lane change的time-gap条件, 返回
    // 不做任何操作
    if (!LaneChangeDecider::IsPerceptionBlocked(
            *reference_line_info,
            rule_based_stop_decider_config_.search_beam_length(),
            rule_based_stop_decider_config_.search_beam_radius_intensity(),
            rule_based_stop_decider_config_.search_range(),
            rule_based_stop_decider_config_.is_block_angle_threshold()) &&
        LaneChangeDecider::IsClearToChangeLane(reference_line_info)) {
      return;
    }
    // 如果有障碍物遮挡, 或者不满足lane-change的time-gap条件, 则进行如下检查
    // 如果自车不会在path_data的stop_s_on_pathdata处停止, 则在reference_line对应的位置
    // 构建虚拟障碍物, 并基于该障碍物, 构建对应的stop_decision
    if (!CheckADCStop(path_data, *reference_line_info, stop_s_on_pathdata)) {
      // 将虚拟障碍物的point设置到change_lane_stop_path_point中
      if (!BuildSidePassStopFence(path_data, stop_s_on_pathdata,
                                  &change_lane_stop_path_point, frame,
                                  reference_line_info)) {
        AERROR << "Set side pass stop fail";
      }
    } 
    // 如果自车会在切换点处停止, 检查是否满足lane changde的time-gap条件, 满足的话, 将check_clear置true
    else {
      if (LaneChangeDecider::IsClearToChangeLane(reference_line_info)) {
        check_clear = true;
      }
    }
  }
}

// @brief Check if necessary to set stop fence used for nonscenario side pass
/* 找到目标SL轨迹中, 从IN_LANE切换到OUT_ON_REVERSE_LANE的轨迹点, 并找到对应的纵向距离stop_s_on_pathdata
   如果成功找到该点, 并找到对应的距离, 则返回true */
bool RuleBasedStopDecider::CheckSidePassStop(
    const PathData &path_data, const ReferenceLineInfo &reference_line_info,
    double *stop_s_on_pathdata) {
  const std::vector<std::tuple<double, PathData::PathPointType, double>>
      &path_point_decision_guide = path_data.path_point_decision_guide();
  PathData::PathPointType last_path_point_type =
      PathData::PathPointType::UNKNOWN;
  for (const auto &point_guide : path_point_decision_guide) {
    // 如果从IN_LANE切换到OUT_ON_REVERSE_LANE的状态
    // 找到从IN_LANE切换到OUT_ON_REVERSE_LANE的轨迹点path_point
    if (last_path_point_type == PathData::PathPointType::IN_LANE &&
        std::get<1>(point_guide) ==
            PathData::PathPointType::OUT_ON_REVERSE_LANE) {
      // 提取该切换点在path_data上的纵向距离s
      *stop_s_on_pathdata = std::get<0>(point_guide);
      // Approximate the stop fence s based on the vehicle position
      const auto &vehicle_config =
          common::VehicleConfigHelper::Instance()->GetConfig();
      const double ego_front_to_center =
          vehicle_config.vehicle_param().front_edge_to_center();
      common::PathPoint stop_pathpoint;
      // 找到path_data中, stop_s_on_pathdata对应的stop_pathpoint点
      if (!path_data.GetPathPointWithRefS(*stop_s_on_pathdata,
                                          &stop_pathpoint)) {
        AERROR << "Can't get stop point on path data";
        return false;
      }
      const double ego_theta = stop_pathpoint.theta();
      Vec2d shift_vec{ego_front_to_center * std::cos(ego_theta),
                      ego_front_to_center * std::sin(ego_theta)};
      const Vec2d stop_fence_pose =
          shift_vec + Vec2d(stop_pathpoint.x(), stop_pathpoint.y());
      // stop_fence_pose为IN_LANE->OUT_REVERSE切换点处, 自车最前端的点的<x,y>坐标
      double stop_l_on_pathdata = 0.0;
      const auto &nearby_path = reference_line_info.reference_line().map_path();

      // 找到自车最前端点对应的reference-line上的纵向/横向距离s, l
      nearby_path.GetNearestPoint(stop_fence_pose, stop_s_on_pathdata,
                                  &stop_l_on_pathdata);
      return true;
    }
    last_path_point_type = std::get<1>(point_guide);
  }
  return false;
}

// @brief Set stop fence for side pass
/* 基于给定的path_data与path_data上的纵向停止距离stop_s_on_pathdata,
   -> 在reference-line上生成并添加对应的静态障碍物
   -> 对该虚拟静态障碍物, 生成对应的stop decision, 标注reason code = STOP_REASON_SIDEPASS_SAFETY */
bool RuleBasedStopDecider::BuildSidePassStopFence(
    const PathData &path_data, const double stop_s_on_pathdata,
    common::PathPoint *stop_point, Frame *const frame,
    ReferenceLineInfo *const reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  // 得到stop_s_on_pathdata对应的轨迹上的点stop_point <x, y, heading>
  if (!path_data.GetPathPointWithRefS(stop_s_on_pathdata, stop_point)) {
    AERROR << "Can't get stop point on path data";
    return false;
  }

  const std::string stop_wall_id = "Side_Pass_Stop";
  std::vector<std::string> wait_for_obstacles;

  const auto &nearby_path = reference_line_info->reference_line().map_path();
  double stop_point_s = 0.0;
  double stop_point_l = 0.0;
  // 得到stop_point在reference_line上对应的纵向距离stop_point_s
  nearby_path.GetNearestPoint({stop_point->x(), stop_point->y()}, &stop_point_s,
                              &stop_point_l);

  // 基于stop_point_s在reference_line上生成对应的虚拟静态障碍物, 并添加到reference_line中
  // 针对虚拟静态障碍物生成对应的stop decision
  util::BuildStopDecision(stop_wall_id, stop_point_s, 0.0,
                          StopReasonCode::STOP_REASON_SIDEPASS_SAFETY,
                          wait_for_obstacles, "RuleBasedStopDecider", frame,
                          reference_line_info);
  return true;
}

// @brief Check if ADV stop at a stop fence
/* 检查是否ADC将会在stop_s_on_pathdata前停止
   -> 如果车速较高(>1m/s), 则返回false
   -> 如果车速较低(<=1m/s), 但是距离小于设定值(1~4m), 则返回false */
bool RuleBasedStopDecider::CheckADCStop(
    const PathData &path_data, const ReferenceLineInfo &reference_line_info,
    const double stop_s_on_pathdata) {
  common::PathPoint stop_point;
  // get stop_point with respect to stop_s_on_pathdata
  // 返回规划轨迹上的点stop_point
  if (!path_data.GetPathPointWithRefS(stop_s_on_pathdata, &stop_point)) {
    AERROR << "Can't get stop point on path data";
    return false;
  }

  // check if adc is stopped by checking its speed
  // 自车速度大于0.5m/s, 则返回false
  const double adc_speed = injector_->vehicle_state()->linear_velocity();
  if (adc_speed > rule_based_stop_decider_config_.max_adc_stop_speed()) {
    ADEBUG << "ADC not stopped: speed[" << adc_speed << "]";
    return false;
  }

  // check stop close enough to stop line of the stop_sign
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  const auto &nearby_path = reference_line_info.reference_line().map_path();
  double stop_point_s = 0.0;
  double stop_point_l = 0.0;
  // 返回SL轨迹上的stop-point在reference-line下的SL坐标
  nearby_path.GetNearestPoint({stop_point.x(), stop_point.y()}, &stop_point_s,
                              &stop_point_l);

  const double distance_stop_line_to_adc_front_edge =
      stop_point_s - adc_front_edge_s;

  // 如果stop_s离自车较远, 则返回false
  // 应该是 "<" 号?
  if (distance_stop_line_to_adc_front_edge >
      rule_based_stop_decider_config_.max_valid_stop_distance()) {
    ADEBUG << "not a valid stop. too far from stop line.";
    return false;
  }

  return true;
}

/*  */
bool RuleBasedStopDecider::CheckClearDone(
    const ReferenceLineInfo &reference_line_info,
    const common::PathPoint &stop_point) {
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  const double adc_back_edge_s = reference_line_info.AdcSlBoundary().start_s();
  const double adc_start_l = reference_line_info.AdcSlBoundary().start_l();
  const double adc_end_l = reference_line_info.AdcSlBoundary().end_l();
  double lane_left_width = 0.0;
  double lane_right_width = 0.0;
  // 提取自车位置处的车道宽度
  reference_line_info.reference_line().GetLaneWidth(
      (adc_front_edge_s + adc_back_edge_s) / 2.0, &lane_left_width,
      &lane_right_width);
  SLPoint stop_sl_point;
  // 提取stop_point在reference_line对应的SL坐标系的值
  reference_line_info.reference_line().XYToSL(stop_point, &stop_sl_point);
  // use distance to last stop point to determine if needed to check clear
  // again
  // 如果自车位置越过stop_sl_point, 且自车在reference_line范围内, 则返回true
  if (adc_back_edge_s > stop_sl_point.s()) {
    if (adc_start_l > -lane_right_width || adc_end_l < lane_left_width) {
      return true;
    }
  }
  return false;
}

}  // namespace planning
}  // namespace apollo
