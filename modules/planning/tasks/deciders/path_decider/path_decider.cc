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

#include "modules/planning/tasks/deciders/path_decider/path_decider.h"

#include <memory>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/util/util.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/proto/decision.pb.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::VehicleConfigHelper;

PathDecider::PathDecider(const TaskConfig &config,
                         const std::shared_ptr<DependencyInjector> &injector)
    : Task(config, injector) {}

Status PathDecider::Execute(Frame *frame,
                            ReferenceLineInfo *reference_line_info) {
  Task::Execute(frame, reference_line_info);
  return Process(reference_line_info, reference_line_info->path_data(),
                 reference_line_info->path_decision());
}

/* 主处理函数:
   -> path_data为path_assessment_decider基于规则选出的所有candidate-path中的最优path
   -> path_decision包含基于SL的轨迹path_data, 对每个静态障碍物所做的横纵向决策: Ignore, Stop, Nudge */
Status PathDecider::Process(const ReferenceLineInfo *reference_line_info,
                            const PathData &path_data,
                            PathDecision *const path_decision) {
  // skip path_decider if reused path
  // enable_skip_path_tasks 该配置项默认为true
  if (FLAGS_enable_skip_path_tasks && reference_line_info->path_reusable()) {
    return Status::OK();
  }

  std::string blocking_obstacle_id;
  if (reference_line_info->GetBlockingObstacle() != nullptr) {
    blocking_obstacle_id = reference_line_info->GetBlockingObstacle()->Id();
  }
  // 基于PIECEWISE_JERK_PATH_OPTIMIZER生成的SL Path, 以及PATH_ASSESSMENT_DECIDER选择的结果
  // 对每个静态目标更新其横纵向决策(decision): Ignore, Stop, Nudge
  if (!MakeObjectDecision(path_data, blocking_obstacle_id, path_decision)) {
    const std::string msg = "Failed to make decision based on tunnel";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  return Status::OK();
}

/* 对path_decision中的所有obstacle分别更新纵向&横向decision */
bool PathDecider::MakeObjectDecision(const PathData &path_data,
                                     const std::string &blocking_obstacle_id,
                                     PathDecision *const path_decision) {
  if (!MakeStaticObstacleDecision(path_data, blocking_obstacle_id,
                                  path_decision)) {
    AERROR << "Failed to make decisions for static obstacles";
    return false;
  }
  return true;
}

// TODO(jiacheng): eventually this entire "path_decider" should be retired.
// Before it gets retired, its logics are slightly modified so that everything
// still works well for now.
/* 对path_decision中的每个obstacle, 生成对应的横纵向决策 (决策结果设置在path_decision中):
   -> 只对静止障碍物生成横/纵向决策
   -> 跳过以下静止障碍物
      -> 同时存在横纵向decision, 且均为Ignore
      -> 存在纵向的stop decision
   -> 对path_data对应的blocking_obstacle, 设置纵向的stop decision
   -> 对不在path_data纵向范围内的obstacle, 设置横纵向的ignore decision
   -> obstacle与path_data的横向距离:
      -> 过大: 设置横向的ignore decision
      -> 过小: 设置纵向的stop decision
      -> 适中: 设置对应的left/right nudge decision*/
bool PathDecider::MakeStaticObstacleDecision(
    const PathData &path_data, const std::string &blocking_obstacle_id,
    PathDecision *const path_decision) {
  // Sanity checks and get important values.
  ACHECK(path_decision);
  const auto &frenet_path = path_data.frenet_frame_path();
  if (frenet_path.empty()) {
    AERROR << "Path is empty.";
    return false;
  }
  const double half_width =
      common::VehicleConfigHelper::GetConfig().vehicle_param().width() / 2.0;
  const double lateral_radius = half_width + FLAGS_lateral_ignore_buffer;

  // Go through every obstacle and make decisions.
  for (const auto *obstacle : path_decision->obstacles().Items()) {
    const std::string &obstacle_id = obstacle->Id();
    const std::string obstacle_type_name =
        PerceptionObstacle_Type_Name(obstacle->Perception().type());
    ADEBUG << "obstacle_id[<< " << obstacle_id << "] type["
           << obstacle_type_name << "]";

    // 跳过 非静止 && virtual 的障碍物
    if (!obstacle->IsStatic() || obstacle->IsVirtual()) {
      continue;
    }
    // - skip decision making for obstacles with IGNORE/STOP decisions already.
    // 如果障碍物已经有纵向&横向decision, 且decision都为ignore
    if (obstacle->HasLongitudinalDecision() &&
        obstacle->LongitudinalDecision().has_ignore() &&
        obstacle->HasLateralDecision() &&
        obstacle->LateralDecision().has_ignore()) {
      continue;
    }
    // 如果障碍物已经有纵向decision, 且decision为stop
    if (obstacle->HasLongitudinalDecision() &&
        obstacle->LongitudinalDecision().has_stop()) {
      // STOP decision
      continue;
    }
    // - add STOP decision for blocking obstacles.
    // 如果目标是静止的blocking obstacle, 且处于非lane borrow的场景, 则对其添加stop decision
    if (obstacle->Id() == blocking_obstacle_id &&
        !injector_->planning_context()
             ->planning_status()
             .path_decider()
             .is_in_path_lane_borrow_scenario()) {
      // Add stop decision
      ADEBUG << "Blocking obstacle = " << blocking_obstacle_id;
      ObjectDecisionType object_decision;
      // 生成相对obstacle的stop decision: 刹停位置 & 刹停原因
      *object_decision.mutable_stop() = GenerateObjectStopDecision(*obstacle);
      // 将stop decision的结果设置到path_decision中对应id的obstacle中
      path_decision->AddLongitudinalDecision("PathDecider/blocking_obstacle",
                                             obstacle->Id(), object_decision);
      continue;
    }
    // - skip decision making for clear-zone obstacles.
    if (obstacle->reference_line_st_boundary().boundary_type() ==
        STBoundary::BoundaryType::KEEP_CLEAR) {
      continue;
    }

    // 0. IGNORE by default and if obstacle is not in path s at all.
    // 如果obstacle的纵向距离s不在规划的path范围内, 则对目标添加纵向与横向的ignore decision
    ObjectDecisionType object_decision;
    object_decision.mutable_ignore();
    const auto &sl_boundary = obstacle->PerceptionSLBoundary();
    if (sl_boundary.end_s() < frenet_path.front().s() ||
        sl_boundary.start_s() > frenet_path.back().s()) {
      path_decision->AddLongitudinalDecision("PathDecider/not-in-s",
                                             obstacle->Id(), object_decision);
      path_decision->AddLateralDecision("PathDecider/not-in-s", obstacle->Id(),
                                        object_decision);
      continue;
    }

    const auto frenet_point = frenet_path.GetNearestPoint(sl_boundary);
    const double curr_l = frenet_point.l();
    double min_nudge_l =
        half_width +
        config_.path_decider_config().static_obstacle_buffer() / 2.0;

    // 如果obstacle的横向距离与规划轨迹对应的点的横向距离相差较大的话, 则添加横向的ignore decision
    if (curr_l - lateral_radius > sl_boundary.end_l() ||
        curr_l + lateral_radius < sl_boundary.start_l()) {
      // 1. IGNORE if laterally too far away.
      path_decision->AddLateralDecision("PathDecider/not-in-l", obstacle->Id(),
                                        object_decision);
    }
    // 如果obstacle的横向距离与规划轨迹对应的点的横向距离存在横向的overlap, 则对障碍物添加纵向的stop decision
     else if (sl_boundary.end_l() >= curr_l - min_nudge_l &&
               sl_boundary.start_l() <= curr_l + min_nudge_l) {
      // 2. STOP if laterally too overlapping.
      *object_decision.mutable_stop() = GenerateObjectStopDecision(*obstacle);

      // 基于新的obstacle stop决策, 更新最近的停止点main_stop
      // 如果该stop被用作为新的最近stop点, 则打tag为"nearest-stop"
      if (path_decision->MergeWithMainStop(
              object_decision.stop(), obstacle->Id(),
              reference_line_info_->reference_line(),
              reference_line_info_->AdcSlBoundary())) {
        path_decision->AddLongitudinalDecision("PathDecider/nearest-stop",
                                               obstacle->Id(), object_decision);
      }
      // 如果该obstacle stop距离不是最近的, 则对该障碍物添加纵向的ignore决策, 并打tag为not-nearest-stop
       else {
        ObjectDecisionType object_decision;
        object_decision.mutable_ignore();
        path_decision->AddLongitudinalDecision("PathDecider/not-nearest-stop",
                                               obstacle->Id(), object_decision);
      }
    } else {
      // 3. NUDGE if laterally very close.
      // nudge决策
      // 如果obstacle距离自车不是过远, 也没有overlap, 则添加横向的nudge决策
      if (sl_boundary.end_l() < curr_l - min_nudge_l) {  // &&
        // sl_boundary.end_l() > curr_l - min_nudge_l - 0.3) {
        // LEFT_NUDGE
        ObjectNudge *object_nudge_ptr = object_decision.mutable_nudge();
        object_nudge_ptr->set_type(ObjectNudge::LEFT_NUDGE);
        object_nudge_ptr->set_distance_l(
            config_.path_decider_config().static_obstacle_buffer());
        path_decision->AddLateralDecision("PathDecider/left-nudge",
                                          obstacle->Id(), object_decision);
      } else if (sl_boundary.start_l() > curr_l + min_nudge_l) {  // &&
        // sl_boundary.start_l() < curr_l + min_nudge_l + 0.3) {
        // RIGHT_NUDGE
        ObjectNudge *object_nudge_ptr = object_decision.mutable_nudge();
        object_nudge_ptr->set_type(ObjectNudge::RIGHT_NUDGE);
        object_nudge_ptr->set_distance_l(
            -config_.path_decider_config().static_obstacle_buffer());
        path_decision->AddLateralDecision("PathDecider/right-nudge",
                                          obstacle->Id(), object_decision);
      }
    }
  }

  return true;
}

/* 计算针对输入目标的stop decision:
   -> stop_distance: 刹停后与前车的相对距离
   -> stop_ref_point: 刹停后的自车位置参考点
   -> stop_reason: stop decision的原因 */
ObjectStop PathDecider::GenerateObjectStopDecision(
    const Obstacle &obstacle) const {
  ObjectStop object_stop;

  // 计算stop_distance, 也即自车刹停后与前车的纵向距离stop_distance
  double stop_distance = obstacle.MinRadiusStopDistance(
      VehicleConfigHelper::GetConfig().vehicle_param());
  object_stop.set_reason_code(StopReasonCode::STOP_REASON_OBSTACLE);
  object_stop.set_distance_s(-stop_distance);

  // 计算针对obstacle的参考刹停位置stop_ref_s, 以及参考刹停点stop_ref_point
  const double stop_ref_s =
      obstacle.PerceptionSLBoundary().start_s() - stop_distance;
  const auto stop_ref_point =
      reference_line_info_->reference_line().GetReferencePoint(stop_ref_s);
  // 将stop_ref_point的位置与heading信息设置到object_stop中
  object_stop.mutable_stop_point()->set_x(stop_ref_point.x());
  object_stop.mutable_stop_point()->set_y(stop_ref_point.y());
  object_stop.set_stop_heading(stop_ref_point.heading());
  return object_stop;
}

}  // namespace planning
}  // namespace apollo
