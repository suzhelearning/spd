#include "tianji_qp_ik/pico_wrist_alignment.hpp"

#include "tianji_qp_ik/so3.hpp"

#include <cmath>
#include <stdexcept>

namespace tianji_qp_ik {
namespace {

constexpr double kRotationValidationTolerance = 1.0e-6;


}  // namespace

PicoWristAlignment::PicoWristAlignment(WristAlignmentConfig config)
    : config_(config) {
  if (config_.stable_frames == 0U) {
    throw std::invalid_argument("wrist stable frame count must be non-zero");
  }
  if (!std::isfinite(config_.max_stable_position_step_m) ||
      config_.max_stable_position_step_m <= 0.0) {
    throw std::invalid_argument(
        "wrist max stable position step must be finite and positive");
  }
  if (!std::isfinite(config_.max_stable_orientation_step_rad) ||
      config_.max_stable_orientation_step_rad <= 0.0) {
    throw std::invalid_argument(
        "wrist max stable orientation step must be finite and positive");
  }
  if (!std::isfinite(config_.position_scale) || config_.position_scale <= 0.0) {
    throw std::invalid_argument("wrist position scale must be finite and positive");
  }
}

WristAlignmentUpdate PicoWristAlignment::update(const PicoTeleopFrame& frame,
                                                 const Pose& current_left,
                                                 const Pose& current_right) {
  bool alignment_reset = frame.wrist_alignment_reset;
  if (!has_epoch_) {
    has_epoch_ = true;
    epoch_ = frame.tracking_epoch;
  } else if (frame.tracking_epoch != epoch_) {
    clearSide(left_);
    clearSide(right_);
    epoch_ = frame.tracking_epoch;
    alignment_reset = true;
  }
  if (alignment_reset) {
    resetSide(ArmSide::kLeft);
    resetSide(ArmSide::kRight);
  }

  const bool left_active = frame.wrist_pose_input && frame.left_wrist_active;
  const bool right_active = frame.wrist_pose_input && frame.right_wrist_active;
  return {updateSide(left_, left_active, frame.left, current_left,
                     alignment_reset),
          updateSide(right_, right_active, frame.right, current_right,
                     alignment_reset)};
}

void PicoWristAlignment::reset() noexcept {
  clearSide(left_);
  clearSide(right_);
  has_epoch_ = false;
  epoch_ = 0U;
}

void PicoWristAlignment::resetSide(ArmSide side) noexcept {
  clearSide(side == ArmSide::kLeft ? left_ : right_);
}

std::size_t PicoWristAlignment::stableCount(ArmSide side) const noexcept {
  const SideState& state = side == ArmSide::kLeft ? left_ : right_;
  return state.candidate_count;
}

bool PicoWristAlignment::aligned(ArmSide side) const noexcept {
  const SideState& state = side == ArmSide::kLeft ? left_ : right_;
  return state.has_reference;
}

bool PicoWristAlignment::isValidPose(const Pose& pose) noexcept {
  return pose.position.allFinite() &&
         isProperRotation(pose.rotation, kRotationValidationTolerance);
}

Pose PicoWristAlignment::compose(const Pose& first, const Pose& second) noexcept {
  Pose result;
  result.rotation = first.rotation * second.rotation;
  result.position = first.position + first.rotation * second.position;
  return result;
}

Pose PicoWristAlignment::relative(const Pose& reference,
                                  const Pose& current) noexcept {
  Pose result;
  result.rotation = reference.rotation.transpose() * current.rotation;
  result.position = reference.rotation.transpose() *
                    (current.position - reference.position);
  return result;
}

double PicoWristAlignment::positionStep(const Pose& first,
                                         const Pose& second) noexcept {
  return (first.position - second.position).norm();
}

double PicoWristAlignment::orientationStep(const Pose& first,
                                            const Pose& second) noexcept {
  return rotationDistance(first.rotation, second.rotation);
}

WristAlignmentSideResult PicoWristAlignment::updateSide(
    SideState& state, bool active, const Pose& pico_pose, const Pose& robot_pose,
    bool alignment_reset) {
  WristAlignmentSideResult result;
  result.target = state.has_last_target ? state.last_target : robot_pose;
  if (!active) {
    clearSide(state);
    result.target = state.has_last_target ? state.last_target : robot_pose;
    result.hold_reason = "inactive";
    return result;
  }
  if (!isValidPose(pico_pose) || !isValidPose(robot_pose)) {
    const Pose last_target = state.has_last_target ? state.last_target : robot_pose;
    clearSide(state);
    result.target = last_target;
    result.hold_reason = "invalid_pose";
    return result;
  }

  if (state.has_reference) {
    if (positionStep(pico_pose, state.candidate) >
            config_.max_stable_position_step_m ||
        orientationStep(pico_pose, state.candidate) >
            config_.max_stable_orientation_step_rad) {
      const Pose last_target = state.has_last_target ? state.last_target : robot_pose;
      clearSide(state);
      state.candidate = pico_pose;
      state.has_candidate = true;
      state.candidate_count = 1U;
      result.target = last_target;
      result.hold_reason = "stable_window";
      return result;
    }
    state.candidate = pico_pose;
    Pose delta = relative(state.pico_reference, pico_pose);
    delta.position *= config_.position_scale;
    result.target = compose(state.robot_reference, delta);
    result.valid = true;
    result.aligned = true;
    state.last_target = result.target;
    state.has_last_target = true;
    return result;
  }

  if (state.has_candidate &&
      (positionStep(pico_pose, state.candidate) >
           config_.max_stable_position_step_m ||
       orientationStep(pico_pose, state.candidate) >
           config_.max_stable_orientation_step_rad)) {
    state.candidate = pico_pose;
    state.candidate_count = 1U;
  } else {
    state.candidate = pico_pose;
    state.has_candidate = true;
    ++state.candidate_count;
  }

  if (state.candidate_count >= config_.stable_frames) {
    state.pico_reference = pico_pose;
    state.robot_reference = robot_pose;
    state.has_reference = true;
    result.target = robot_pose;
    result.valid = true;
    result.aligned = true;
    state.last_target = result.target;
    state.has_last_target = true;
    return result;
  }

  result.target = state.has_last_target ? state.last_target : robot_pose;
  result.hold_reason = alignment_reset ? "alignment_reset" : "stable_window";
  return result;
}

void PicoWristAlignment::clearSide(SideState& state) noexcept {
  state.has_candidate = false;
  state.candidate_count = 0U;
  state.has_reference = false;
}

}  // namespace tianji_qp_ik
