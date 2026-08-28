#pragma once

#include "tianji_qp_ik/pico_teleop_protocol.hpp"

#include <cstdint>
#include <cstddef>
#include <string>

namespace tianji_qp_ik {

struct WristAlignmentConfig {
  std::size_t stable_frames{10};
  double max_stable_position_step_m{0.02};
  double max_stable_orientation_step_rad{0.15};
  double position_scale{1.0};
};

struct WristAlignmentSideResult {
  bool valid{false};
  bool aligned{false};
  Pose target;
  std::string hold_reason;
};

struct WristAlignmentUpdate {
  WristAlignmentSideResult left;
  WristAlignmentSideResult right;
};

class PicoWristAlignment {
 public:
  explicit PicoWristAlignment(WristAlignmentConfig config = {});

  WristAlignmentUpdate update(const PicoTeleopFrame& frame,
                              const Pose& current_left,
                              const Pose& current_right);
  void reset() noexcept;
  void resetSide(ArmSide side) noexcept;

  std::size_t stableCount(ArmSide side) const noexcept;
  bool aligned(ArmSide side) const noexcept;

 private:
  struct SideState {
    bool has_candidate{false};
    std::size_t candidate_count{0U};
    Pose candidate;
    bool has_reference{false};
    Pose pico_reference;
    Pose robot_reference;
    Pose last_target;
    bool has_last_target{false};
  };

  static bool isValidPose(const Pose& pose) noexcept;
  static Pose compose(const Pose& first, const Pose& second) noexcept;
  static Pose relative(const Pose& reference, const Pose& current) noexcept;
  static double positionStep(const Pose& first, const Pose& second) noexcept;
  static double orientationStep(const Pose& first, const Pose& second) noexcept;

  WristAlignmentSideResult updateSide(SideState& state, bool active,
                                      const Pose& pico_pose,
                                      const Pose& robot_pose,
                                      bool alignment_reset);
  void clearSide(SideState& state) noexcept;

  WristAlignmentConfig config_;
  SideState left_;
  SideState right_;
  bool has_epoch_{false};
  std::uint64_t epoch_{0U};
};

}  // namespace tianji_qp_ik
