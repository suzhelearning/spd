#pragma once

#include "tianji_qp_ik/mujoco_robot.hpp"

#include <memory>
#include <string>

namespace tianji_qp_ik {

class PinocchioArmKinematics {
 public:
  explicit PinocchioArmKinematics(const std::string& urdf_path);
  ~PinocchioArmKinematics();

  PinocchioArmKinematics(const PinocchioArmKinematics&) = delete;
  PinocchioArmKinematics& operator=(const PinocchioArmKinematics&) = delete;
  PinocchioArmKinematics(PinocchioArmKinematics&&) noexcept;
  PinocchioArmKinematics& operator=(PinocchioArmKinematics&&) noexcept;

  int configurationSize() const noexcept;
  int velocitySize() const noexcept;
  ArmKinematicSample sample(ArmSide side, const Vec7& q);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace tianji_qp_ik
