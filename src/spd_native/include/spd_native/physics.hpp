#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include <mujoco/mujoco.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

namespace spd_native {

namespace py = pybind11;
using JointValues = std::array<double, 54>;

// Shared ROS/Python boundary decoder; physics applies the servo envelope afterwards.
std::pair<JointValues, int> decode_joint_command(py::handle message);

// Model-local material policy; absent policies retain ordinary MuJoCo dynamics.
void material_step(const mjModel* model, mjData* data);
void material_forward(const mjModel* model, mjData* data);

struct PhysicsStep {
  std::int64_t tick;
  std::int64_t sim_time_ns;
  bool finite;
};

class PhysicsCheckpoint;

class Physics {
 public:
  Physics(py::object model, py::object data,
          py::array_t<int, py::array::c_style | py::array::forcecast> qpos_indices,
          py::array_t<int, py::array::c_style | py::array::forcecast> dof_indices,
          py::array_t<int, py::array::c_style | py::array::forcecast> actuator_indices,
          py::array_t<double, py::array::c_style | py::array::forcecast> limits,
          py::array_t<double, py::array::c_style | py::array::forcecast> home);

  JointValues positions() const;
  JointValues velocities() const;
  JointValues start_positions() const;
  const JointValues& targets() const noexcept { return targets_; }
  JointValues validate_values(const JointValues& values, int ready_mask) const;
  void submit_values(const JointValues& values, int ready_mask, int hold_mask = 0);
  void set_hold(int hold_mask);
  PhysicsStep physics_tick();
  std::shared_ptr<PhysicsCheckpoint> capture_checkpoint();
  void restore_checkpoint(const std::shared_ptr<PhysicsCheckpoint>& checkpoint);
  void inherit_robot_state(const Physics& previous);
  void close() noexcept { closed_ = true; }

  std::int64_t tick() const noexcept { return tick_; }
  std::int64_t sim_time_ns() const;
  int hold_mask() const noexcept { return hold_mask_; }
  bool fresh() const noexcept { return fresh_; }

 private:
  void require_open() const;
  void validate_checkpoint(const PhysicsCheckpoint& checkpoint);
  void initialize_home();

  py::object model_owner_;
  py::object data_owner_;
  mjModel* model_;
  mjData* data_;
  const mjtNum* material_friction_ = nullptr;
  std::array<int, 54> qpos_indices_;
  std::array<int, 54> dof_indices_;
  std::array<int, 54> actuator_indices_;
  std::array<std::array<double, 2>, 54> limits_;
  JointValues home_;
  JointValues targets_;
  std::shared_ptr<const int> identity_ = std::make_shared<const int>(0);
  std::vector<mjtNum> state_buffer_;
  std::int64_t tick_ = 0;
  int hold_mask_ = 7;
  bool fresh_ = true;
  bool closed_ = false;
};

}  // namespace spd_native

void bind_physics(pybind11::module_& module);
