#include "spd_native/physics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include <pybind11/stl.h>

namespace spd_native {
namespace {

constexpr int kReadyMask = 7;
constexpr std::size_t kJointCount = 54;
using NumericArray = py::array_t<double, py::array::c_style | py::array::forcecast>;
using IndexArray = py::array_t<int, py::array::c_style | py::array::forcecast>;

void require_model_data(const py::object& model, const py::object& data) {
  const auto mujoco = py::module_::import("mujoco");
  if (!py::isinstance(model, mujoco.attr("MjModel")) ||
      !py::isinstance(data, mujoco.attr("MjData"))) {
    throw py::type_error("physics requires MuJoCo MjModel and MjData objects");
  }
  if (data.attr("model").ptr() != model.ptr()) {
    throw py::value_error("physics data belongs to a different model");
  }
}

template <typename T>
T* address(const py::object& object) {
  const auto pointer = object.attr("_address").cast<std::uintptr_t>();
  if (!pointer) {
    throw py::value_error("MuJoCo object has no native storage");
  }
  return reinterpret_cast<T*>(pointer);
}

void validate_mask(int mask) {
  if (mask < 0 || (mask & ~kReadyMask)) {
    throw py::value_error("joint command mask contains reserved bits");
  }
}

int group_bit(std::size_t index) {
  return index < 14 ? 1 : (index < 34 ? 4 : 2);
}

template <typename T>
bool all_finite(const T* values, std::size_t count) {
  for (std::size_t i = 0; i < count; ++i) {
    if (!std::isfinite(values[i])) return false;
  }
  return true;
}

void copy_indices(const IndexArray& values, std::array<int, 54>& output, int size) {
  if (values.ndim() != 1 || values.shape(0) != 54) {
    throw py::value_error("joint mappings must contain 54 indices");
  }
  for (std::size_t i = 0; i < kJointCount; ++i) {
    const int index = values.data()[i];
    if (index < 0 || index >= size ||
        std::find(output.begin(), output.begin() + i, index) != output.begin() + i) {
      throw py::value_error("joint mappings must contain unique in-bounds indices");
    }
    output[i] = index;
  }
}

JointValues joint_values(const NumericArray& values) {
  if (values.ndim() != 1 || values.shape(0) != 54) {
    throw py::value_error("position_rad must contain 54 finite values");
  }
  JointValues result;
  std::copy_n(values.data(), result.size(), result.begin());
  return result;
}

py::array_t<double> joint_array(const JointValues& values, bool readonly = false) {
  py::array_t<double> result(values.size());
  std::copy(values.begin(), values.end(), result.mutable_data());
  if (readonly) result.attr("setflags")(false);
  return result;
}

py::array_t<bool> contact_array(const std::vector<unsigned char>& values,
                              std::size_t object_count, bool readonly = false) {
  py::array_t<bool> result({py::ssize_t(2), static_cast<py::ssize_t>(object_count)});
  bool* output = result.mutable_data();
  for (std::size_t i = 0; i < values.size(); ++i) output[i] = values[i] != 0;
  if (readonly) result.attr("setflags")(false);
  return result;
}

constexpr int kMaterialCount = 8;

const mjtNum* material_friction(const mjModel* model) {
  const int id = mj_name2id(model, mjOBJ_NUMERIC, "spd_material_friction");
  if (id < 0) return nullptr;
  const mjtNum* numeric = model->numeric_data + model->numeric_adr[id];
  if (model->numeric_size[id] != 1 + kMaterialCount * kMaterialCount ||
      numeric[0] != 2 || model->nuser_geom < 4) {
    throw std::invalid_argument("invalid spd_material_friction version-2 policy");
  }
  for (int i = 1; i <= kMaterialCount * kMaterialCount; ++i) {
    if (!std::isfinite(numeric[i]) || (numeric[i] < 0 && numeric[i] != -1)) {
      throw std::invalid_argument("material friction must be finite, nonnegative or -1");
    }
  }
  return numeric + 1;
}

int contact_material(const mjModel* model, const mjData* data,
                     const mjContact& contact, int side) {
  const int geom = contact.geom[side];
  if (geom < 0 || geom >= model->ngeom) return 0;
  const mjtNum* user = model->geom_user + geom * model->nuser_geom;
  if (!(user[2] >= 0 && user[2] < kMaterialCount) ||
      !(user[3] >= 0 && user[3] <= 3)) {
    return 0;
  }
  const int material = static_cast<int>(user[2]);
  const int region = static_cast<int>(user[3]);
  if (user[2] != material || user[3] != region) return 0;
  if (!region) return material;

  // Contact normal points geom0 -> geom1. Rotate each outward normal into
  // BODY (not geom or inertial) coordinates using a column of body xmat.
  const mjtNum* rotation = data->xmat + 9 * model->geom_bodyid[geom];
  const int axis = region == 1 ? 2 : 1;
  const mjtNum component = (side == 0 ? 1 : -1) *
      (rotation[axis] * contact.frame[0] +
       rotation[3 + axis] * contact.frame[1] +
       rotation[6 + axis] * contact.frame[2]);
  if (region == 1) return component < -0.5 ? 4 : material;
  if (region == 2) return component < 0 ? 7 : 0;
  return component > 0 ? 7 : 0;
}

bool override_material_contacts(const mjModel* model, mjData* data,
                                const mjtNum* friction) {
  bool changed = false;
  for (int i = 0; i < data->ncon; ++i) {
    mjContact& contact = data->contact[i];
    if (contact.geom[0] < 0 || contact.geom[1] < 0) continue;
    int first = contact_material(model, data, contact, 0);
    int second = contact_material(model, data, contact, 1);
    if (!first || !second) continue;
    if (first > second) std::swap(first, second);
    const mjtNum mu = friction[first * kMaterialCount + second];
    if (mu < 0) continue;
    if (contact.friction[0] != mu || contact.friction[1] != mu) {
      contact.friction[0] = contact.friction[1] = mu;
      changed = true;
    }
  }
  return changed;
}

bool rebuild_material_constraints(const mjModel* model, mjData* data,
                                  const mjtNum* friction) {
  if (data->efm_active) {
    throw std::invalid_argument("material friction does not support an active flex effective metric");
  }
  if (!override_material_contacts(model, data, friction)) return false;
  // MuJoCo 3.12 rewinds its arena to the end of the existing contacts here.
  // Rebuild the cone Jacobian, regularization R/contact.mu, islands and
  // projected inertia before reference velocities or the solver consume them.
  // Actuator transmission/moments live in the main buffer in 3.12 and survive.
  mj_makeConstraint(model, data);
  mj_island(model, data);
  mj_projectConstraint(model, data);
  return true;
}

void step_with_material(const mjModel* model, mjData* data, const mjtNum* friction) {
  if (!friction) {
    mj_step(model, data);
    return;
  }
  if (model->opt.integrator != mjINT_EULER &&
      model->opt.integrator != mjINT_IMPLICIT &&
      model->opt.integrator != mjINT_IMPLICITFAST) {
    throw std::invalid_argument("material_step supports Euler, implicit and implicitfast");
  }
  mj_step1(model, data);
  if (rebuild_material_constraints(model, data, friction)) {
    mj_referenceConstraint(model, data);
  }
  mj_step2(model, data);
}

void forward_with_material(const mjModel* model, mjData* data, const mjtNum* friction) {
  if (!friction) {
    mj_forward(model, data);
    return;
  }
  mj_fwdPosition(model, data);
  rebuild_material_constraints(model, data, friction);
  mj_sensorPos(model, data);
  if (!data->flg_energypos) {
    if (model->opt.enableflags & mjENBL_ENERGY) {
      mj_energyPos(model, data);
    } else {
      data->energy[0] = data->energy[1] = 0;
    }
  }
  // Recompute velocity references, actuation, acceleration, constraints and
  // force sensors without another collision pass or advancing simulation time.
  mj_forwardSkip(model, data, mjSTAGE_POS, 0);
}

}  // namespace

class PhysicsCheckpoint {
 public:
  py::object model;
  py::object data;
  std::shared_ptr<const int> identity;
  JointValues targets;
  std::int64_t tick;
  int hold_mask;
};

void material_step(const mjModel* model, mjData* data) {
  step_with_material(model, data, material_friction(model));
}

void material_forward(const mjModel* model, mjData* data) {
  forward_with_material(model, data, material_friction(model));
}

Physics::Physics(py::object model, py::object data, IndexArray qpos_indices,
                 IndexArray dof_indices, IndexArray actuator_indices,
                 NumericArray limits, NumericArray home)
    : model_owner_(std::move(model)), data_owner_(std::move(data)) {
  require_model_data(model_owner_, data_owner_);
  model_ = address<mjModel>(model_owner_);
  data_ = address<mjData>(data_owner_);
  material_friction_ = material_friction(model_);
  copy_indices(qpos_indices, qpos_indices_, model_->nq);
  copy_indices(dof_indices, dof_indices_, model_->nv);
  copy_indices(actuator_indices, actuator_indices_, model_->nu);
  if (limits.ndim() != 2 || limits.shape(0) != 54 || limits.shape(1) != 2) {
    throw py::value_error("joint limits must have shape (54, 2)");
  }
  home_ = joint_values(home);
  for (std::size_t i = 0; i < kJointCount; ++i) {
    limits_[i] = {limits.data()[2 * i], limits.data()[2 * i + 1]};
    if (!std::isfinite(limits_[i][0]) || !std::isfinite(limits_[i][1]) ||
        limits_[i][0] > limits_[i][1]) {
      throw py::value_error("joint limits must be finite nonempty intervals");
    }
  }
  if (validate_values(home_, kReadyMask) != home_) {
    throw py::value_error("home is outside the manifest/actuator limits");
  }
  state_buffer_.resize(mj_stateSize(model_, mjSTATE_INTEGRATION));
  initialize_home();
}

void Physics::initialize_home() {
  // Scene free joints retain qpos0; only mapped robot joints are sent HOME.
  mju_copy(data_->qpos, model_->qpos0, model_->nq);
  mju_zero(data_->qvel, model_->nv);
  mju_zero(data_->ctrl, model_->nu);
  mju_zero(data_->act, model_->na);
  targets_ = home_;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    data_->qpos[qpos_indices_[i]] = home_[i];
    data_->ctrl[actuator_indices_[i]] = home_[i];
  }
  data_->time = 0;
  forward_with_material(model_, data_, material_friction_);
}

void Physics::require_open() const {
  if (closed_) throw std::runtime_error("plant is shut down");
}

JointValues Physics::positions() const {
  JointValues result;
  for (std::size_t i = 0; i < kJointCount; ++i) result[i] = data_->qpos[qpos_indices_[i]];
  return result;
}

JointValues Physics::velocities() const {
  JointValues result;
  for (std::size_t i = 0; i < kJointCount; ++i) result[i] = data_->qvel[dof_indices_[i]];
  return result;
}

JointValues Physics::start_positions() const {
  JointValues result = positions();
  for (std::size_t i = 0; i < kJointCount; ++i) {
    if (!std::isfinite(result[i])) {
      throw py::value_error("transition origin contains non-finite joint positions");
    }
    result[i] = std::clamp(result[i], limits_[i][0], limits_[i][1]);
  }
  return result;
}

JointValues Physics::validate_values(const JointValues& values, int ready_mask) const {
  validate_mask(ready_mask);
  if (!all_finite(values.data(), values.size())) {
    throw py::value_error("position_rad must contain 54 finite values");
  }
  JointValues targets;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    targets[i] = std::clamp(values[i], limits_[i][0], limits_[i][1]);
  }
  return targets;
}

void Physics::submit_values(const JointValues& values, int ready_mask, int hold_mask) {
  require_open();
  const auto targets = validate_values(values, ready_mask);
  validate_mask(hold_mask);
  const int effective_hold = (kReadyMask ^ ready_mask) | hold_mask;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    if (!(effective_hold & group_bit(i))) targets_[i] = targets[i];
  }
  hold_mask_ = effective_hold;
  fresh_ = false;
}

void Physics::set_hold(int hold_mask) {
  validate_mask(hold_mask);
  hold_mask_ = hold_mask;
}

std::int64_t Physics::sim_time_ns() const {
  const double nanoseconds = std::nearbyint(data_->time * 1e9);
  if (!std::isfinite(nanoseconds) || nanoseconds < -0x1p63 || nanoseconds >= 0x1p63) {
    throw py::value_error("simulation time cannot be represented in nanoseconds");
  }
  return static_cast<std::int64_t>(nanoseconds);
}

PhysicsStep Physics::physics_tick() {
  require_open();
  if (tick_ == std::numeric_limits<std::int64_t>::max()) {
    throw std::overflow_error("physics tick counter exhausted");
  }
  fresh_ = false;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    data_->ctrl[actuator_indices_[i]] = targets_[i];
  }
  step_with_material(model_, data_, material_friction_);
  ++tick_;
  const bool finite = all_finite(data_->qpos, model_->nq) &&
                      all_finite(data_->qvel, model_->nv) &&
                      all_finite(data_->ctrl, model_->nu) && std::isfinite(data_->time);
  return {tick_, std::isfinite(data_->time) ? sim_time_ns() : 0, finite};
}

void Physics::validate_checkpoint(const PhysicsCheckpoint& checkpoint) {
  if (checkpoint.identity != identity_ || !checkpoint.model.is(model_owner_)) {
    throw py::value_error("checkpoint belongs to a different plant or model");
  }
  if (checkpoint.tick < 0) {
    throw py::value_error("checkpoint has invalid tick, targets or hold mask");
  }
  validate_mask(checkpoint.hold_mask);
  if (validate_values(checkpoint.targets, kReadyMask) != checkpoint.targets) {
    throw py::value_error("checkpoint targets are outside the manifest/actuator limits");
  }
  require_model_data(checkpoint.model, checkpoint.data);
  const auto* data = address<mjData>(checkpoint.data);
  mj_getState(model_, data, state_buffer_.data(), mjSTATE_INTEGRATION);
  if (!all_finite(state_buffer_.data(), state_buffer_.size()) ||
      !all_finite(data->qacc, model_->nv) || data->time < 0) {
    throw py::value_error("checkpoint contains invalid physics state");
  }
}

std::shared_ptr<PhysicsCheckpoint> Physics::capture_checkpoint() {
  require_open();
  auto checkpoint = std::make_shared<PhysicsCheckpoint>();
  checkpoint->model = model_owner_;
  checkpoint->data = py::module_::import("mujoco").attr("MjData")(model_owner_);
  checkpoint->identity = identity_;
  checkpoint->targets = targets_;
  checkpoint->tick = tick_;
  checkpoint->hold_mask = hold_mask_;
  mj_copyData(address<mjData>(checkpoint->data), model_, data_);
  validate_checkpoint(*checkpoint);
  return checkpoint;
}

void Physics::restore_checkpoint(const std::shared_ptr<PhysicsCheckpoint>& checkpoint) {
  require_open();
  if (!checkpoint) throw py::value_error("checkpoint belongs to a different plant or model");
  validate_checkpoint(*checkpoint);
  mj_copyData(data_, model_, address<mjData>(checkpoint->data));
  tick_ = checkpoint->tick;
  targets_ = checkpoint->targets;
  hold_mask_ = checkpoint->hold_mask;
  fresh_ = false;
}

void Physics::inherit_robot_state(const Physics& previous) {
  require_open();
  previous.require_open();
  if (&previous == this || previous.model_ == model_ || previous.data_ == data_) {
    throw py::value_error("robot state requires a new model and independent physics data");
  }
  if (!fresh_ || tick_ != 0 || data_->time != 0) {
    throw py::value_error("robot state can only be inherited by a fresh plant");
  }
  if (validate_values(previous.targets_, kReadyMask) != previous.targets_) {
    throw py::value_error("retained targets are outside the manifest/actuator limits");
  }
  for (std::size_t i = 0; i < kJointCount; ++i) {
    data_->qpos[qpos_indices_[i]] = previous.data_->qpos[previous.qpos_indices_[i]];
    data_->qvel[dof_indices_[i]] = previous.data_->qvel[previous.dof_indices_[i]];
    data_->ctrl[actuator_indices_[i]] = previous.targets_[i];
  }
  targets_ = previous.targets_;
  hold_mask_ = kReadyMask;
  fresh_ = false;
  forward_with_material(model_, data_, material_friction_);
}


class ContactState {
 public:
  std::shared_ptr<const int> identity;
  std::vector<unsigned char> contacts;
  std::size_t object_count;
};

class ContactCollector {
 public:
  ContactCollector(py::object model, py::object data,
                   const std::vector<std::vector<int>>& hand_geoms,
                   const std::vector<std::vector<int>>& object_geoms)
      : model_owner_(std::move(model)), data_owner_(std::move(data)),
        scratch_(nullptr, mj_deleteData), object_count_(object_geoms.size()),
        contacts_(2 * object_count_, 0) {
    require_model_data(model_owner_, data_owner_);
    model_ = address<mjModel>(model_owner_);
    data_ = address<mjData>(data_owner_);
    if (hand_geoms.size() != 2) {
      throw py::value_error("contact collection requires two hand geometry mappings");
    }
    geom_hand_.assign(model_->ngeom, -1);
    geom_object_.assign(model_->ngeom, -1);
    const auto map = [this](const std::vector<std::vector<int>>& groups,
                            std::vector<int>& output) {
      for (std::size_t index = 0; index < groups.size(); ++index) {
        for (const int geom : groups[index]) {
          if (geom < 0 || geom >= model_->ngeom || geom_hand_[geom] >= 0 ||
              geom_object_[geom] >= 0) {
            throw py::value_error("hand and object geometry mappings must be disjoint and in bounds");
          }
          output[geom] = static_cast<int>(index);
        }
      }
    };
    map(hand_geoms, geom_hand_);
    map(object_geoms, geom_object_);
    scratch_.reset(mj_makeData(model_));
    if (!scratch_) throw std::bad_alloc();
  }

  void reset() { std::fill(contacts_.begin(), contacts_.end(), 0); }

  void observe() {
    for (int index = 0; index < data_->ncon; ++index) {
      const auto& contact = data_->contact[index];
      if (!active(contact)) continue;
      const int first = contact.geom[0], second = contact.geom[1];
      accumulate(geom_hand_[first], geom_object_[second]);
      accumulate(geom_hand_[second], geom_object_[first]);
    }
  }

  bool current() {
    if (!object_count_) return false;
    // mj_step contacts describe the pre-integration pose. Refresh only scratch data.
    mj_copyData(scratch_.get(), model_, data_);
    mj_fwdPosition(model_, scratch_.get());
    for (int index = 0; index < scratch_->ncon; ++index) {
      const auto& contact = scratch_->contact[index];
      if (!active(contact)) continue;
      const int first = contact.geom[0], second = contact.geom[1];
      if ((geom_hand_[first] >= 0 && geom_object_[second] >= 0) ||
          (geom_hand_[second] >= 0 && geom_object_[first] >= 0)) return true;
    }
    return false;
  }

  std::shared_ptr<ContactState> capture_state() const {
    auto state = std::make_shared<ContactState>();
    state->identity = identity_;
    state->contacts = contacts_;
    state->object_count = object_count_;
    return state;
  }

  void restore_state(const std::shared_ptr<ContactState>& state) {
    if (!state || state->identity != identity_ || state->object_count != object_count_ ||
        state->contacts.size() != contacts_.size()) {
      throw py::value_error("invalid contact interval checkpoint");
    }
    std::copy(state->contacts.begin(), state->contacts.end(), contacts_.begin());
  }

  py::array_t<bool> contacts() const { return contact_array(contacts_, object_count_); }

  py::array_t<bool> hand_contact() const {
    py::array_t<bool> result(2);
    for (std::size_t hand = 0; hand < 2; ++hand) {
      const auto first = contacts_.begin() + hand * object_count_;
      result.mutable_data()[hand] =
          std::any_of(first, first + object_count_, [](unsigned char value) { return value != 0; });
    }
    return result;
  }

 private:
  bool active(const mjContact& contact) const {
    return contact.geom[0] >= 0 && contact.geom[0] < model_->ngeom &&
           contact.geom[1] >= 0 && contact.geom[1] < model_->ngeom && contact.efc_address >= 0;
  }

  void accumulate(int hand, int object) {
    if (hand >= 0 && object >= 0) contacts_[hand * object_count_ + object] = 1;
  }

  py::object model_owner_;
  py::object data_owner_;
  mjModel* model_;
  mjData* data_;
  std::unique_ptr<mjData, decltype(&mj_deleteData)> scratch_;
  std::size_t object_count_;
  std::vector<int> geom_hand_;
  std::vector<int> geom_object_;
  std::vector<unsigned char> contacts_;
  std::shared_ptr<const int> identity_ = std::make_shared<const int>(0);
};

}  // namespace spd_native

void bind_physics(pybind11::module_& module) {
  namespace py = pybind11;
  using namespace spd_native;
  module.def("material_step", [](const py::object& model, const py::object& data) {
    require_model_data(model, data);
    const mjModel* native_model = address<mjModel>(model);
    mjData* native_data = address<mjData>(data);
    py::gil_scoped_release release;
    material_step(native_model, native_data);
  }, py::arg("model"), py::arg("data"));
  module.def("material_forward", [](const py::object& model, const py::object& data) {
    require_model_data(model, data);
    const mjModel* native_model = address<mjModel>(model);
    mjData* native_data = address<mjData>(data);
    py::gil_scoped_release release;
    material_forward(native_model, native_data);
  }, py::arg("model"), py::arg("data"));
  py::class_<PhysicsStep>(module, "PhysicsStep")
      .def(py::init<std::int64_t, std::int64_t, bool>(), py::arg("tick"),
           py::arg("sim_time_ns"), py::arg("finite"))
      .def_readonly("tick", &PhysicsStep::tick)
      .def_readonly("sim_time_ns", &PhysicsStep::sim_time_ns)
      .def_readonly("finite", &PhysicsStep::finite);

  py::class_<PhysicsCheckpoint, std::shared_ptr<PhysicsCheckpoint>>(module, "PhysicsCheckpoint")
      .def_readonly("model", &PhysicsCheckpoint::model)
      .def_readonly("data", &PhysicsCheckpoint::data)
      .def_readonly("tick", &PhysicsCheckpoint::tick)
      .def_readonly("hold_mask", &PhysicsCheckpoint::hold_mask)
      .def_property_readonly("targets", [](const PhysicsCheckpoint& self) {
        return joint_array(self.targets, true);
      });

  py::class_<Physics, std::shared_ptr<Physics>>(module, "Physics")
      .def(py::init<py::object, py::object, IndexArray, IndexArray, IndexArray,
                    NumericArray, NumericArray>(),
           py::arg("model"), py::arg("data"), py::arg("qpos_indices"),
           py::arg("dof_indices"), py::arg("actuator_indices"), py::arg("limits"), py::arg("home"))
      .def_property_readonly("tick", &Physics::tick)
      .def_property_readonly("sim_time_ns", &Physics::sim_time_ns)
      .def_property("hold_mask", &Physics::hold_mask, &Physics::set_hold)
      .def_property_readonly("fresh", &Physics::fresh)
      .def("validate_joint_command", [](const Physics& self, py::handle snapshot) {
        const auto [values, ready_mask] = decode_joint_command(snapshot);
        return joint_array(self.validate_values(values, ready_mask));
      }, py::arg("snapshot"))
      .def("submit_joint_command", [](Physics& self, py::handle snapshot, int hold_mask) {
        const auto [values, ready_mask] = decode_joint_command(snapshot);
        self.submit_values(values, ready_mask, hold_mask);
      }, py::arg("snapshot"), py::kw_only(), py::arg("hold_mask") = 0)
      .def("positions", [](const Physics& self) { return joint_array(self.positions()); })
      .def("velocities", [](const Physics& self) { return joint_array(self.velocities()); })
      .def("targets", [](const Physics& self) { return joint_array(self.targets()); })
      .def("start_positions", [](const Physics& self) { return joint_array(self.start_positions()); })
      .def("validate_values", [](const Physics& self, const NumericArray& values, int ready_mask) {
        return joint_array(self.validate_values(joint_values(values), ready_mask));
      }, py::arg("values"), py::arg("ready_mask"))
      .def("submit_values", [](Physics& self, const NumericArray& values, int ready_mask, int hold_mask) {
        self.submit_values(joint_values(values), ready_mask, hold_mask);
      }, py::arg("values"), py::arg("ready_mask"), py::arg("hold_mask") = 0)
      .def("set_hold", &Physics::set_hold)
      .def("physics_tick", &Physics::physics_tick, py::call_guard<py::gil_scoped_release>())
      .def("capture_checkpoint", &Physics::capture_checkpoint)
      .def("restore_checkpoint", &Physics::restore_checkpoint)
      .def("inherit_robot_state", &Physics::inherit_robot_state)
      .def("close", &Physics::close);

  py::class_<ContactState, std::shared_ptr<ContactState>>(module, "ContactState")
      .def_property_readonly("contacts", [](const ContactState& self) {
        return contact_array(self.contacts, self.object_count, true);
      });
  py::class_<ContactCollector>(module, "ContactCollector")
      .def(py::init<py::object, py::object, const std::vector<std::vector<int>>&,
                    const std::vector<std::vector<int>>&>(),
           py::arg("model"), py::arg("data"), py::arg("hand_geom_ids"), py::arg("object_geom_ids"))
      .def("reset", &ContactCollector::reset)
      .def("observe", &ContactCollector::observe)
      .def("current", &ContactCollector::current)
      .def("capture_state", &ContactCollector::capture_state)
      .def("restore_state", &ContactCollector::restore_state)
      .def("contacts", &ContactCollector::contacts)
      .def("hand_contact", &ContactCollector::hand_contact);
}
