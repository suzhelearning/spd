#include <pybind11/pybind11.h>

void bind_physics(pybind11::module_ &);
void bind_executor(pybind11::module_ &);
void bind_control(pybind11::module_ &);

PYBIND11_MODULE(_spd_native, m) {
  m.doc() = "ROS 2 native command, control, MuJoCo and contact runtime";
  bind_physics(m);
  bind_executor(m);
  bind_control(m);
}
