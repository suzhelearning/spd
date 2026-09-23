#include <pybind11/embed.h>
#include <iostream>

namespace py = pybind11;

int main(int argc, char **argv) {
  py::scoped_interpreter interpreter(true, argc, argv);
  try {
    // Libraries such as GLFW launch Python helpers through sys.executable.
    // The embedding executable is not a Python CLI and must never be respawned.
    auto sys = py::module_::import("sys");
    sys.attr("executable") = py::str(sys.attr("prefix")).cast<std::string>() + "/bin/python";
    return py::module_::import("simulation.ros_viewer").attr("main")().cast<int>();
  } catch (const py::error_already_set &error) {
    if (error.matches(PyExc_SystemExit)) {
      const auto code = error.value().attr("code");
      return code.is_none() ? 0 : (py::isinstance<py::int_>(code) ? code.cast<int>() : 1);
    }
    std::cerr << error.what() << '\n';
    return 1;
  }
}
