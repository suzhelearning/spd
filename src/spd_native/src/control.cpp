#include <pybind11/pybind11.h>

#include <chrono>
#include <cstdint>
#include <thread>

namespace py = pybind11;

namespace {
using SteadyClock = std::chrono::steady_clock;
constexpr std::int64_t kDisplayPeriodNs = 50'000'000;

std::int64_t monotonic_now_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               SteadyClock::now().time_since_epoch()).count();
}

void run_loop(const py::object &app) {
    const auto physics_hz = app.attr("plant").attr("physics_hz").cast<std::int64_t>();
    if (physics_hz <= 0) {
        throw py::value_error("physics_hz must be positive");
    }
    const std::int64_t period_ns = 1'000'000'000 / physics_hz;
    auto deadline = monotonic_now_ns();
    auto display_deadline = deadline;
    while (!app.attr("stop").cast<bool>() &&
           app.attr("executor").attr("context_ok").cast<bool>() &&
           app.attr("window").attr("is_running")().cast<bool>()) {
        if (PyErr_CheckSignals() != 0) {
            throw py::error_already_set();
        }
        if (app.attr("stop").cast<bool>()) break;
        app.attr("executor").attr("spin_once")();
        app.attr("three_key").attr("poll")();
        app.attr("_process_actions")();
        if (app.attr("stop").cast<bool>()) break;

        // The coordinator may replace the plant at a completed episode boundary.
        // Acquire all owners after polling; never step a stale scene reference.
        const py::object collection = app.attr("collection");
        const py::object executor = app.attr("executor");
        const py::object control = app.attr("three_key");
        collection.attr("poll")();
        const auto now = monotonic_now_ns();
        if (!collection.attr("physics_paused").cast<bool>()) {
            executor.attr("apply_pending")(py::arg("now_ns") = now);
            if (!executor.attr("mailbox").attr("enabled").cast<bool>() ||
                (executor.attr("hold_mask").cast<unsigned>() & 1U)) {
                control.attr("poll")();
            } else {
                const py::object step = app.attr("plant").attr("physics_tick")();
                collection.attr("tick")(step,
                    py::arg("recovery") = control.attr("recovery"),
                    py::arg("control_flags") = control.attr("control_flags"));
            }
        }
        app.attr("collection_ros").attr("heartbeat")();
        if (now >= display_deadline) {
            app.attr("_update_display")(now);
            display_deadline = now + kDisplayPeriodNs;
        }
        deadline += period_ns;
        const auto remaining = deadline - monotonic_now_ns();
        if (remaining > 0) {
            py::gil_scoped_release release;
            std::this_thread::sleep_for(std::chrono::nanoseconds(remaining));
        } else {
            deadline = monotonic_now_ns();
        }
    }
}
}  // namespace

void bind_control(py::module_ &module) {
    module.def("run_loop", &run_loop, py::arg("app"));
}
