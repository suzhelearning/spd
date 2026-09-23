#include <pybind11/pybind11.h>

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <thread>
#include <utility>

namespace py = pybind11;

namespace {

constexpr std::int64_t kMaxAgeNs = 100'000'000;
constexpr std::int64_t kDisplayPeriodNs = 50'000'000;
using SteadyClock = std::chrono::steady_clock;

std::int64_t monotonic_now_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               SteadyClock::now().time_since_epoch())
        .count();
}

std::int64_t wall_now_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

class ThreeKeyControl {
public:
    ThreeKeyControl(py::object collection, py::object executor,
                    py::object clock_ns, py::object monotonic_ns)
        : collection_(std::move(collection)), executor_(std::move(executor)),
          clock_ns_(std::move(clock_ns)), monotonic_ns_(std::move(monotonic_ns)) {
        if ((!clock_ns_.is_none() && !PyCallable_Check(clock_ns_.ptr())) ||
            (!monotonic_ns_.is_none() && !PyCallable_Check(monotonic_ns_.ptr()))) {
            throw py::type_error("clock_ns and monotonic_ns must be callable or None");
        }
        set_freeze(true);
    }

    const std::string &stage() const { return stage_; }
    const std::string &notice() const { return notice_; }
    void set_notice(std::string notice) { notice_ = std::move(notice); }
    bool freeze() const { return collection_.attr("control_paused").cast<bool>(); }
    void set_freeze(bool value) { collection_.attr("control_paused") = value; }
    int recovery() const {
        return executor_.attr("transition_frame").cast<bool>() ? recovery_ : 0;
    }

    void cancel_confirmation() { save_confirmation_.reset(); }

    void key(std::string key) {
        // All transition-time keys are dropped rather than queued for replay.
        if (stage_ == "blending" || stage_ == "preparing" ||
            stage_ == "preparation_failed" || stage_ == "reverting" ||
            stage_ == "rewind_wait" || stage_ == "saving" || stage_ == "aborting") {
            return;
        }
        for (char &character : key) {
            if (character >= 'A' && character <= 'Z') {
                character = static_cast<char>(character - 'A' + 'a');
            }
        }
        if (key != "r") {
            cancel_confirmation();
        }
        if (key == "r") {
            if (stage_ == "idle" && collection_state() == "idle") {
                start_episode();
            } else if (stage_ == "recording" && collection_state() == "recording") {
                notice_ = request("checkpoint").message;
            } else if (stage_ == "paused" && collection_state() == "paused") {
                SaveContext context{
                    collection_.attr("operation_id").cast<std::string>(),
                    collection_.attr("state_frames").cast<std::int64_t>()};
                if (!save_confirmation_ ||
                    save_confirmation_->operation_id != context.operation_id ||
                    save_confirmation_->state_frames != context.state_frames) {
                    save_confirmation_ = std::move(context);
                    notice_ = "Press r again to confirm successful completion; s resumes, d rewinds";
                    return;
                }
                cancel_confirmation();
                const auto response = request("save");
                notice_ = response.message;
                if (response.accepted) {
                    executor_.attr("clear")();
                    set_stage("saving", "Saving successful episode; next task waits for r");
                }
            }
        } else if (key == "s") {
            if (stage_ == "recording") {
                pause("Paused: checkpoint hand ghost shown; s resumes, d rewinds, r then r saves");
            } else if (stage_ == "paused" && collection_state() == "paused") {
                begin_recovery(2);
            }
        } else if (key == "d") {
            if (stage_ == "paused" && collection_state() == "paused") {
                const auto response = request("revert");
                notice_ = response.message;
                if (response.accepted) {
                    set_freeze(true);
                    set_stage("reverting", "Restoring checkpoint; automatic 1 second recovery follows");
                }
            }
        }
    }

    void poll() {
        if (stage_ == "preparing" || stage_ == "recording" || stage_ == "blending") {
            const py::object mailbox = executor_.attr("mailbox");
            const py::object candidate = mailbox.attr("latest");
            std::string reason = ready_reason(candidate);
            if (!mailbox.attr("enabled").cast<bool>()) {
                reason = mailbox.attr("last_reject_reason").cast<std::string>();
                if (reason.empty()) {
                    reason = "Motion authorization was revoked";
                }
            } else if (!candidate.is_none() &&
                       candidate.attr("session_id").cast<std::string>() != session_id_) {
                reason = "Upstream session changed";
            } else if ((executor_.attr("hold_mask").cast<unsigned>() & 1U) && reason.empty()) {
                reason = "Arm targets are held";
            }
            if (!reason.empty()) {
                pause(reason + "; explicit s required after recovery");
                return;
            }
        }

        const std::string state = collection_state();
        if (stage_ == "preparing") {
            if (state == "recording") {
                begin_recovery(1);
            } else if (state != "preparing") {
                pause("Collector is " + state);
            }
        } else if (stage_ == "blending") {
            if (state != "recording") {
                pause("Collector is " + state);
            } else if (!executor_.attr("transition_active").cast<bool>()) {
                recovery_ = 0;
                set_stage("recording", "r checkpoint; s pause; d is available only while paused");
            }
        } else if (stage_ == "recording") {
            if (state != "recording") {
                pause("Collector is " + state);
            }
        } else if (stage_ == "reverting") {
            if (state == "paused") {
                // Restoring clears the mailbox. A new 60 Hz target need not arrive
                // on the first 480 Hz iteration after the asynchronous restore.
                rewind_deadline_ns_ = monotonic_ns() + kMaxAgeNs;
                set_stage("rewind_wait", "Checkpoint restored; waiting for fresh target to auto-resume");
            } else if (state != "reverting") {
                pause("Rewind did not complete: " + collection_.attr("message").cast<std::string>());
            }
        } else if (stage_ == "rewind_wait") {
            const py::object candidate = executor_.attr("mailbox").attr("latest");
            if (!candidate.is_none() &&
                candidate.attr("session_id").cast<std::string>() != session_id_) {
                pause("Upstream session changed during rewind; explicit s required");
            } else if (ready_reason(candidate).empty()) {
                begin_recovery(3);
            } else if (monotonic_ns() >= rewind_deadline_ns_) {
                pause("No fresh target after rewind; explicit s required");
            }
        } else if (stage_ == "saving" || stage_ == "aborting" || stage_ == "preparation_failed") {
            set_freeze(true);
            if (state == "idle") {
                set_stage("idle", collection_.attr("message").cast<std::string>() +
                                      "; r starts the next episode");
            } else if (state == "error") {
                set_stage("error", collection_.attr("message").cast<std::string>() +
                                       "; frozen, restart required");
            } else if (state == "aborting" && stage_ != "aborting") {
                set_stage("aborting", collection_.attr("message").cast<std::string>());
            }
        }
    }

    void close() { pause("Local collector stopping; upstream continues independently"); }

private:
    struct SaveContext {
        std::string operation_id;
        std::int64_t state_frames;
    };
    struct RequestResult {
        bool accepted;
        std::string message;
    };

    std::string collection_state() const { return collection_.attr("state").cast<std::string>(); }
    std::int64_t clock_ns() const {
        return clock_ns_.is_none() ? wall_now_ns() : clock_ns_().cast<std::int64_t>();
    }
    std::int64_t monotonic_ns() const {
        return monotonic_ns_.is_none() ? monotonic_now_ns() : monotonic_ns_().cast<std::int64_t>();
    }
    RequestResult request(const char *operation) {
        const py::tuple result = collection_.attr("request")(operation).cast<py::tuple>();
        return {result[0].cast<bool>(), result[1].cast<py::dict>()["message"].cast<std::string>()};
    }
    void set_stage(std::string stage, std::string notice) {
        if (stage_ != stage) {
            cancel_confirmation();
        }
        stage_ = std::move(stage);
        notice_ = std::move(notice);
        py::print("SPD keys [" + stage_ + "]: " + notice_, py::arg("flush") = true);
    }
    std::string ready_reason(const py::object &candidate) const {
        if (candidate.is_none()) {
            return "Waiting for upstream targets: calibrate with r and start with s upstream";
        }
        if (!(candidate.attr("ready_mask").cast<unsigned>() & 1U)) {
            return "Upstream arm targets are not ready";
        }
        const auto stamp = candidate.attr("stamp_ns").cast<std::int64_t>();
        const auto now = clock_ns();
        if (stamp > now || stamp < now - kMaxAgeNs) {
            return "Upstream joint candidate is stale";
        }
        return {};
    }
    void pause(const std::string &reason) {
        set_freeze(true);
        if (collection_state() == "recording") {
            collection_.attr("request")("pause");
        }
        executor_.attr("clear")();
        recovery_ = 0;
        cancel_confirmation();
        auto stage = collection_state();
        if (stage == "preparing") {
            // The writer still owns its in-flight open; wait to preserve the partial.
            stage = "preparation_failed";
        }
        set_stage(std::move(stage), reason);
    }
    void start_episode() {
        const py::object mailbox = executor_.attr("mailbox");
        const auto reason = ready_reason(mailbox.attr("latest"));
        if (!reason.empty()) {
            notice_ = reason;
            return;
        }
        set_freeze(true);
        if (!executor_.attr("authorize_transition")().cast<bool>()) {
            notice_ = mailbox.attr("last_reject_reason").cast<std::string>();
            return;
        }
        session_id_ = mailbox.attr("authorized_session").cast<std::string>();
        const auto response = request("start");
        if (!response.accepted) {
            executor_.attr("clear")();
            notice_ = response.message;
            return;
        }
        set_stage("preparing", "Opening episode with checkpoint 0; no motion until ready");
    }
    void begin_recovery(int kind) {
        const py::object mailbox = executor_.attr("mailbox");
        const auto reason = ready_reason(mailbox.attr("latest"));
        if (!reason.empty() || !executor_.attr("authorize_transition")().cast<bool>()) {
            pause(reason.empty() ? mailbox.attr("last_reject_reason").cast<std::string>() : reason);
            return;
        }
        session_id_ = mailbox.attr("authorized_session").cast<std::string>();
        if (collection_state() == "paused") {
            const auto response = request("resume");
            if (!response.accepted) {
                pause(response.message);
                return;
            }
        }
        if (collection_state() != "recording") {
            pause("Collector is not ready for the transition");
            return;
        }
        recovery_ = kind;
        set_freeze(false);
        set_stage("blending", "1 second live-target transition; r/s/d ignored; recovery samples labelled");
    }

    py::object collection_;
    py::object executor_;
    py::object clock_ns_;
    py::object monotonic_ns_;
    std::string stage_ = "idle";
    std::string notice_ = "Upstream r calibrates / s follows; local r starts an episode";
    std::optional<SaveContext> save_confirmation_;
    std::string session_id_;
    int recovery_ = 0;
    std::int64_t rewind_deadline_ns_ = 0;
};

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
        if (app.attr("stop").cast<bool>()) {
            break;
        }
        app.attr("executor").attr("spin_once")();
        app.attr("three_key").cast<ThreeKeyControl &>().poll();
        app.attr("_maybe_rotate_task")();
        if (app.attr("_task_change_pending")().cast<bool>()) {
            app.attr("_discard_scene_actions")();
        } else {
            app.attr("_process_actions")();
        }
        if (app.attr("stop").cast<bool>()) {
            break;
        }

        // Scene rotation may replace all three native objects and the window.
        const py::object collection = app.attr("collection");
        const py::object executor = app.attr("executor");
        const py::object control = app.attr("three_key");
        collection.attr("poll")();
        const auto now = monotonic_now_ns();
        if (!collection.attr("physics_paused").cast<bool>() &&
            !app.attr("_task_change_pending")().cast<bool>()) {
            const py::object applied = executor.attr("apply_pending")(py::arg("now_ns") = now);
            if (!applied.is_none()) {
                app.attr("_last_applied") = applied;
            }
            // Revalidate at the integration boundary: revocation or held arms
            // must not advance physics or append even a single uncontrolled sample.
            if (!executor.attr("mailbox").attr("enabled").cast<bool>() ||
                (executor.attr("hold_mask").cast<unsigned>() & 1U)) {
                control.cast<ThreeKeyControl &>().poll();
                continue;
            }
            const py::object step = app.attr("plant").attr("physics_tick")();
            collection.attr("tick")(step, py::arg("recovery") = control.cast<ThreeKeyControl &>().recovery());
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
    py::class_<ThreeKeyControl>(module, "ThreeKeyControl")
        .def(py::init<py::object, py::object, py::object, py::object>(),
             py::arg("collection"), py::arg("executor"), py::kw_only(),
             py::arg("clock_ns") = py::none(), py::arg("monotonic_ns") = py::none())
        .def_property_readonly("stage", &ThreeKeyControl::stage)
        .def_property("freeze", &ThreeKeyControl::freeze, &ThreeKeyControl::set_freeze)
        .def_property("notice", &ThreeKeyControl::notice, &ThreeKeyControl::set_notice)
        .def_property_readonly("recovery", &ThreeKeyControl::recovery)
        .def("key", &ThreeKeyControl::key, py::arg("key"))
        .def("poll", &ThreeKeyControl::poll)
        .def("cancel_confirmation", &ThreeKeyControl::cancel_confirmation)
        .def("close", &ThreeKeyControl::close);
    module.def("run_loop", &run_loop, py::arg("app"));
}
