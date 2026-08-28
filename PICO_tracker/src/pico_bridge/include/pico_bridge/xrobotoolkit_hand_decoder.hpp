#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <string_view>

namespace pico_bridge {

constexpr std::size_t XR_HAND_JOINT_COUNT = 26;
using XrJointPose = std::array<double, 7>;

struct XrHandSideSnapshot {
    std::array<XrJointPose, XR_HAND_JOINT_COUNT> joints;
    bool active;
    double scale;
    bool structurally_valid;
    std::string rejection_reason;

    XrHandSideSnapshot();
};

struct XrHandSnapshot {
    std::int64_t source_timestamp_ns;
    XrHandSideSnapshot left;
    XrHandSideSnapshot right;

    XrHandSnapshot();
};

struct XrHandDecodeResult {
    bool valid;
    std::string rejection_reason;
    XrHandSnapshot snapshot;
};

XrHandDecodeResult decode_xrobotoolkit_state_json(std::string_view json_text);

}  // namespace pico_bridge
