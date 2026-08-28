#include "pico_bridge/xrobotoolkit_hand_decoder.hpp"

#include <cmath>
#include <cstdlib>
#include <sstream>
#include <string>

#include <nlohmann/json.hpp>

namespace pico_bridge {
namespace {

using json = nlohmann::json;

XrJointPose identity_pose() { return {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0}; }

XrHandSideSnapshot rejected_side(const char* reason) {
    XrHandSideSnapshot side;
    side.rejection_reason = reason;
    return side;
}

bool parse_pose(const std::string& text, XrJointPose& pose) {
    std::istringstream stream(text);
    for (double& value : pose) {
        std::string token;
        if (!(stream >> token)) {
            return false;
        }
        char* end = nullptr;
        const double parsed = std::strtod(token.c_str(), &end);
        if (end != token.c_str() + token.size() || !std::isfinite(parsed)) {
            return false;
        }
        value = parsed;
    }

    std::string extra;
    return !(stream >> extra);
}

XrHandSideSnapshot decode_side(const json& hand, const char* side_name) {
    if (!hand.contains(side_name) || !hand.at(side_name).is_object()) {
        return rejected_side("hand_missing");
    }
    const json& side_json = hand.at(side_name);

    if (!side_json.contains("scale") || !side_json.at("scale").is_number()) {
        return rejected_side("hand_scale_invalid");
    }
    const double scale = side_json.at("scale").get<double>();
    if (!std::isfinite(scale) || scale <= 0.0) {
        return rejected_side("hand_scale_invalid");
    }
    if (!side_json.contains("isActive") || !side_json.at("isActive").is_boolean()) {
        return rejected_side("hand_missing");
    }
    if (!side_json.contains("HandJointLocations") ||
        !side_json.at("HandJointLocations").is_array() ||
        side_json.at("HandJointLocations").size() != XR_HAND_JOINT_COUNT) {
        return rejected_side("hand_joint_count_invalid");
    }

    XrHandSideSnapshot decoded;
    decoded.scale = scale;
    decoded.active = side_json.at("isActive").get<bool>();
    const auto& joints = side_json.at("HandJointLocations");
    for (std::size_t index = 0; index < XR_HAND_JOINT_COUNT; ++index) {
        if (!joints.at(index).is_object() || !joints.at(index).contains("p") ||
            !joints.at(index).at("p").is_string() ||
            !parse_pose(joints.at(index).at("p").get<std::string>(), decoded.joints[index])) {
            return rejected_side("hand_pose_non_finite");
        }

        const double quaternion_norm = std::hypot(
            std::hypot(decoded.joints[index][3], decoded.joints[index][4]),
            std::hypot(decoded.joints[index][5], decoded.joints[index][6]));
        if (!std::isfinite(quaternion_norm) || quaternion_norm <= 0.0) {
            return rejected_side("hand_quaternion_invalid");
        }
        for (std::size_t component = 3; component < decoded.joints[index].size(); ++component) {
            decoded.joints[index][component] /= quaternion_norm;
        }
    }

    decoded.structurally_valid = true;
    return decoded;
}

}  // namespace

XrHandSideSnapshot::XrHandSideSnapshot()
    : joints{}, active(false), scale(1.0), structurally_valid(false), rejection_reason() {
    joints.fill(identity_pose());
}

XrHandSnapshot::XrHandSnapshot() : source_timestamp_ns(0), left(), right() {}

XrHandDecodeResult decode_xrobotoolkit_state_json(std::string_view json_text) {
    XrHandDecodeResult result{false, "", XrHandSnapshot{}};

    json outer;
    try {
        outer = json::parse(json_text.begin(), json_text.end());
    } catch (const json::exception&) {
        result.rejection_reason = "outer_json_malformed";
        return result;
    }
    if (!outer.is_object() || !outer.contains("value") || !outer.at("value").is_string()) {
        result.rejection_reason = "nested_value_invalid";
        return result;
    }

    json nested;
    try {
        const auto& nested_text = outer.at("value").get_ref<const std::string&>();
        nested = json::parse(nested_text);
    } catch (const json::exception&) {
        result.rejection_reason = "nested_value_invalid";
        return result;
    }

    if (!nested.is_object() || !nested.contains("timeStampNs") ||
        !nested.at("timeStampNs").is_number_integer()) {
        result.rejection_reason = "timestamp_invalid";
        return result;
    }
    try {
        result.snapshot.source_timestamp_ns = nested.at("timeStampNs").get<std::int64_t>();
    } catch (const json::exception&) {
        result.rejection_reason = "timestamp_invalid";
        return result;
    }
    if (result.snapshot.source_timestamp_ns <= 0) {
        result.rejection_reason = "timestamp_invalid";
        return result;
    }

    if (!nested.contains("Hand") || !nested.at("Hand").is_object()) {
        result.rejection_reason = "hand_missing";
        return result;
    }
    const auto& hand = nested.at("Hand");
    if (!hand.contains("leftHand") && !hand.contains("rightHand")) {
        result.rejection_reason = "hand_missing";
        return result;
    }

    result.valid = true;
    result.snapshot.left = decode_side(hand, "leftHand");
    result.snapshot.right = decode_side(hand, "rightHand");
    return result;
}

}  // namespace pico_bridge
