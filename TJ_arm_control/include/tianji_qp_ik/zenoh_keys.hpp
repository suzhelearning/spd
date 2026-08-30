#pragma once

#include <string_view>

namespace tianji_qp_ik {

inline constexpr std::string_view kTrackingKey{"spd/vr/v1/tracking"};
inline constexpr std::string_view kArmTargetsKey{"spd/vr/v1/arm_targets"};
inline constexpr std::string_view kControlKey{"spd/vr/v1/control"};
inline constexpr std::string_view kBridgeStatusKey{"spd/vr/v1/status/bridge"};
inline constexpr std::string_view kIkStatusKey{"spd/vr/v1/status/ik"};
inline constexpr std::string_view kViewerStatusKey{"spd/vr/v1/status/viewer"};

}  // namespace tianji_qp_ik
