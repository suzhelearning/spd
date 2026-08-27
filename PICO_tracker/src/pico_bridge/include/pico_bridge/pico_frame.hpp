// include/pico_bridge/pico_frame.hpp
#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cstddef>

namespace pico_bridge {

constexpr int HEADER_SIZE = 1 + 1 + 8 + 4;         // 14 bytes
constexpr uint8_t FRAME_MAGIC = 0xAB;
constexpr uint32_t MAX_PAYLOAD_BYTES = 64 * 1024 * 1024;

// Frame type IDs — mirror receiver.py / PICO_Streaming_Guide.md.
constexpr uint8_t TYPE_CAM_LEFT    = 0x01;
constexpr uint8_t TYPE_CAM_RIGHT   = 0x02;
constexpr uint8_t TYPE_POSE_LEFT   = 0x03;
constexpr uint8_t TYPE_POSE_RIGHT  = 0x04;
constexpr uint8_t TYPE_POSE_HEAD   = 0x05;
constexpr uint8_t TYPE_WORLD_RESET = 0x06;  // 4B float yaw, published as /pico/world_reset
constexpr uint8_t TYPE_CTRL_LEFT   = 0x07;  // controller button state, payload undocumented
constexpr uint8_t TYPE_CTRL_RIGHT  = 0x08;  // controller button state, payload undocumented
constexpr uint8_t TYPE_RECORD_FLAG = 0x09;  // 1B, non-zero = start recording
constexpr uint8_t TYPE_BLE_LEFT    = 0x10;
constexpr uint8_t TYPE_BLE_RIGHT   = 0x11;

// Full PICO BodyTrackerRole stream. Frame type is 0x20 + enum index and each
// payload is pos.xyz + quat.xyzw (7 little-endian float32 values). The order is
// the protocol contract in pico_stream_record_receiver.py.
constexpr uint8_t TYPE_BODY_BASE = 0x20;
constexpr uint8_t TYPE_BODY_LAST = 0x37;
constexpr size_t BODY_JOINT_COUNT = 24;
constexpr std::array<const char*, BODY_JOINT_COUNT> BODY_JOINT_NAMES = {
    "Pelvis",
    "LEFT_HIP", "RIGHT_HIP", "SPINE1",
    "LEFT_KNEE", "RIGHT_KNEE", "SPINE2",
    "LEFT_ANKLE", "RIGHT_ANKLE", "SPINE3",
    "LEFT_FOOT", "RIGHT_FOOT", "NECK",
    "LEFT_COLLAR", "RIGHT_COLLAR", "HEAD",
    "LEFT_SHOULDER", "RIGHT_SHOULDER",
    "LEFT_ELBOW", "RIGHT_ELBOW",
    "LEFT_WRIST", "RIGHT_WRIST", "LEFT_HAND", "RIGHT_HAND",
};

// OpenXR hand stream. The two values are deliberately outside the body range
// so a hand frame can never be mistaken for a SMPL joint.
constexpr uint8_t TYPE_HAND_LEFT = 0x38;
constexpr uint8_t TYPE_HAND_RIGHT = 0x39;
constexpr size_t HAND_JOINT_COUNT = 26;
constexpr size_t POSE_FLOAT_COUNT = 7;
constexpr size_t POSE_BYTES = POSE_FLOAT_COUNT * sizeof(float);
constexpr size_t HAND_PAYLOAD_BYTES = sizeof(uint8_t) + sizeof(float) +
    HAND_JOINT_COUNT * POSE_BYTES;  // 733 bytes

inline bool is_body_pose_type(uint8_t type) {
    return type >= TYPE_BODY_BASE && type <= TYPE_BODY_LAST;
}

inline size_t body_joint_index(uint8_t type) {
    return static_cast<size_t>(type - TYPE_BODY_BASE);
}

inline bool is_hand_type(uint8_t type) {
    return type == TYPE_HAND_LEFT || type == TYPE_HAND_RIGHT;
}

inline uint32_t read_u32_le(const uint8_t* bytes) {
    return static_cast<uint32_t>(bytes[0]) |
           (static_cast<uint32_t>(bytes[1]) << 8U) |
           (static_cast<uint32_t>(bytes[2]) << 16U) |
           (static_cast<uint32_t>(bytes[3]) << 24U);
}

inline uint64_t read_u64_le(const uint8_t* bytes) {
    uint64_t value = 0;
    for (size_t index = 0; index < sizeof(uint64_t); ++index) {
        value |= static_cast<uint64_t>(bytes[index]) << (8U * index);
    }
    return value;
}

inline int64_t read_i64_le(const uint8_t* bytes) {
    const uint64_t bits = read_u64_le(bytes);
    int64_t value = 0;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

inline float read_f32_le(const uint8_t* bytes) {
    const uint32_t bits = read_u32_le(bytes);
    float value = 0.0F;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

struct FrameHeader {
    uint8_t  magic;
    uint8_t  type;
    int64_t  ts_ms;
    uint32_t payload_len;
};

// Parse 14-byte little-endian header. Returns false on bad magic or oversized payload.
inline bool parse_frame_header(const uint8_t* buf, FrameHeader& out) {
    if (buf == nullptr) return false;
    out.magic = buf[0];
    out.type  = buf[1];
    out.ts_ms = read_i64_le(buf + 2);
    out.payload_len = read_u32_le(buf + 10);
    if (out.magic != FRAME_MAGIC) return false;
    if (out.payload_len > MAX_PAYLOAD_BYTES) return false;
    return true;
}

// Decode 7×float32 little-endian pose payload into pos[3] + quat[4] (xyzw).
// Returns false if payload_len < 28.
inline bool parse_pose_payload(const uint8_t* payload, size_t len,
                                float pos[3], float quat_xyzw[4]) {
    if (payload == nullptr || len < POSE_BYTES) return false;
    for (size_t index = 0; index < 3; ++index) pos[index] = read_f32_le(payload + index * sizeof(float));
    for (size_t index = 0; index < 4; ++index) quat_xyzw[index] = read_f32_le(payload + 12 + index * sizeof(float));
    return true;
}

// A parsed OpenXR hand packet. The joint order is the APK's 26-joint OpenXR
// order and each pose is xyz + quaternion xyzw in metres.
struct HandPayload {
    bool active = false;
    float scale = 1.0F;
    std::array<std::array<float, POSE_FLOAT_COUNT>, HAND_JOINT_COUNT> joints{};
};

inline bool parse_hand_payload(const uint8_t* payload, size_t len, HandPayload& out) {
    if (payload == nullptr || len != HAND_PAYLOAD_BYTES) return false;
    if (payload[0] > 1) return false;
    out.active = payload[0] != 0;
    out.scale = read_f32_le(payload + 1);
    if (!std::isfinite(out.scale) || out.scale <= 0.0F) return false;
    for (size_t joint = 0; joint < HAND_JOINT_COUNT; ++joint) {
        for (size_t value = 0; value < POSE_FLOAT_COUNT; ++value) {
            out.joints[joint][value] = read_f32_le(
                payload + 1 + sizeof(float) + joint * POSE_BYTES + value * sizeof(float));
            if (!std::isfinite(out.joints[joint][value])) return false;
        }
    }
    return true;
}

inline bool parse_world_reset_payload(const uint8_t* payload, size_t len, float& yaw) {
    if (payload == nullptr || len < sizeof(float)) return false;
    yaw = read_f32_le(payload);
    return true;
}

// Split a BLE payload [4B esp32_ts | sensor_bytes] → esp32_ts + (data_ptr, data_len).
// Returns false if payload is shorter than 4 bytes.
inline bool split_ble_payload(const uint8_t* payload, size_t len,
                               uint32_t& esp32_ts,
                               const uint8_t*& data_ptr, size_t& data_len) {
    if (payload == nullptr || len < 4) return false;
    esp32_ts = read_u32_le(payload);
    data_ptr = payload + 4;
    data_len = len - 4;
    return true;
}

}  // namespace pico_bridge
