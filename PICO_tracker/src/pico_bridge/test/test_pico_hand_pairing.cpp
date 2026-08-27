#include <array>
#include <cstring>
#include <limits>
#include <gtest/gtest.h>

#include "pico_bridge/pico_hand_pairing.hpp"

namespace {

pico_bridge::HandPayload sample(bool active, float marker) {
    pico_bridge::HandPayload hand;
    hand.active = active;
    hand.scale = 1.25F;
    for (size_t joint = 0; joint < pico_bridge::HAND_JOINT_COUNT; ++joint) {
        hand.joints[joint] = {
            marker + static_cast<float>(joint),
            marker + 0.1F,
            marker + 0.2F,
            0.0F,
            0.0F,
            0.0F,
            1.0F,
        };
    }
    return hand;
}

std::array<uint8_t, pico_bridge::HAND_PAYLOAD_BYTES> wire_sample() {
    std::array<uint8_t, pico_bridge::HAND_PAYLOAD_BYTES> wire{};
    wire[0] = 1;
    const float scale = 1.25F;
    std::memcpy(wire.data() + 1, &scale, sizeof(scale));
    for (size_t joint = 0; joint < pico_bridge::HAND_JOINT_COUNT; ++joint) {
        const float values[7] = {
            static_cast<float>(joint), static_cast<float>(joint) + 0.1F,
            static_cast<float>(joint) + 0.2F, 0.0F, 0.0F, 0.0F, 1.0F,
        };
        std::memcpy(wire.data() + 1 + sizeof(float) + joint * pico_bridge::POSE_BYTES,
                    values, sizeof(values));
    }
    return wire;
}

}  // namespace

TEST(PicoHandFrame, GoldenPayloadIsExactly733Bytes) {
    EXPECT_EQ(pico_bridge::TYPE_HAND_LEFT, 0x38);
    EXPECT_EQ(pico_bridge::TYPE_HAND_RIGHT, 0x39);
    EXPECT_EQ(pico_bridge::HAND_PAYLOAD_BYTES, 733u);

    const auto wire = wire_sample();
    pico_bridge::HandPayload decoded;
    ASSERT_TRUE(pico_bridge::parse_hand_payload(wire.data(), wire.size(), decoded));
    EXPECT_TRUE(decoded.active);
    EXPECT_FLOAT_EQ(decoded.scale, 1.25F);
    EXPECT_FLOAT_EQ(decoded.joints[0][0], 0.0F);
    EXPECT_FLOAT_EQ(decoded.joints[25][0], 25.0F);
    EXPECT_FLOAT_EQ(decoded.joints[25][6], 1.0F);
}

TEST(PicoHandFrame, RejectsTruncatedAndMalformedPayloads) {
    const auto wire = wire_sample();
    pico_bridge::HandPayload decoded;
    EXPECT_FALSE(pico_bridge::parse_hand_payload(wire.data(), wire.size() - 1, decoded));
    EXPECT_FALSE(pico_bridge::parse_hand_payload(wire.data(), wire.size() + 1, decoded));

    auto bad_active = wire;
    bad_active[0] = 2;
    EXPECT_FALSE(pico_bridge::parse_hand_payload(bad_active.data(), bad_active.size(), decoded));

    auto bad_scale = wire;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    std::memcpy(bad_scale.data() + 1, &nan, sizeof(nan));
    EXPECT_FALSE(pico_bridge::parse_hand_payload(bad_scale.data(), bad_scale.size(), decoded));
}

TEST(PicoHandPairing, EmitsOnePairForEqualTimestampAndEpoch) {
    pico_bridge::HandPairAccumulator accumulator;
    accumulator.reset(7);
    EXPECT_FALSE(accumulator.accept(pico_bridge::HandSide::Left, 7, 100,
                                    sample(true, 1.0F)).has_value());
    const auto pair = accumulator.accept(pico_bridge::HandSide::Right, 7, 100,
                                          sample(true, 2.0F));
    ASSERT_TRUE(pair.has_value());
    EXPECT_EQ(pair->timestamp_ms, 100);
    EXPECT_EQ(pair->tracking_epoch, 7u);
    EXPECT_FLOAT_EQ(pair->left.joints[0][0], 1.0F);
    EXPECT_FLOAT_EQ(pair->right.joints[0][0], 2.0F);
    EXPECT_FALSE(accumulator.accept(pico_bridge::HandSide::Right, 7, 100,
                                    sample(true, 3.0F)).has_value());
}

TEST(PicoHandPairing, TimestampAdvanceDropsUnpairedSide) {
    pico_bridge::HandPairAccumulator accumulator;
    accumulator.reset(3);
    accumulator.accept(pico_bridge::HandSide::Left, 3, 100, sample(true, 1.0F));
    accumulator.accept(pico_bridge::HandSide::Right, 3, 101, sample(true, 2.0F));
    EXPECT_FALSE(accumulator.accept(pico_bridge::HandSide::Left, 3, 100,
                                    sample(true, 4.0F)).has_value());
    const auto pair = accumulator.accept(pico_bridge::HandSide::Left, 3, 101,
                                         sample(false, 5.0F));
    ASSERT_TRUE(pair.has_value());
    EXPECT_FALSE(pair->left.active);
    EXPECT_TRUE(pair->right.active);
}

TEST(PicoHandPairing, EpochChangeClearsBothSides) {
    pico_bridge::HandPairAccumulator accumulator;
    accumulator.reset(1);
    accumulator.accept(pico_bridge::HandSide::Left, 1, 10, sample(true, 1.0F));
    accumulator.accept(pico_bridge::HandSide::Right, 2, 10, sample(true, 2.0F));
    EXPECT_FALSE(accumulator.accept(pico_bridge::HandSide::Right, 2, 10,
                                    sample(true, 3.0F)).has_value());
    const auto pair = accumulator.accept(pico_bridge::HandSide::Left, 2, 10,
                                         sample(true, 4.0F));
    ASSERT_TRUE(pair.has_value());
    EXPECT_EQ(pair->tracking_epoch, 2u);
    EXPECT_FLOAT_EQ(pair->right.joints[0][0], 3.0F);
}
