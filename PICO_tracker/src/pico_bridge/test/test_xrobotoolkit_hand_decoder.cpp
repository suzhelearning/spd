#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include "pico_bridge/xrobotoolkit_hand_decoder.hpp"

namespace {

using json = nlohmann::json;

std::string readFixture() {
    const auto path = std::filesystem::path(__FILE__).parent_path() /
                      "fixtures/xrobotoolkit_hand_state.json";
    std::ifstream input(path);
    EXPECT_TRUE(input.is_open()) << path;
    std::ostringstream contents;
    contents << input.rdbuf();
    return contents.str();
}

json readOuter() { return json::parse(readFixture()); }
json readNested(json& outer) { return json::parse(outer.at("value").get<std::string>()); }

std::string encode(json& outer, const json& nested) {
    outer["value"] = nested.dump();
    return outer.dump();
}

TEST(XroboToolkitHandDecoder, DecodesAtomicCallbackFixture) {
    const auto result = pico_bridge::decode_xrobotoolkit_state_json(readFixture());

    ASSERT_TRUE(result.valid);
    EXPECT_EQ(result.snapshot.source_timestamp_ns, 1724846400000000000LL);
    EXPECT_DOUBLE_EQ(result.snapshot.left.joints[1][0], 0.101);

    EXPECT_DOUBLE_EQ(result.snapshot.right.scale, 1.02);
    EXPECT_EQ(result.snapshot.right.joints[1][6], 1.0);
    EXPECT_TRUE(result.snapshot.left.active);
    EXPECT_FALSE(result.snapshot.right.active);
    EXPECT_DOUBLE_EQ(result.snapshot.left.scale, 0.98);
    EXPECT_TRUE(result.snapshot.left.structurally_valid);
    EXPECT_TRUE(result.snapshot.right.structurally_valid);
}

TEST(XroboToolkitHandDecoder, InvalidLeftQuaternionDoesNotDiscardRightSide) {
    auto outer = readOuter();
    auto nested = readNested(outer);
    nested["Hand"]["leftHand"]["HandJointLocations"][0]["p"] = "0 0 0 0 0 0 0";

    const auto result = pico_bridge::decode_xrobotoolkit_state_json(encode(outer, nested));

    ASSERT_TRUE(result.valid);
    EXPECT_FALSE(result.snapshot.left.structurally_valid);
    EXPECT_FALSE(result.snapshot.left.active);
    EXPECT_EQ(result.snapshot.left.rejection_reason, "hand_quaternion_invalid");
    EXPECT_TRUE(result.snapshot.right.structurally_valid);
    EXPECT_DOUBLE_EQ(result.snapshot.right.joints[1][0], 1.101);
    EXPECT_DOUBLE_EQ(result.snapshot.right.joints[1][6], 1.0);
}

TEST(XroboToolkitHandDecoder, MissingSideUsesInactiveIdentitySnapshot) {
    auto outer = readOuter();
    auto nested = readNested(outer);
    nested["Hand"].erase("rightHand");

    const auto result = pico_bridge::decode_xrobotoolkit_state_json(encode(outer, nested));

    ASSERT_TRUE(result.valid);
    EXPECT_TRUE(result.snapshot.left.structurally_valid);
    EXPECT_FALSE(result.snapshot.right.structurally_valid);
    EXPECT_FALSE(result.snapshot.right.active);
    EXPECT_DOUBLE_EQ(result.snapshot.right.scale, 1.0);
    EXPECT_EQ(result.snapshot.right.rejection_reason, "hand_missing");
    EXPECT_DOUBLE_EQ(result.snapshot.right.joints[0][6], 1.0);
}

TEST(XroboToolkitHandDecoder, RejectsMalformedOuterJson) {
    const auto result = pico_bridge::decode_xrobotoolkit_state_json("{");

    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.rejection_reason, "outer_json_malformed");
}

TEST(XroboToolkitHandDecoder, RejectsMalformedNestedJson) {
    json outer = {{"value", "not json"}};

    const auto result = pico_bridge::decode_xrobotoolkit_state_json(outer.dump());

    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.rejection_reason, "nested_value_invalid");
}

TEST(XroboToolkitHandDecoder, RejectsInvalidTimestamp) {
    auto outer = readOuter();
    auto nested = readNested(outer);
    nested["timeStampNs"] = 0;

    const auto result = pico_bridge::decode_xrobotoolkit_state_json(encode(outer, nested));

    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.rejection_reason, "timestamp_invalid");
}

TEST(XroboToolkitHandDecoder, RejectsMissingHand) {
    auto outer = readOuter();
    auto nested = readNested(outer);
    nested.erase("Hand");

    const auto result = pico_bridge::decode_xrobotoolkit_state_json(encode(outer, nested));

    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.rejection_reason, "hand_missing");
}

TEST(XroboToolkitHandDecoder, RejectsWrongJointCountPerSide) {
    auto outer = readOuter();
    auto nested = readNested(outer);
    nested["Hand"]["leftHand"]["HandJointLocations"].erase(0);

    const auto result = pico_bridge::decode_xrobotoolkit_state_json(encode(outer, nested));

    ASSERT_TRUE(result.valid);
    EXPECT_FALSE(result.snapshot.left.structurally_valid);
    EXPECT_EQ(result.snapshot.left.rejection_reason, "hand_joint_count_invalid");
    EXPECT_TRUE(result.snapshot.right.structurally_valid);
}

TEST(XroboToolkitHandDecoder, RejectsNonFinitePose) {
    auto outer = readOuter();
    auto nested = readNested(outer);
    nested["Hand"]["leftHand"]["HandJointLocations"][1]["p"] =
        "nan 0 0 0 0 0 1";

    const auto result = pico_bridge::decode_xrobotoolkit_state_json(encode(outer, nested));

    ASSERT_TRUE(result.valid);
    EXPECT_FALSE(result.snapshot.left.structurally_valid);
    EXPECT_EQ(result.snapshot.left.rejection_reason, "hand_pose_non_finite");
}

TEST(XroboToolkitHandDecoder, RejectsInvalidScale) {
    auto outer = readOuter();
    auto nested = readNested(outer);
    nested["Hand"]["leftHand"]["scale"] = 0.0;

    const auto result = pico_bridge::decode_xrobotoolkit_state_json(encode(outer, nested));

    ASSERT_TRUE(result.valid);
    EXPECT_FALSE(result.snapshot.left.structurally_valid);
    EXPECT_EQ(result.snapshot.left.rejection_reason, "hand_scale_invalid");
}

TEST(XroboToolkitHandDecoder, NormalizesQuaternionsBeforeStoring) {
    auto outer = readOuter();
    auto nested = readNested(outer);
    nested["Hand"]["leftHand"]["HandJointLocations"][1]["p"] =
        "0.101 0.201 0.301 0 0 0 2";

    const auto result = pico_bridge::decode_xrobotoolkit_state_json(encode(outer, nested));

    ASSERT_TRUE(result.valid);
    ASSERT_TRUE(result.snapshot.left.structurally_valid);
    EXPECT_DOUBLE_EQ(result.snapshot.left.joints[1][6], 1.0);
}

}  // namespace
