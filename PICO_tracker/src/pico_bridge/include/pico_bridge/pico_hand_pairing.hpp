// include/pico_bridge/pico_hand_pairing.hpp
#pragma once

#include <cstdint>
#include <optional>

#include "pico_bridge/pico_frame.hpp"

namespace pico_bridge {

enum class HandSide { Left, Right };

struct PairedHands {
    int64_t timestamp_ms = 0;
    uint64_t tracking_epoch = 0;
    HandPayload left;
    HandPayload right;
};

// Accumulates one packet per side and emits at most one pair for each
// (tracking_epoch, timestamp_ms). A timestamp advance invalidates the
// incomplete pair; an older packet can never join a newer one.
class HandPairAccumulator {
public:
    void reset(uint64_t tracking_epoch) {
        tracking_epoch_ = tracking_epoch;
        left_.reset();
        right_.reset();
        timestamp_ms_.reset();
        emitted_ = false;
    }

    uint64_t tracking_epoch() const { return tracking_epoch_; }

    std::optional<PairedHands> accept(HandSide side, uint64_t tracking_epoch,
                                      int64_t timestamp_ms,
                                      const HandPayload& payload) {
        if (tracking_epoch != tracking_epoch_) reset(tracking_epoch);

        if (timestamp_ms_.has_value() && timestamp_ms < *timestamp_ms_) {
            return std::nullopt;
        }
        if (!timestamp_ms_.has_value() || timestamp_ms > *timestamp_ms_) {
            timestamp_ms_ = timestamp_ms;
            left_.reset();
            right_.reset();
            emitted_ = false;
        }

        (side == HandSide::Left ? left_ : right_) = payload;
        if (!left_.has_value() || !right_.has_value() || emitted_) {
            return std::nullopt;
        }

        emitted_ = true;
        return PairedHands{*timestamp_ms_, tracking_epoch_, *left_, *right_};
    }

private:
    uint64_t tracking_epoch_ = 0;
    std::optional<int64_t> timestamp_ms_;
    std::optional<HandPayload> left_;
    std::optional<HandPayload> right_;
    bool emitted_ = false;
};

}  // namespace pico_bridge
