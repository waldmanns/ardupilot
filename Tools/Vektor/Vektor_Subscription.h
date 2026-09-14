#pragma once

#include <stdint.h>

namespace Vektor {

class SubscriptionTable {
public:
    static constexpr uint16_t max_subscriptions = 4;
    // Large enough for attitude (Euler, quaternion, body rates), all sixteen
    // RC inputs, and all twelve physical RC/PWM outputs in one sample.
    static constexpr uint16_t max_fields = 40;
    static constexpr uint8_t max_quality_bytes =
        (max_fields * 2U + 7U) / 8U;

    enum class AddResult : uint8_t {
        OK,
        INVALID_FIELD_COUNT,
        FULL,
    };

    struct Entry {
        bool active;
        uint16_t id;
        uint32_t period_us;
        uint64_t next_sample_us;
        uint16_t sample_sequence;
        uint16_t field_count;
        uint32_t field_ids[max_fields];
    };

    void reset(uint32_t scheduler_period_us);
    AddResult add(uint32_t requested_period_us,
                  const uint32_t *field_ids,
                  uint16_t field_count,
                  uint64_t now_us,
                  const Entry *&created);
    bool remove(uint16_t subscription_id);
    Entry *claim_due(uint64_t now_us);
    const Entry *find(uint16_t subscription_id) const;
    uint16_t active_count() const;
    uint32_t scheduler_period_us() const { return _scheduler_period_us; }

private:
    uint16_t allocate_id();
    uint32_t accepted_period(uint32_t requested_period_us) const;

    uint32_t _scheduler_period_us = 1;
    uint16_t _next_id = 1;
    Entry _entries[max_subscriptions] {};
};

} // namespace Vektor
