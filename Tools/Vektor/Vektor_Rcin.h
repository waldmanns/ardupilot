#pragma once

#include "Vektor_Signal.h"

#include <stdint.h>

namespace Vektor {

class RcinSource {
public:
    static constexpr uint8_t max_channels = 16;
    static constexpr uint32_t default_timeout_us = 500000;

    void reset(uint32_t timeout_us = default_timeout_us);
    void set_timeout_us(uint32_t timeout_us);

    void ingest_normalized(const float *values,
                           const uint16_t *pwm,
                           uint8_t channel_count,
                           bool failsafe,
                           uint64_t timestamp_us);
    void ingest_pwm(const uint16_t *pwm,
                    uint8_t channel_count,
                    bool failsafe,
                    uint64_t timestamp_us);
    void update(uint64_t now_us);

    const SignalSample<float> *channel(uint8_t index) const;
    uint16_t raw_pwm(uint8_t index) const;
    uint8_t channel_count() const { return _channel_count; }
    uint32_t frame_count() const { return _frame_count; }
    uint64_t last_frame_us() const { return _last_frame_us; }
    bool healthy() const { return _healthy; }
    bool failsafe() const { return _failsafe; }

private:
    static float normalize_pwm(uint16_t pwm);
    void set_all_quality(SignalQuality quality);

    SignalSample<float> _channels[max_channels] {};
    uint16_t _pwm[max_channels] {};
    bool _value_valid[max_channels] {};
    uint32_t _timeout_us = default_timeout_us;
    uint32_t _frame_count = 0;
    uint64_t _last_frame_us = 0;
    uint8_t _channel_count = 0;
    bool _have_frame = false;
    bool _failsafe = false;
    bool _healthy = false;
};

} // namespace Vektor
