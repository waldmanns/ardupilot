#include "Vektor_Rcin.h"

#include <math.h>

namespace {

static constexpr uint16_t pwm_valid_min = 900;
static constexpr uint16_t pwm_valid_max = 2100;
static constexpr uint16_t pwm_min = 1000;
static constexpr uint16_t pwm_trim = 1500;
static constexpr uint16_t pwm_max = 2000;
static constexpr uint16_t pwm_deadzone = 20;

float constrain_normalized(float value)
{
    if (value < -1.0f) {
        return -1.0f;
    }
    if (value > 1.0f) {
        return 1.0f;
    }
    return value;
}

} // namespace

namespace Vektor {

void RcinSource::reset(uint32_t timeout_us)
{
    _timeout_us = timeout_us == 0 ? 1 : timeout_us;
    _frame_count = 0;
    _last_frame_us = 0;
    _channel_count = 0;
    _have_frame = false;
    _failsafe = false;
    _healthy = false;
    for (uint8_t i = 0; i < max_channels; i++) {
        _channels[i] = {};
        _pwm[i] = 0;
        _value_valid[i] = false;
    }
}

void RcinSource::set_timeout_us(uint32_t timeout_us)
{
    _timeout_us = timeout_us == 0 ? 1 : timeout_us;
}

void RcinSource::ingest_normalized(const float *values,
                                   const uint16_t *pwm,
                                   uint8_t channel_count,
                                   bool failsafe,
                                   uint64_t timestamp_us)
{
    _channel_count = channel_count > max_channels ? max_channels : channel_count;
    _frame_count++;
    _last_frame_us = timestamp_us;
    _have_frame = true;
    _failsafe = failsafe;

    bool any_valid = false;
    for (uint8_t i = 0; i < max_channels; i++) {
        _pwm[i] = (i < _channel_count && pwm != nullptr) ? pwm[i] : 0;
        _value_valid[i] = i < _channel_count &&
                          values != nullptr &&
                          isfinite(values[i]);
        if (_value_valid[i]) {
            _channels[i].value = constrain_normalized(values[i]);
            _channels[i].timestamp_us = timestamp_us;
            _channels[i].quality = failsafe ? SignalQuality::INVALID :
                                              SignalQuality::VALID;
            any_valid = true;
        } else {
            _channels[i] = {};
        }
    }
    _healthy = !failsafe && _channel_count != 0 && any_valid;
}

void RcinSource::ingest_pwm(const uint16_t *pwm,
                            uint8_t channel_count,
                            bool failsafe,
                            uint64_t timestamp_us)
{
    float normalized[max_channels] {};
    bool valid[max_channels] {};
    const uint8_t count = channel_count > max_channels ? max_channels :
                                                       channel_count;
    for (uint8_t i = 0; i < count; i++) {
        valid[i] = pwm != nullptr &&
                   pwm[i] >= pwm_valid_min &&
                   pwm[i] <= pwm_valid_max;
        normalized[i] = valid[i] ? normalize_pwm(pwm[i]) : NAN;
    }
    ingest_normalized(normalized, pwm, count, failsafe, timestamp_us);
}

void RcinSource::update(uint64_t now_us)
{
    if (!_have_frame || _last_frame_us > now_us) {
        set_all_quality(SignalQuality::INVALID);
        _healthy = false;
        return;
    }

    if (_failsafe) {
        set_all_quality(SignalQuality::INVALID);
        _healthy = false;
        return;
    }

    if (now_us - _last_frame_us > _timeout_us) {
        set_all_quality(SignalQuality::STALE);
        _healthy = false;
        return;
    }

    set_all_quality(SignalQuality::VALID);
    _healthy = _channel_count != 0;
    if (_healthy) {
        bool any_valid = false;
        for (uint8_t i = 0; i < _channel_count; i++) {
            any_valid |= _value_valid[i];
        }
        _healthy = any_valid;
    }
}

const SignalSample<float> *RcinSource::channel(uint8_t index) const
{
    return index < max_channels ? &_channels[index] : nullptr;
}

uint16_t RcinSource::raw_pwm(uint8_t index) const
{
    return index < max_channels ? _pwm[index] : 0;
}

float RcinSource::normalize_pwm(uint16_t pwm)
{
    if (pwm > pwm_trim + pwm_deadzone) {
        return constrain_normalized(
            float(pwm - (pwm_trim + pwm_deadzone)) /
            float(pwm_max - (pwm_trim + pwm_deadzone)));
    }
    if (pwm < pwm_trim - pwm_deadzone) {
        return constrain_normalized(
            -float((pwm_trim - pwm_deadzone) - pwm) /
            float((pwm_trim - pwm_deadzone) - pwm_min));
    }
    return 0.0f;
}

void RcinSource::set_all_quality(SignalQuality quality)
{
    for (uint8_t i = 0; i < max_channels; i++) {
        _channels[i].quality = _value_valid[i] ? quality :
                                                 SignalQuality::INVALID;
    }
}

} // namespace Vektor
