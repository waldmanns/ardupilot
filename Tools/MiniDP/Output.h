#pragma once

#include <stdint.h>

enum class MiniDP_OutputSafeAction : uint8_t {
    DISABLE_PWM = 0,
    SEND_NEUTRAL,
    SEND_FAILSAFE,
};

enum class MiniDP_OutputState : uint8_t {
    DISARMED = 0,
    ARMED_NEUTRAL,
    ARMED_ACTIVE,
    ACTUATOR_TEST,
    FAILSAFE,
    KILL,
};

enum class MiniDP_FrameType : int16_t {
    OMNI_PLUS = 901,
};

enum class MiniDP_ScrewPosition : int8_t {
    AFT = -1,
    CENTER = 0,
    FORWARD = 1,
};

struct MiniDP_FrameGeometryConfig {
    MiniDP_ScrewPosition screw_position;
    float screw_yaw_scale;
};

struct MiniDP_AxisCommand {
    float surge;
    float sway;
    float yaw;
};

struct MiniDP_ActuatorConfig {
    bool enabled;
    uint8_t pwm_channel;

    uint16_t pwm_min;
    uint16_t pwm_trim;
    uint16_t pwm_max;
    uint16_t pwm_failsafe;

    bool reversed;

    float forward_scale;
    float reverse_scale;
    float deadband;
    float min_effective_output;

    float k_surge;
    float k_sway;
    float k_yaw;

    MiniDP_OutputSafeAction disarmed_action;
    MiniDP_OutputSafeAction failsafe_action;
    MiniDP_OutputSafeAction kill_action;
};

struct MiniDP_ActuatorOutput {
    bool configured;
    bool pwm_enabled;
    uint8_t pwm_channel;
    uint16_t pwm_us;
    float demand;
    bool saturated;
};

struct MiniDP_OutputFrame {
    MiniDP_OutputState state;
    MiniDP_ActuatorOutput actuator[6];
    bool saturated;
    uint8_t active_pwm_count;
};

struct MiniDP_ActuatorTestCommand {
    bool active;
    uint8_t actuator_index;
    bool use_pwm;
    uint16_t pwm_us;
    float demand;
};

class MiniDP_OutputManager {
public:
    static constexpr uint8_t max_actuators = 6;
    static constexpr int16_t default_frame_type =
        int16_t(MiniDP_FrameType::OMNI_PLUS);

    void init(int16_t frame_type = default_frame_type);
    bool set_frame_type(int16_t frame_type);
    void set_frame_geometry(const MiniDP_FrameGeometryConfig &new_config);

    const MiniDP_OutputFrame &update(
        MiniDP_OutputState state,
        const MiniDP_AxisCommand &command);
    const MiniDP_OutputFrame &update_actuator_test(
        const MiniDP_ActuatorTestCommand &command);

    bool set_actuator_config(
        uint8_t index,
        const MiniDP_ActuatorConfig &config);

    const MiniDP_ActuatorConfig &actuator_config(uint8_t index) const;
    const MiniDP_OutputFrame &frame() const { return output_frame; }
    int16_t frame_type() const { return configured_frame_type; }

    static MiniDP_ActuatorConfig default_actuator_config(uint8_t index);
    static MiniDP_FrameGeometryConfig default_frame_geometry_config();
    static const char *frame_type_name(int16_t frame_type);
    static const char *screw_position_name(MiniDP_ScrewPosition position);
    static const char *motor_name(uint8_t index);
    static const char *state_name(MiniDP_OutputState state);
    static const char *safe_action_name(MiniDP_OutputSafeAction action);

private:
    MiniDP_ActuatorConfig configs[max_actuators];
    MiniDP_OutputFrame output_frame{};
    MiniDP_FrameGeometryConfig frame_geometry{};
    int16_t configured_frame_type = default_frame_type;

    void reset_frame_configs();
    void configure_motor(
        uint8_t index,
        float k_surge,
        float k_sway,
        float k_yaw);
    void configure_omni_plus_frame();
    void apply_omni_plus_geometry();
    void apply_safe_outputs(MiniDP_OutputState state);
    void apply_active_outputs(MiniDP_OutputState state, const MiniDP_AxisCommand &command);
    void apply_actuator_test_output(const MiniDP_ActuatorTestCommand &command);
    void set_disabled_output(uint8_t index);
    void set_safe_output(uint8_t index, MiniDP_OutputSafeAction action);
    void set_pwm_output(uint8_t index, uint16_t pwm_us, float demand, bool saturated);
    uint16_t constrain_pwm_to_actuator(const MiniDP_ActuatorConfig &config, uint16_t pwm_us) const;
    uint16_t pwm_from_demand(const MiniDP_ActuatorConfig &config, float demand) const;
};
