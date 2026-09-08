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
    DUAL_AZ_180_BOW = 902,
};

enum class MiniDP_ScrewPosition : int8_t {
    AFT = -1,
    CENTER = 0,
    FORWARD = 1,
};

struct MiniDP_FrameGeometryConfig {
    MiniDP_ScrewPosition screw_position;
    float screw_yaw_scale;
    float aft_arm_m = 1.0f;
    float bow_arm_m = 1.0f;
    float screw_half_span_m = 1.0f;
};

struct MiniDP_AxisCommand {
    float surge;
    float sway;
    float yaw;
};

enum class MiniDP_ThrusterType : uint8_t {
    NONE = 0,
    FIXED = 1,
    AZIMUTH_180 = 2,
    VOITH = 3,
};

struct MiniDP_ThrusterConfig {
    bool enabled;
    MiniDP_ThrusterType type;
    uint8_t output1;
    uint8_t output2;
    float k_surge;
    float k_sway;
    float k_yaw;
    float angle_min_rad;
    float angle_max_rad;
    bool allow_reverse_fold;
};

struct MiniDP_AzipodConfig {
    float angle_min_rad;
    float angle_max_rad;
    bool allow_reverse_fold;
    float steering_rate_rad_s = 1.57079632679f;
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

struct MiniDP_ThrusterOutput {
    bool configured;
    MiniDP_ThrusterType type;
    uint16_t flags;
    uint8_t output1;
    uint8_t output2;
    float force_x;
    float force_y;
    float value;
    float value2;
    uint16_t pwm1_us;
    uint16_t pwm2_us;
};

struct MiniDP_OutputFrame {
    MiniDP_OutputState state;
    MiniDP_ActuatorOutput actuator[6];
    MiniDP_ThrusterOutput thruster[4];
    bool saturated;
    uint8_t active_pwm_count;
    uint8_t configured_thruster_count;
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
    static constexpr uint8_t max_thrusters = 4;
    static constexpr int16_t default_frame_type =
        int16_t(MiniDP_FrameType::OMNI_PLUS);

    static constexpr uint16_t thruster_flag_active = 1U << 0;
    static constexpr uint16_t thruster_flag_saturated = 1U << 1;
    static constexpr uint16_t thruster_flag_angle_limited = 1U << 2;
    static constexpr uint16_t thruster_flag_reverse_folded = 1U << 3;

    void init(int16_t frame_type = default_frame_type);
    bool set_frame_type(int16_t frame_type);
    void set_frame_geometry(const MiniDP_FrameGeometryConfig &new_config);
    bool set_azipod_config(uint8_t index, const MiniDP_AzipodConfig &config);

    const MiniDP_OutputFrame &update(
        MiniDP_OutputState state,
        const MiniDP_AxisCommand &command, float dt_s = 1.0f);
    const MiniDP_OutputFrame &update_actuator_test(
        const MiniDP_ActuatorTestCommand &command);

    bool set_actuator_config(
        uint8_t index,
        const MiniDP_ActuatorConfig &config);

    bool set_actuator_configs(const MiniDP_ActuatorConfig (&config)[max_actuators]);
    const MiniDP_ActuatorConfig &actuator_config(uint8_t index) const;
    const MiniDP_AzipodConfig &azipod_config(uint8_t index) const;
    const MiniDP_OutputFrame &frame() const { return output_frame; }
    int16_t frame_type() const { return configured_frame_type; }

    static MiniDP_ActuatorConfig default_actuator_config(uint8_t index);
    static MiniDP_FrameGeometryConfig default_frame_geometry_config();
    static const char *frame_type_name(int16_t frame_type);
    static const char *screw_position_name(MiniDP_ScrewPosition position);
    static const char *thruster_type_name(MiniDP_ThrusterType type);
    static const char *motor_name(uint8_t index);
    static const char *state_name(MiniDP_OutputState state);
    static const char *safe_action_name(MiniDP_OutputSafeAction action);

private:
    MiniDP_ActuatorConfig configs[max_actuators];
    MiniDP_AzipodConfig azipod_configs[2];
    float pod_angle_rad[2]{};
    bool pod_in_transit[2]{};
    float pod_settle_s[2]{};
    float output_dt_s = 0.0f;
    MiniDP_ThrusterConfig thruster_configs[max_thrusters];
    MiniDP_OutputFrame output_frame{};
    MiniDP_FrameGeometryConfig frame_geometry{};
    int16_t configured_frame_type = default_frame_type;

    void reset_frame_configs();
    void reset_thruster_configs();
    void configure_motor(
        uint8_t index,
        float k_surge,
        float k_sway,
        float k_yaw);
    void configure_fixed_thruster(
        uint8_t thruster_index,
        uint8_t output_index,
        float k_surge,
        float k_sway,
        float k_yaw);
    void configure_azimuth_180_thruster(
        uint8_t thruster_index,
        uint8_t thrust_output_index,
        uint8_t azimuth_output_index);
    void configure_omni_plus_frame();
    void configure_dual_az_180_bow_frame();
    void apply_omni_plus_geometry();
    void initialise_output_frame(MiniDP_OutputState state);
    void apply_safe_outputs(MiniDP_OutputState state);
    void apply_active_outputs(MiniDP_OutputState state, const MiniDP_AxisCommand &command);
    void apply_scalar_mixer_outputs(MiniDP_OutputState state, const MiniDP_AxisCommand &command);
    void apply_dual_az_180_bow_outputs(MiniDP_OutputState state, const MiniDP_AxisCommand &command);
    void apply_actuator_test_output(const MiniDP_ActuatorTestCommand &command);
    void set_disabled_output(uint8_t index);
    void set_safe_output(uint8_t index, MiniDP_OutputSafeAction action);
    void set_pwm_output(uint8_t index, uint16_t pwm_us, float demand, bool saturated);
    void set_fixed_thruster_output(uint8_t thruster_index, float demand, bool saturated);
    void set_azimuth_180_thruster_output(uint8_t thruster_index, float force_x, float force_y, bool saturated);
    void sync_thruster_outputs_from_actuators();
    uint16_t constrain_pwm_to_actuator(const MiniDP_ActuatorConfig &config, uint16_t pwm_us) const;
    uint16_t pwm_from_demand(const MiniDP_ActuatorConfig &config, float demand) const;
    uint16_t pwm_from_angle(const MiniDP_ActuatorConfig &config, float angle_rad, float angle_min_rad, float angle_max_rad) const;
};
