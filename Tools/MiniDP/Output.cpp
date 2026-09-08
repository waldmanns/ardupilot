#include "Output.h"
#include "ControlMath.h"

#include <math.h>

namespace {

constexpr float pi = 3.14159265358979323846f;
constexpr float half_pi = pi * 0.5f;
constexpr uint8_t output_none = 255U;

float constrain_unit(const float value)
{
    if (!isfinite(value)) {
        return 0.0f;
    }
    if (value > 1.0f) {
        return 1.0f;
    }
    if (value < -1.0f) {
        return -1.0f;
    }
    return value;
}

float absf(const float value)
{
    return value < 0.0f ? -value : value;
}

float sanitized_scale(const float scale)
{
    return isfinite(scale) && scale > 0.0f ? fminf(scale, 10.0f) : 1.0f;
}


float constrain_angle(
    const float angle_rad,
    const float angle_min_rad,
    const float angle_max_rad)
{
    if (angle_rad < angle_min_rad) {
        return angle_min_rad;
    }
    if (angle_rad > angle_max_rad) {
        return angle_max_rad;
    }
    return angle_rad;
}

float sanitize_screw_yaw_scale(const float scale)
{
    if (!isfinite(scale) || scale < 0.0f) {
        return 1.0f;
    }
    if (scale > 2.0f) {
        return 2.0f;
    }
    return scale;
}

MiniDP_ScrewPosition sanitize_screw_position(const MiniDP_ScrewPosition position)
{
    switch (position) {
    case MiniDP_ScrewPosition::AFT:
    case MiniDP_ScrewPosition::CENTER:
    case MiniDP_ScrewPosition::FORWARD:
        return position;
    }
    return MiniDP_ScrewPosition::AFT;
}

uint16_t sane_pwm(const uint16_t pwm, const uint16_t fallback)
{
    if (pwm < 800U || pwm > 2200U) {
        return fallback;
    }
    return pwm;
}

} // namespace

constexpr int16_t MiniDP_OutputManager::default_frame_type;

MiniDP_ActuatorConfig MiniDP_OutputManager::default_actuator_config(
    const uint8_t index)
{
    MiniDP_ActuatorConfig config{};
    config.enabled = false;
    config.pwm_channel = index;
    config.pwm_min = 1100U;
    config.pwm_trim = 1500U;
    config.pwm_max = 1900U;
    config.pwm_failsafe = 1500U;
    config.forward_scale = 1.0f;
    config.reverse_scale = 1.0f;
    config.disarmed_action = MiniDP_OutputSafeAction::DISABLE_PWM;
    config.failsafe_action = MiniDP_OutputSafeAction::SEND_NEUTRAL;
    config.kill_action = MiniDP_OutputSafeAction::DISABLE_PWM;
    return config;
}

MiniDP_FrameGeometryConfig MiniDP_OutputManager::default_frame_geometry_config()
{
    MiniDP_FrameGeometryConfig config{};
    config.screw_position = MiniDP_ScrewPosition::AFT;
    config.screw_yaw_scale = 1.0f;
    return config;
}

void MiniDP_OutputManager::reset_frame_configs()
{
    for (uint8_t i = 0; i < max_actuators; i++) {
        configs[i] = default_actuator_config(i);
    }
    reset_thruster_configs();
}

void MiniDP_OutputManager::reset_thruster_configs()
{
    for (uint8_t i = 0; i < max_thrusters; i++) {
        thruster_configs[i] = {};
        thruster_configs[i].type = MiniDP_ThrusterType::NONE;
        thruster_configs[i].output1 = output_none;
        thruster_configs[i].output2 = output_none;
        thruster_configs[i].angle_min_rad = -half_pi;
        thruster_configs[i].angle_max_rad = half_pi;
        thruster_configs[i].allow_reverse_fold = true;
    }
}

void MiniDP_OutputManager::configure_motor(
    const uint8_t index,
    const float k_surge,
    const float k_sway,
    const float k_yaw)
{
    if (index >= max_actuators) {
        return;
    }

    configs[index].enabled = true;
    configs[index].k_surge = k_surge;
    configs[index].k_sway = k_sway;
    configs[index].k_yaw = k_yaw;
}

void MiniDP_OutputManager::configure_fixed_thruster(
    const uint8_t thruster_index,
    const uint8_t output_index,
    const float k_surge,
    const float k_sway,
    const float k_yaw)
{
    if (thruster_index >= max_thrusters || output_index >= max_actuators) {
        return;
    }

    configure_motor(output_index, k_surge, k_sway, k_yaw);

    MiniDP_ThrusterConfig &thruster = thruster_configs[thruster_index];
    thruster = {};
    thruster.enabled = true;
    thruster.type = MiniDP_ThrusterType::FIXED;
    thruster.output1 = output_index;
    thruster.output2 = output_none;
    thruster.k_surge = k_surge;
    thruster.k_sway = k_sway;
    thruster.k_yaw = k_yaw;
}

void MiniDP_OutputManager::configure_azimuth_180_thruster(
    const uint8_t thruster_index,
    const uint8_t thrust_output_index,
    const uint8_t azimuth_output_index)
{
    if (thruster_index >= max_thrusters ||
        thrust_output_index >= max_actuators ||
        azimuth_output_index >= max_actuators) {
        return;
    }

    configs[thrust_output_index].enabled = true;
    configs[azimuth_output_index].enabled = true;

    MiniDP_ThrusterConfig &thruster = thruster_configs[thruster_index];
    thruster = {};
    thruster.enabled = true;
    thruster.type = MiniDP_ThrusterType::AZIMUTH_180;
    thruster.output1 = thrust_output_index;
    thruster.output2 = azimuth_output_index;
    thruster.angle_min_rad = azipod_configs[thruster_index].angle_min_rad;
    thruster.angle_max_rad = azipod_configs[thruster_index].angle_max_rad;
    thruster.allow_reverse_fold =
        azipod_configs[thruster_index].allow_reverse_fold;
}

void MiniDP_OutputManager::configure_omni_plus_frame()
{
    reset_frame_configs();

    configure_fixed_thruster(0U, 0U, 1.0f, 0.0f, 1.0f);   // Motor1: port propulsion screw
    configure_fixed_thruster(1U, 1U, 1.0f, 0.0f, -1.0f);  // Motor2: starboard propulsion screw
    configure_fixed_thruster(2U, 2U, 0.0f, 1.0f, 1.0f);   // Motor3: bow tunnel thruster
    configure_fixed_thruster(3U, 3U, 0.0f, 1.0f, -1.0f);  // Motor4: stern tunnel thruster
    apply_omni_plus_geometry();
}

void MiniDP_OutputManager::configure_dual_az_180_bow_frame()
{
    reset_frame_configs();

    configure_azimuth_180_thruster(0U, 0U, 1U);            // Motor1 thrust, Motor2 azimuth
    configure_azimuth_180_thruster(1U, 2U, 3U);            // Motor3 thrust, Motor4 azimuth
    configure_fixed_thruster(2U, 4U, 0.0f, 1.0f, 1.0f);    // Motor5 bow thruster
}

void MiniDP_OutputManager::apply_omni_plus_geometry()
{
    // Longitudinal screws create yaw through lateral offset, independent
    // of forward/aft placement. Use the scale to disable their yaw authority.
    const float screw_yaw = frame_geometry.screw_yaw_scale / frame_geometry.screw_half_span_m;
    const float arm_sum = frame_geometry.aft_arm_m + frame_geometry.bow_arm_m;
    configs[2].k_sway = 2.0f * frame_geometry.aft_arm_m / arm_sum;
    configs[3].k_sway = 2.0f * frame_geometry.bow_arm_m / arm_sum;
    configs[2].k_yaw = 1.0f / frame_geometry.bow_arm_m;
    configs[3].k_yaw = -1.0f / frame_geometry.aft_arm_m;

    configs[0].k_yaw = screw_yaw;
    configs[1].k_yaw = -screw_yaw;
    thruster_configs[0].k_yaw = screw_yaw;
    thruster_configs[1].k_yaw = -screw_yaw;
}

bool MiniDP_OutputManager::set_frame_type(const int16_t frame_type)
{
    switch (frame_type) {
    case int16_t(MiniDP_FrameType::OMNI_PLUS):
        configure_omni_plus_frame();
        configured_frame_type = frame_type;
        return true;
    case int16_t(MiniDP_FrameType::DUAL_AZ_180_BOW):
        configure_dual_az_180_bow_frame();
        configured_frame_type = frame_type;
        return true;
    default:
        return false;
    }
}

void MiniDP_OutputManager::init(const int16_t frame_type)
{
    frame_geometry = default_frame_geometry_config();
    for (uint8_t i = 0; i < 2U; i++) {
        azipod_configs[i].angle_min_rad = -half_pi;
        azipod_configs[i].angle_max_rad = half_pi;
        azipod_configs[i].allow_reverse_fold = true;
        azipod_configs[i].steering_rate_rad_s = half_pi;
        pod_angle_rad[i] = 0.0f;
    }
    if (!set_frame_type(frame_type)) {
        (void)set_frame_type(default_frame_type);
    }
    apply_safe_outputs(MiniDP_OutputState::DISARMED);
}

bool MiniDP_OutputManager::set_azipod_config(
    const uint8_t index,
    const MiniDP_AzipodConfig &config)
{
    if (index >= 2U) {
        return false;
    }

    MiniDP_AzipodConfig sanitized = config;
    if (!isfinite(sanitized.angle_min_rad)) {
        sanitized.angle_min_rad = -half_pi;
    }
    if (!isfinite(sanitized.angle_max_rad)) {
        sanitized.angle_max_rad = half_pi;
    }
    sanitized.angle_min_rad = constrain_angle(sanitized.angle_min_rad, -pi, 0.0f);
    sanitized.angle_max_rad = constrain_angle(sanitized.angle_max_rad, 0.0f, pi);
    if (sanitized.angle_max_rad - sanitized.angle_min_rad > pi) {
        const float center =
            0.5f * (sanitized.angle_min_rad + sanitized.angle_max_rad);
        sanitized.angle_min_rad = center - half_pi;
        sanitized.angle_max_rad = center + half_pi;
    }
    sanitized.steering_rate_rad_s = isfinite(sanitized.steering_rate_rad_s) ?
        fminf(2.0f * pi, fmaxf(0.01f, sanitized.steering_rate_rad_s)) : half_pi;
    azipod_configs[index] = sanitized;

    if (configured_frame_type == int16_t(MiniDP_FrameType::DUAL_AZ_180_BOW)) {
        MiniDP_ThrusterConfig &thruster = thruster_configs[index];
        thruster.angle_min_rad = sanitized.angle_min_rad;
        thruster.angle_max_rad = sanitized.angle_max_rad;
        thruster.allow_reverse_fold = sanitized.allow_reverse_fold;
    }
    return true;
}

void MiniDP_OutputManager::set_frame_geometry(
    const MiniDP_FrameGeometryConfig &new_config)
{
    frame_geometry = new_config;
    frame_geometry.screw_position =
        sanitize_screw_position(frame_geometry.screw_position);
    frame_geometry.screw_yaw_scale =
        sanitize_screw_yaw_scale(frame_geometry.screw_yaw_scale);
    const auto arm = [](float value) {
        return isfinite(value) ? fminf(100.0f, fmaxf(0.05f, value)) : 1.0f;
    };
    frame_geometry.aft_arm_m = arm(frame_geometry.aft_arm_m);
    frame_geometry.bow_arm_m = arm(frame_geometry.bow_arm_m);
    frame_geometry.screw_half_span_m = arm(frame_geometry.screw_half_span_m);

    switch (configured_frame_type) {
    case int16_t(MiniDP_FrameType::OMNI_PLUS):
        apply_omni_plus_geometry();
        break;
    default:
        break;
    }
}

bool MiniDP_OutputManager::set_actuator_config(
    const uint8_t index,
    const MiniDP_ActuatorConfig &config)
{
    if (index >= max_actuators) {
        return false;
    }

    MiniDP_ActuatorConfig sanitized = config;
    if (sanitized.pwm_channel >= 32U && sanitized.pwm_channel != output_none) {
        return false;
    }
    sanitized.pwm_trim = sane_pwm(sanitized.pwm_trim, 1500U);
    sanitized.pwm_min = sane_pwm(sanitized.pwm_min, 1100U);
    sanitized.pwm_max = sane_pwm(sanitized.pwm_max, 1900U);
    sanitized.pwm_failsafe = sane_pwm(sanitized.pwm_failsafe, sanitized.pwm_trim);

    if (sanitized.pwm_min > sanitized.pwm_trim) {
        sanitized.pwm_min = sanitized.pwm_trim;
    }
    if (sanitized.pwm_max < sanitized.pwm_trim) {
        sanitized.pwm_max = sanitized.pwm_trim;
    }

    sanitized.forward_scale = sanitized_scale(sanitized.forward_scale);
    sanitized.reverse_scale = sanitized_scale(sanitized.reverse_scale);
    sanitized.deadband = constrain_unit(absf(sanitized.deadband));
    sanitized.min_effective_output =
        constrain_unit(absf(sanitized.min_effective_output));

    if (sanitized.enabled && sanitized.pwm_channel != output_none) {
        for (uint8_t i = 0; i < max_actuators; i++) {
            if (i != index &&
                configs[i].enabled &&
                configs[i].pwm_channel == sanitized.pwm_channel) {
                return false;
            }
        }
    }

    configs[index] = sanitized;
    return true;
}

bool MiniDP_OutputManager::set_actuator_configs(
    const MiniDP_ActuatorConfig (&new_configs)[max_actuators])
{
    // Validate the whole permutation before replacing any calibration.
    MiniDP_ActuatorConfig saved[max_actuators];
    for (uint8_t i = 0; i < max_actuators; i++) {
        saved[i] = configs[i];
        configs[i].enabled = false;
    }
    for (uint8_t i = 0; i < max_actuators; i++) {
        if (!set_actuator_config(i, new_configs[i])) {
            for (uint8_t j = 0; j < max_actuators; j++) {
                configs[j] = saved[j];
            }
            return false;
        }
    }
    return true;
}

const MiniDP_ActuatorConfig &MiniDP_OutputManager::actuator_config(
    const uint8_t index) const
{
    return configs[index < max_actuators ? index : 0U];
}

const MiniDP_AzipodConfig &MiniDP_OutputManager::azipod_config(
    const uint8_t index) const
{
    return azipod_configs[index < 2U ? index : 0U];
}

void MiniDP_OutputManager::initialise_output_frame(
    const MiniDP_OutputState state)
{
    output_frame = {};
    output_frame.state = state;

    for (uint8_t i = 0; i < max_thrusters; i++) {
        MiniDP_ThrusterOutput &output = output_frame.thruster[i];
        const MiniDP_ThrusterConfig &config = thruster_configs[i];
        output = {};
        output.type = config.type;
        output.output1 = config.output1;
        output.output2 = config.output2;
        output.value = 0.0f;
        output.value2 = 0.0f;
        if (config.enabled) {
            output.configured = true;
            output_frame.configured_thruster_count++;
        }
    }
}

const MiniDP_OutputFrame &MiniDP_OutputManager::update(
    const MiniDP_OutputState state,
    const MiniDP_AxisCommand &command, const float dt_s)
{
    output_dt_s = isfinite(dt_s) ? fminf(1.0f, fmaxf(0.0f, dt_s)) : 0.0f;
    const MiniDP_AxisCommand safe_command{constrain_unit(command.surge),
        constrain_unit(command.sway), constrain_unit(command.yaw)};
    switch (state) {
    case MiniDP_OutputState::ARMED_ACTIVE:
    case MiniDP_OutputState::ACTUATOR_TEST:
        apply_active_outputs(state, safe_command);
        break;
    case MiniDP_OutputState::DISARMED:
    case MiniDP_OutputState::ARMED_NEUTRAL:
    case MiniDP_OutputState::FAILSAFE:
    case MiniDP_OutputState::KILL:
        apply_safe_outputs(state);
        break;
    }

    return output_frame;
}

const MiniDP_OutputFrame &MiniDP_OutputManager::update_actuator_test(
    const MiniDP_ActuatorTestCommand &command)
{
    apply_actuator_test_output(command);
    return output_frame;
}

void MiniDP_OutputManager::set_disabled_output(const uint8_t index)
{
    MiniDP_ActuatorOutput &output = output_frame.actuator[index];
    output = {};
    output.configured = configs[index].enabled;
    output.pwm_channel = configs[index].pwm_channel;
}

void MiniDP_OutputManager::set_pwm_output(
    const uint8_t index,
    const uint16_t pwm_us,
    const float demand,
    const bool saturated)
{
    MiniDP_ActuatorOutput &output = output_frame.actuator[index];
    output.configured = configs[index].enabled;
    output.pwm_enabled = true;
    output.pwm_channel = configs[index].pwm_channel;
    output.pwm_us = pwm_us;
    output.demand = demand;
    output.saturated = saturated;
    output_frame.active_pwm_count++;
}

void MiniDP_OutputManager::set_safe_output(
    const uint8_t index,
    const MiniDP_OutputSafeAction action)
{
    if (!configs[index].enabled) {
        set_disabled_output(index);
        return;
    }

    switch (action) {
    case MiniDP_OutputSafeAction::DISABLE_PWM:
        set_disabled_output(index);
        break;
    case MiniDP_OutputSafeAction::SEND_NEUTRAL:
        set_pwm_output(index, configs[index].pwm_trim, 0.0f, false);
        break;
    case MiniDP_OutputSafeAction::SEND_FAILSAFE:
        set_pwm_output(index, configs[index].pwm_failsafe, 0.0f, false);
        break;
    }
}

void MiniDP_OutputManager::apply_safe_outputs(const MiniDP_OutputState state)
{
    initialise_output_frame(state);
    // Hold the last steering demand on neutral/failsafe instead of recentering
    // unexpectedly. Disarmed/kill PWM policies remain authoritative.

    for (uint8_t i = 0; i < max_actuators; i++) {
        switch (state) {
        case MiniDP_OutputState::DISARMED:
            set_safe_output(i, configs[i].disarmed_action);
            break;
        case MiniDP_OutputState::ARMED_NEUTRAL:
            set_safe_output(i, MiniDP_OutputSafeAction::SEND_NEUTRAL);
            break;
        case MiniDP_OutputState::FAILSAFE:
            set_safe_output(i, configs[i].failsafe_action);
            break;
        case MiniDP_OutputState::KILL:
            set_safe_output(i, configs[i].kill_action);
            break;
        case MiniDP_OutputState::ARMED_ACTIVE:
        case MiniDP_OutputState::ACTUATOR_TEST:
            set_disabled_output(i);
            break;
        }
    }

    if (configured_frame_type == int16_t(MiniDP_FrameType::DUAL_AZ_180_BOW)) {
        for (uint8_t i = 0; i < 2; i++) {
            const uint8_t motor = thruster_configs[i].output2;
            auto &steering = output_frame.actuator[motor];
            MiniDP_OutputSafeAction action = MiniDP_OutputSafeAction::SEND_NEUTRAL;
            if (state == MiniDP_OutputState::DISARMED) {
                action = configs[motor].disarmed_action;
            } else if (state == MiniDP_OutputState::FAILSAFE) {
                action = configs[motor].failsafe_action;
            } else if (state == MiniDP_OutputState::KILL) {
                action = configs[motor].kill_action;
            }
            if (!steering.pwm_enabled || action == MiniDP_OutputSafeAction::SEND_FAILSAFE) {
                // With no position feedback, allow a full calibrated sweep to
                // reacquire the commanded angle after PWM loss or an external
                // steering command. Thrust remains neutral during this period.
                pod_settle_s[i] = (thruster_configs[i].angle_max_rad -
                    thruster_configs[i].angle_min_rad) / azipod_configs[i].steering_rate_rad_s;
            }
            if (steering.pwm_enabled && action == MiniDP_OutputSafeAction::SEND_NEUTRAL) {
                steering.pwm_us = pwm_from_angle(configs[motor], pod_angle_rad[i],
                    thruster_configs[i].angle_min_rad, thruster_configs[i].angle_max_rad);
            }
            output_frame.thruster[i].value2 = pod_angle_rad[i];
        }
    }
    sync_thruster_outputs_from_actuators();
}

void MiniDP_OutputManager::apply_active_outputs(
    const MiniDP_OutputState state,
    const MiniDP_AxisCommand &command)
{
    switch (configured_frame_type) {
    case int16_t(MiniDP_FrameType::DUAL_AZ_180_BOW):
        apply_dual_az_180_bow_outputs(state, command);
        break;
    case int16_t(MiniDP_FrameType::OMNI_PLUS):
    default:
        apply_scalar_mixer_outputs(state, command);
        break;
    }
}

void MiniDP_OutputManager::apply_scalar_mixer_outputs(
    const MiniDP_OutputState state,
    const MiniDP_AxisCommand &command)
{
    initialise_output_frame(state);

    float raw[max_actuators]{};
    float max_abs = 0.0f;
    for (uint8_t i = 0; i < max_actuators; i++) {
        if (!configs[i].enabled) {
            continue;
        }
        raw[i] =
            (configs[i].k_surge * command.surge) +
            (configs[i].k_sway * command.sway) +
            (configs[i].k_yaw * command.yaw);
        const float actuator_abs = absf(raw[i]);
        if (actuator_abs > max_abs) {
            max_abs = actuator_abs;
        }
    }

    const bool saturated = max_abs > 1.0f;
    const float saturation_scale = saturated ? max_abs : 1.0f;
    output_frame.saturated = saturated;

    for (uint8_t i = 0; i < max_actuators; i++) {
        if (!configs[i].enabled) {
            set_disabled_output(i);
            continue;
        }

        float demand = raw[i] / saturation_scale;
        if (absf(demand) < configs[i].deadband) {
            demand = 0.0f;
        } else if (absf(demand) > 0.0f && configs[i].min_effective_output > 0.0f &&
                   absf(demand) < configs[i].min_effective_output) {
            demand = demand < 0.0f ?
                -configs[i].min_effective_output :
                configs[i].min_effective_output;
        }
        demand = constrain_unit(demand);

        set_pwm_output(i, pwm_from_demand(configs[i], demand), demand, saturated);
    }

    sync_thruster_outputs_from_actuators();
}

void MiniDP_OutputManager::apply_dual_az_180_bow_outputs(
    const MiniDP_OutputState state,
    const MiniDP_AxisCommand &command)
{
    initialise_output_frame(state);

    for (uint8_t i = 0; i < max_actuators; i++) {
        if (!configs[i].enabled) {
            set_disabled_output(i);
        }
    }

    float pod1_x = 0.5f * command.surge;
    float pod2_x = 0.5f * command.surge;
    // Solve Fy=aft+bow and Mz=-aft_arm*aft+bow_arm*bow.
    const float arm_sum = frame_geometry.aft_arm_m + frame_geometry.bow_arm_m;
    float bow = (frame_geometry.aft_arm_m * command.sway + command.yaw) / arm_sum;
    float pod1_y = 0.5f * (frame_geometry.bow_arm_m * command.sway - command.yaw) / arm_sum;
    float pod2_y = pod1_y;

    float max_abs = sqrtf((pod1_x * pod1_x) + (pod1_y * pod1_y));
    const float pod2_abs = sqrtf((pod2_x * pod2_x) + (pod2_y * pod2_y));
    if (pod2_abs > max_abs) {
        max_abs = pod2_abs;
    }
    if (absf(bow) > max_abs) {
        max_abs = absf(bow);
    }

    const bool saturated = max_abs > 1.0f;
    const float saturation_scale = saturated ? max_abs : 1.0f;
    output_frame.saturated = saturated;

    pod1_x /= saturation_scale;
    pod1_y /= saturation_scale;
    pod2_x /= saturation_scale;
    pod2_y /= saturation_scale;
    bow /= saturation_scale;

    set_azimuth_180_thruster_output(0U, pod1_x, pod1_y, saturated);
    set_azimuth_180_thruster_output(1U, pod2_x, pod2_y, saturated);
    set_fixed_thruster_output(2U, bow, saturated);
    if (pod_in_transit[0] || pod_in_transit[1]) {
        // Do not apply the other half of a force/moment allocation while a
        // pod is still steering. Keep steering PWM active, all thrust neutral.
        for (uint8_t i = 0; i < 3; i++) {
            const uint8_t motor = thruster_configs[i].output1;
            output_frame.actuator[motor].demand = 0.0f;
            output_frame.actuator[motor].pwm_us = configs[motor].pwm_trim;
            output_frame.thruster[i].force_x = 0.0f;
            output_frame.thruster[i].force_y = 0.0f;
            output_frame.thruster[i].value = 0.0f;
            output_frame.thruster[i].pwm1_us = configs[motor].pwm_trim;
        }
    }
}

void MiniDP_OutputManager::apply_actuator_test_output(
    const MiniDP_ActuatorTestCommand &command)
{
    initialise_output_frame(MiniDP_OutputState::ACTUATOR_TEST);
    for (uint8_t i = 0; i < 2; i++) {
        pod_settle_s[i] = (azipod_configs[i].angle_max_rad -
            azipod_configs[i].angle_min_rad) / azipod_configs[i].steering_rate_rad_s;
    }

    for (uint8_t i = 0; i < max_actuators; i++) {
        set_disabled_output(i);
    }

    if (!command.active ||
        command.actuator_index >= max_actuators ||
        !configs[command.actuator_index].enabled) {
        return;
    }

    const MiniDP_ActuatorConfig &config = configs[command.actuator_index];
    if (command.use_pwm) {
        set_pwm_output(
            command.actuator_index,
            constrain_pwm_to_actuator(config, command.pwm_us),
            0.0f,
            false);
        sync_thruster_outputs_from_actuators();
        return;
    }

    const float demand = constrain_unit(command.demand);
    set_pwm_output(
        command.actuator_index,
        pwm_from_demand(config, demand),
        demand,
        false);

    sync_thruster_outputs_from_actuators();
}

void MiniDP_OutputManager::set_fixed_thruster_output(
    const uint8_t thruster_index,
    float demand,
    const bool saturated)
{
    if (thruster_index >= max_thrusters ||
        !thruster_configs[thruster_index].enabled ||
        thruster_configs[thruster_index].type != MiniDP_ThrusterType::FIXED) {
        return;
    }

    const uint8_t output_index = thruster_configs[thruster_index].output1;
    if (output_index >= max_actuators || !configs[output_index].enabled) {
        return;
    }

    if (absf(demand) < configs[output_index].deadband) {
        demand = 0.0f;
    } else if (absf(demand) > 0.0f && configs[output_index].min_effective_output > 0.0f &&
               absf(demand) < configs[output_index].min_effective_output) {
        demand = demand < 0.0f ?
            -configs[output_index].min_effective_output :
            configs[output_index].min_effective_output;
    }
    demand = constrain_unit(demand);

    const uint16_t pwm_us = pwm_from_demand(configs[output_index], demand);
    set_pwm_output(output_index, pwm_us, demand, saturated);

    MiniDP_ThrusterOutput &output = output_frame.thruster[thruster_index];
    output.configured = true;
    output.type = MiniDP_ThrusterType::FIXED;
    output.flags = thruster_flag_active;
    if (saturated) {
        output.flags |= thruster_flag_saturated;
    }
    output.output1 = output_index;
    output.output2 = output_none;
    output.force_x = thruster_configs[thruster_index].k_surge * demand;
    output.force_y = thruster_configs[thruster_index].k_sway * demand;
    output.value = demand;
    output.value2 = 0.0f;
    output.pwm1_us = pwm_us;
    output.pwm2_us = 0U;
}

void MiniDP_OutputManager::set_azimuth_180_thruster_output(
    const uint8_t thruster_index,
    const float force_x,
    const float force_y,
    const bool saturated)
{
    if (thruster_index >= max_thrusters ||
        !thruster_configs[thruster_index].enabled ||
        thruster_configs[thruster_index].type != MiniDP_ThrusterType::AZIMUTH_180) {
        return;
    }

    const MiniDP_ThrusterConfig &thruster = thruster_configs[thruster_index];
    const uint8_t thrust_output_index = thruster.output1;
    const uint8_t azimuth_output_index = thruster.output2;
    if (thrust_output_index >= max_actuators ||
        azimuth_output_index >= max_actuators ||
        !configs[thrust_output_index].enabled ||
        !configs[azimuth_output_index].enabled) {
        return;
    }

    const float magnitude = hypotf(force_x, force_y);
    const float desired_angle = magnitude > 1.0e-6f ? atan2f(force_y, force_x) : pod_angle_rad[thruster_index];
    float angle_rad = pod_angle_rad[thruster_index];
    float thrust = 0.0f;
    float best_cost = INFINITY;
    // Project the requested vector onto each feasible direction. Consider
    // reverse thrust too; choose a continuous solution near the fold boundary.
    const float candidates[] = {
        constrain_angle(desired_angle, thruster.angle_min_rad, thruster.angle_max_rad),
        constrain_angle(MiniDP_Math::wrap_pi(desired_angle + pi), thruster.angle_min_rad, thruster.angle_max_rad),
        thruster.angle_min_rad, thruster.angle_max_rad,
        constrain_angle(angle_rad, thruster.angle_min_rad, thruster.angle_max_rad)
    };
    for (float candidate : candidates) {
        float projected = force_x * cosf(candidate) + force_y * sinf(candidate);
        if (!thruster.allow_reverse_fold) {
            projected = fmaxf(0.0f, projected);
        }
        const float dx = force_x - projected * cosf(candidate);
        const float dy = force_y - projected * sinf(candidate);
        const float travel = (candidate - pod_angle_rad[thruster_index]) / pi;
        const float cost = dx * dx + dy * dy + 0.0025f * magnitude * magnitude * travel * travel;
        if (cost < best_cost) {
            best_cost = cost;
            angle_rad = candidate;
            thrust = projected;
        }
    }
    uint16_t flags = thruster_flag_active;
    const float max_step = azipod_configs[thruster_index].steering_rate_rad_s * output_dt_s;
    const float old_angle = pod_angle_rad[thruster_index];
    pod_angle_rad[thruster_index] = constrain_angle(angle_rad, old_angle - max_step, old_angle + max_step);
    pod_settle_s[thruster_index] = fmaxf(0.0f, pod_settle_s[thruster_index] - output_dt_s);
    const bool steering_in_transit = pod_settle_s[thruster_index] > 0.0f ||
        fabsf(angle_rad - pod_angle_rad[thruster_index]) > 0.01f;
    pod_in_transit[thruster_index] = steering_in_transit;
    angle_rad = pod_angle_rad[thruster_index];
    if (steering_in_transit) {
        output_frame.saturated = true;
        thrust = 0.0f;
    }
    const float residual = hypotf(force_x - thrust * cosf(angle_rad), force_y - thrust * sinf(angle_rad));
    if (residual > 0.01f * magnitude + 1.0e-5f) {
        flags |= thruster_flag_angle_limited;
        output_frame.saturated = true;
    }
    if (saturated || output_frame.saturated) {
        flags |= thruster_flag_saturated;
    }
    if (thrust < 0.0f) {
        flags |= thruster_flag_reverse_folded;
    }

    if (absf(thrust) < configs[thrust_output_index].deadband) {
        thrust = 0.0f;
    } else if (absf(thrust) > 0.0f && configs[thrust_output_index].min_effective_output > 0.0f &&
               absf(thrust) < configs[thrust_output_index].min_effective_output) {
        thrust = thrust < 0.0f ?
            -configs[thrust_output_index].min_effective_output :
            configs[thrust_output_index].min_effective_output;
    }
    thrust = constrain_unit(thrust);

    const uint16_t thrust_pwm =
        pwm_from_demand(configs[thrust_output_index], thrust);
    const uint16_t azimuth_pwm =
        pwm_from_angle(
            configs[azimuth_output_index],
            angle_rad,
            thruster.angle_min_rad,
            thruster.angle_max_rad);
    const float servo_demand = constrain_unit(angle_rad / half_pi);

    set_pwm_output(thrust_output_index, thrust_pwm, thrust, saturated);
    set_pwm_output(azimuth_output_index, azimuth_pwm, servo_demand, false);

    MiniDP_ThrusterOutput &output = output_frame.thruster[thruster_index];
    output.configured = true;
    output.type = MiniDP_ThrusterType::AZIMUTH_180;
    output.flags = flags;
    output.output1 = thrust_output_index;
    output.output2 = azimuth_output_index;
    output.force_x = cosf(angle_rad) * thrust;
    output.force_y = sinf(angle_rad) * thrust;
    output.value = thrust;
    output.value2 = angle_rad;
    output.pwm1_us = thrust_pwm;
    output.pwm2_us = azimuth_pwm;
}

void MiniDP_OutputManager::sync_thruster_outputs_from_actuators()
{
    for (uint8_t i = 0; i < max_thrusters; i++) {
        MiniDP_ThrusterOutput &output = output_frame.thruster[i];
        const MiniDP_ThrusterConfig &config = thruster_configs[i];
        if (!config.enabled) {
            continue;
        }

        output.configured = true;
        output.type = config.type;
        output.output1 = config.output1;
        output.output2 = config.output2;
        if (config.output1 < max_actuators) {
            const MiniDP_ActuatorOutput &actuator = output_frame.actuator[config.output1];
            output.pwm1_us = actuator.pwm_enabled ? actuator.pwm_us : uint16_t(0);
            if (actuator.pwm_enabled) {
                output.flags |= thruster_flag_active;
            }
            if (config.type == MiniDP_ThrusterType::FIXED) {
                output.value = actuator.configured ? actuator.demand : 0.0f;
                output.value2 = 0.0f;
                output.force_x = config.k_surge * actuator.demand;
                output.force_y = config.k_sway * actuator.demand;
                if (actuator.saturated) {
                    output.flags |= thruster_flag_saturated;
                }
            }
        }
        if (config.output2 < max_actuators) {
            const MiniDP_ActuatorOutput &actuator = output_frame.actuator[config.output2];
            output.pwm2_us = actuator.pwm_enabled ? actuator.pwm_us : uint16_t(0);
            if (actuator.pwm_enabled) {
                output.flags |= thruster_flag_active;
            }
        }
    }
}

uint16_t MiniDP_OutputManager::constrain_pwm_to_actuator(
    const MiniDP_ActuatorConfig &config,
    const uint16_t pwm_us) const
{
    if (pwm_us < config.pwm_min) {
        return config.pwm_min;
    }
    if (pwm_us > config.pwm_max) {
        return config.pwm_max;
    }
    return pwm_us;
}

uint16_t MiniDP_OutputManager::pwm_from_demand(
    const MiniDP_ActuatorConfig &config,
    float demand) const
{
    demand = constrain_unit(config.reversed ? -demand : demand);

    float pwm = config.pwm_trim;
    if (demand >= 0.0f) {
        pwm += demand *
            float(config.pwm_max - config.pwm_trim) *
            config.forward_scale;
    } else {
        pwm += demand *
            float(config.pwm_trim - config.pwm_min) *
            config.reverse_scale;
    }

    if (pwm < config.pwm_min) {
        pwm = config.pwm_min;
    } else if (pwm > config.pwm_max) {
        pwm = config.pwm_max;
    }

    return uint16_t(pwm + 0.5f);
}

uint16_t MiniDP_OutputManager::pwm_from_angle(
    const MiniDP_ActuatorConfig &config,
    float angle_rad,
    const float angle_min_rad,
    const float angle_max_rad) const
{
    angle_rad = constrain_angle(angle_rad, angle_min_rad, angle_max_rad);
    const float range = angle_rad >= 0.0f ? angle_max_rad : -angle_min_rad;
    // Normalize in the physical range before reversing the electrical range.
    const float demand = range > 0.0f ? angle_rad / range : 0.0f;
    return pwm_from_demand(config, demand);

}

const char *MiniDP_OutputManager::state_name(const MiniDP_OutputState state)
{
    switch (state) {
    case MiniDP_OutputState::DISARMED:
        return "DISARMED";
    case MiniDP_OutputState::ARMED_NEUTRAL:
        return "ARMED_NEUTRAL";
    case MiniDP_OutputState::ARMED_ACTIVE:
        return "ARMED_ACTIVE";
    case MiniDP_OutputState::ACTUATOR_TEST:
        return "ACTUATOR_TEST";
    case MiniDP_OutputState::FAILSAFE:
        return "FAILSAFE";
    case MiniDP_OutputState::KILL:
        return "KILL";
    }
    return "UNKNOWN";
}

const char *MiniDP_OutputManager::frame_type_name(const int16_t frame_type)
{
    switch (frame_type) {
    case int16_t(MiniDP_FrameType::OMNI_PLUS):
        return "OMNI_PLUS";
    case int16_t(MiniDP_FrameType::DUAL_AZ_180_BOW):
        return "DUAL_AZ_180_BOW";
    }
    return "UNKNOWN";
}

const char *MiniDP_OutputManager::screw_position_name(
    const MiniDP_ScrewPosition position)
{
    switch (position) {
    case MiniDP_ScrewPosition::AFT:
        return "aft";
    case MiniDP_ScrewPosition::CENTER:
        return "center";
    case MiniDP_ScrewPosition::FORWARD:
        return "forward";
    }
    return "unknown";
}

const char *MiniDP_OutputManager::thruster_type_name(
    const MiniDP_ThrusterType type)
{
    switch (type) {
    case MiniDP_ThrusterType::NONE:
        return "NONE";
    case MiniDP_ThrusterType::FIXED:
        return "FIXED";
    case MiniDP_ThrusterType::AZIMUTH_180:
        return "AZIMUTH_180";
    case MiniDP_ThrusterType::VOITH:
        return "VOITH";
    }
    return "UNKNOWN";
}

const char *MiniDP_OutputManager::motor_name(const uint8_t index)
{
    switch (index) {
    case 0:
        return "Motor1";
    case 1:
        return "Motor2";
    case 2:
        return "Motor3";
    case 3:
        return "Motor4";
    case 4:
        return "Motor5";
    case 5:
        return "Motor6";
    }
    return "Motor?";
}

const char *MiniDP_OutputManager::safe_action_name(
    const MiniDP_OutputSafeAction action)
{
    switch (action) {
    case MiniDP_OutputSafeAction::DISABLE_PWM:
        return "disable-pwm";
    case MiniDP_OutputSafeAction::SEND_NEUTRAL:
        return "send-neutral";
    case MiniDP_OutputSafeAction::SEND_FAILSAFE:
        return "send-failsafe";
    }
    return "unknown";
}
