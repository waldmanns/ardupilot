#include "Output.h"

#include <math.h>

namespace {

constexpr float pi = 3.14159265358979323846f;
constexpr float half_pi = pi * 0.5f;
constexpr uint8_t output_none = 255U;

float constrain_unit(const float value)
{
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
    return scale > 0.0f ? scale : 1.0f;
}

float wrap_pi_local(float angle_rad)
{
    while (angle_rad > pi) {
        angle_rad -= 2.0f * pi;
    }
    while (angle_rad < -pi) {
        angle_rad += 2.0f * pi;
    }
    return angle_rad;
}

bool angle_in_range(
    const float angle_rad,
    const float angle_min_rad,
    const float angle_max_rad)
{
    return angle_rad >= angle_min_rad && angle_rad <= angle_max_rad;
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

float screw_position_yaw_sign(const MiniDP_ScrewPosition position)
{
    switch (position) {
    case MiniDP_ScrewPosition::AFT:
        return 1.0f;
    case MiniDP_ScrewPosition::CENTER:
        return 0.0f;
    case MiniDP_ScrewPosition::FORWARD:
        return -1.0f;
    }
    return 1.0f;
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
    thruster.angle_min_rad = -half_pi;
    thruster.angle_max_rad = half_pi;
    thruster.allow_reverse_fold = true;
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
    const float screw_yaw =
        screw_position_yaw_sign(frame_geometry.screw_position) *
        frame_geometry.screw_yaw_scale;

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
    if (!set_frame_type(frame_type)) {
        (void)set_frame_type(default_frame_type);
    }
    apply_safe_outputs(MiniDP_OutputState::DISARMED);
}

void MiniDP_OutputManager::set_frame_geometry(
    const MiniDP_FrameGeometryConfig &new_config)
{
    frame_geometry = new_config;
    frame_geometry.screw_position =
        sanitize_screw_position(frame_geometry.screw_position);
    frame_geometry.screw_yaw_scale =
        sanitize_screw_yaw_scale(frame_geometry.screw_yaw_scale);

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
    if (sanitized.pwm_channel >= max_actuators) {
        sanitized.pwm_channel = index;
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

    if (sanitized.enabled) {
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

const MiniDP_ActuatorConfig &MiniDP_OutputManager::actuator_config(
    const uint8_t index) const
{
    return configs[index < max_actuators ? index : 0U];
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
    const MiniDP_AxisCommand &command)
{
    switch (state) {
    case MiniDP_OutputState::ARMED_ACTIVE:
    case MiniDP_OutputState::ACTUATOR_TEST:
        apply_active_outputs(state, command);
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
        } else if (configs[i].min_effective_output > 0.0f &&
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
    float pod1_y = 0.25f * command.sway + 0.25f * command.yaw;
    float pod2_y = 0.25f * command.sway - 0.25f * command.yaw;
    float bow = 0.5f * command.sway + 0.5f * command.yaw;

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
}

void MiniDP_OutputManager::apply_actuator_test_output(
    const MiniDP_ActuatorTestCommand &command)
{
    initialise_output_frame(MiniDP_OutputState::ACTUATOR_TEST);

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
    } else if (configs[output_index].min_effective_output > 0.0f &&
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

    float thrust = sqrtf((force_x * force_x) + (force_y * force_y));
    float angle_rad = thrust > 0.0f ? atan2f(force_y, force_x) : 0.0f;
    uint16_t flags = thruster_flag_active;
    if (saturated) {
        flags |= thruster_flag_saturated;
    }

    angle_rad = wrap_pi_local(angle_rad);
    if (!angle_in_range(angle_rad, thruster.angle_min_rad, thruster.angle_max_rad)) {
        const float folded_angle = wrap_pi_local(angle_rad + pi);
        if (thruster.allow_reverse_fold &&
            angle_in_range(folded_angle, thruster.angle_min_rad, thruster.angle_max_rad)) {
            angle_rad = folded_angle;
            thrust = -thrust;
            flags |= thruster_flag_reverse_folded;
        } else {
            angle_rad = constrain_angle(
                angle_rad,
                thruster.angle_min_rad,
                thruster.angle_max_rad);
            flags |= thruster_flag_angle_limited;
        }
    }

    if (absf(thrust) < configs[thrust_output_index].deadband) {
        thrust = 0.0f;
    } else if (configs[thrust_output_index].min_effective_output > 0.0f &&
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
    angle_rad = constrain_angle(
        config.reversed ? -angle_rad : angle_rad,
        angle_min_rad,
        angle_max_rad);

    float pwm = config.pwm_trim;
    if (angle_rad >= 0.0f) {
        const float range = angle_max_rad > 0.0f ? angle_max_rad : half_pi;
        pwm += (angle_rad / range) * float(config.pwm_max - config.pwm_trim);
    } else {
        const float range = angle_min_rad < 0.0f ? -angle_min_rad : half_pi;
        pwm += (angle_rad / range) * float(config.pwm_trim - config.pwm_min);
    }

    if (pwm < config.pwm_min) {
        pwm = config.pwm_min;
    } else if (pwm > config.pwm_max) {
        pwm = config.pwm_max;
    }

    return uint16_t(pwm + 0.5f);
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
