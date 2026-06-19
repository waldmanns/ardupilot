#include "Output.h"

namespace {

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

void MiniDP_OutputManager::reset_frame_configs()
{
    for (uint8_t i = 0; i < max_actuators; i++) {
        configs[i] = default_actuator_config(i);
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

void MiniDP_OutputManager::configure_omni_plus_frame()
{
    reset_frame_configs();

    configure_motor(0U, 1.0f, 0.0f, 1.0f);   // Motor1: port propulsion screw
    configure_motor(1U, 1.0f, 0.0f, -1.0f);  // Motor2: starboard propulsion screw
    configure_motor(2U, 0.0f, 1.0f, 1.0f);   // Motor3: bow tunnel thruster
    configure_motor(3U, 0.0f, 1.0f, -1.0f);  // Motor4: stern tunnel thruster
}

bool MiniDP_OutputManager::set_frame_type(const int16_t frame_type)
{
    switch (frame_type) {
    case int16_t(MiniDP_FrameType::OMNI_PLUS):
        configure_omni_plus_frame();
        configured_frame_type = frame_type;
        return true;
    default:
        return false;
    }
}

void MiniDP_OutputManager::init(const int16_t frame_type)
{
    if (!set_frame_type(frame_type)) {
        (void)set_frame_type(default_frame_type);
    }
    apply_safe_outputs(MiniDP_OutputState::DISARMED);
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
    output_frame = {};
    output_frame.state = state;

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
}

void MiniDP_OutputManager::apply_active_outputs(
    const MiniDP_OutputState state,
    const MiniDP_AxisCommand &command)
{
    output_frame = {};
    output_frame.state = state;

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
}

void MiniDP_OutputManager::apply_actuator_test_output(
    const MiniDP_ActuatorTestCommand &command)
{
    output_frame = {};
    output_frame.state = MiniDP_OutputState::ACTUATOR_TEST;

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
        return;
    }

    const float demand = constrain_unit(command.demand);
    set_pwm_output(
        command.actuator_index,
        pwm_from_demand(config, demand),
        demand,
        false);
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
