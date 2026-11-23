#include "Sequencer.h"

namespace Vektor2 {

static bool valid_trigger(const Sequencer::Trigger& t)
{
    return t.edge <= Sequencer::Edge::Falling && t.low >= 800 &&
        t.high <= 2200 && t.low <= t.high;
}

static bool valid_value(Sequencer::Output type, uint16_t value)
{
    return type == Sequencer::Output::Servo ? value >= 1000 && value <= 2000 : value <= 1;
}

bool Sequencer::Document::valid() const
{
    if (rate > max_rate_hz || !valid_trigger(shared)) {
        return false;
    }
    for (const auto& lane : lanes) {
        if (lane.enabled > 1 || lane.autostart > 1 || lane.repeat > 1 ||
            lane.output > Output::Binary || lane.clock > Clock::Independent ||
            !valid_value(lane.output, lane.idle) || !valid_trigger(lane.trigger) ||
            (lane.clock == Clock::Shared && lane.trigger.edge != Edge::Off)) {
            return false;
        }
        bool ended = false;
        for (const uint8_t ch : lane.name) {
            if (ch == 0) {
                ended = true;
            } else if (ended || ch < 32 || ch > 126) {
                return false;
            }
        }
    }
    uint32_t positions[lane_count] {};
    for (const auto& block : blocks) {
        if (block.lane == 0) {
            continue;
        }
        if (block.lane > lane_count || block.position == 0 || block.position > per_lane ||
            block.duration < min_duration_ms || block.duration > max_duration_ms || block.type > Type::Wait) {
            return false;
        }
        const auto output = lanes[block.lane - 1].output;
        if ((block.type != Type::Wait && !valid_value(output, block.to)) ||
            (block.type == Type::Wait && block.to != 0) ||
            (block.type == Type::Ramp ?
                output == Output::Binary || !valid_value(output, block.from) : block.from != 0)) {
            return false;
        }
        const uint32_t bit = uint32_t(1) << (block.position - 1);
        auto& set = positions[block.lane - 1];
        if (set & bit) {
            return false;
        }
        set |= bit;
    }
    for (const auto set : positions) {
        if ((set & (set + 1)) != 0) {
            return false;
        }
    }
    static_assert(uint32_t(per_lane) * max_duration_ms < INT32_MAX, "Elapsed time must be unambiguous");
    return true;
}

void Sequencer::load(const Document* document)
{
    cancel_boot_starts(UINT32_MAX);
    _powered = false;
    _doc = document;
    _participants = 0;
    _shared_length = 0;
    reset_baselines();
    for (uint8_t n = 0; n < lane_count; n++) {
        _length[n] = 0;
        for (auto& slot : _index[n]) {
            slot = 0;
        }
    }
    if (_doc != nullptr) {
        for (uint16_t n = 0; n < block_count; n++) {
            const auto& block = _doc->blocks[n];
            if (block.lane != 0) {
                _index[block.lane - 1][block.position - 1] = n + 1;
                _length[block.lane - 1] += block.duration;
            }
        }
    }
    if (_doc != nullptr) {
        for (uint8_t lane = 0; lane < lane_count; lane++) {
            if (_doc->lanes[lane].enabled && _doc->lanes[lane].clock == Clock::Shared &&
                _length[lane] > _shared_length) {
                _shared_length = _length[lane];
            }
        }
    }
    for (uint8_t n = 0; n <= shared_slot; n++) {
        _status[n] = {};
        finish(n, State::Idle, _doc == nullptr ? Reason::Configuration : Reason::None);
    }
}

void Sequencer::schedule_boot_start(uint32_t now, uint32_t inhibit_starts)
{
    if (!_active || _powered || _doc == nullptr) {
        return;
    }
    uint32_t targets = 0;
    _boot_shared_lanes = 0;
    for (uint8_t n = 0; n < lane_count; n++) {
        const auto& lane = _doc->lanes[n];
        if (!lane.enabled || !lane.autostart || _length[n] == 0) {
            continue;
        }
        if (lane.clock == Clock::Shared) {
            targets |= 1UL << shared_slot;
            _boot_shared_lanes |= 1U << n;
        } else {
            targets |= 1UL << n;
        }
    }
    _boot_targets = targets & ~inhibit_starts;
    if (boot_start_pending()) {
        _boot_start_ms = now;
        ++_status_revision;
    }
}

void Sequencer::cancel_boot_starts(uint32_t targets)
{
    if (_boot_targets & targets) {
        _boot_targets &= ~targets;
        ++_status_revision;
    }
}

void Sequencer::activate(bool enabled)
{
    if (_active == enabled) {
        return;
    }
    cancel_boot_starts(UINT32_MAX);
    _active = enabled;
    _powered = false;
    stop_all(enabled ? (_doc == nullptr ? Reason::Configuration : Reason::None) : Reason::Deactivated);
    reset_baselines();
}

bool Sequencer::power(bool on)
{
    if (on && (!_active || _doc == nullptr)) {
        return false;
    }
    cancel_boot_starts(UINT32_MAX);
    set_power(on);
    return true;
}

void Sequencer::set_power(bool on, bool keep_samples)
{
    if (on == _powered && on) {
        return;
    }
    _powered = on;
    reset_baselines(keep_samples);
    stop_all(on ? Reason::None : Reason::PowerOff);
}

bool Sequencer::running() const
{
    for (const auto& run : _runs) {
        if (run.total != 0) {
            return true;
        }
    }
    return false;
}

const Sequencer::Trigger& Sequencer::trigger(uint8_t target) const
{
    return target == shared_slot ? _doc->shared : _doc->lanes[target].trigger;
}

void Sequencer::reset_baselines(bool keep_samples)
{
    for (uint8_t n = 0; n <= shared_slot; n++) {
        auto& trigger_state = _triggers[n];
        trigger_state.valid = false;
        trigger_state.high = false;
        if (!keep_samples) {
            trigger_state.sample = {};
        }
        _status[n].trigger_valid = false;
    }
}

bool Sequencer::start(uint8_t target, uint32_t now)
{
    return start(target, now, UINT16_MAX);
}

bool Sequencer::start(uint8_t target, uint32_t now, uint16_t shared_lanes)
{
    if (!_powered || _gap || target > shared_slot ||
        (trigger(target).edge != Edge::Off && !_triggers[target].valid)) {
        return false;
    }
    if (_runs[target].total != 0) {
        return true;
    }
    uint32_t total = 0;
    bool repeat = false;
    if (target == shared_slot) {
        for (uint8_t lane = 0; lane < lane_count; lane++) {
            if ((shared_lanes & (1U << lane)) && _doc->lanes[lane].enabled &&
                _doc->lanes[lane].clock == Clock::Shared && _length[lane] != 0) {
                _participants |= 1U << lane;
                repeat |= _doc->lanes[lane].repeat != 0;
                if (_length[lane] > total) {
                    total = _length[lane];
                }
            }
        }
    } else if (_doc->lanes[target].enabled && _doc->lanes[target].clock == Clock::Independent) {
        total = _length[target];
        repeat = _doc->lanes[target].repeat != 0;
    }
    if (total == 0) {
        return false;
    }
    if (++_next_run == 0) {
        ++_next_run;
    }
    auto& run = _runs[target];
    run.start = now;
    run.total = total;
    run.repeat = repeat;
    _status[target].run = _next_run;
    _status[target].state = State::Running;
    _status[target].reason = Reason::None;
    if (target == shared_slot) {
        for (uint8_t lane = 0; lane < lane_count; lane++) {
            if (_participants & (1U << lane)) {
                _status[lane].run = _next_run;
                _status[lane].state = State::Running;
                _status[lane].reason = Reason::None;
                evaluate(lane, 0);
            }
        }
    } else {
        evaluate(target, 0);
    }
    ++_status_revision;
    return true;
}

void Sequencer::update_idle(uint8_t lane)
{
    auto& status = _status[lane];
    const bool available = _active && _doc != nullptr;
    if (lane == shared_slot) {
        if (!available) {
            status.state = State::Disabled;
        }
        return;
    }
    status.valid = available && _doc->lanes[lane].enabled;
    status.binary = _doc != nullptr && _doc->lanes[lane].output == Output::Binary;
    if (!status.valid) {
        status.state = State::Disabled;
    }
    status.logical = status.valid ? _doc->lanes[lane].idle : 0;
    status.pulse = status.valid ? (status.binary ? 1000 + 1000 * status.logical : status.logical) : 0;
}

void Sequencer::finish(uint8_t lane, State state, Reason reason)
{
    _runs[lane].total = 0;
    auto& status = _status[lane];
    status.state = state;
    status.reason = reason;
    status.block = 0;
    status.position = 0;
    status.block_elapsed = 0;
    status.duration = 0;
    status.elapsed = state == State::Complete && lane < lane_count ? _length[lane] : 0;
    status.holding = false;
    update_idle(lane);
    ++_status_revision;
}

void Sequencer::stop(uint8_t target, Reason reason)
{
    if (target > shared_slot || (target < lane_count && (_participants & (1U << target)))) {
        return;
    }
    if (reason == Reason::Stopped) {
        cancel_boot_starts(1UL << target);
    }
    if (target == shared_slot) {
        for (uint8_t lane = 0; lane < lane_count; lane++) {
            if ((_participants & (1U << lane)) ||
                (_doc != nullptr && _doc->lanes[lane].clock == Clock::Shared)) {
                finish(lane, State::Idle, reason);
            }
        }
        _participants = 0;
    }
    finish(target, State::Idle, reason);
}

void Sequencer::stop_all(Reason reason)
{
    stop(shared_slot, reason);
    for (uint8_t lane = 0; lane < lane_count; lane++) {
        if (_doc == nullptr || _doc->lanes[lane].clock == Clock::Independent) {
            stop(lane, reason);
        }
    }
}

void Sequencer::routes_changed(uint32_t triggers, uint32_t outputs)
{
    if (_doc == nullptr) {
        return;
    }
    for (uint8_t lane = 0; lane < lane_count; lane++) {
        if (!_doc->lanes[lane].enabled) {
            continue;
        }
        const uint32_t bit = 1UL << lane;
        if (_doc->lanes[lane].clock == Clock::Independent) {
            triggers |= outputs & bit;
        } else if ((outputs & bit) && _length[lane] != 0) {
            triggers |= 1UL << shared_slot;
        }
    }
    for (uint8_t n = 0; n <= shared_slot; n++) {
        if (triggers & (1UL << n)) {
            _triggers[n] = {};
            if (_runs[n].total != 0) {
                stop(n, Reason::RouteChanged);
            }
        }
    }
}

void Sequencer::sample(uint8_t target, Input input)
{
    _triggers[target].sample = input;
}

static bool valid_sample(Sequencer::Input input)
{
    return input.valid && input.pulse >= 800 && input.pulse <= 2200;
}

bool Sequencer::consume_edge(uint8_t target)
{
    auto& state = _triggers[target];
    const auto& config = trigger(target);
    const bool valid = config.edge != Edge::Off && valid_sample(state.sample);
    const bool high = state.sample.pulse >= config.low && state.sample.pulse <= config.high;
    const bool fire = state.valid && valid && state.high != high &&
        (high ? config.edge == Edge::Rising : config.edge == Edge::Falling);
    state.valid = valid;
    state.high = high;
    if (!valid && config.edge != Edge::Off && _runs[target].total != 0) {
        stop(target, Reason::InputLost);
    }
    return fire;
}

void Sequencer::evaluate(uint8_t lane, uint32_t elapsed)
{
    auto& status = _status[lane];
    const uint32_t length = _length[lane];
    status.elapsed = elapsed < length ? elapsed : length;
    status.holding = elapsed >= length;
    uint32_t t = status.elapsed;
    uint16_t value = _doc->lanes[lane].idle;
    for (uint8_t pos = 0; pos < per_lane; pos++) {
        const auto slot = _index[lane][pos];
        if (slot == 0) {
            break;
        }
        const auto& block = _doc->blocks[slot - 1];
        const bool last = pos + 1 == per_lane || _index[lane][pos + 1] == 0;
        if (t < block.duration || last) {
            status.block = slot;
            status.position = pos + 1;
            status.duration = block.duration;
            status.block_elapsed = t;
            if (block.type == Type::Ramp) {
                const int32_t delta = int32_t(block.to) - block.from;
                const int32_t scaled = delta * int32_t(t);
                const int32_t half = block.duration / 2;
                value = block.from + (scaled + (scaled < 0 ? -half : half)) / block.duration;
            } else if (block.type == Type::Set) {
                value = block.to;
            }
            break;
        }
        if (block.type != Type::Wait) {
            value = block.to;
        }
        t -= block.duration;
    }
    status.logical = value;
    status.pulse = status.binary ? 1000 + 1000 * value : value;
}

void Sequencer::tick(uint32_t now, uint32_t inhibit_starts)
{
    _gap = _seen_tick && now - _last_tick > max_gap_ms;
    _seen_tick = true;
    _last_tick = now;
    uint32_t boot_ready = 0;
    cancel_boot_starts(inhibit_starts);
    if (boot_start_pending() && now - _boot_start_ms >= autostart_window_ms) {
        cancel_boot_starts(UINT32_MAX);
    }
    if (boot_start_pending() && !_gap) {
        for (uint8_t n = 0; n <= shared_slot; n++) {
            const uint32_t bit = 1UL << n;
            if ((_boot_targets & bit) && (trigger(n).edge == Edge::Off || valid_sample(_triggers[n].sample))) {
                boot_ready |= bit;
            }
        }
        if (boot_ready != 0) {
            // Preserve the fresh inputs used to qualify these starts and
            // establish their baselines in the normal sampling pass.
            set_power(true, true);
            cancel_boot_starts(boot_ready);
        }
    }
    if (!_powered) {
        if (_gap && boot_start_pending()) {
            reset_baselines();
        }
        return;
    }
    if (_gap) {
        if (running()) {
            stop_all(Reason::LoopGap);
        }
        reset_baselines();
    }

    // Capture running membership before advancing so a simultaneous edge at
    // completion is ignored just like an edge during any other active tick.
    for (uint8_t n = 0; n <= shared_slot; n++) {
        const bool was_running = _runs[n].total != 0;
        const bool boot = (boot_ready & (1UL << n)) != 0;
        if ((consume_edge(n) || boot) && !was_running && !(inhibit_starts & (1UL << n))) {
            start(n, now, boot ? _boot_shared_lanes : UINT16_MAX);
        }
    }
    // Advance shared and independent clocks before evaluating any lane. Keep
    // the original phase when a tick falls just after a cycle boundary.
    uint32_t ended_cycles = 0;
    for (uint8_t n = 0; n <= shared_slot; n++) {
        auto& run = _runs[n];
        const uint32_t elapsed = now - run.start;
        if (run.total != 0 && elapsed >= run.total) {
            ended_cycles |= 1UL << n;
            if (run.repeat) {
                run.start = now - elapsed % run.total;
                ++_status_revision;
            }
        }
    }
    for (uint8_t n = 0; n <= shared_slot; n++) {
        const uint8_t target = n < lane_count && _doc->lanes[n].clock == Clock::Shared ?
            shared_slot : n;
        auto& status = _status[n];
        const bool trigger_valid = _triggers[target].valid;
        if (status.trigger_valid != trigger_valid) {
            status.trigger_valid = trigger_valid;
            ++_status_revision;
        }
        if (status.state == State::Idle || status.state == State::Armed) {
            const bool has_blocks = n == shared_slot ? _shared_length != 0 : _length[n] != 0;
            const auto state = has_blocks && trigger_valid ? State::Armed : State::Idle;
            if (state != status.state) {
                status.state = state;
                ++_status_revision;
            }
        }
        const auto& run = _runs[target];
        if (run.total == 0 || status.state != State::Running) {
            continue;
        }
        if ((ended_cycles & (1UL << target)) &&
            (!run.repeat || (n < lane_count && !_doc->lanes[n].repeat))) {
            finish(n, State::Complete, Reason::None);
            if (n == shared_slot) {
                _participants = 0;
            }
        } else if (n < lane_count) {
            evaluate(n, now - run.start);
        }
    }
}

} // namespace Vektor2
