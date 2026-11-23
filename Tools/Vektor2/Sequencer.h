#pragma once

#include <stdint.h>

#ifndef VEKTOR2_SEQ_LARGE
#define VEKTOR2_SEQ_LARGE 0
#endif

namespace Vektor2 {

class Sequencer {
public:
    static constexpr uint8_t lane_count = VEKTOR2_SEQ_LARGE ? 16 : 4;
    static constexpr uint8_t per_lane = VEKTOR2_SEQ_LARGE ? 32 : 16;
    static constexpr uint16_t block_count = VEKTOR2_SEQ_LARGE ? 256 : 48;
    static constexpr uint8_t shared_slot = lane_count;
    static constexpr uint32_t autostart_window_ms = 5000;
    static constexpr uint8_t name_size = 12;
    static constexpr uint32_t max_gap_ms = 100;
    static constexpr uint16_t min_duration_ms = 100;
    static constexpr uint16_t max_duration_ms = 60000;
    static constexpr uint8_t max_rate_hz = 10;
    static constexpr uint8_t default_rate_hz = 5;

    enum class State : uint8_t { Disabled, Idle, Armed, Running, Complete };
    enum class Reason : uint8_t {
        None, Stopped, PowerOff, InputLost, RouteChanged, LoopGap, Configuration, Deactivated
    };
    enum class Type : uint8_t { Set, Ramp, Wait };
    enum class Output : uint8_t { Servo, Binary };
    enum class Clock : uint8_t { Shared, Independent };
    enum class Edge : uint8_t { Off, Rising, Falling };

    struct Trigger {
        Trigger() = default;
        Trigger(uint16_t lower, uint16_t upper, Edge which) : low(lower), high(upper), edge(which) {}
        uint16_t low = 1500;
        uint16_t high = 2000;
        Edge edge = Edge::Off;
    };
    struct Lane {
        char name[name_size] {};
        Trigger trigger;
        uint16_t idle = 1500;
        uint8_t enabled = 0;
        uint8_t autostart = 0;
        uint8_t repeat = 0;
        Output output = Output::Servo;
        Clock clock = Clock::Shared;
    };
    struct Block {
        Block() = default;
        Block(uint16_t ms, uint16_t start, uint16_t end, uint8_t owner, uint8_t order, Type kind) :
            duration(ms), from(start), to(end), lane(owner), position(order), type(kind) {}
        uint16_t duration = 0;
        uint16_t from = 0;
        uint16_t to = 0;
        uint8_t lane = 0;
        uint8_t position = 0;
        Type type = Type::Set;
    };
    struct Document {
        Lane lanes[lane_count];
        Block blocks[block_count];
        Trigger shared;
        uint32_t revision = 0;
        uint8_t rate = default_rate_hz;
        bool valid() const;
    };
    struct Input {
        Input() = default;
        Input(uint16_t sample, bool available) : pulse(sample), valid(available) {}
        uint16_t pulse = 0;
        bool valid = false;
    };
    struct Status {
        uint32_t run = 0;
        uint32_t elapsed = 0;
        uint16_t block = 0;
        uint16_t block_elapsed = 0;
        uint16_t duration = 0;
        uint16_t logical = 0;
        uint16_t pulse = 0;
        uint8_t position = 0;
        State state = State::Disabled;
        Reason reason = Reason::Configuration;
        bool holding = false;
        bool valid = false;
        bool trigger_valid = false;
        bool binary = false;
    };

    void load(const Document* document);
    void schedule_boot_start(uint32_t now, uint32_t inhibit_starts = 0);
    void activate(bool enabled);
    bool power(bool on);
    bool start(uint8_t target, uint32_t now);
    void stop(uint8_t target, Reason reason = Reason::Stopped);
    void stop_all(Reason reason = Reason::Stopped);
    void routes_changed(uint32_t triggers, uint32_t outputs = 0);
    void sample(uint8_t target, Input input);
    void tick(uint32_t now, uint32_t inhibit_starts = 0);

    bool active() const { return _active; }
    bool powered() const { return _powered; }
    bool boot_start_pending() const { return _boot_targets != 0; }
    bool running() const;
    const Status& status(uint8_t lane) const { return _status[lane]; }
    const Status& shared_status() const { return _status[shared_slot]; }
    uint32_t status_revision() const { return _status_revision; }

private:
    struct Run {
        uint32_t start = 0;
        uint32_t total = 0;
        bool repeat = false;
    };
    struct TriggerState {
        Input sample;
        bool valid = false;
        bool high = false;
    };

    const Trigger& trigger(uint8_t target) const;
    void cancel_boot_starts(uint32_t targets);
    void set_power(bool on, bool keep_samples = false);
    void reset_baselines(bool keep_samples = false);
    bool start(uint8_t target, uint32_t now, uint16_t shared_lanes);
    void finish(uint8_t lane, State state, Reason reason);
    void evaluate(uint8_t lane, uint32_t elapsed);
    bool consume_edge(uint8_t target);
    void update_idle(uint8_t lane);

    const Document* _doc = nullptr;
    uint16_t _index[lane_count][per_lane] {}; // One-based block-pool slot
    uint32_t _length[lane_count] {};
    uint16_t _participants = 0;
    uint32_t _shared_length = 0;
    Run _runs[lane_count + 1];
    TriggerState _triggers[lane_count + 1];
    Status _status[lane_count + 1];
    uint32_t _last_tick = 0;
    uint32_t _next_run = 0;
    uint32_t _status_revision = 0;
    uint32_t _boot_start_ms = 0;
    uint32_t _boot_targets = 0;
    uint16_t _boot_shared_lanes = 0;
    bool _seen_tick = false;
    bool _gap = false;
    bool _active = false;
    bool _powered = false;
};

} // namespace Vektor2
