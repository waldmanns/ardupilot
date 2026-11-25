#pragma once

#include "ComponentParameters.h"
#include "FunctionParameters.h"
#include "MainPropulsion.h"
#include "VspBackend.h"
#include "AzipodControl.h"

#include <AP_Param/AP_Param.h>
#include <stdint.h>

namespace Vektor2 {

using ComponentLoopFn = void (*)(const uint16_t* inputs,
                                 uint16_t* outputs,
                                 const void* parameter_store,
                                 RollStabilization* stabilization);

struct ComponentDefinition {
    ComponentId id;
    const char* name;
    const char* prefix;
    uint8_t input_count;
    uint8_t output_count;
    ComponentLoopFn loop;
};

class Logic {
public:
    static constexpr uint8_t component_count = uint8_t(ComponentId::Count);
    static constexpr uint8_t max_inputs = 3;
    static constexpr uint8_t max_outputs = 3;

    static const AP_Param::GroupInfo var_info[];

    // Called after AP_Param::load_all(), before route construction or PWM.
    void initialize_main_propulsion();
    MainPropulsionType main_propulsion_type() const { return _main.type(); }
    bool vsp_selected() const { return _main.vsp() != nullptr; }
    bool component_selected(ComponentId id) const { return Vektor2::component_selected(_main.type(), azipod_layout(), id); }
    AzipodLayout azipod_layout() const { const auto* a = _main.azi(); return a ? a->layout : AzipodLayout::Unspecified; }
    const VspBackend* vsp_backend() const { return _main.vsp(); }
    const Components::AzipodControl* azipod_controller(ComponentId id) const {
        const auto* a = _main.azi();
        return a && is_azipod(id) && component_selected(id) ? &a->drive(runtime_slot(id)) : nullptr;
    }
    RollStabilization* roll_controller() { auto* v = _main.vsp(); return v ? v->vrs.controller() : nullptr; }
    void begin_cycle(const VspRpmGovernor::Feedback (&shaft_rpm)[2]);
    void set_azipod_routes(uint8_t ready, bool rc_valid) { _azi_routes = ready; _azi_rc_valid = rc_valid; }
    uint16_t azipod_motor_stop(ComponentId id) const { return _main.azi()->drive(runtime_slot(id)).motor_stop(); }
    void azipod_output_failed(ComponentId id) { _main.azi()->drive(runtime_slot(id)).output_failed(); }
    void set_thruster_routes(uint8_t mask, uint32_t revision);
    const ThrusterControl& thruster(uint8_t n) const { return _thrusters[n]; }
    void set_thruster_output_reasons(uint8_t n, uint16_t reasons) { _thrusters[n].set_output_reasons(reasons); }
    void apply_thruster_heading(uint8_t n, float command);
    void release_thruster_heading(uint8_t n);
    void run(ComponentId id);
    void calibrate_azipod(ComponentId id, bool selected, uint16_t steering, int8_t motor_percent);

    bool set_input(ComponentId id, uint8_t port, uint16_t value_us, bool source_valid = true);
    uint16_t input(ComponentId id, uint8_t port) const;
    uint16_t output(ComponentId id, uint8_t port) const;
    bool output_valid(ComponentId id) const;
    bool output_valid(ComponentId id, uint8_t port) const;
    bool input_valid(ComponentId id, uint8_t port) const;

    static const ComponentDefinition& definition(ComponentId id);
    static bool valid(ComponentId id)
    {
        return uint8_t(id) < component_count;
    }

    // Persistent parameters belong to the framework, not to VSP1.cpp/VSP2.cpp.
    // The AP_Param prefixes are provided by Logic::var_info.
    VSP1ParameterStore vsp1_params;
    VSP2ParameterStore vsp2_params;
    ThrusterBowParameterStore thruster_bow_params;
    ThrusterSternParameterStore thruster_stern_params;
    AP_Int16 vsp_conf;
    AP_Int16 vsp_tel_hz;
    AP_Int16 thr_tel_hz;
    RollStabilizationParameters vrs_params;
    AP_Int8 main_prop_type;
    AP_Int8 azi_conf;
    AP_Int8 azi_tel_hz;
    AzipodParameterStore azi1_params;
    AzipodParameterStore azi2_params;
    AP_Int8 vrpm_enable;
    AP_Float vrpm_low;
    AP_Float vrpm_high;
    AP_Float vrpm_deadband;
    AP_Float vrpm_fast;
    AP_Float vrpm_slow;
    AP_Float vrpm_near;
    AP_Int16 vrpm_start_us;
    AP_Float vrpm_start_timeout;
    FunctionParameters vfn_params;
    Functions vfn;

    // Runtime rod ownership/contribution flags, independent of RPM ownership.
    enum VspAuthority : uint8_t { Manual = 1, HeadingAssist = 2, RollAssist = 4, NeutralGuard = 8, DPAssist = 16 };
    uint8_t vsp_authority(uint8_t drive) const { const auto* v = _main.vsp(); return v && drive < 2 ? v->authority[drive] : 0; }
    uint16_t vsp_rpm_status(uint8_t drive) const { const auto* v = _main.vsp(); return v && drive < 2 ? v->rpm_governors[drive].status() : 0; }

    VspAllocator* vsp_allocator() { auto* v=_main.vsp(); return v ? &v->allocator : nullptr; }
    uint16_t vsp_alloc_status() const { const auto* v=_main.vsp(); return v ? v->allocator.status() : 0; }
    bool vsp_pair_ready() const;
    bool vsp_centered() const;
    VspMapper::Vector vsp_pilot() const;
    VspMapper::Vector vsp_commanded(uint8_t n) const { return _main.vsp()->vectors[n].commanded(); }
    uint32_t vsp_revision(uint8_t n) const { return _main.vsp()->vectors[n].revision(); }
    bool apply_vsp_allocation(const VspAllocator::Result& result);
    void neutral_vsp_allocation(bool bow);
    void confirm_vsp_allocation(bool dp);
    bool roll_enabled() const { const auto* v = _main.vsp(); return v && v->roll_enabled; }
    VspVectorControl::State vector_state(uint8_t drive) const { const auto* v = _main.vsp(); return v && drive < 2 ? v->vectors[drive].state() : VspVectorControl::State::Invalid; }

    uint16_t vsp_configuration() const
    {
        return vsp_selected() ? vsp_parameter_value(vsp_conf.get()) : 0;
    }

private:
    struct Runtime {
        uint16_t inputs[max_inputs] {};
        uint16_t outputs[max_outputs] {};
        bool sources_valid[max_inputs] {};
    };

    const void* parameter_store(ComponentId id) const;

    uint8_t runtime_slot(ComponentId id) const { return component_runtime_slot(_main.type(), azipod_layout(), id); }
    Runtime _runtime[runtime_slot_count] {};
    MainPropulsionStorage<VspBackend, AzipodBackend> _main;
    ThrusterControl _thrusters[2];
    void record_vsp_authority(uint8_t drive);
    uint8_t _azi_routes = 0;
    bool _azi_rc_valid = false;
    uint8_t _thruster_routes = 0;
    uint32_t _thruster_route_revision = 0;
    bool prepare_vsp(uint8_t drive);
    void finish_vsp(uint8_t drive, bool vector_owned);
};

} // namespace Vektor2
