#include "NavigationSupervisor.h"
#include <cassert>
#include <cstdint>
#include <iostream>

using N = Vektor2::NavigationSupervisor;

int main()
{
    N nav;
    N::Input in;
    in.rc_valid = true;
    nav.update(in);
    assert(nav.status().state == N::State::Free && nav.status().reason_bits == 0);

    in.requested_mode = 1;
    nav.update(in);
    assert(nav.status().state == N::State::HeadingWait);
    assert(nav.status().reason_bits & N::Disabled);
    assert(nav.status().reason_bits & N::OutputNotReady);

    in.enabled = in.heading_enabled = in.heading_valid = true;
    in.heading_command_valid = true;
    in.heading_target_valid = in.output_ready = in.yaw_owned = true;
    nav.update(in);
    assert(nav.status().state == N::State::HeadingActive);
    assert(nav.status().reason_bits == 0);

    // A good heading estimate can coexist with an unusable yaw command.
    in.heading_command_valid = false;
    in.configuration_valid = false;
    nav.update(in);
    assert(nav.status().state == N::State::HeadingWait);
    assert(nav.status().reason_bits == (N::ConfigInvalid | N::OutputNotReady));
    in.configuration_valid = true;
    nav.update(in);
    assert(nav.status().state == N::State::HeadingWait);
    assert(nav.status().reason_bits == N::OutputNotReady);
    in.heading_command_valid = true;

    in.vsp_unavailable=true; nav.update(in); assert(nav.status().reason_bits & N::VspUnavailable);
    in.vsp_unavailable=false;
    in.requested_mode = 2;
    nav.update(in);
    assert(nav.status().state == N::State::DPWait);
    assert(nav.status().reason_bits & N::Disabled);
    assert(nav.status().reason_bits & N::PositionInvalid);
    assert(nav.status().reason_bits & N::VelocityInvalid);

    in.dp_enabled = in.position_valid = in.velocity_valid = true;
    in.position_target_valid = in.translation_owned = true;
    nav.update(in);
    assert(nav.status().state == N::State::DPActive);
    assert(nav.status().reason_bits == 0);

    in.position_valid = false;
    nav.update(in);
    assert(nav.status().state == N::State::DPWait);
    assert(nav.status().reason_bits == N::PositionInvalid);
    in.heading_valid = false;
    nav.update(in);
    assert(nav.status().state == N::State::HeadingWait);
    assert(nav.status().reason_bits & N::HeadingInvalid);

    in.heading_valid = in.position_valid = true;
    in.yaw_owned = false;
    nav.update(in);
    assert(nav.status().state == N::State::HeadingWait);
    assert(nav.status().reason_bits & N::OutputNotReady);
    in.yaw_owned = true;
    in.translation_owned = false;
    nav.update(in);
    assert(nav.status().state == N::State::DPWait);
    assert(nav.status().reason_bits & N::OutputNotReady);

    // A DP-only controller fault blocks translation, not shared Heading Hold.
    in.translation_owned = true;
    in.dp_controller_reasons = N::ConfigInvalid | N::VelocityInvalid;
    nav.update(in);
    assert(nav.status().state == N::State::DPWait);
    assert((nav.status().reason_bits & (N::ConfigInvalid | N::VelocityInvalid)) ==
           (N::ConfigInvalid | N::VelocityInvalid));
    in.requested_mode = 1;
    nav.update(in);
    assert(nav.status().state == N::State::HeadingActive);
    assert((nav.status().reason_bits & (N::ConfigInvalid | N::VelocityInvalid)) == 0);
    in.requested_mode = 2;
    in.dp_controller_reasons = 0;
    nav.update(in);
    assert(nav.status().state == N::State::DPActive);

    in.pilot_override = true;
    in.rc_valid = false;
    nav.update(in);
    assert(nav.status().state == N::State::HeadingWait);
    assert(nav.status().reason_bits & N::PilotOverride);
    assert(nav.status().reason_bits & N::RCInvalid);
    in.rc_valid = true;
    in.pilot_override = false;
    in.estimator_reset = true;
    nav.update(in);
    assert(nav.status().reason_bits & N::EstimatorReset);

    in.estimator_reset = false;
    in.fault = true;
    nav.update(in);
    assert(nav.status().state == N::State::Fault);
    in.fault = false;
    in.requested_mode = 0;
    in.heading_valid = in.position_valid = in.velocity_valid = false;
    nav.update(in);
    assert(nav.status().state == N::State::Free);
    assert(!(nav.status().reason_bits & (N::HeadingInvalid | N::PositionInvalid | N::VelocityInvalid)));

    in.requested_mode = 3;
    nav.update(in);
    assert(nav.status().state == N::State::Free);
    assert(nav.status().reason_bits & N::ConfigInvalid);

    // Observation-only integration: requested DP cannot claim active control.
    in = N::Input{};
    in.requested_mode = 2;
    in.enabled = in.heading_enabled = in.dp_enabled = true;
    in.heading_valid = in.position_valid = in.velocity_valid = in.rc_valid = true;
    nav.update(in);
    assert(nav.status().state == N::State::HeadingWait);
    assert(nav.status().reason_bits & N::OutputNotReady);

    std::cout << "Navigation supervisor tests passed\n";
}
