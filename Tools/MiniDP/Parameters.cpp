#include "MiniDP.h"

const AP_Param::Info MiniDP::var_info[] = {
    // @Param: FORMAT_VERSION
    // @DisplayName: MiniDP parameter storage format
    // @Description: Internal MiniDP storage format marker. MiniDP resets parameters when the expected format changes.
    // @User: Advanced
    GSCALAR(format_version, "FORMAT_VERSION", 0),

    // @Param: LOG_BITMASK
    // @DisplayName: MiniDP log bitmask
    // @Description: Controls MiniDP standard and vehicle-specific DataFlash logging. Set to -1 to enable all useful logs.
    // @Bitmask: 0:Attitude AHRS EKF,1:GPS,2:Battery and power,3:IMU and vibration,4:Compass,5:RC input and servo output,6:MiniDP controller output and status
    // @User: Standard
    GSCALAR(log_bitmask, "LOG_BITMASK", -1),

    GOBJECT(board_config, "BRD_", AP_BoardConfig),
#if HAL_CANMANAGER_ENABLED
    // @Group: CAN_
    // @Path: ../libraries/AP_CANManager/AP_CANManager.cpp
    GOBJECT(can_mgr, "CAN_", AP_CANManager),
#endif
    GOBJECT(serial_manager, "SERIAL", AP_SerialManager),
    GOBJECT(ins, "INS", AP_InertialSensor),
    GOBJECT(compass, "COMPASS_", Compass),
    GOBJECT(gps, "GPS", AP_GPS),
#if AP_BATTERY_ENABLED
    GOBJECT(battery, "BATT", AP_BattMonitor),
#endif
#if AP_RSSI_ENABLED
    // @Group: RSSI_
    // @Path: ../libraries/AP_RSSI/AP_RSSI.cpp
    GOBJECT(rssi, "RSSI_", AP_RSSI),
#endif
    // @Group: SCHED_
    // @Path: ../libraries/AP_Scheduler/AP_Scheduler.cpp
    GOBJECT(scheduler, "SCHED_", AP_Scheduler),
    GOBJECT(logger, "LOG", AP_Logger),
    GOBJECT(notify, "NTF_", AP_Notify),
    GOBJECT(rc_channels, "RC", RC_Channels),
    GOBJECT(servo_channels, "SERVO", SRV_Channels),
#if HAL_GCS_ENABLED
    GOBJECT(gcs_backend, "MAV", GCS),
#endif
#if MINIDP_BARO_ENABLED
    GOBJECT(barometer, "BARO", AP_Baro),
#endif
    GOBJECT(ahrs, "AHRS_", AP_AHRS),
#if HAL_NAVEKF2_AVAILABLE
    GOBJECTN(ahrs.EKF2, NavEKF2, "EK2_", NavEKF2),
#endif
#if HAL_NAVEKF3_AVAILABLE
    GOBJECTN(ahrs.EKF3, NavEKF3, "EK3_", NavEKF3),
#endif

    // @Param: AUTH_MAV_MAN
    // @DisplayName: Allow MAVLink manual control authority
    // @Description: Allows MAVLink MANUAL_CONTROL input to own MiniDP manual control when MAN_ENABLE is also enabled.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(auth_mav_manual, "AUTH_MAV_MAN", 0),

    // @Param: AUTH_MAV_TGT
    // @DisplayName: Allow MAVLink target authority
    // @Description: Allows MAVLink mode requests to own MiniDP target modes including HEADING_HOLD and DP_HOLD.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(auth_mav_target, "AUTH_MAV_TGT", 0),

    // @Param: AUTH_TEST
    // @DisplayName: Allow actuator test authority
    // @Description: Allows MAVLink motor test commands to enter MiniDP actuator test mode.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(auth_test, "AUTH_TEST", 0),

    // @Param: AUTH_RC_TAKE
    // @DisplayName: Allow RC takeover
    // @Description: Allows healthy RC manual input to take over from MAVLink manual or target authority.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(auth_rc_takeover, "AUTH_RC_TAKE", 0),

    // @Param: AUTH_RC_FB
    // @DisplayName: RC fallback on MAVLink loss
    // @Description: Falls back to healthy RC authority when MAVLink manual or target authority is lost.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(auth_rc_fallback, "AUTH_RC_FB", 1),

    // @Param: AUTH_MAV_FB
    // @DisplayName: MAVLink fallback on RC loss
    // @Description: Allows MAVLink manual authority to take over when RC authority is lost and MAVLink manual input is healthy.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(auth_mav_fallback, "AUTH_MAV_FB", 0),

    // @Param: AUTH_TGT_LOSS
    // @DisplayName: Target link loss action
    // @Description: Action when MAVLink target authority loses its link while MiniDP is in a target mode.
    // @Values: 0:Failsafe,1:Hold last target,2:Fall back to RC
    // @User: Standard
    GSCALAR(auth_target_loss, "AUTH_TGT_LOSS", 0),

    // @Param: AUTH_RC_TMO
    // @DisplayName: RC health timeout
    // @Description: RC input is unhealthy if no fresh RC input arrives within this time.
    // @Units: s
    // @Range: 0.05 5
    // @Increment: 0.05
    // @User: Standard
    GSCALAR(auth_rc_timeout, "AUTH_RC_TMO", 0.5f),

    // @Param: AUTH_MAV_TMO
    // @DisplayName: MAVLink health timeout
    // @Description: MAVLink is unhealthy if no GCS heartbeat or valid GCS traffic arrives within this time.
    // @Units: s
    // @Range: 0.1 30
    // @Increment: 0.1
    // @User: Standard
    GSCALAR(auth_mav_timeout, "AUTH_MAV_TMO", 3.0f),

    // @Param: FRAME_TYPE
    // @DisplayName: MiniDP frame type
    // @Description: Selects the MiniDP output mixer frame. Frame 901 is OMNI_PLUS: port and starboard propulsion screws plus bow and stern tunnel thrusters. Frame 902 is DUAL_AZ_180_BOW: aft port 180-degree azimuth pod on Motor1 thrust and Motor2 azimuth, aft starboard 180-degree azimuth pod on Motor3 thrust and Motor4 azimuth, and bow thruster on Motor5.
    // @Values: 901:OMNI_PLUS,902:DUAL_AZ_180_BOW
    // @User: Standard
    GSCALAR(frame_type, "FRAME_TYPE", MiniDP_OutputManager::default_frame_type),

    // @Param: FRAME_SCR_POS
    // @DisplayName: Propulsion screw position
    // @Description: Position of the Motor1 and Motor2 propulsion screws relative to the yaw center for frame 901. Aft preserves the default differential screw yaw sign, center removes screw yaw contribution, and forward flips the screw yaw sign.
    // @Values: -1:Aft,0:Center no yaw,1:Forward
    // @User: Standard
    GSCALAR(frame_screw_position, "FRAME_SCR_POS", int8_t(MiniDP_ScrewPosition::AFT)),

    // @Param: FRAME_SCR_YAW
    // @DisplayName: Propulsion screw yaw scale
    // @Description: Scales the Motor1 and Motor2 differential yaw contribution for frame 901.
    // @Range: 0 2
    // @Increment: 0.05
    // @User: Standard
    GSCALAR(frame_screw_yaw_scale, "FRAME_SCR_YAW", 1.0f),

    // @Param: IN_RC_SURGE
    // @DisplayName: RC surge channel
    // @Description: One-based RC input channel for manual surge command. Set to 0 to disable this axis.
    // @Range: 0 16
    // @User: Standard
    GSCALAR(in_rc_surge_channel, "IN_RC_SURGE", 2),

    // @Param: IN_RC_SWAY
    // @DisplayName: RC sway channel
    // @Description: One-based RC input channel for manual sway command. Set to 0 to disable this axis.
    // @Range: 0 16
    // @User: Standard
    GSCALAR(in_rc_sway_channel, "IN_RC_SWAY", 1),

    // @Param: IN_RC_YAW
    // @DisplayName: RC yaw channel
    // @Description: One-based RC input channel for manual yaw command. Set to 0 to disable this axis.
    // @Range: 0 16
    // @User: Standard
    GSCALAR(in_rc_yaw_channel, "IN_RC_YAW", 4),

    // @Param: IN_RC_KILL
    // @DisplayName: RC kill channel
    // @Description: One-based RC input channel for output kill. Set to 0 to disable RC kill.
    // @Range: 0 16
    // @User: Standard
    GSCALAR(in_rc_kill_channel, "IN_RC_KILL", 0),

    // @Param: IN_KILL_PWM
    // @DisplayName: RC kill threshold PWM
    // @Description: RC kill is active when IN_RC_KILL is at or above this PWM.
    // @Units: PWM
    // @Range: 800 2200
    // @Increment: 1
    // @User: Standard
    GSCALAR(in_rc_kill_pwm, "IN_KILL_PWM", 1800),

    // @Param: IN_RC_ARM
    // @DisplayName: RC arm switch channel
    // @Description: One-based RC input channel for switch arming. Set to 0 to disable RC switch arming.
    // @Range: 0 16
    // @User: Standard
    GSCALAR(in_rc_arm_channel, "IN_RC_ARM", 0),

    // @Param: IN_ARM_PWM
    // @DisplayName: RC arm threshold PWM
    // @Description: RC arm switch arms MiniDP on a rising edge when IN_RC_ARM is at or above this PWM.
    // @Units: PWM
    // @Range: 800 2200
    // @Increment: 1
    // @User: Standard
    GSCALAR(in_rc_arm_pwm, "IN_ARM_PWM", 1800),

    // @Param: IN_RC_DISARM
    // @DisplayName: RC disarm switch channel
    // @Description: One-based RC input channel for switch disarming. Set to 0 to disable RC switch disarming.
    // @Range: 0 16
    // @User: Standard
    GSCALAR(in_rc_disarm_channel, "IN_RC_DISARM", 0),

    // @Param: IN_DISARM_PWM
    // @DisplayName: RC disarm threshold PWM
    // @Description: RC disarm switch disarms MiniDP on a falling edge when IN_RC_DISARM is at or below this PWM.
    // @Units: PWM
    // @Range: 800 2200
    // @Increment: 1
    // @User: Standard
    GSCALAR(in_rc_disarm_pwm, "IN_DISARM_PWM", 1200),

    // @Param: MAN_ENABLE
    // @DisplayName: Enable manual control
    // @Description: Enables manual RC and authorized MAVLink manual output.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(manual_enable, "MAN_ENABLE", 0),

    // @Param: MAN_DZ
    // @DisplayName: Manual input deadband
    // @Description: Deadband applied to manual RC and MAVLink manual axes before manual limits.
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard
    GSCALAR(manual_deadband, "MAN_DZ", 0.03f),

    // @Param: MAN_SRG_LIM
    // @DisplayName: Manual surge limit
    // @Description: Maximum normalized manual surge output.
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard
    GSCALAR(manual_surge_limit, "MAN_SRG_LIM", 0.5f),

    // @Param: MAN_SWY_LIM
    // @DisplayName: Manual sway limit
    // @Description: Maximum normalized manual sway output.
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard
    GSCALAR(manual_sway_limit, "MAN_SWY_LIM", 0.5f),

    // @Param: MAN_YAW_LIM
    // @DisplayName: Manual yaw limit
    // @Description: Maximum normalized manual yaw output.
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard
    GSCALAR(manual_yaw_limit, "MAN_YAW_LIM", 0.5f),

    // @Param: MAN_MAV_TMO
    // @DisplayName: MAVLink manual timeout
    // @Description: Authorized MAVLink manual command times out after this many seconds without a fresh MANUAL_CONTROL message.
    // @Units: s
    // @Range: 0.1 5
    // @Increment: 0.05
    // @User: Standard
    GSCALAR(manual_mavlink_timeout, "MAN_MAV_TMO", 0.5f),

    // @Param: AXIS_DZ
    // @DisplayName: Output axis deadband
    // @Description: Deadband applied after manual or DP controller output and before frame mixing.
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard
    GSCALAR(axis_deadband, "AXIS_DZ", 0.0f),

    // @Param: AXIS_SRG_MAX
    // @DisplayName: Output surge limit
    // @Description: Final normalized surge output limit after manual or DP controller output.
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard
    GSCALAR(axis_surge_limit, "AXIS_SRG_MAX", 1.0f),

    // @Param: AXIS_SWY_MAX
    // @DisplayName: Output sway limit
    // @Description: Final normalized sway output limit after manual or DP controller output.
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard
    GSCALAR(axis_sway_limit, "AXIS_SWY_MAX", 1.0f),

    // @Param: AXIS_YAW_MAX
    // @DisplayName: Output yaw limit
    // @Description: Final normalized yaw output limit after manual or DP controller output.
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard
    GSCALAR(axis_yaw_limit, "AXIS_YAW_MAX", 1.0f),

    // @Param: AXIS_SRG_SLW
    // @DisplayName: Output surge slew rate
    // @Description: Maximum surge command change rate in normalized output per second. Set to 0 to disable slew limiting.
    // @Units: 1/s
    // @Range: 0 10
    // @Increment: 0.1
    // @User: Standard
    GSCALAR(axis_surge_slew, "AXIS_SRG_SLW", 0.0f),

    // @Param: AXIS_SWY_SLW
    // @DisplayName: Output sway slew rate
    // @Description: Maximum sway command change rate in normalized output per second. Set to 0 to disable slew limiting.
    // @Units: 1/s
    // @Range: 0 10
    // @Increment: 0.1
    // @User: Standard
    GSCALAR(axis_sway_slew, "AXIS_SWY_SLW", 0.0f),

    // @Param: AXIS_YAW_SLW
    // @DisplayName: Output yaw slew rate
    // @Description: Maximum yaw command change rate in normalized output per second. Set to 0 to disable slew limiting.
    // @Units: 1/s
    // @Range: 0 10
    // @Increment: 0.1
    // @User: Standard
    GSCALAR(axis_yaw_slew, "AXIS_YAW_SLW", 0.0f),

    // @Param: DP_YAW_P
    // @DisplayName: DP yaw proportional gain
    // @Description: Heading hold proportional gain. Multiplies yaw error in radians to produce normalized yaw output.
    // @Range: 0 5
    // @Increment: 0.01
    // @User: Standard
    GSCALAR(dp_yaw_p, "DP_YAW_P", 0.0f),

    // @Param: DP_YAW_I
    // @DisplayName: DP yaw integral gain
    // @Description: Heading hold integral gain. Integrates yaw error in radians over time to correct steady-state yaw bias.
    // @Range: 0 2
    // @Increment: 0.01
    // @User: Standard
    GSCALAR(dp_yaw_i, "DP_YAW_I", 0.0f),

    // @Param: DP_YAW_D
    // @DisplayName: DP yaw damping gain
    // @Description: Heading hold yaw-rate damping gain. Multiplies yaw rate in radians per second to reduce yaw output.
    // @Range: 0 5
    // @Increment: 0.01
    // @User: Standard
    GSCALAR(dp_yaw_d, "DP_YAW_D", 0.0f),

    // @Param: DP_POS_P
    // @DisplayName: DP position proportional gain
    // @Description: Position hold proportional gain. Multiplies body-frame position error in meters to produce surge and sway output.
    // @Range: 0 5
    // @Increment: 0.01
    // @User: Standard
    GSCALAR(dp_position_p, "DP_POS_P", 0.0f),

    // @Param: DP_POS_I
    // @DisplayName: DP position integral gain
    // @Description: Position hold integral gain. Integrates shaped north/east position error over time to correct steady wind or current bias.
    // @Range: 0 2
    // @Increment: 0.01
    // @User: Standard
    GSCALAR(dp_position_i, "DP_POS_I", 0.0f),

    // @Param: DP_VEL_D
    // @DisplayName: DP velocity damping gain
    // @Description: Position hold velocity damping gain. Multiplies body-frame velocity in meters per second to reduce surge and sway output.
    // @Range: 0 5
    // @Increment: 0.01
    // @User: Standard
    GSCALAR(dp_velocity_d, "DP_VEL_D", 0.0f),

    // @Param: DP_SRG_MAX
    // @DisplayName: DP surge limit
    // @Description: Maximum normalized surge output from the DP controller before the final AXIS limits.
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard
    GSCALAR(dp_surge_limit, "DP_SRG_MAX", 0.5f),

    // @Param: DP_SWY_MAX
    // @DisplayName: DP sway limit
    // @Description: Maximum normalized sway output from the DP controller before the final AXIS limits.
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard
    GSCALAR(dp_sway_limit, "DP_SWY_MAX", 0.5f),

    // @Param: DP_YAW_MAX
    // @DisplayName: DP yaw limit
    // @Description: Maximum normalized yaw output from the DP controller before the final AXIS limits.
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard
    GSCALAR(dp_yaw_limit, "DP_YAW_MAX", 0.5f),

    // @Param: DP_YAW_IMAX
    // @DisplayName: DP yaw integral limit
    // @Description: Maximum normalized yaw output that the yaw integral term may add. Set to 0 to disable yaw integral authority even if DP_YAW_I is nonzero.
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard
    GSCALAR(dp_yaw_imax, "DP_YAW_IMAX", 0.2f),

    // @Param: DP_POS_IMAX
    // @DisplayName: DP position integral limit
    // @Description: Maximum normalized surge/sway vector output that the position integral term may add. Set to 0 to disable position integral authority even if DP_POS_I is nonzero.
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard
    GSCALAR(dp_position_imax, "DP_POS_IMAX", 0.2f),

    // @Param: DP_RETARGET
    // @DisplayName: Relatch DP target on mode request
    // @Description: Relatches current position and heading as the target whenever DP_HOLD is requested.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(dp_retarget, "DP_RETARGET", 1),

    // @Param: DP_POS_RAD
    // @DisplayName: DP soft position radius
    // @Description: Position error is gently shaped inside this radius before entering the position controller.
    // @Units: m
    // @Range: 0 50
    // @Increment: 0.1
    // @User: Standard
    GSCALAR(dp_position_radius, "DP_POS_RAD", 2.0f),

    // @Param: DP_POS_DZ
    // @DisplayName: DP position deadband
    // @Description: Position correction is zero inside this distance from the DP target.
    // @Units: m
    // @Range: 0 10
    // @Increment: 0.1
    // @User: Standard
    GSCALAR(dp_position_deadband, "DP_POS_DZ", 0.5f),

    // @Param: DP_HACC_MAX
    // @DisplayName: DP maximum GPS horizontal accuracy
    // @Description: Maximum GPS horizontal accuracy accepted for entering or continuing DP_HOLD. Set to 0 to disable this check.
    // @Units: m
    // @Range: 0 100
    // @Increment: 0.1
    // @User: Standard
    GSCALAR(dp_hacc_max, "DP_HACC_MAX", 10.0f),

    // @Param: DP_SACC_MAX
    // @DisplayName: DP maximum GPS speed accuracy
    // @Description: Maximum GPS speed accuracy accepted for entering or continuing DP_HOLD. Set to 0 to disable this check.
    // @Units: m/s
    // @Range: 0 20
    // @Increment: 0.1
    // @User: Standard
    GSCALAR(dp_sacc_max, "DP_SACC_MAX", 2.0f),

    // @Param: ARMING_REQUIRE
    // @DisplayName: Require arming for active outputs
    // @Description: Requires MiniDP to be armed before active motor output. Hardware safety can still block outputs.
    // @Values: 0:Not required,1:Required
    // @User: Standard
    GSCALAR(arm_require, "ARMING_REQUIRE", 1),

    // @Param: ARM_GPS_REQ
    // @DisplayName: GPS arming requirement
    // @Description: GPS and EKF requirement for arming MiniDP.
    // @Values: 0:None,1:GPS fix and accuracy,2:DP-ready EKF and GPS accuracy
    // @User: Standard
    GSCALAR(arm_gps_require, "ARM_GPS_REQ", 0),

    // @Param: ARM_HACC_MAX
    // @DisplayName: Arming maximum GPS horizontal accuracy
    // @Description: Maximum GPS horizontal accuracy accepted for GPS-required arming. Set to 0 to disable this accuracy check.
    // @Units: m
    // @Range: 0 100
    // @Increment: 0.1
    // @User: Standard
    GSCALAR(arm_hacc_max, "ARM_HACC_MAX", 10.0f),

    // @Param: ARM_SACC_MAX
    // @DisplayName: Arming maximum GPS speed accuracy
    // @Description: Maximum GPS speed accuracy accepted for GPS-required arming. Set to 0 to disable this accuracy check.
    // @Units: m/s
    // @Range: 0 20
    // @Increment: 0.1
    // @User: Standard
    GSCALAR(arm_sacc_max, "ARM_SACC_MAX", 2.0f),

    // @Param: OUT_DARM_ACT
    // @DisplayName: Disarmed output action
    // @Description: Output behavior for managed MiniDP motor channels when disarmed.
    // @Values: 0:Disable PWM,1:Send neutral trim,2:Send OUT_FS_PWM
    // @User: Standard
    GSCALAR(out_disarmed_action, "OUT_DARM_ACT", 0),

    // @Param: OUT_FS_ACT
    // @DisplayName: Failsafe output action
    // @Description: Output behavior for managed MiniDP motor channels in failsafe.
    // @Values: 0:Disable PWM,1:Send neutral trim,2:Send OUT_FS_PWM
    // @User: Standard
    GSCALAR(out_failsafe_action, "OUT_FS_ACT", 1),

    // @Param: OUT_KILL_ACT
    // @DisplayName: RC kill output action
    // @Description: Output behavior for managed MiniDP motor channels when RC kill is active.
    // @Values: 0:Disable PWM,1:Send neutral trim,2:Send OUT_FS_PWM
    // @User: Standard
    GSCALAR(out_kill_action, "OUT_KILL_ACT", 0),

    // @Param: OUT_FS_PWM
    // @DisplayName: Failsafe PWM
    // @Description: PWM used when an output safe action is set to send failsafe PWM. Set to 0 to use each motor channel trim.
    // @Units: PWM
    // @Range: 0 2200
    // @Increment: 1
    // @User: Standard
    GSCALAR(out_failsafe_pwm, "OUT_FS_PWM", 1500),

    AP_VAREND
};
