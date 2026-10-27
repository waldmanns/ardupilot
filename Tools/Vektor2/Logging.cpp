#include "Vektor2.h"

namespace Vektor2 {
void App::init_logging()
{
    static const LogStructure structures[] = {LOG_COMMON_STRUCTURES};
    logger.init(log_bitmask, structures, ARRAY_SIZE(structures));
    // Never fake an armed vehicle to enable recording: downloads must remain
    // available, and Vektor has no vehicle arming architecture.
    logger.set_vehicle_armed(false);
}

void App::logging_task()
{
    storage.report_changes();
    if (storage.formatting() || !logger.log_while_disarmed()) {
        return;
    }
    storage.recording_active();
    logger.periodic_tasks();
    const int16_t rate = log_rate_hz.get();
    const uint32_t now_ms = AP_HAL::millis();
    if (rate <= 0 || now_ms - _last_log_ms < 1000U / MIN(rate, 50)) {
        return;
    }
    _last_log_ms = now_ms;
    const uint64_t time_us = AP_HAL::micros64();
    if (logger.should_log(1U << 0)) {
        nav_filter_status status{};
        const bool valid = ahrs.get_filter_status(status) && status.flags.attitude;
        logger.WriteStreaming("VATT", "TimeUS,Roll,Pitch,Yaw,Valid", "QfffB", time_us,
                              ahrs.get_roll_deg(), ahrs.get_pitch_deg(), ahrs.get_yaw_deg(), valid);
        const auto& accel = ins.get_accel();
        const auto& gyro = ins.get_gyro();
        logger.WriteStreaming("VIMU", "TimeUS,AX,AY,AZ,GX,GY,GZ,Healthy", "QffffffB", time_us,
                              accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z, ins.healthy());
    }
    if (logger.should_log(1U << 1)) {
        const auto& raw = gps.location();
        logger.WriteStreaming("VGPS", "TimeUS,Status,Sats,Lat,Lng,Alt,Speed,Course,HDop", "QBBLLffff",
                              time_us, uint8_t(gps.status()), gps.num_sats(), raw.lat, raw.lng,
                              raw.alt * 0.01f, gps.ground_speed(), gps.ground_course_cd() * 0.01f,
                              gps.get_hdop() * 0.01f);
        Location location;
        Vector3f velocity;
        const bool position_valid = ahrs.get_location(location);
        const bool velocity_valid = ahrs.get_velocity_NED(velocity);
        logger.WriteStreaming("VPOS", "TimeUS,Lat,Lng,Alt,VN,VE,VD,PosOK,VelOK", "QLLffffBB",
                              time_us, position_valid ? location.lat : 0,
                              position_valid ? location.lng : 0,
                              position_valid ? location.alt * 0.01f : NAN,
                              velocity_valid ? velocity.x : NAN, velocity_valid ? velocity.y : NAN,
                              velocity_valid ? velocity.z : NAN, position_valid, velocity_valid);
    }
    if (logger.should_log(1U << 2)) {
        for (uint8_t n = 0; n < 2; n++) {
            const auto id = ComponentId(n);
            logger.WriteStreaming("VVSP", "TimeUS,Drive,InX,InY,InRPM,OutX,OutY,OutRPM", "QBHHHHHH",
                                  time_us, n + 1, logic.input(id, 0), logic.input(id, 1), logic.input(id, 2),
                                  logic.output(id, 0), logic.output(id, 1), logic.output(id, 2));
        }
        for (uint8_t channel = 0; channel < rcin.channel_count(); channel++) {
            logger.WriteStreaming("VRC", "TimeUS,Chan,PWM,Valid", "QBHB", time_us,
                                  channel + 1, rcin.channel_us(channel), rcin.valid());
        }
        for (uint8_t channel = 0; channel < max_pwm_channels; channel++) {
            if (pwm.enabled(channel)) {
                logger.WriteStreaming("VPWM", "TimeUS,Chan,PWM", "QBH", time_us,
                                      channel + 1, pwm.value_us(channel));
            }
        }
    }
    if (logger.should_log(1U << 3)) {
        const auto& hdg = logic.vhdg.controller();
        const bool holding = hdg.state() == HeadingHold::State::Hold;
        logger.WriteStreaming("VHDG", "TimeUS,En,State,Inhib,Hdg,Tgt,Err,Rate,Req,I,Scale,Wait,Y1,Y2",
                              "QBBHffffffffhh", time_us, hdg.enabled(), uint8_t(hdg.state()), hdg.inhibited(),
                              hdg.heading(), holding ? hdg.target() : NAN, holding ? hdg.error() : NAN,
                              hdg.rate(), hdg.demand(), hdg.integral(), hdg.travel_scale(),
                              hdg.wait_ms() * 0.001f, int(hdg.correction_us(0)), int(hdg.correction_us(1)));
    }
    if (logger.should_log(1U << 4)) {
        const auto* roll = logic.vrs.controller();
        if (roll != nullptr) {
            logger.WriteStreaming("VROL", "TimeUS,State,Inhib,Roll,Rate,Req", "QBHfff", time_us,
                                  uint8_t(roll->state()), roll->inhibited(), roll->roll(), roll->rate(), roll->demand());
            for (uint8_t n = 0; n < 2; n++) {
                const auto& drive = roll->status(n);
                logger.WriteStreaming("VRSD", "TimeUS,Drive,Cmd,Room,Duty,Limited,Bursts", "QBfffHI",
                                      time_us, n + 1, drive.command, drive.headroom, drive.duty,
                                      drive.limited, drive.bursts);
            }
        }
    }
    if (logger.should_log(1U << 5)) {
        const auto card = storage.status();
        logger.WriteStreaming("VSTA", "TimeUS,SD,RCValid,Total,Free,Dropped", "QBBqqI", time_us,
                              card.state, rcin.valid(), card.total_bytes, card.free_bytes, logger.num_dropped());
    }
}
} // namespace Vektor2
