#include "Vektor_Capability.h"

#include <AP_HAL/AP_HAL_Boards.h>

#include <string.h>

namespace {

const Vektor::TimerGroup core_evo_pwm_timer_groups[] = {
    { "TIM2", 2 },
    { "TIM4", 4 },
    { "TIM8", 4 },
    { "TIM1", 2 },
};

const Vektor::TimerGroup core_evo_flex_timer_groups[] = {
    { "TIM5", 2 },
    { "TIM3", 4 },
};

const Vektor::TimerGroup core_reduced_pwm_timer_groups[] = {
    { "TIM3", 2 },
    { "TIM2", 4 },
};

const Vektor::BoardCapability board_capabilities[] = {
    {
        Vektor::BoardProfile::CORE_EVO_H743,
        "Vektor Core Evo H743",
        "unassigned",
        "STM32H743VIT6",
        0,
        12,
        6,
        1,
        4,
        2,
        1,
        0,
        "ICM-20602",
        "MMC5983MA",
        Vektor::CAP_NATIVE_USB |
        Vektor::CAP_PWM_OUTPUTS |
        Vektor::CAP_FLEX_TIMER_CHANNELS |
        Vektor::CAP_DEDICATED_RECEIVER_ROW |
        Vektor::CAP_EXTERNAL_UARTS |
        Vektor::CAP_CLASSIC_CAN |
        Vektor::CAP_ADC_OBSERVABLES |
        Vektor::CAP_ONBOARD_IMU |
        Vektor::CAP_ONBOARD_COMPASS,
        core_evo_pwm_timer_groups,
        uint8_t(sizeof(core_evo_pwm_timer_groups) / sizeof(core_evo_pwm_timer_groups[0])),
        core_evo_flex_timer_groups,
        uint8_t(sizeof(core_evo_flex_timer_groups) / sizeof(core_evo_flex_timer_groups[0])),
        -1, // assigned when the H743 ChibiOS hwdef maps its status LED
    },
    {
        Vektor::BoardProfile::CORE_REDUCED_F405,
        "Vektor Core Reduced F405",
        "revo-mini",
        "STM32F405xx",
        124,
        6,
        0,
        0,
        3,
        0,
        3,
        2,
        "BMI088",
        "external I2C probe",
        Vektor::CAP_NATIVE_USB |
        Vektor::CAP_PWM_OUTPUTS |
        Vektor::CAP_EXTERNAL_UARTS |
        Vektor::CAP_ADC_OBSERVABLES |
        Vektor::CAP_STORAGE |
        Vektor::CAP_ONBOARD_IMU,
        core_reduced_pwm_timer_groups,
        uint8_t(sizeof(core_reduced_pwm_timer_groups) / sizeof(core_reduced_pwm_timer_groups[0])),
        nullptr,
        0,
        0, // LED_BLUE is GPIO(0) in the revo-mini hwdef
    },
};

bool chibios_board_is(const char *expected)
{
#if defined(CHIBIOS_BOARD_NAME)
    return strcmp(CHIBIOS_BOARD_NAME, expected) == 0;
#else
    (void)expected;
    return false;
#endif
}

} // namespace

namespace Vektor {

const BoardCapability &core_evo_h743_capability()
{
    return board_capabilities[0];
}

const BoardCapability &core_reduced_f405_capability()
{
    return board_capabilities[1];
}

const BoardCapability *supported_capabilities()
{
    return board_capabilities;
}

uint8_t supported_capability_count()
{
    return uint8_t(sizeof(board_capabilities) / sizeof(board_capabilities[0]));
}

const BoardCapability &default_capability_for_build()
{
    if (chibios_board_is("revo-mini")) {
        return core_reduced_f405_capability();
    }

    return core_evo_h743_capability();
}

const char *board_profile_name(BoardProfile profile)
{
    switch (profile) {
    case BoardProfile::CORE_EVO_H743:
        return "core-evo-h743";
    case BoardProfile::CORE_REDUCED_F405:
        return "core-reduced-f405";
    }

    return "unknown";
}

bool capability_has(const BoardCapability &capability, CapabilityFlag flag)
{
    return (capability.flags & uint32_t(flag)) != 0;
}

} // namespace Vektor
