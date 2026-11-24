#include "VTelemetryCore.h"

#include <cmath>
#include <cstring>
#include <limits.h>

namespace Vektor2 {
namespace VTelemetryCore {

// Stable source IDs. A source has one canonical native unit. For convenient
// transmitter display, both m/s and km/h are separately exposed. IDs must
// never be recycled. Unavailable board features yield invalid samples.
#define SRC(id, name, type, unit, ibus_type, width, scale, bias) \
    {id, name, Type::type, unit, ibus_type, width, scale, bias}
static const Source sources[] = {
    SRC(1, "GPS_SPEED_MS", Number, "m/s", 0x13, 2, 100, 0),
    SRC(2, "GPS_SPEED_KMH", Number, "km/h", 0x7E, 2, 1, 0),
    SRC(3, "GPS_COURSE", Number, "deg", 0x0A, 2, 100, 0),
    SRC(4, "GPS_SATS", Number, "count", 0x02, 2, 1, 0),
    SRC(5, "GPS_FIX", Number, "enum", 0x02, 2, 1, 0),
    SRC(6, "GPS_LAT", CoordinateE7, "degE7", 0x80, 4, 1, 0),
    SRC(7, "GPS_LON", CoordinateE7, "degE7", 0x81, 4, 1, 0),
    SRC(8, "GPS_ALT_M", Number, "m", 0xF9, 2, 1, 0),
    SRC(9, "AHRS_CLIMB_MS", Number, "m/s", 0x09, 2, 100, 0),
    SRC(10, "AHRS_ROLL", Number, "deg", 0x0F, 2, 100, 0),
    SRC(11, "AHRS_PITCH", Number, "deg", 0x10, 2, 100, 0),
    SRC(12, "AHRS_HEADING", Number, "deg", 0x08, 2, 1, 0),
    SRC(13, "AHRS_VN_MS", Number, "m/s", 0x02, 2, 1, 0),
    SRC(14, "AHRS_VE_MS", Number, "m/s", 0x02, 2, 1, 0),
    SRC(15, "GPS_SACC_MS", Number, "m/s", 0x02, 2, 1, 0),
    SRC(16, "GPS_HACC_M", Number, "m", 0x02, 2, 1, 0),
    // i-BUS GPS_STATUS packs status (low byte) and satellite count (high).
    SRC(18, "IBUS_GPS_STATUS", Number, "packed", 0x0B, 2, 1, 0),
    SRC(20, "RC1", Number, "us", 0x02, 2, 1, 0),
    SRC(21, "RC2", Number, "us", 0x02, 2, 1, 0),
    SRC(22, "RC3", Number, "us", 0x02, 2, 1, 0),
    SRC(23, "RC4", Number, "us", 0x02, 2, 1, 0),
    SRC(24, "RC5", Number, "us", 0x02, 2, 1, 0),
    SRC(25, "RC6", Number, "us", 0x02, 2, 1, 0),
    SRC(26, "RC7", Number, "us", 0x02, 2, 1, 0),
    SRC(27, "RC8", Number, "us", 0x02, 2, 1, 0),
    SRC(28, "RC9", Number, "us", 0x02, 2, 1, 0),
    SRC(29, "RC10", Number, "us", 0x02, 2, 1, 0),
    SRC(30, "RC11", Number, "us", 0x02, 2, 1, 0),
    SRC(31, "RC12", Number, "us", 0x02, 2, 1, 0),
    SRC(32, "RC13", Number, "us", 0x02, 2, 1, 0),
    SRC(33, "RC14", Number, "us", 0x02, 2, 1, 0),
    SRC(34, "RC15", Number, "us", 0x02, 2, 1, 0),
    SRC(35, "RC16", Number, "us", 0x02, 2, 1, 0),
    SRC(36, "RC17", Number, "us", 0x02, 2, 1, 0),
    SRC(37, "RC18", Number, "us", 0x02, 2, 1, 0),
    SRC(40, "VSP1_ROD_X", Number, "us", 0x02, 2, 1, 0),
    SRC(41, "VSP1_ROD_Y", Number, "us", 0x02, 2, 1, 0),
    SRC(42, "VSP1_RPM_OUT", Number, "us", 0x02, 2, 1, 0),
    SRC(43, "VSP2_ROD_X", Number, "us", 0x02, 2, 1, 0),
    SRC(44, "VSP2_ROD_Y", Number, "us", 0x02, 2, 1, 0),
    SRC(45, "VSP2_RPM_OUT", Number, "us", 0x02, 2, 1, 0),
    SRC(50, "BOW_PWM", Number, "us", 0x02, 2, 1, 0),
    SRC(51, "STERN_PWM", Number, "us", 0x02, 2, 1, 0),
    SRC(60, "VSP1_TDIR", Number, "deg", 0x08, 2, 1, 0),
    SRC(61, "VSP1_TMAG", Number, "norm", 0x07, 2, 100, 0),
    SRC(62, "VSP2_TDIR", Number, "deg", 0x08, 2, 1, 0),
    SRC(63, "VSP2_TMAG", Number, "norm", 0x07, 2, 100, 0),
    SRC(64, "VSP1_VST", Number, "enum", 0x02, 2, 1, 0),
    SRC(65, "VSP2_VST", Number, "enum", 0x02, 2, 1, 0),
    SRC(66, "VSP1_AUTH", Number, "mask", 0x02, 2, 1, 0),
    SRC(67, "VSP2_AUTH", Number, "mask", 0x02, 2, 1, 0),
    SRC(80, "VRS_STATE", Number, "enum", 0x02, 2, 1, 0),
    SRC(81, "VRS_DEMAND", Number, "norm", 0x07, 2, 100, 0),
    SRC(82, "VRS_INHIB", Number, "mask", 0x02, 2, 1, 0),
    SRC(90, "VRPM1_RPM", Number, "rpm", 0x02, 2, 1, 0),
    SRC(91, "VRPM2_RPM", Number, "rpm", 0x02, 2, 1, 0),
    SRC(100, "MP1_VOLT", Number, "V", 0x03, 2, 100, 0),
    SRC(101, "MP2_VOLT", Number, "V", 0x03, 2, 100, 0),
    SRC(102, "MP3_VOLT", Number, "V", 0x03, 2, 100, 0),
    SRC(103, "MP4_VOLT", Number, "V", 0x03, 2, 100, 0),
    SRC(110, "VFN1_FLAGS", Number, "mask", 0x02, 2, 1, 0),
    SRC(111, "VFN2_FLAGS", Number, "mask", 0x02, 2, 1, 0),
    SRC(112, "VFN3_FLAGS", Number, "mask", 0x02, 2, 1, 0),
    SRC(113, "VFN4_FLAGS", Number, "mask", 0x02, 2, 1, 0),
    SRC(114, "VFN5_FLAGS", Number, "mask", 0x02, 2, 1, 0),
    SRC(115, "VFN6_FLAGS", Number, "mask", 0x02, 2, 1, 0),
    SRC(116, "VFN7_FLAGS", Number, "mask", 0x02, 2, 1, 0),
    SRC(117, "VFN8_FLAGS", Number, "mask", 0x02, 2, 1, 0),
    SRC(118, "VFN9_FLAGS", Number, "mask", 0x02, 2, 1, 0),
    SRC(119, "VFN10_FLAGS", Number, "mask", 0x02, 2, 1, 0),
    SRC(120, "VFN1_ERRORS", Number, "mask", 0x02, 2, 1, 0),
    SRC(121, "VFN2_ERRORS", Number, "mask", 0x02, 2, 1, 0),
    SRC(122, "VFN3_ERRORS", Number, "mask", 0x02, 2, 1, 0),
    SRC(123, "VFN4_ERRORS", Number, "mask", 0x02, 2, 1, 0),
    SRC(124, "VFN5_ERRORS", Number, "mask", 0x02, 2, 1, 0),
    SRC(125, "VFN6_ERRORS", Number, "mask", 0x02, 2, 1, 0),
    SRC(126, "VFN7_ERRORS", Number, "mask", 0x02, 2, 1, 0),
    SRC(127, "VFN8_ERRORS", Number, "mask", 0x02, 2, 1, 0),
    SRC(128, "VFN9_ERRORS", Number, "mask", 0x02, 2, 1, 0),
    SRC(129, "VFN10_ERRORS", Number, "mask", 0x02, 2, 1, 0),
    SRC(130, "VFN1_PULSE", Number, "us", 0x02, 2, 1, 0),
    SRC(131, "VFN2_PULSE", Number, "us", 0x02, 2, 1, 0),
    SRC(132, "VFN3_PULSE", Number, "us", 0x02, 2, 1, 0),
    SRC(133, "VFN4_PULSE", Number, "us", 0x02, 2, 1, 0),
    SRC(134, "VFN5_PULSE", Number, "us", 0x02, 2, 1, 0),
    SRC(135, "VFN6_PULSE", Number, "us", 0x02, 2, 1, 0),
    SRC(136, "VFN7_PULSE", Number, "us", 0x02, 2, 1, 0),
    SRC(137, "VFN8_PULSE", Number, "us", 0x02, 2, 1, 0),
    SRC(138, "VFN9_PULSE", Number, "us", 0x02, 2, 1, 0),
    SRC(139, "VFN10_PULSE", Number, "us", 0x02, 2, 1, 0),
    SRC(140, "NAV_STATE", Number, "enum", 0x02, 2, 1, 0),
    SRC(141, "NAV_RBITS", Number, "mask", 0x02, 2, 1, 0),
    SRC(150, "SEQ1_STATE", Number, "enum", 0x02, 2, 1, 0),
    SRC(151, "SEQ2_STATE", Number, "enum", 0x02, 2, 1, 0),
    SRC(152, "SEQ3_STATE", Number, "enum", 0x02, 2, 1, 0),
    SRC(153, "SEQ4_STATE", Number, "enum", 0x02, 2, 1, 0),
    SRC(154, "SEQ5_STATE", Number, "enum", 0x02, 2, 1, 0),
    SRC(155, "SEQ6_STATE", Number, "enum", 0x02, 2, 1, 0),
    SRC(156, "SEQ7_STATE", Number, "enum", 0x02, 2, 1, 0),
    SRC(157, "SEQ8_STATE", Number, "enum", 0x02, 2, 1, 0),
    SRC(158, "SEQ9_STATE", Number, "enum", 0x02, 2, 1, 0),
    SRC(159, "SEQ10_STATE", Number, "enum", 0x02, 2, 1, 0),
    SRC(160, "SEQ11_STATE", Number, "enum", 0x02, 2, 1, 0),
    SRC(161, "SEQ12_STATE", Number, "enum", 0x02, 2, 1, 0),
    SRC(162, "SEQ13_STATE", Number, "enum", 0x02, 2, 1, 0),
    SRC(163, "SEQ14_STATE", Number, "enum", 0x02, 2, 1, 0),
    SRC(164, "SEQ15_STATE", Number, "enum", 0x02, 2, 1, 0),
    SRC(165, "SEQ16_STATE", Number, "enum", 0x02, 2, 1, 0),
    SRC(170, "SEQ1_PULSE", Number, "us", 0x02, 2, 1, 0),
    SRC(171, "SEQ2_PULSE", Number, "us", 0x02, 2, 1, 0),
    SRC(172, "SEQ3_PULSE", Number, "us", 0x02, 2, 1, 0),
    SRC(173, "SEQ4_PULSE", Number, "us", 0x02, 2, 1, 0),
    SRC(174, "SEQ5_PULSE", Number, "us", 0x02, 2, 1, 0),
    SRC(175, "SEQ6_PULSE", Number, "us", 0x02, 2, 1, 0),
    SRC(176, "SEQ7_PULSE", Number, "us", 0x02, 2, 1, 0),
    SRC(177, "SEQ8_PULSE", Number, "us", 0x02, 2, 1, 0),
    SRC(178, "SEQ9_PULSE", Number, "us", 0x02, 2, 1, 0),
    SRC(179, "SEQ10_PULSE", Number, "us", 0x02, 2, 1, 0),
    SRC(180, "SEQ11_PULSE", Number, "us", 0x02, 2, 1, 0),
    SRC(181, "SEQ12_PULSE", Number, "us", 0x02, 2, 1, 0),
    SRC(182, "SEQ13_PULSE", Number, "us", 0x02, 2, 1, 0),
    SRC(183, "SEQ14_PULSE", Number, "us", 0x02, 2, 1, 0),
    SRC(184, "SEQ15_PULSE", Number, "us", 0x02, 2, 1, 0),
    SRC(185, "SEQ16_PULSE", Number, "us", 0x02, 2, 1, 0),
    // Vessel DP telemetry. Geographic E7 coordinates must use CoordinateE7,
    // not a lossy float. i-BUS assignment and receiver polling stay optional.
    SRC(240, "DP_TGT_LAT", CoordinateE7, "degE7", 0x80, 4, 1, 0),
    SRC(241, "DP_TGT_LON", CoordinateE7, "degE7", 0x81, 4, 1, 0),
    SRC(242, "DP_ACT_LAT", CoordinateE7, "degE7", 0x80, 4, 1, 0),
    SRC(243, "DP_ACT_LON", CoordinateE7, "degE7", 0x81, 4, 1, 0),
    SRC(244, "DP_DIST", Number, "m", 0x02, 2, 100, 0),
    SRC(245, "DP_BRG", Number, "deg", 0x08, 2, 100, 0),
    SRC(246, "HDG_TGT", Number, "deg", 0x08, 2, 100, 0),
    SRC(247, "HDG_ACT", Number, "deg", 0x08, 2, 100, 0),
    SRC(248, "DP_ERR_N", Number, "m", 0x02, 2, 100, 0),
    SRC(249, "DP_ERR_E", Number, "m", 0x02, 2, 100, 0),
    SRC(250, "DP_CMD_X", Number, "norm", 0x02, 2, 1000, 0),
    SRC(251, "DP_CMD_Y", Number, "norm", 0x02, 2, 1000, 0),
    SRC(252, "HDG_CMD", Number, "norm", 0x02, 2, 1000, 0),
    SRC(253, "DP_VN", Number, "m/s", 0x02, 2, 100, 0),
    SRC(254, "DP_VE", Number, "m/s", 0x02, 2, 100, 0),
    SRC(255, "DP_CMD_MAG", Number, "norm", 0x02, 2, 1000, 0),
    SRC(256, "DP_CMD_BRG", Number, "deg", 0x08, 2, 100, 0),
    SRC(257, "HDG_ERR", Number, "deg", 0x02, 2, 100, 0),
    SRC(258, "DP_CMD_N", Number, "norm", 0x02, 2, 1000, 0),
    SRC(259, "DP_CMD_E", Number, "norm", 0x02, 2, 1000, 0),
    // Controller maths readiness, independent of NAV_STATE/actuator ownership.
    SRC(260, "DP_CALC", Number, "bool", 0x02, 2, 1, 0),
    SRC(261, "HDG_CALC", Number, "bool", 0x02, 2, 1, 0),
    SRC(262, "VSP_AL_ST", Number, "mask", 0x02, 2, 1, 0),
    SRC(200, "PWM1_OUT", Number, "us", 0x02, 2, 1, 0),
    SRC(201, "PWM2_OUT", Number, "us", 0x02, 2, 1, 0),
    SRC(202, "PWM3_OUT", Number, "us", 0x02, 2, 1, 0),
    SRC(203, "PWM4_OUT", Number, "us", 0x02, 2, 1, 0),
    SRC(204, "PWM5_OUT", Number, "us", 0x02, 2, 1, 0),
    SRC(205, "PWM6_OUT", Number, "us", 0x02, 2, 1, 0),
    SRC(206, "PWM7_OUT", Number, "us", 0x02, 2, 1, 0),
    SRC(207, "PWM8_OUT", Number, "us", 0x02, 2, 1, 0),
    SRC(208, "PWM9_OUT", Number, "us", 0x02, 2, 1, 0),
    SRC(209, "PWM10_OUT", Number, "us", 0x02, 2, 1, 0),
    SRC(210, "PWM11_OUT", Number, "us", 0x02, 2, 1, 0),
    SRC(211, "PWM12_OUT", Number, "us", 0x02, 2, 1, 0),
    SRC(212, "PWM13_OUT", Number, "us", 0x02, 2, 1, 0),
    SRC(213, "PWM14_OUT", Number, "us", 0x02, 2, 1, 0),
    SRC(214, "PWM15_OUT", Number, "us", 0x02, 2, 1, 0),
    SRC(215, "PWM16_OUT", Number, "us", 0x02, 2, 1, 0),
    SRC(216, "PWM17_OUT", Number, "us", 0x02, 2, 1, 0),
    SRC(217, "PWM18_OUT", Number, "us", 0x02, 2, 1, 0),
    SRC(218, "PWM19_OUT", Number, "us", 0x02, 2, 1, 0),
    SRC(219, "PWM20_OUT", Number, "us", 0x02, 2, 1, 0),
    SRC(220, "PWM21_OUT", Number, "us", 0x02, 2, 1, 0),
    SRC(221, "PWM22_OUT", Number, "us", 0x02, 2, 1, 0),
    SRC(222, "PWM23_OUT", Number, "us", 0x02, 2, 1, 0),
    SRC(223, "PWM24_OUT", Number, "us", 0x02, 2, 1, 0),
    SRC(224, "PWM25_OUT", Number, "us", 0x02, 2, 1, 0),
    SRC(225, "PWM26_OUT", Number, "us", 0x02, 2, 1, 0),
    SRC(226, "PWM27_OUT", Number, "us", 0x02, 2, 1, 0),
    SRC(227, "PWM28_OUT", Number, "us", 0x02, 2, 1, 0),
    SRC(228, "PWM29_OUT", Number, "us", 0x02, 2, 1, 0),
    SRC(229, "PWM30_OUT", Number, "us", 0x02, 2, 1, 0),
    SRC(230, "PWM31_OUT", Number, "us", 0x02, 2, 1, 0),
    SRC(231, "PWM32_OUT", Number, "us", 0x02, 2, 1, 0),
    // Active propulsion configuration and AZI commissioning commands.
    SRC(270, "MAIN_PROP_TYPE", Number, "enum", 0x02, 2, 1, 0),
    SRC(271, "VSP_CONF", Number, "enum", 0x02, 2, 1, 0),
    SRC(272, "AZI_CONF", Number, "enum", 0x02, 2, 1, 0),
    SRC(280, "AZI1_AZ_PWM", Number, "us", 0x02, 2, 1, 0),
    SRC(281, "AZI1_MOT_PWM", Number, "us", 0x02, 2, 1, 0),
    SRC(282, "AZI1_ANG_CMD", Number, "deg", 0x02, 2, 10, 0),
    SRC(283, "AZI1_ANG_TGT", Number, "deg", 0x02, 2, 10, 0),
    SRC(284, "AZI1_MOT_CMD", Number, "norm", 0x02, 2, 1000, 0),
    SRC(285, "AZI1_STATE", Number, "enum", 0x02, 2, 1, 0),
    SRC(286, "AZI1_RB", Number, "mask", 0x02, 2, 1, 0),
    SRC(290, "AZI2_AZ_PWM", Number, "us", 0x02, 2, 1, 0),
    SRC(291, "AZI2_MOT_PWM", Number, "us", 0x02, 2, 1, 0),
    SRC(292, "AZI2_ANG_CMD", Number, "deg", 0x02, 2, 10, 0),
    SRC(293, "AZI2_ANG_TGT", Number, "deg", 0x02, 2, 10, 0),
    SRC(294, "AZI2_MOT_CMD", Number, "norm", 0x02, 2, 1000, 0),
    SRC(295, "AZI2_STATE", Number, "enum", 0x02, 2, 1, 0),
    SRC(296, "AZI2_RB", Number, "mask", 0x02, 2, 1, 0),
};
#undef SRC

// Only numeric HoTT fields are assignable. Reserved flags, packet headers,
// warning tones, protocol versions, GPS text, and stop bytes are NOT slots.
// Units and ranges follow HoTT wire format; assignment deliberately ignores
// their physical meaning. The field's scale is the only conversion applied.
#define H(p, name, type, offset, width, scale, bias, sign) \
    {Protocol::p, name, Type::type, offset, width, scale, bias, sign}
static const Slot slots[slot_count] = {
    // i-BUS address 1..15: type is advertised from the selected source.
    H(IBus,"IBUS_01",Number,0,0,1,0,false), H(IBus,"IBUS_02",Number,0,0,1,0,false),
    H(IBus,"IBUS_03",Number,0,0,1,0,false), H(IBus,"IBUS_04",Number,0,0,1,0,false),
    H(IBus,"IBUS_05",Number,0,0,1,0,false), H(IBus,"IBUS_06",Number,0,0,1,0,false),
    H(IBus,"IBUS_07",Number,0,0,1,0,false), H(IBus,"IBUS_08",Number,0,0,1,0,false),
    H(IBus,"IBUS_09",Number,0,0,1,0,false), H(IBus,"IBUS_10",Number,0,0,1,0,false),
    H(IBus,"IBUS_11",Number,0,0,1,0,false), H(IBus,"IBUS_12",Number,0,0,1,0,false),
    H(IBus,"IBUS_13",Number,0,0,1,0,false), H(IBus,"IBUS_14",Number,0,0,1,0,false),
    H(IBus,"IBUS_15",Number,0,0,1,0,false),
    // EAM 7 low cells, 7 high cells: voltage in 20-mV increments.
    H(HottEAM,"EAM_CELL1_LOW",Number,6,1,50,0,false),
    H(HottEAM,"EAM_CELL2_LOW",Number,7,1,50,0,false),
    H(HottEAM,"EAM_CELL3_LOW",Number,8,1,50,0,false),
    H(HottEAM,"EAM_CELL4_LOW",Number,9,1,50,0,false),
    H(HottEAM,"EAM_CELL5_LOW",Number,10,1,50,0,false),
    H(HottEAM,"EAM_CELL6_LOW",Number,11,1,50,0,false),
    H(HottEAM,"EAM_CELL7_LOW",Number,12,1,50,0,false),
    H(HottEAM,"EAM_CELL1_HIGH",Number,13,1,50,0,false),
    H(HottEAM,"EAM_CELL2_HIGH",Number,14,1,50,0,false),
    H(HottEAM,"EAM_CELL3_HIGH",Number,15,1,50,0,false),
    H(HottEAM,"EAM_CELL4_HIGH",Number,16,1,50,0,false),
    H(HottEAM,"EAM_CELL5_HIGH",Number,17,1,50,0,false),
    H(HottEAM,"EAM_CELL6_HIGH",Number,18,1,50,0,false),
    H(HottEAM,"EAM_CELL7_HIGH",Number,19,1,50,0,false),
    H(HottEAM,"EAM_BATT1",Number,20,2,10,0,false),
    H(HottEAM,"EAM_BATT2",Number,22,2,10,0,false),
    H(HottEAM,"EAM_TEMP1",Number,24,1,1,20,false),
    H(HottEAM,"EAM_TEMP2",Number,25,1,1,20,false),
    H(HottEAM,"EAM_ALT",Number,26,2,1,500,false),
    H(HottEAM,"EAM_CURRENT",Number,28,2,10,0,false),
    H(HottEAM,"EAM_VOLT",Number,30,2,10,0,false),
    H(HottEAM,"EAM_CAPACITY",Number,32,2,0.1f,0,false),
    H(HottEAM,"EAM_CLIMB",Number,34,2,100,30000,false),
    H(HottEAM,"EAM_CLIMB3",Number,36,1,3,120,false),
    H(HottEAM,"EAM_RPM",Number,37,2,0.1f,0,false),
    H(HottEAM,"EAM_MIN",Number,39,1,1,0,false),
    H(HottEAM,"EAM_SEC",Number,40,1,1,0,false),
    H(HottEAM,"EAM_SPEED",Number,41,2,1,0,false),
    // HoTT GPS, including two coordinate-only locations.
    H(HottGPS,"GPS_DIR",Number,6,1,0.5f,0,false),
    H(HottGPS,"GPS_SPEED",Number,7,2,1,0,false),
    H(HottGPS,"GPS_LAT",CoordinateE7,9,0,1,0,false),
    H(HottGPS,"GPS_LON",CoordinateE7,14,0,1,0,false),
    H(HottGPS,"GPS_DISTANCE",Number,19,2,1,0,false),
    H(HottGPS,"GPS_ALT",Number,21,2,1,500,false),
    H(HottGPS,"GPS_CLIMB",Number,23,2,100,30000,false),
    H(HottGPS,"GPS_CLIMB3",Number,25,1,3,120,false),
    H(HottGPS,"GPS_SATS",Number,26,1,1,0,false),
    H(HottGPS,"GPS_FIX",Number,27,1,1,0,false),
    H(HottGPS,"GPS_HOME_DIR",Number,28,1,0.5f,0,false),
    H(HottGPS,"GPS_VN",Number,29,2,1000,0,true),
    H(HottGPS,"GPS_SACC",Number,31,1,100,0,false),
    H(HottGPS,"GPS_VE",Number,36,2,1000,0,true),
    H(HottGPS,"GPS_HACC",Number,38,1,100,0,false),
    // HoTT VARIO: numeric fields only (text remains intentionally blank).
    H(HottVario,"VARIO_ALT",Number,5,2,1,500,false),
    H(HottVario,"VARIO_MAX",Number,7,2,1,500,false),
    H(HottVario,"VARIO_MIN",Number,9,2,1,500,false),
    H(HottVario,"VARIO_CLIMB",Number,11,2,100,30000,false),
    H(HottVario,"VARIO_CLIMB3",Number,13,2,300,30000,false),
    H(HottVario,"VARIO_CLIMB10",Number,15,2,1000,30000,false),
    H(HottVario,"VARIO_YAW",Number,41,1,0.5f,0,false),
};
#undef H

static_assert(sizeof(slots)/sizeof(slots[0]) == slot_count, "VTEL slot count changed");

uint8_t source_count() { return uint8_t(sizeof(sources)/sizeof(sources[0])); }
const Source* source_at(uint8_t index) { return index < source_count() ? &sources[index] : nullptr; }
const Source* find_source(uint16_t id)
{
    for (const Source& source : sources) {
        if (source.id == id) { return &source; }
    }
    return nullptr;
}
const Slot* slot_definition(uint8_t n) { return n >= 1 && n <= slot_count ? &slots[n-1] : nullptr; }

static bool compatible(const Slot& slot, const Source& source)
{
    return slot.protocol == Protocol::IBus || slot.type == source.type;
}

uint8_t Router::configure(const int16_t (&ids)[slot_count])
{
    uint8_t rejected = 0;
    for (uint8_t i = 0; i < slot_count; i++) {
        const Source* src = ids[i] > 0 ? find_source(uint16_t(ids[i])) : nullptr;
        if (ids[i] != 0 && (src == nullptr || !compatible(slots[i], *src))) {
            rejected++;
        }
        _assigned[i] = (src && compatible(slots[i], *src)) ? src : nullptr;
        _samples[i] = {};
    }
    return rejected;
}

void Router::sample(Reader reader, void* context)
{
    if (reader == nullptr) { return; }
    for (uint8_t i = 0; i < slot_count; i++) {
        Sample s{};
        const Source* src = _assigned[i];
        if (src && reader(src->id, s, context) &&
            (src->type == Type::CoordinateE7 || std::isfinite(s.number))) {
            s.valid = true;
        } else {
            s.valid = false;
        }
        _samples[i] = s;
    }
}

const Source* Router::assigned(uint8_t n) const { return n >= 1 && n <= slot_count ? _assigned[n-1] : nullptr; }
const Sample& Router::value(uint8_t n) const
{
    static const Sample invalid{};
    return n >= 1 && n <= slot_count ? _samples[n-1] : invalid;
}

bool Router::module_enabled(Protocol p) const
{
    for (uint8_t i = 0; i < slot_count; i++) {
        if (slots[i].protocol == p && _assigned[i] != nullptr) { return true; }
    }
    return false;
}

static void sum16(uint8_t* p, uint8_t length)
{
    uint16_t crc = 0xFFFF;
    for (uint8_t n = 0; n < length-2; n++) { crc -= p[n]; }
    p[length-2] = uint8_t(crc);
    p[length-1] = uint8_t(crc >> 8);
}

static int64_t rounded_clamped(float v, float minimum, float maximum)
{
    if (v < minimum) { return int64_t(minimum); }
    if (v > maximum) { return int64_t(maximum); }
    return int64_t(std::round(v));
}

uint8_t Router::ibus_reply(const uint8_t request[4], uint8_t out[8]) const
{
    if (request[0] != 4 || request[1] < 0x81 || (request[1] & 0x0F) == 0) { return 0; }
    const uint8_t address = request[1] & 0x0F;
    if (address > ibus_count || _assigned[address-1] == nullptr) { return 0; }
    uint8_t check[4];
    memcpy(check, request, 4);
    sum16(check, 4);
    if (check[2] != request[2] || check[3] != request[3]) { return 0; }

    const uint8_t command = request[1] & 0xF0;
    const Source& src = *_assigned[address-1];
    const Sample& sample = _samples[address-1];
    if (command == 0x80) {
        out[0] = 4; out[1] = request[1];
        sum16(out, 4);
        return 4;
    }
    if (command == 0x90) {
        out[0] = 6; out[1] = request[1];
        out[2] = src.ibus_sensor_type; out[3] = src.ibus_width;
        sum16(out, 6);
        return 6;
    }
    if (command != 0xA0 || !sample.valid) { return 0; }

    const uint8_t width = src.ibus_width;
    const uint8_t count = uint8_t(width + 4);
    out[0] = count;
    out[1] = request[1];
    uint32_t raw = 0;
    if (src.type == Type::CoordinateE7) {
        raw = uint32_t(sample.coordinate_e7);
    } else {
        const float scaled = sample.number * src.ibus_scale + src.ibus_bias;
        if (!std::isfinite(scaled)) { return 0; }
        // All scalar i-BUS encodings used here are 16 bits. Negative values
        // retain their two's-complement bits for signed sensor types.
        raw = uint16_t(int32_t(rounded_clamped(scaled, -32768, 65535)));
    }
    for (uint8_t i = 0; i < width; i++) { out[2+i] = uint8_t(raw >> (8*i)); }
    sum16(out, count);
    return count;
}

static void coordinate_to_ddm(int32_t value, uint8_t* out)
{
    const bool neg = value < 0;
    const uint64_t absval = neg ? uint64_t(-int64_t(value)) : uint64_t(value);
    uint32_t degrees = absval / 10000000ULL;
    const uint64_t minutes_e7 = (absval % 10000000ULL) * 60ULL;
    uint32_t minutes = minutes_e7 / 10000000ULL;
    uint32_t fraction = uint32_t(((minutes_e7 % 10000000ULL) + 500ULL) / 1000ULL);
    if (fraction >= 10000) {
        fraction -= 10000;
        if (++minutes >= 60) { minutes = 0; degrees++; }
    }
    const uint16_t dm = uint16_t(degrees * 100U + minutes);
    out[0] = uint8_t(neg); out[1] = uint8_t(dm); out[2] = uint8_t(dm >> 8);
    out[3] = uint8_t(fraction); out[4] = uint8_t(fraction >> 8);
}

bool Router::hott_frame(uint8_t sensor_id, uint8_t out[44]) const
{
    Protocol protocol;
    uint8_t text_id;
    switch (sensor_id) {
    case 0x8E: protocol = Protocol::HottEAM; text_id = 0xE0; break;
    case 0x8A: protocol = Protocol::HottGPS; text_id = 0xA0; break;
    case 0x89: protocol = Protocol::HottVario; text_id = 0x90; break;
    default: return false;
    }
    if (!module_enabled(protocol)) { return false; }
    memset(out, 0, 44);
    out[0] = 0x7C; out[1] = sensor_id; out[3] = text_id;
    out[43] = 0x7D;
    if (protocol == Protocol::HottEAM) {
        out[24] = out[25] = 20;
        out[26] = uint8_t(500); out[27] = uint8_t(500 >> 8);
        out[34] = uint8_t(30000); out[35] = uint8_t(30000 >> 8);
        out[36] = 120;
    } else if (protocol == Protocol::HottGPS) {
        out[21] = uint8_t(500); out[22] = uint8_t(500 >> 8);
        out[23] = uint8_t(30000); out[24] = uint8_t(30000 >> 8);
        out[25] = 120;
        out[27] = '-'; out[41] = '-'; out[42] = 1;
    } else {
        for (uint8_t o = 5; o <= 9; o += 2) {
            out[o] = uint8_t(500); out[o+1] = uint8_t(500 >> 8);
        }
        for (uint8_t o = 11; o <= 15; o += 2) {
            out[o] = uint8_t(30000); out[o+1] = uint8_t(30000 >> 8);
        }
        out[42] = 1;
    }
    for (uint8_t i = ibus_count; i < slot_count; i++) {
        const Slot& field = slots[i];
        if (field.protocol != protocol || _assigned[i] == nullptr || !_samples[i].valid) { continue; }
        const Sample& v = _samples[i];
        const uint8_t offset = field.offset;
        if (field.type == Type::CoordinateE7) {
            coordinate_to_ddm(v.coordinate_e7, &out[offset]);
        } else if (field.name && offset == 27 && protocol == Protocol::HottGPS) {
            // GPS status is an ASCII character rather than an integer on wire.
            out[offset] = v.number >= 3 ? '3' : (v.number >= 2 ? '2' : '-');
            out[41] = out[offset];
        } else {
            const float scaled = v.number * field.scale + field.bias;
            if (!std::isfinite(scaled)) { continue; }
            const int64_t raw = rounded_clamped(scaled,
                                field.signed_value ? -32768.0f : 0.0f,
                                field.signed_value ? 32767.0f : (field.width == 1 ? 255.0f : 65535.0f));
            out[offset] = uint8_t(raw);
            if (field.width == 2) { out[offset+1] = uint8_t(raw >> 8); }
        }
    }
    return true;
}

} // namespace VTelemetryCore
} // namespace Vektor2
