#!/usr/bin/env python3
"""Exercise actual measurement code with deterministic ADCs and bounded MAVLink links."""
from pathlib import Path
import os
import subprocess
import tempfile


def main():
    source = Path(__file__).resolve().parents[1]
    board = source.parents[1] / 'libraries/AP_HAL_ChibiOS/hwdef/MatekH743/hwdef.dat'
    declaration = next(line for line in board.read_text().splitlines()
                       if line.startswith('define HAL_VEKTOR2_MEASURE_POINTS '))
    headers = {
        'AP_HAL/AP_HAL.h': '''#pragma once
#include <cstdint>
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
namespace AP_HAL {
class AnalogSource { public: virtual float voltage_average() = 0; };
class AnalogIn { public: virtual AnalogSource* channel(int16_t) = 0; };
struct HAL { AnalogIn* analogin; };
}
''',
        'AP_Param/AP_Param.h': '''#pragma once
#include <cstdint>
#include <cstddef>
class AP_Int16 {
public:
    int16_t value = 0;
    int16_t get() const { return value; }
};
class AP_Param {
public:
    struct GroupInfo { const char* name; size_t offset; int16_t default_value; };
    static void setup_object_defaults(void* object, const GroupInfo* info) {
        for (; info->name; info++) {
            set(object, *info, info->default_value);
        }
    }
    static void set(void* object, const GroupInfo& info, int16_t value) {
        reinterpret_cast<AP_Int16*>(static_cast<char*>(object) + info.offset)->value = value;
    }
};
#define AP_GROUPINFO(name, index, cls, member, value) {name, offsetof(cls, member), value}
#define AP_GROUPEND {nullptr, 0, 0}
''',
        'GCS_MAVLink/GCS.h': '''#pragma once
#include <vector>
#include <cstdint>
#define HAL_GCS_ENABLED 1
#define HAL_HIGH_LATENCY2_ENABLED 1
#define MAVLINK_COMM_NUM_BUFFERS 3
struct mavlink_named_value_float_t { uint32_t time_boot_ms; char name[10]; float value; };
struct Link {
    uint8_t channel;
    Link(uint8_t n) : channel(n) {}
    bool active = true, private_link = false, is_high_latency_link = false;
    bool is_active() const { return active; }
    bool is_private() const { return private_link; }
    uint8_t get_chan() const { return channel; }
};
struct GCS {
    Link links[3] {{0}, {1}, {2}};
    uint8_t num_gcs() const { return 3; }
    Link* chan(uint8_t n) { return &links[n]; }
};
extern GCS ground_station;
inline GCS& gcs() { return ground_station; }
extern unsigned space[3];
extern std::vector<mavlink_named_value_float_t> sent[3];
#define WITH_SEMAPHORE(lock)
#define HAVE_PAYLOAD_SPACE(channel, message) (space[channel] > 0)
inline void mavlink_msg_named_value_float_send_struct(uint8_t channel,
                                                     const mavlink_named_value_float_t* value) {
    --space[channel];
    sent[channel].push_back(*value);
}
''',
    }
    with tempfile.TemporaryDirectory(prefix='vektor2-mp-test-') as directory:
        directory = Path(directory)
        for name, contents in headers.items():
            path = directory / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(contents)
        config = directory / 'board_points.h'
        config.write_text('#' + declaration + '\n')
        for enabled in (True, False):
            binary = directory / ('test_mp' if enabled else 'test_no_mp')
            command = [
                os.environ.get('CXX', '/usr/bin/g++'), '-std=c++11', '-O1', '-g',
                '-Wall', '-Wextra', '-Werror', '-fsanitize=address,undefined',
                '-fno-omit-frame-pointer', '-I', str(directory), '-I', str(source),
            ]
            if enabled:
                command += ['-include', str(config)]
            command += [str(source / 'tests/test_measure_points.cpp'),
                        str(source / 'MeasurePoints.cpp'), '-o', str(binary)]
            subprocess.run(command, check=True)
            subprocess.run([str(binary)], check=True)


if __name__ == '__main__':
    main()
