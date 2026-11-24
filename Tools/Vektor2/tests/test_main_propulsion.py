#!/usr/bin/env python3
"""Compile real Logic/Routing/RouteParameters with small host HAL/parameter adapters.
Does not validate ArduPilot persistent storage, board outputs or target RAM.
"""
from pathlib import Path
import os
import subprocess
import tempfile

HEADERS = {
    'AP_Param/AP_Param.h': r'''#pragma once
#include <stdint.h>
template<class T> struct Scalar {
    T value{};
    T get() const { return value; }
    void set(T v) { value = v; }
};
using AP_Int8 = Scalar<int8_t>;
using AP_Int16 = Scalar<int16_t>;
using AP_Float = Scalar<float>;
class AP_Param {
public:
    struct GroupInfo { const char* name; int index; void (*defaults)(void*); };
    static void setup_object_defaults(void* obj, const GroupInfo* info) {
        for (; info->name; ++info) { info->defaults(obj); }
    }
};
#define AP_GROUPINFO(name,index,cls,member,def) {name,index,[](void* p){static_cast<cls*>(p)->member.set(def);}}
#define AP_SUBGROUPINFO(member,name,index,cls,sub) {name,index,[](void* p){AP_Param::setup_object_defaults(&static_cast<cls*>(p)->member, sub::var_info);}}
#define AP_GROUPEND {nullptr,0,nullptr}
''',
    'AP_HAL/AP_HAL.h': r'''#pragma once
#include <stdint.h>
namespace AP_HAL { uint32_t millis(); uint64_t micros64(); }
''',
    'RC_Channel/RC_Channel.h': r'''#pragma once
#include <stdint.h>
#define NUM_RC_CHANNELS 16
class RC_Channel {};
class RC_Channels {
public:
    virtual RC_Channel* channel(uint8_t) = 0;
    virtual bool in_rc_failsafe() const = 0;
    virtual bool has_valid_input() const = 0;
    virtual int8_t flight_mode_channel_number() const = 0;
};
''',
    'AP_AHRS/AP_AHRS_config.h': '#pragma once\n',
    'AP_InertialSensor/AP_InertialSensor_config.h': '#pragma once\n',
}


def main():
    source = Path(__file__).resolve().parents[1]
    with tempfile.TemporaryDirectory(prefix='vektor2-main-propulsion-') as directory:
        tmp = Path(directory)
        for name, content in HEADERS.items():
            path = tmp / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(content)
        files = ['tests/test_main_propulsion.cpp', 'Logic.cpp', 'Routing.cpp',
                 'RouteParameters.cpp', 'ComponentParameters.cpp',
                 'FunctionParameters.cpp', 'RollStabilizationParams.cpp',
                 'Functions.cpp', 'ThrusterControl.cpp', 'VSP1.cpp', 'VSP2.cpp',
                 'AzipodControl.cpp', 'VesselThrustAllocator.cpp', 'VspMapper.cpp', 'VspVectorControl.cpp',
                 'VspRpmGovernor.cpp', 'RollStabilization.cpp',
                 'LibVSP.c', 'Sequencer.cpp']
        for large in (0, 1):
            binary = tmp / f'test_main_propulsion_{large}'
            subprocess.run([os.environ.get('CXX', '/usr/bin/g++'), '-std=c++17',
                            '-O1', '-g', '-Wall', '-Wextra', '-Werror',
                            '-fsanitize=address,undefined', '-fno-omit-frame-pointer',
                            '-fno-pie', '-no-pie', f'-DVEKTOR2_SEQ_LARGE={large}',
                            '-I', str(tmp), '-I', str(source),
                            *(str(source / f) for f in files), '-o', str(binary)], check=True)
            subprocess.run([str(binary)], check=True)


if __name__ == '__main__':
    main()
