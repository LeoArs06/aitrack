#pragma once
#include "ImuProtocol.h"
#include "Quat.h"
namespace hybrid {
struct SampleSI {
    Vec3 gyro; // rad/s, IMU body axes
    Vec3 accel; // m/s^2, IMU body axes
    double temperatureC=0;
    uint8_t flags=0;
    uint64_t sequence=0;
    bool discontinuity=false;
};
inline SampleSI scale(const SampleRaw& s,uint8_t whoAmI=0x68) {
    constexpr double g=9.80665/8192.0, w=radians/32.8;
    return {{s.gx*w,s.gy*w,s.gz*w},{s.ax*g,s.ay*g,s.az*g},
            whoAmI==0x68 ? s.temp_raw/340.0+36.53 : s.temp_raw/333.87+21.0,
            s.flags,s.sequence,s.discontinuity};
}
}
