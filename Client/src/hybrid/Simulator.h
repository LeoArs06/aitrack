#pragma once
#include "ImuScaling.h"
#include "PoseConvention.h"
#include <cstdint>
#include <utility>
#include <vector>
namespace hybrid {
class Simulator {
public:
    enum class Motion { Natural,Calibration,MicroMovement,SlowRamp };
    struct Config {
        double durationSec=24,imuHz=200,opticalHz=30,latencyMs=60;
        double opticalNoiseDeg=0.65,translationNoiseCm=0.10,gyroNoiseDegPerSec=0.08;
        double biasWalkDegPerSqrtSec=0.003,accelNoise=0.025;
        double swayDeg=0; // breathing-like head sway, present also during the still prompt
        double clockDriftPpm=80,usbJitterUs=900,opticalJitterUs=600;
        uint64_t mcuOffsetUs=4294000000ULL;
        int64_t hostOffsetUs=10000000;
        Vec3 bias{0.30*radians,-0.20*radians,0.15*radians};
        Quat mounting=Quat::exp({0.7,-0.45,1.1}); // IMU vector -> head vector
        Motion motion=Motion::Natural;
        std::vector<std::pair<double,double>> faceLost{{14,19}},imuDropouts;
        bool outliers=true;
        uint32_t seed=73421;
    };
    struct ImuEvent {
        int64_t arrivalUs=0,truthHostUs=0;
        uint64_t mcuUs=0;
        SampleSI sample;
        SampleRaw raw() const;
    };
    struct OpticalEvent { int64_t arrivalUs=0,captureUs=0; OpticalPose pose; };
    Simulator();
    explicit Simulator(Config config);
    const Config& config() const { return config_; }
    const std::vector<ImuEvent>& imu() const { return imu_; }
    const std::vector<OpticalEvent>& optical() const { return optical_; }
    Quat orientation(double seconds) const;
    Vec3 angularVelocity(double seconds) const;
    Vec3 translation(double seconds) const;
private:
    Config config_;
    std::vector<ImuEvent> imu_;
    std::vector<OpticalEvent> optical_;
    void generate();
};
}
