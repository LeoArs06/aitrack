#pragma once
#include "Calibration.h"
#include <deque>
namespace hybrid {
enum class FusionState { Waiting,Hybrid,GyroOnly,OpticalOnly,Crossfading };
const char* stateName(FusionState state);
struct FusionDiagnostics {
    double imuHz=0,opticalHz=0,latencyMs=60,driftDegPerMin=0;
    uint64_t lost=0,rejectedOptical=0;
    FusionState state=FusionState::Waiting;
};
struct FusionOutput { Pose6 pose; FusionState state=FusionState::Waiting; bool valid=false; };
// Single-owner object: the Qt adapter must serialize IMU, camera and output calls.
// All host times are signed steady-clock microseconds; gyro uses rad/s.
class HybridFusion {
public:
    void addImuSample(int64_t hostTimeUs,const SampleSI& sample);
    void addOptical(int64_t arrivalHostUs,const OpticalPose& pose);
    FusionOutput output(int64_t hostNowUs);
    void setStrength(double strength);
    void setCalibration(const CalibrationResult& calibration);
    const FusionDiagnostics& diagnostics() const { return diag_; }
    double timeConstant() const { return tau_; }
    Vec3 bias() const { return bias_; }
private:
    struct State { int64_t t; Quat q; };
    struct Accel { int64_t t; Vec3 a; };
    std::deque<State> history_,opticalHistory_;
    std::deque<Accel> accelHistory_;
    CalibrationResult calibration_;
    FusionDiagnostics diag_;
    Quat q_,optical_,previousOptical_,out_,recoveryOrigin_;
    Vec3 bias_,lastRate_,translation_,translationTarget_,correctionRate_;
    int64_t lastImu_=0,lastOptical_=0,lastValidOptical_=0,lastOutput_=0,disagreeSince_=0,reacquireSince_=0,recoverySince_=0;
    uint64_t lastSequence_=0;
    bool haveImu_=false,haveOptical_=false,haveOutput_=false,faceValid_=false,seeded_=false,recovering_=false;
    double tau_=std::sqrt(10.0),fallback_=0;
    Quat at(int64_t t) const;
    bool still(int64_t t,const SampleSI& s) const;
};
}
