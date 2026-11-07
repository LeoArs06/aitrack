#pragma once
#include "ImuScaling.h"
#include "PoseConvention.h"
#include <cstdint>
#include <deque>
#include <string>
#include <vector>
namespace hybrid {
struct CalibrationResult {
    Vec3 bias;
    Quat imuToHead;
    double latencyMs=60,quality=0,residualDegPerSec=0;
    Vec3 excitationDegPerSec;
    std::string serialize() const;
    static bool deserialize(const std::string& text,CalibrationResult& result);
};
class Calibration {
public:
    enum class Phase { Idle,Still,Motion,Complete,Failed };
    void begin(int64_t hostUs);
    void addImuSample(int64_t hostUs,const SampleSI& sample);
    void addOptical(int64_t arrivalHostUs,const OpticalPose& pose);
    bool finish(); // Call after the motion prompt (~10 s); never installs a failed result.
    Phase phase() const { return phase_; }
    const std::string& reason() const { return reason_; }
    const CalibrationResult& result() const { return result_; }
private:
    struct Gyro { int64_t t; Vec3 w; };
    struct Opt { int64_t t; Quat q; };
    Phase phase_=Phase::Idle;
    int64_t start_=0;
    std::vector<Gyro> gyro_;
    std::vector<Opt> optical_;
    struct Still { Vec3 w,a; };
    std::vector<Still> still_;
    double stillSpread_=0; // gyro spread during the still prompt (rad/s)
    CalibrationResult result_;
    std::string reason_;
    bool fail(const std::string& reason);
    bool endStill();
    bool meanGyro(int64_t a,int64_t b,Vec3& w) const;
    void stillBiasCorrection(Quat imuToHead,int delayMs);
};
// Background calibration from ordinary head motion: no still phase, no
// button. Fits camera body rate ~= R*(gyro - bias) over the last ~90 s for
// each candidate camera delay and keeps the best fit once it is reliable.
class AutoCalibrator {
public:
    void addImuSample(int64_t hostUs,const SampleSI& sample);
    void addOptical(int64_t arrivalHostUs,const OpticalPose& pose);
    // Re-solves at most every 2 s. True when a new accepted result is ready.
    bool update(int64_t nowUs);
    void reset() { *this=AutoCalibrator{}; }
    bool ready() const { return ready_; }
    const CalibrationResult& result() const { return result_; }
    double progress() const { return progress_; } // 0..1 while learning
    double relativeResidual() const { return residual_; }
private:
    struct Gyro { int64_t t; Vec3 w; };
    struct Opt { int64_t t; Quat q; };
    std::deque<Gyro> gyro_;
    std::deque<Opt> optical_;
    int64_t lastSolve_=0;
    bool ready_=false;
    double progress_=0,residual_=1;
    CalibrationResult result_;
    bool meanGyro(int64_t a,int64_t b,Vec3& w) const;
};
}
