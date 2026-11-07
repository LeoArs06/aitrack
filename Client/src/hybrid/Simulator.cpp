#include "Simulator.h"
#include <algorithm>
#include <random>
#include <stdexcept>
namespace hybrid {
namespace {
bool inside(double t,const std::vector<std::pair<double,double>>& segments) {
    for(auto s:segments) if(t>=s.first && t<s.second) return true; return false;
}
int16_t quantize(double v) { return int16_t(std::clamp(std::round(v),-32768.0,32767.0)); }
}
SampleRaw Simulator::ImuEvent::raw() const {
    SampleRaw r; r.seq=uint32_t(sample.sequence);r.t_us=uint32_t(mcuUs);r.sequence=sample.sequence;r.timeUs=mcuUs;
    r.ax=quantize(sample.accel.x*8192/9.80665);r.ay=quantize(sample.accel.y*8192/9.80665);r.az=quantize(sample.accel.z*8192/9.80665);
    r.gx=quantize(sample.gyro.x*degrees*32.8);r.gy=quantize(sample.gyro.y*degrees*32.8);r.gz=quantize(sample.gyro.z*degrees*32.8);
    r.temp_raw=quantize((sample.temperatureC-36.53)*340);r.flags=sample.flags;
    if(std::abs(int(r.ax))>=32767 || std::abs(int(r.ay))>=32767 || std::abs(int(r.az))>=32767) r.flags|=1;
    if(std::abs(int(r.gx))>=32767 || std::abs(int(r.gy))>=32767 || std::abs(int(r.gz))>=32767) r.flags|=2;
    return r;
}
Simulator::Simulator():Simulator(Config{}) {}
Simulator::Simulator(Config c):config_(std::move(c)) {
    if(config_.imuHz<=0 || config_.opticalHz<=0 || config_.durationSec<0 || config_.usbJitterUs<0 || config_.latencyMs<0)
        throw std::invalid_argument("Invalid simulator rates, duration or delay");
    config_.mounting=config_.mounting.normalized(); generate();
}
Quat Simulator::orientation(double seconds) const {
    double t=std::max(0.0,seconds-2.2);
    // Smooth start avoids an artificial angular impulse after the still prompt.
    double fade=1-std::exp(-t*t/0.15);
    if(config_.motion==Motion::MicroMovement) return poseQuaternion(0.2*std::sin(2*pi*t)*fade,0,0);
    if(config_.motion==Motion::SlowRamp) return poseQuaternion(2*t,0,0);
    double sw=config_.swayDeg,sy=sw*std::sin(2*pi*0.3*seconds),sp=0.6*sw*std::sin(2*pi*0.23*seconds+1);
    if(config_.motion==Motion::Calibration)
        return poseQuaternion(sy+fade*(32*std::sin(1.6*t)+14*std::sin(4.3*t)),
            sp+fade*(23*std::sin(2.1*t)+9*std::sin(5.1*t)),fade*(19*std::sin(2.7*t)+8*std::sin(5.7*t)));
    double burst=std::exp(-std::pow((t-5)/0.65,2))+std::exp(-std::pow((t-17)/0.65,2));
    return poseQuaternion(fade*(19*std::sin(0.7*t)+14*std::sin(1.9*t)+32*std::sin(7*t)*burst)+0.3*t,
        fade*(12*std::sin(1.3*t)+6*std::sin(3.1*t)),fade*(9*std::sin(1.7*t)+4*std::sin(3.9*t)));
}
Vec3 Simulator::angularVelocity(double t) const {
    constexpr double h=0.0001;
    return (orientation(t-h).conj()*orientation(t+h)).log()/(2*h);
}
Vec3 Simulator::translation(double seconds) const {
    double t=std::max(0.0,seconds-2.2);return {1.5*std::sin(0.8*t),0.8*std::sin(0.6*t),3*std::sin(0.4*t)};
}
void Simulator::generate() {
    std::mt19937 random(config_.seed);std::normal_distribution<double> normal(0,1);
    std::exponential_distribution<double> jitter(1/std::max(1.0,config_.usbJitterUs));
    auto noise=[&](double s) { return Vec3{normal(random)*s,normal(random)*s,normal(random)*s}; };
    Vec3 bias=config_.bias; double dt=1/config_.imuHz;
    int64_t lastArrival=0;
    for(uint64_t i=0;double(i)/config_.imuHz<=config_.durationSec;++i) {
        double t=double(i)/config_.imuHz;
        bias+=noise(config_.biasWalkDegPerSqrtSec*radians*std::sqrt(dt));
        if(inside(t,config_.imuDropouts)) continue;
        ImuEvent e;e.truthHostUs=config_.hostOffsetUs+int64_t(std::llround(t*1e6));
        e.mcuUs=config_.mcuOffsetUs+uint64_t(std::llround(t*1e6*(1+config_.clockDriftPpm*1e-6)));
        double transport=200+(config_.usbJitterUs>0 ? jitter(random) : 0);
        // CDC batches preserve sample order even when their arrival jitter overlaps.
        e.arrivalUs=std::max(lastArrival+1,e.truthHostUs+int64_t(transport));lastArrival=e.arrivalUs;
        e.sample.gyro=config_.mounting.conj().rotate(angularVelocity(t))+bias+noise(config_.gyroNoiseDegPerSec*radians);
        e.sample.accel=(orientation(t)*config_.mounting).conj().rotate({0,9.80665,0})+noise(config_.accelNoise);
        e.sample.temperatureC=30;e.sample.sequence=i;imu_.push_back(e);
    }
    for(uint64_t i=0;double(i)/config_.opticalHz<=config_.durationSec;++i) {
        double t=double(i)/config_.opticalHz;
        OpticalEvent e;e.captureUs=config_.hostOffsetUs+int64_t(std::llround(t*1e6));
        e.arrivalUs=e.captureUs+int64_t(config_.latencyMs*1000+normal(random)*config_.opticalJitterUs);
        Quat q=(orientation(t)*Quat::exp(noise(config_.opticalNoiseDeg*radians))).normalized();
        if(config_.outliers && i>90 && i%211==0) q=(q*Quat::exp({0,100*radians,0})).normalized();
        Vec3 tr=translation(t)+noise(config_.translationNoiseCm);auto p=quaternionPose(q,tr);
        e.pose={!inside(t,config_.faceLost),p.yaw,p.pitch,p.roll,p.x,p.y,p.z};optical_.push_back(e);
    }
    std::sort(optical_.begin(),optical_.end(),[](const OpticalEvent& a,const OpticalEvent& b){return a.arrivalUs<b.arrivalUs;});
}
}
