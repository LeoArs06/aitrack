#include "HybridFusion.h"
#include <algorithm>
namespace hybrid {
const char* stateName(FusionState s) {
    switch(s) { case FusionState::Waiting:return "waiting";case FusionState::Hybrid:return "hybrid";
    case FusionState::GyroOnly:return "gyro only";case FusionState::OpticalOnly:return "optical only";case FusionState::Crossfading:return "crossfading"; } return "waiting";
}
void HybridFusion::setStrength(double s) { if(std::isfinite(s)) tau_=20*std::pow(0.5/20,std::clamp(s,0.0,1.0)); }
void HybridFusion::setCalibration(const CalibrationResult& c) {
    calibration_=c; calibration_.imuToHead=c.imuToHead.normalized();
    calibration_.latencyMs=std::clamp(c.latencyMs,0.0,250.0); bias_=c.bias; diag_.latencyMs=calibration_.latencyMs;
    history_.clear();accelHistory_.clear();correctionRate_={};
}
Quat HybridFusion::at(int64_t t) const {
    if(history_.empty()) return q_;
    if(t<=history_.front().t) return history_.front().q;
    for(size_t i=1;i<history_.size();++i) if(t<=history_[i].t) {
        auto a=history_[i-1],b=history_[i]; return Quat::slerp(a.q,b.q,double(t-a.t)/double(b.t-a.t));
    }
    return history_.back().q;
}
bool HybridFusion::still(int64_t t,const SampleSI& s) const {
    if((s.flags&3) || !faceValid_ || t-lastValidOptical_>150000 || opticalHistory_.size()<8 || accelHistory_.size()<60) return false;
    if(t-opticalHistory_.front().t<500000 || (s.gyro-bias_).norm()>0.8*radians) return false;
    if(angularDistance(opticalHistory_.front().q,optical_)>1.3*radians) return false;
    Vec3 mean,second;
    for(auto a:accelHistory_) { mean+=a.a; second+={a.a.x*a.a.x,a.a.y*a.a.y,a.a.z*a.a.z}; }
    double n=double(accelHistory_.size());mean=mean/n;second=second/n;
    double variance=second.x+second.y+second.z-mean.dot(mean);
    return variance<0.04 && std::abs(mean.norm()-9.80665)<0.5;
}
void HybridFusion::addImuSample(int64_t t,const SampleSI& s) {
    if(!s.gyro.finite() || !s.accel.finite() || (haveImu_ && t<=lastImu_)) return;
    double dt=haveImu_ ? double(t-lastImu_)*1e-6 : 0;
    bool gap=haveImu_ && (dt>0.1 || s.discontinuity || (s.flags&4));
    if(haveImu_ && !s.discontinuity && s.sequence>lastSequence_) diag_.lost+=s.sequence-lastSequence_-1;
    lastSequence_=s.sequence;
    if(dt>0 && dt<0.1) diag_.imuHz=diag_.imuHz==0 ? 1/dt : diag_.imuHz*0.95+0.05/dt;
    if(gap || (!haveImu_ && haveOutput_)) {
        history_.clear();accelHistory_.clear();
        if(haveOptical_ && faceValid_ && t-lastValidOptical_<200000) q_=optical_;
        else if(haveOutput_) q_=out_;
        recoveryOrigin_=haveOutput_ ? out_ : q_; recoverySince_=t; recovering_=true; dt=0;
    }
    accelHistory_.push_back({t,s.accel}); while(!accelHistory_.empty() && t-accelHistory_.front().t>700000) accelHistory_.pop_front();
    if(dt>0 && still(t,s)) bias_+=(s.gyro-bias_)*(1-std::exp(-dt/60));
    Vec3 rate=calibration_.imuToHead.rotate(s.gyro-bias_);
    if(dt>0 && !(s.flags&2)) q_=(q_*Quat::exp((lastRate_+rate)*(0.5*dt))).normalized();
    // Do not let a clipped gyro contaminate the following trapezoid or prediction.
    if(!(s.flags&2)) lastRate_=rate;
    lastImu_=t;haveImu_=true;
    history_.push_back({t,q_}); while(!history_.empty() && t-history_.front().t>1200000) history_.pop_front();
}
void HybridFusion::addOptical(int64_t t,const OpticalPose& p) {
    if(haveOptical_ && t<=lastOptical_) return;
    double dt=lastOptical_ ? double(t-lastOptical_)*1e-6 : 1.0/30;
    lastOptical_=t;
    if(!p.valid || !std::isfinite(p.yaw) || !std::isfinite(p.pitch) || !std::isfinite(p.roll) ||
        !Vec3{p.x,p.y,p.z}.finite()) { faceValid_=false; return; }
    Quat measurement=poseQuaternion(p.yaw,p.pitch,p.roll);
    int64_t capture=t-int64_t(calibration_.latencyMs*1000);
    Quat past=at(capture); Vec3 error=(measurement*past.conj()).log();
    double opticalStep=haveOptical_ ? angularDistance(previousOptical_,measurement) : 0;
    // Allow real fast turns, but reject isolated PnP flips. Persistent disagreement
    // is accepted only after a consistent stream, with a gradual gain ramp.
    bool jump=haveOptical_ && opticalStep>std::max(25*radians,dt*400*radians);
    bool disagreement=seeded_ && error.norm()>35*radians;
    if(jump || disagreement) {
        ++diag_.rejectedOptical;
        if(!disagreeSince_ || jump) disagreeSince_=t;
        previousOptical_=measurement;
        if(t-disagreeSince_<500000) return;
        if(!reacquireSince_) reacquireSince_=t;
    } else { disagreeSince_=0; if(error.norm()<3*radians) reacquireSince_=0; }
    if(haveOptical_ && dt>0) diag_.opticalHz=diag_.opticalHz==0 ? 1/dt : diag_.opticalHz*0.9+0.1/dt;
    previousOptical_=measurement; optical_=measurement;haveOptical_=true;faceValid_=true;lastValidOptical_=t;
    opticalHistory_.push_back({capture,measurement}); while(!opticalHistory_.empty() && capture-opticalHistory_.front().t>800000) opticalHistory_.pop_front();
    translationTarget_={p.x,p.y,p.z};
    if(!seeded_) {
        q_=measurement;for(auto& h:history_) h.q=measurement;
        translation_=translationTarget_;seeded_=true;return;
    }
    if(!haveImu_ || t-lastImu_>100000 || history_.empty() || capture<history_.front().t) return;
    double effectiveTau=tau_;
    if(reacquireSince_) {
        double ramp=std::clamp(double(t-reacquireSince_)/1000000,0.0,1.0);
        effectiveTau=tau_*(1-ramp)+0.5*ramp;
    }
    double alpha=1-std::exp(-std::min(dt,0.1)/effectiveTau);
    Vec3 correction=error*alpha;
    // Large re-acquisition errors are speed-limited to 20 deg/s.
    double limit=20*radians*std::min(dt,0.1);
    if(correction.norm()>limit) correction=correction*(limit/correction.norm());
    Quat dq=Quat::exp(correction);q_=(dq*q_).normalized();
    // Correct buffered descendants as well, otherwise subsequent frames would
    // repeatedly correct the same historical error.
    for(auto& h:history_) if(h.t>=capture) h.q=(dq*h.q).normalized();
    double k=1-std::exp(-std::min(dt,0.1)/10);
    correctionRate_+=(correction/std::max(0.001,dt)-correctionRate_)*k;
    diag_.driftDegPerMin=correctionRate_.norm()*degrees*60;
}
FusionOutput HybridFusion::output(int64_t t) {
    if(haveOutput_ && t<lastOutput_) return {quaternionPose(out_,translation_),diag_.state,true};
    double dt=haveOutput_ ? std::min(0.1,double(t-lastOutput_)*1e-6) : 0;
    bool imu=haveImu_ && t-lastImu_<=100000;
    bool face=haveOptical_ && faceValid_ && t-lastValidOptical_<250000;
    double targetFallback=imu ? 0.0 : (face ? 1.0 : fallback_);
    if(dt>0) fallback_+=std::clamp(targetFallback-fallback_,-dt/0.5,dt/0.5);
    Quat target=(!imu && !face && haveOutput_) ? out_ : q_;
    if(imu) target=(q_*Quat::exp(lastRate_*(std::clamp(double(t-lastImu_)*1e-6,0.0,0.02)))).normalized();
    if(face) target=Quat::slerp(target,optical_,fallback_);
    if(recovering_ && (imu || face)) {
        double a=std::clamp(double(t-recoverySince_)/500000,0.0,1.0);
        target=Quat::slerp(recoveryOrigin_,target,a);if(a>=1) recovering_=false;
    } else if(!imu && !face) recovering_=false;
    bool transition=recovering_ || std::abs(fallback_-targetFallback)>1e-6;
    // Optical-only poses are interpolated at the output cadence, never stepped
    // directly at camera cadence. Limit transition speed for dropout continuity.
    if(haveOutput_ && (transition || fallback_>0)) {
        Vec3 d=(out_.conj()*target).log();
        double a=fallback_>0.999 ? 1-std::exp(-dt/0.045) : 1;
        if(transition && d.norm()*a>150*radians*dt) a=150*radians*dt/std::max(1e-12,d.norm());
        out_=(out_*Quat::exp(d*a)).normalized();
    } else out_=target;
    if(dt>0 && face) translation_+=(translationTarget_-translation_)*(1-std::exp(-dt/0.08));
    diag_.state=transition ? FusionState::Crossfading : imu ? (face ? FusionState::Hybrid : FusionState::GyroOnly) : face ? FusionState::OpticalOnly : FusionState::Waiting;
    if(!haveImu_ && !haveOptical_) return {{},FusionState::Waiting,false};
    haveOutput_=true;lastOutput_=t;
    return {quaternionPose(out_,translation_),diag_.state,true};
}
}
