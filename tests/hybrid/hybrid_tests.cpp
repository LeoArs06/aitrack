#include "Calibration.h"
#include "ClockSync.h"
#include "HybridFusion.h"
#include "ImuProtocol.h"
#include "Simulator.h"
#include <algorithm>
#include <cmath>
#include <functional>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
using namespace hybrid;
namespace {
struct Failure:std::runtime_error { using std::runtime_error::runtime_error; };
#define CHECK(condition) do { if(!(condition)) { std::ostringstream message; message<<__FILE__<<':'<<__LINE__<<": "<<#condition; throw Failure(message.str()); } } while(false)
void near(double a,double b,double tolerance) { if(std::abs(a-b)>tolerance || !std::isfinite(a)) {
    std::ostringstream m;m<<"Expected "<<a<<" ~= "<<b<<" (tolerance "<<tolerance<<')';throw Failure(m.str()); } }
struct Metric { std::string name;double value;std::string unit; };
std::vector<Metric> metrics;
void metric(const std::string& name,double value,const std::string& unit) { metrics.push_back({name,value,unit}); }
CalibrationResult recovered;
bool calibrated=false;
void protocolTests() {
    std::string known="123456789";CHECK(crc16(reinterpret_cast<const uint8_t*>(known.data()),known.size())==0x29B1);
    SampleRaw sample;sample.seq=34;sample.t_us=1000;sample.ax=-32768;sample.ay=32767;sample.gz=-1234;sample.temp_raw=-20;
    auto frame=encode(sample);CHECK(frame.size()==30);
    for(size_t split=0;split<=frame.size();++split) {
        ImuProtocol p;auto a=p.feed(frame.data(),split);auto b=p.feed(frame.data()+split,frame.size()-split);
        CHECK(a.size()+b.size()==1);auto s=std::get<SampleRaw>(a.empty()?b[0]:a[0]);
        CHECK(s.seq==34 && s.ax==-32768 && s.ay==32767 && s.gz==-1234 && s.temp_raw==-20);
    }
    ImuProtocol oneByte;size_t count=0;for(auto b:frame) count+=oneByte.feed(&b,1).size();CHECK(count==1);
    Info info;info.whoAmI=0x71;info.txDrops=123;Status status{2,0x68,0,987};
    ImuProtocol p;auto messages=p.feed(encode(info));CHECK(std::get<Info>(messages.at(0)).txDrops==123);
    messages=p.feed(encode(status));CHECK(std::get<Status>(messages.at(0)).i2cErrors==987);
    auto bad=frame;bad[14]^=0x23;
    std::vector<uint8_t> mixed{0x1,0xA5,0xA5,0x5A,0x01,65};mixed.insert(mixed.end(),bad.begin(),bad.end());mixed.insert(mixed.end(),frame.begin(),frame.end());
    messages=p.feed(mixed);CHECK(messages.size()==1);CHECK(p.counters().crcErrors==1);CHECK(p.counters().resyncs>0);CHECK(p.counters().malformed>0);
    // A valid frame embedded inside a damaged outer frame must survive rescan.
    auto nested=frame;auto sf=encode(status);std::copy(sf.begin(),sf.end(),nested.begin()+5);nested.back()^=0xFF;
    messages=p.feed(nested);CHECK(messages.size()==1 && std::holds_alternative<Status>(messages[0]));
    p.reset();sample.seq=0xFFFFFFFEu;sample.t_us=0xFFFFFFF0u;p.feed(encode(sample));
    sample.seq=0xFFFFFFFFu;sample.t_us=4;messages=p.feed(encode(sample));
    auto s=std::get<SampleRaw>(messages[0]);CHECK(s.timeUs==0x100000004ULL);
    sample.seq=0;sample.t_us=24;messages=p.feed(encode(sample));CHECK(std::get<SampleRaw>(messages[0]).sequence==0x100000000ULL);
    sample.seq=3;sample.t_us=84;p.feed(encode(sample));CHECK(p.counters().lost==2);
    p.feed(encode(sample));CHECK(p.counters().lost==2); // duplicate is not a 2^32 loss
    sample.seq=0;sample.t_us=10;sample.flags=4;messages=p.feed(encode(sample));
    CHECK(std::get<SampleRaw>(messages[0]).discontinuity);CHECK(p.counters().lost==2);
    sample.ax=8192;sample.gx=3280;sample.temp_raw=340;
    auto si=scale(sample);near(si.accel.x,9.80665,1e-12);near(si.gyro.x,100*radians,1e-12);near(si.temperatureC,37.53,1e-12);
    for(int id:{0x70,0x71,0x73,0x98,0x19,0x12}) near(scale(sample,uint8_t(id)).temperatureC,340/333.87+21,1e-12);
    // Arbitrary garbage and long chunks exercise bounded buffering and recovery.
    std::vector<uint8_t> garbage(100000,0xA5);garbage.insert(garbage.end(),frame.begin(),frame.end());
    CHECK(p.feed(garbage).size()==1);
}
void mathTests() {
    for(double pitch:{-179.0,-90.0,-30.0,0.0,80.0,179.0}) for(double yaw:{-89.9,-60.0,-1.0,0.0,45.0,89.9})
        for(double roll:{-179.0,-75.0,0.0,25.0,90.0,150.0}) {
            auto q=poseQuaternion(yaw,pitch,roll);auto p=quaternionPose(q);
            near(p.yaw,yaw,1e-9);near(p.pitch,pitch,1e-9);near(p.roll,roll,1e-9);
            near(angularDistance(q,poseQuaternion(p.yaw,p.pitch,p.roll)),0,1e-11);
            near(angularDistance(q,Quat::fromMatrix(q.matrix())),0,1e-12);
        }
    for(double yaw:{-90.0,90.0}) { auto q=poseQuaternion(yaw,46,-37);auto p=quaternionPose(q);near(angularDistance(q,poseQuaternion(p.yaw,p.pitch,p.roll)),0,1e-7); }
    {   // Independent reference built from explicit elementary matrices (AITrack order).
        auto mul=[](const double a[3][3],const double b[3][3],double o[3][3]){for(int i=0;i<3;i++)for(int j=0;j<3;j++){o[i][j]=0;for(int k=0;k<3;k++)o[i][j]+=a[i][k]*b[k][j];}};
        for(double yaw:{-70.0,-20.0,0.0,35.0,80.0}) for(double pitch:{-150.0,-40.0,0.0,25.0,170.0}) for(double roll:{-120.0,0.0,60.0,90.0,175.0}) {
            double a=pitch*radians,b=yaw*radians,c=roll*radians;
            double X[3][3]={{1,0,0},{0,std::cos(a),-std::sin(a)},{0,std::sin(a),std::cos(a)}};
            double Y[3][3]={{std::cos(b),0,std::sin(b)},{0,1,0},{-std::sin(b),0,std::cos(b)}};
            double Z[3][3]={{std::cos(c),-std::sin(c),0},{std::sin(c),std::cos(c),0},{0,0,1}};
            double ZY[3][3],R[3][3]; mul(Z,Y,ZY); mul(ZY,X,R);
            auto m=poseQuaternion(yaw,pitch,roll).matrix();
            for(int i=0;i<3;i++)for(int j=0;j<3;j++) near(m.m[i][j],R[i][j],1e-12);
        }
    }
    for(Vec3 v:{Vec3{},Vec3{1e-12,-2e-12,3e-12},Vec3{0.3,-0.5,1.2},Vec3{pi,0,0}}) near((Quat::exp(v).log()-v).norm(),0,1e-12);
    Quat a=Quat::exp({0.3,0.1,-0.7}),b=Quat::exp({-0.8,0.4,0.3});
    near(angularDistance(a,Quat::slerp(a,b,0.5)),angularDistance(a,b)*0.5,1e-12);
    near((a.rotate({1,2,3})-a.matrix()*Vec3{1,2,3}).norm(),0,1e-12);
    Pose6 pose{1,2,3,4,5,6};double packet[6];pose.packet(packet);for(int i=0;i<6;++i) near(packet[i],i+1,0);
}
void clockTests() {
    ClockSync clock;std::mt19937 random(111);std::exponential_distribution<double> jitter(1.0/1200);
    double square=0,maxError=0;int n=0;int64_t lastArrival=0;
    constexpr uint64_t origin=0x100000000ULL+3500;
    for(int i=0;i<12000;++i) {
        int64_t truth=900000000000LL+int64_t(i)*5000;
        uint64_t mcu=origin+uint64_t(std::llround(i*5000.0*1.000125));
        int64_t arrival=std::max(lastArrival+1,truth+250+int64_t(jitter(random))+(i%91==0?18000:0));lastArrival=arrival;
        auto estimate=clock.observe(mcu,arrival);
        if(i>4000) { double e=double(estimate-truth-250);square+=e*e;maxError=std::max(maxError,std::abs(e));++n; }
    }
    double rms=std::sqrt(square/n);metric("Clock mapping RMS",rms,"us");metric("Clock mapping max error",maxError,"us");metric("Clock drift error",std::abs(clock.driftPpm()+124.9844),"ppm");
    CHECK(rms<250);CHECK(maxError<600);near(clock.driftPpm(),-124.9844,5);
    // Change oscillator drift without a timestamp discontinuity. The old
    // envelope must age out and the running fit must follow the new slope.
    for(int i=1;i<=8000;++i) {
        int64_t truth=900000000000LL+60000000LL+int64_t(i)*5000;
        uint64_t mcu=origin+60007500ULL+uint64_t(std::llround(i*5000.0*0.99985));
        int64_t arrival=std::max(lastArrival+1,truth+250+int64_t(jitter(random)));lastArrival=arrival;
        clock.observe(mcu,arrival);
    }
    near(clock.driftPpm(),150.0225,5);
    auto before=clock.resets();auto mapped=clock.observe(10,960000000000LL,true);CHECK(clock.resets()==before+1);CHECK(mapped==960000000000LL);
}
void runCalibration(Calibration& calibration,const Simulator& sim) {
    calibration.begin(sim.config().hostOffsetUs);size_t i=0,j=0;ClockSync clock;
    while(i<sim.imu().size() || j<sim.optical().size()) {
        if(i<sim.imu().size() && (j==sim.optical().size() || sim.imu()[i].arrivalUs<=sim.optical()[j].arrivalUs)) {
            auto e=sim.imu()[i++];calibration.addImuSample(clock.observe(e.mcuUs,e.arrivalUs),scale(e.raw()));
        } else { auto e=sim.optical()[j++];calibration.addOptical(e.arrivalUs,e.pose); }
    }
}
void calibrationTests() {
    Simulator::Config c;c.durationSec=12.5;c.motion=Simulator::Motion::Calibration;c.faceLost.clear();
    Simulator sim(c);Calibration cal;runCalibration(cal,sim);
    if(!cal.finish()) throw Failure("Calibration: "+cal.reason());
    recovered=cal.result();calibrated=true;
    double error=angularDistance(recovered.imuToHead,c.mounting)*degrees;
    metric("Mounting rotation error",error,"deg");metric("Recovered camera latency",recovered.latencyMs,"ms");
    metric("Calibration residual",recovered.residualDegPerSec,"deg/s");metric("Calibration quality",recovered.quality,"0..1");
    CHECK(error<1);CHECK(std::abs(recovered.latencyMs-c.latencyMs)<10);CHECK((recovered.bias-c.bias).norm()*degrees<0.03);
    CalibrationResult restored;CHECK(CalibrationResult::deserialize(recovered.serialize(),restored));
    near(angularDistance(restored.imuToHead,recovered.imuToHead),0,1e-12);near((restored.bias-recovered.bias).norm(),0,1e-12);
    CHECK(!CalibrationResult::deserialize("bad",restored));CHECK(!CalibrationResult::deserialize(recovered.serialize()+" trailing",restored));
    CalibrationResult invalid=recovered;invalid.imuToHead={0,0,0,0};CHECK(!CalibrationResult::deserialize(invalid.serialize(),restored));
    double worstMount=error,worstLatency=std::abs(recovered.latencyMs-c.latencyMs);
    for(int variant=0;variant<3;++variant) {
        auto alternate=c;alternate.latencyMs=variant==0 ? 0 : variant==1 ? 125 : 225;
        alternate.seed+=uint32_t(variant+1);alternate.mounting=Quat::exp({-1.2+variant*0.4,0.8+variant*0.3,-1.7});
        Calibration other;runCalibration(other,Simulator(alternate));if(!other.finish()) throw Failure("Alternate calibration: "+other.reason());
        double rotationError=angularDistance(other.result().imuToHead,alternate.mounting)*degrees;
        double delayError=std::abs(other.result().latencyMs-alternate.latencyMs);
        worstMount=std::max(worstMount,rotationError);worstLatency=std::max(worstLatency,delayError);
        CHECK(rotationError<1);CHECK(delayError<10);
    }
    metric("Mounting worst (4 runs)",worstMount,"deg");metric("Latency worst (4 runs)",worstLatency,"ms");
    Calibration moving;moving.begin(0);
    for(int k=0;k<=400;++k) { SampleSI s;s.gyro={0,0,(k%2?8:-8)*radians};s.accel={0,9.80665,0};moving.addImuSample(k*5000LL,s); }
    CHECK(moving.phase()==Calibration::Phase::Failed);CHECK(!moving.reason().empty());
    // Worn headset: breathing/pulse tremor (~0.9 dps RMS) on top of a 2 dps bias must pass.
    Calibration worn;worn.begin(0);Vec3 wornBias{2*radians,-0.7*radians,1.0*radians};
    for(int k=0;k<=420;++k) { double t=k*0.005;SampleSI s;
        s.gyro=wornBias+Vec3{1.1*std::sin(2*3.14159265*0.3*t),0.6*std::sin(2*3.14159265*1.1*t),0.5*std::cos(2*3.14159265*0.7*t)}*radians;
        s.accel={0,9.80665,0};worn.addImuSample(k*5000LL,s); }
    if(worn.phase()!=Calibration::Phase::Motion) throw Failure("Worn still phase: "+worn.reason());
    auto biased=c;biased.bias={1.9*radians,-1.2*radians,0.8*radians};biased.seed+=11;
    Calibration big;runCalibration(big,Simulator(biased));if(!big.finish()) throw Failure("Large-bias calibration: "+big.reason());
    CHECK((big.result().bias-biased.bias).norm()*degrees<0.05);
    auto swaying=c;swaying.swayDeg=1.0;swaying.bias=biased.bias;swaying.seed+=23;
    Calibration sway;runCalibration(sway,Simulator(swaying));if(!sway.finish()) throw Failure("Sway calibration: "+sway.reason());
    double swayBias=(sway.result().bias-swaying.bias).norm()*degrees;metric("Bias error with 1 deg sway",swayBias,"deg/s");
    CHECK(swayBias<0.25);CHECK(angularDistance(sway.result().imuToHead,swaying.mounting)*degrees<1);
    c.motion=Simulator::Motion::SlowRamp;c.outliers=false;Simulator deficient(c);Calibration poor;runCalibration(poor,deficient);
    CHECK(!poor.finish());CHECK(poor.phase()==Calibration::Phase::Failed);CHECK(!poor.reason().empty());
}
struct Run {
    double rms=0,rawRms=0,faceLostMax=0,maxTransitionStep=0,microAmplitude=0,rampRate=0,maxStep=0;
    double visibleRms=0,visibleRawRms=0;
    uint64_t rejected=0,lost=0;
    bool sawGyroOnly=false,sawOpticalOnly=false,sawCrossfade=false;
    std::vector<double> yaw,time;
};
Run runFusion(const Simulator& sim,CalibrationResult calibration,double strength=0.5,bool wire=true) {
    HybridFusion f;f.setCalibration(calibration);f.setStrength(strength);
    ClockSync clock;ImuProtocol parser;size_t i=0,j=0;Run r;Quat raw,previous;bool haveRaw=false,havePrevious=false;
    double sum=0,rawSum=0,visibleSum=0,visibleRawSum=0;size_t n=0,nVisible=0;bool rawClean=false;auto c=sim.config();
    for(int64_t now=c.hostOffsetUs;now<=c.hostOffsetUs+int64_t(c.durationSec*1e6);now+=5000) {
        while(i<sim.imu().size() || j<sim.optical().size()) {
            bool nextImu=i<sim.imu().size() && (j==sim.optical().size() || sim.imu()[i].arrivalUs<=sim.optical()[j].arrivalUs);
            int64_t next=nextImu ? sim.imu()[i].arrivalUs : sim.optical()[j].arrivalUs;
            if(next>now) break;
            if(nextImu) {
                auto e=sim.imu()[i++];
                if(wire) {
                    auto messages=parser.feed(encode(e.raw()));CHECK(messages.size()==1);auto s=std::get<SampleRaw>(messages[0]);
                    f.addImuSample(clock.observe(s.timeUs,e.arrivalUs,s.discontinuity),scale(s));
                } else f.addImuSample(e.truthHostUs,e.sample);
            } else {
                auto e=sim.optical()[j++];f.addOptical(e.arrivalUs,e.pose);
                rawClean=e.pose.valid;
                if(e.pose.valid) { raw=poseQuaternion(e.pose.yaw,e.pose.pitch,e.pose.roll);haveRaw=true;
                    rawClean=angularDistance(raw,sim.orientation(double(e.captureUs-c.hostOffsetUs)*1e-6))<10*radians; }
            }
        }
        auto o=f.output(now);if(!o.valid) continue;
        auto q=poseQuaternion(o.pose.yaw,o.pose.pitch,o.pose.roll);double t=double(now-c.hostOffsetUs)*1e-6;
        CHECK(q.finite());CHECK(std::isfinite(o.pose.x));
        if(t>3 && haveRaw) { double e=angularDistance(q,sim.orientation(t))*degrees;sum+=e*e;
            double re=angularDistance(raw,sim.orientation(t))*degrees;rawSum+=re*re;++n;
            if(rawClean) { visibleSum+=e*e;visibleRawSum+=re*re;++nVisible; } }
        bool faceLost=false;for(auto segment:c.faceLost) if(t>=segment.first+0.2 && t<segment.second) faceLost=true;
        if(faceLost) r.faceLostMax=std::max(r.faceLostMax,angularDistance(q,sim.orientation(t))*degrees);
        double step=havePrevious ? angularDistance(q,previous)*degrees : 0;r.maxStep=std::max(r.maxStep,step);
        if(o.state==FusionState::Crossfading) r.maxTransitionStep=std::max(r.maxTransitionStep,step);
        r.sawGyroOnly|=o.state==FusionState::GyroOnly;r.sawOpticalOnly|=o.state==FusionState::OpticalOnly;r.sawCrossfade|=o.state==FusionState::Crossfading;
        previous=q;havePrevious=true;
        if(t>5) { r.yaw.push_back(o.pose.yaw);r.time.push_back(t); }
    }
    CHECK(n>0);r.rms=std::sqrt(sum/n);r.rawRms=std::sqrt(rawSum/n);
    CHECK(nVisible>0);r.visibleRms=std::sqrt(visibleSum/nVisible);r.visibleRawRms=std::sqrt(visibleRawSum/nVisible);
    r.rejected=f.diagnostics().rejectedOptical;r.lost=f.diagnostics().lost;
    double s=0,co=0,mt=0,my=0;for(size_t k=0;k<r.yaw.size();++k) {
        double phase=2*pi*(r.time[k]-2.2);s+=r.yaw[k]*std::sin(phase);co+=r.yaw[k]*std::cos(phase);mt+=r.time[k];my+=r.yaw[k]; }
    r.microAmplitude=2*std::sqrt(s*s+co*co)/r.yaw.size();mt/=r.yaw.size();my/=r.yaw.size();
    double xy=0,xx=0;for(size_t k=0;k<r.yaw.size();++k) { xy+=(r.time[k]-mt)*(r.yaw[k]-my);xx+=(r.time[k]-mt)*(r.time[k]-mt); }
    r.rampRate=xy/std::max(1e-12,xx);return r;
}
void fusionTests() {
    CHECK(calibrated);Simulator sim;auto r=runFusion(sim,recovered);
    double peakRate=0;for(const auto& e:sim.imu()) peakRate=std::max(peakRate,sim.angularVelocity(double(e.truthHostUs-sim.config().hostOffsetUs)*1e-6).norm()*degrees);
    metric("Synthetic peak turn rate",peakRate,"deg/s");CHECK(peakRate>200 && peakRate<310);
    metric("Fused rotation RMS",r.rms,"deg");metric("Raw optical rotation RMS",r.rawRms,"deg");metric("5 s face-loss max error",r.faceLostMax,"deg");
    metric("Visible fused RMS",r.visibleRms,"deg");metric("Visible clean optical RMS",r.visibleRawRms,"deg");
    metric("Rejected optical frames",double(r.rejected),"frames");
    CHECK(r.rms<r.rawRms*0.4);CHECK(r.visibleRms<r.visibleRawRms*0.4);CHECK(r.rms<1.5);CHECK(r.faceLostMax<2);CHECK(r.sawGyroOnly);CHECK(r.rejected>=1);
    Simulator::Config c;c.motion=Simulator::Motion::MicroMovement;c.durationSec=16;c.faceLost.clear();c.outliers=false;
    auto micro=runFusion(Simulator(c),recovered);metric("0.2 deg micro amplitude",micro.microAmplitude,"deg");
    CHECK(micro.microAmplitude>0.16 && micro.microAmplitude<0.24);
    c.motion=Simulator::Motion::SlowRamp;c.durationSec=20;auto ramp=runFusion(Simulator(c),recovered);
    metric("Voluntary ramp rate",ramp.rampRate,"deg/s");near(ramp.rampRate,2,0.05);
    c.imuDropouts={{8,9.5}};auto dropout=runFusion(Simulator(c),recovered);
    metric("Dropout transition max step",dropout.maxTransitionStep,"deg/step");
    CHECK(dropout.sawOpticalOnly && dropout.sawCrossfade);CHECK(dropout.maxTransitionStep<1);CHECK(dropout.rms<1.5);CHECK(dropout.lost>=299);
    for(double strength:{0.0,1.0}) { auto extreme=runFusion(sim,recovered,strength);metric(strength==0?"Weak strength RMS":"Strong strength RMS",extreme.rms,"deg");CHECK(extreme.rms<2);CHECK(extreme.faceLostMax<3); }
    HybridFusion f;f.setStrength(0);near(f.timeConstant(),20,1e-12);f.setStrength(1);near(f.timeConstant(),0.5,1e-12);f.setStrength(0.5);near(f.timeConstant(),std::sqrt(10.0),1e-12);
}
void gatingTests() {
    HybridFusion f;CalibrationResult c;c.latencyMs=0;f.setCalibration(c);SampleSI s;s.accel={0,9.80665,0};
    OpticalPose optical{true,10,0,0,1,2,3};
    for(int64_t t=0;t<2000000;t+=5000) { s.sequence++;f.addImuSample(t,s);if(t%35000==0) f.addOptical(t,optical);f.output(t); }
    auto before=f.output(1995000);OpticalPose outlier{true,130,0,0,1000,1000,1000};f.addOptical(2010000,outlier);
    s.sequence++;f.addImuSample(2010000,s);auto after=f.output(2010000);near(after.pose.yaw,before.pose.yaw,0.01);near(after.pose.x,before.pose.x,0.01);
    CHECK(f.diagnostics().rejectedOptical>0);
    // Persistent, consistent optical disagreement must eventually re-acquire,
    // without snapping or resetting toward zero.
    optical.yaw=60;double maxStep=0,last=after.pose.yaw;
    for(int64_t t=2015000;t<12000000;t+=5000) { s.sequence++;f.addImuSample(t,s);if(t%35000==0)f.addOptical(t,optical);
        auto o=f.output(t);maxStep=std::max(maxStep,std::abs(o.pose.yaw-last));last=o.pose.yaw; }
    CHECK(last>55);CHECK(maxStep<1);
    // Continued stillness may track a new small bias, but optical motion forbids it.
    HybridFusion biasFilter;biasFilter.setCalibration(c);OpticalPose stillPose{true,0,0,0,0,0,0};s.gyro={0,0.4*radians,0};
    for(int64_t t=0;t<90000000;t+=5000) { s.sequence++;biasFilter.addImuSample(t,s);if(t%35000==0)biasFilter.addOptical(t,stillPose);biasFilter.output(t); }
    CHECK(biasFilter.bias().y>0.25*radians && biasFilter.bias().y<0.41*radians);
    auto held=biasFilter.bias();s.gyro={0,2*radians,0};
    for(int64_t t=90000000;t<95000000;t+=5000) { s.sequence++;biasFilter.addImuSample(t,s);stillPose.yaw=2*double(t-90000000)*1e-6;
        if(t%35000==0)biasFilter.addOptical(t,stillPose);biasFilter.output(t); }
    near((biasFilter.bias()-held).norm(),0,1e-12);
}
void fallbackTests() {
    CalibrationResult calibration;calibration.latencyMs=0;
    HybridFusion f;f.setCalibration(calibration);SampleSI sample;sample.accel={0,9.80665,0};
    OpticalPose optical{true,0,0,0,0,0,0};
    f.addImuSample(0,sample);f.addOptical(0,optical);f.output(0);
    double previous=0,maxStep=0,heldYaw=0;
    for(int64_t t=5000;t<=1800000;t+=5000) {
        // IMU vanishes, then optical moves and also vanishes. Hold the last
        // output rather than returning to the obsolete last IMU orientation.
        if(t<100000) { sample.sequence++;f.addImuSample(t,sample); }
        if(t<1200000 && t%35000==0) { optical.yaw=t>200000 ? 8 : 0;f.addOptical(t,optical); }
        if(t==1200000) { optical.valid=false;f.addOptical(t,optical);heldYaw=previous; }
        auto o=f.output(t);maxStep=std::max(maxStep,std::abs(o.pose.yaw-previous));previous=o.pose.yaw;
        if(t>=1200000) near(o.pose.yaw,heldYaw,1e-10);
    }
    CHECK(heldYaw>7.9);CHECK(maxStep<1);
    // First IMU connection after optical-only operation seeds from the latest
    // absolute pose, just like recovery from an established IMU connection.
    HybridFusion lateImu;lateImu.setCalibration(calibration);optical={true,0,0,0,0,0,0};
    lateImu.addOptical(0,optical);lateImu.output(0);
    for(int64_t t=5000;t<1500000;t+=5000) {
        if(t%35000==0) { optical.yaw=8;lateImu.addOptical(t,optical); }
        if(t>=800000) { sample.sequence++;lateImu.addImuSample(t,sample); }
        auto pose=lateImu.output(t);if(t>850000) near(pose.pose.yaw,8,0.02);
    }
    // A clipped gyro is excluded both from integration and the next trapezoid.
    HybridFusion clipped;clipped.setCalibration(calibration);sample={};sample.accel={0,9.80665,0};
    clipped.addImuSample(0,sample);clipped.output(0);
    sample.flags=2;sample.gyro={1000*radians,0,0};clipped.addImuSample(5000,sample);auto a=clipped.output(5000);
    sample.flags=0;sample.gyro={};clipped.addImuSample(10000,sample);auto b=clipped.output(10000);
    near(a.pose.pitch,0,1e-12);near(b.pose.pitch,0,1e-12);
    // Translation uses elapsed time rather than the number of output calls.
    auto translationAt=[&](int64_t cadence) {
        HybridFusion filter;filter.setCalibration(calibration);OpticalPose p{true,0,0,0,0,0,0};
        filter.addOptical(0,p);filter.output(0);p.x=10;filter.addOptical(1,p);
        FusionOutput o;
        for(int64_t t=cadence;t<=200000;t+=cadence) o=filter.output(t);
        return o.pose.x;
    };
    near(translationAt(5000),10*(1-std::exp(-0.2/0.08)),1e-10);
    near(translationAt(10000),translationAt(5000),1e-10);
}
}
int main() {
    int failed=0;
    for(const auto& test:std::vector<std::pair<std::string,std::function<void()>>>{{"protocol / scaling",protocolTests},{"quaternion / pose",mathTests},
        {"clock sync",clockTests},{"calibration",calibrationTests},{"fusion simulator",fusionTests},{"gating / re-acquisition / bias",gatingTests},
        {"fallback / saturation / translation",fallbackTests}}) {
        try { test.second();std::cout<<"PASS "<<test.first<<'\n'; }
        catch(const std::exception& e) { ++failed;std::cout<<"FAIL "<<test.first<<": "<<e.what()<<'\n'; }
    }
    std::cout<<"\n"<<std::left<<std::setw(34)<<"Metric"<<std::right<<std::setw(12)<<"Value"<<"  Unit\n";
    for(auto m:metrics) std::cout<<std::left<<std::setw(34)<<m.name<<std::right<<std::fixed<<std::setprecision(4)<<std::setw(12)<<m.value<<"  "<<m.unit<<'\n';
    std::cout<<'\n'<<(failed?"FAILED":"ALL TESTS PASSED")<<" ("<<failed<<" failing groups)\n";return failed?1:0;
}
