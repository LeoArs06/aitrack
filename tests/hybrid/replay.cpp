// Replay a hybrid_log CSV (ImuWorker format) through ClockSync, Calibration
// and HybridFusion. Usage: hybrid_replay log.csv [calStartSec] [out.csv]
// Prints the calibration outcome and writes the fused 200 Hz output next to
// the camera poses so real recordings, not the simulator, are the test.
#include "Calibration.h"
#include "ClockSync.h"
#include "HybridFusion.h"
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
using namespace hybrid;
struct Row { bool imu; int64_t host; SampleRaw raw; OpticalPose pose; };
static std::vector<std::string> split(const std::string& s) {
    std::vector<std::string> f; std::stringstream ss(s); std::string x;
    while(std::getline(ss,x,',')) f.push_back(x);
    while(f.size()<19) f.push_back("");
    return f;
}
int main(int argc,char** argv) {
    if(argc<2) { std::cerr<<"usage: hybrid_replay log.csv [calStartSec] [out.csv]\n"; return 2; }
    std::ifstream in(argv[1]); std::string line; std::getline(in,line);
    std::vector<Row> rows;
    while(std::getline(in,line)) {
        auto f=split(line); Row r{}; r.host=std::stoll(f[1]);
        if(f[0]=="imu") {
            r.imu=true; r.raw.seq=uint32_t(std::stoul(f[2])); r.raw.t_us=uint32_t(std::stoul(f[3]));
            r.raw.ax=int16_t(std::stoi(f[4]));r.raw.ay=int16_t(std::stoi(f[5]));r.raw.az=int16_t(std::stoi(f[6]));
            r.raw.temp_raw=int16_t(std::stoi(f[7]));r.raw.gx=int16_t(std::stoi(f[8]));r.raw.gy=int16_t(std::stoi(f[9]));
            r.raw.gz=int16_t(std::stoi(f[10]));r.raw.flags=uint8_t(std::stoi(f[11]));
        } else if(f[0]=="optical") {
            r.imu=false; r.pose.valid=f[12]=="1";
            r.pose.yaw=std::stod(f[13]);r.pose.pitch=std::stod(f[14]);r.pose.roll=std::stod(f[15]);
            r.pose.x=std::stod(f[16]);r.pose.y=std::stod(f[17]);r.pose.z=std::stod(f[18]);
        } else continue;
        rows.push_back(r);
    }
    // Unwrap MCU time/sequence and map to host time once.
    uint64_t tHigh=0,sHigh=0; uint32_t lastT=0,lastS=0; bool first=true; ClockSync clock;
    std::vector<int64_t> mapped(rows.size(),0); int64_t firstImu=0;
    for(size_t i=0;i<rows.size();++i) if(rows[i].imu) {
        auto& r=rows[i].raw;
        if(!first) { if(r.t_us<lastT) tHigh+=1ull<<32; if(r.seq<lastS) sHigh+=1ull<<32; }
        lastT=r.t_us; lastS=r.seq; r.timeUs=tHigh+r.t_us; r.sequence=sHigh+r.seq; r.discontinuity=first;
        mapped[i]=clock.observe(r.timeUs,rows[i].host,first); if(first) firstImu=mapped[i]; first=false;
    }
    double calStart=argc>2 ? std::stod(argv[2]) : -1;
    CalibrationResult cal; bool haveCal=false;
    if(calStart>=0) {
        Calibration c; int64_t t0=firstImu+int64_t(calStart*1e6); bool begun=false;
        for(size_t i=0;i<rows.size();++i) {
            int64_t t=rows[i].imu ? mapped[i] : rows[i].host;
            if(t<t0) continue;
            if(!begun) { c.begin(t0); begun=true; }
            if(t>t0+12500000) break;
            if(rows[i].imu) c.addImuSample(t,scale(rows[i].raw)); else c.addOptical(t,rows[i].pose);
        }
        if(c.finish()) { cal=c.result(); haveCal=true;
            auto q=cal.imuToHead; std::printf("CAL OK latency=%.0f ms quality=%.2f residual=%.1f dps bias=(%.2f %.2f %.2f) dps q=(%.3f %.3f %.3f %.3f)\n",
                cal.latencyMs,cal.quality,cal.residualDegPerSec,cal.bias.x*degrees,cal.bias.y*degrees,cal.bias.z*degrees,q.w,q.x,q.y,q.z); }
        else std::printf("CAL FAIL: %s\n",c.reason().c_str());
    }
    std::ofstream out(argc>3 ? argv[3] : "replay_out.csv");
    out<<"kind,t_s,yaw,pitch,roll,x,y,z,state\n";
    HybridFusion f; f.setStrength(0.5); if(haveCal) f.setCalibration(cal);
    AutoCalibrator autoCal; bool autoMode=calStart<0;
    int64_t nextOut=0;
    for(size_t i=0;i<rows.size();++i) {
        if(autoMode) {
            int64_t t=rows[i].imu ? mapped[i] : rows[i].host;
            if(rows[i].imu) autoCal.addImuSample(t,scale(rows[i].raw)); else autoCal.addOptical(t,rows[i].pose);
            bool fresh=autoCal.update(t);
            static int64_t lastPrint=0;
            if(fresh || t-lastPrint>5000000) { lastPrint=t; auto& c=autoCal.result();
                std::printf("t=%5.1f s %s progress=%.2f residual=%.2f latency=%.0f bias=(%.2f %.2f %.2f) q=(%.3f %.3f %.3f %.3f)\n",(t-firstImu)*1e-6,
                    fresh?"NEW ":(autoCal.ready()?"ok  ":"learn"),autoCal.progress(),autoCal.relativeResidual(),c.latencyMs,
                    c.bias.x*degrees,c.bias.y*degrees,c.bias.z*degrees,c.imuToHead.w,c.imuToHead.x,c.imuToHead.y,c.imuToHead.z); }
            if(fresh) { if(!haveCal) { f=HybridFusion{}; f.setStrength(0.5); } f.setCalibration(autoCal.result()); haveCal=true; } // as ImuWorker::updateCalibration
        }
        if(rows[i].imu) {
            int64_t t=mapped[i]; f.addImuSample(t,scale(rows[i].raw));
            if(!haveCal) continue;
            if(t>=nextOut) { auto o=f.output(t); nextOut=t+5000;
                if(o.valid) out<<"fused,"<<(t-firstImu)*1e-6<<','<<o.pose.yaw<<','<<o.pose.pitch<<','<<o.pose.roll<<','
                    <<o.pose.x<<','<<o.pose.y<<','<<o.pose.z<<','<<stateName(o.state)<<'\n'; }
        } else {
            f.addOptical(rows[i].host,rows[i].pose);
            auto& p=rows[i].pose; if(p.valid) out<<"camera,"<<(rows[i].host-firstImu)*1e-6<<','<<p.yaw<<','<<p.pitch<<','<<p.roll<<','<<p.x<<','<<p.y<<','<<p.z<<",\n";
        }
    }
    auto d=f.diagnostics(); std::printf("FUSION imu=%.1f Hz optical=%.1f Hz drift=%.1f deg/min rejected=%llu state=%s\n",
        d.imuHz,d.opticalHz,d.driftDegPerMin,(unsigned long long)d.rejectedOptical,stateName(d.state));
    return 0;
}
