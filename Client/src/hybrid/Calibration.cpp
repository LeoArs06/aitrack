#include "Calibration.h"
#include <algorithm>
#include <array>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
namespace hybrid {
namespace {
Vec3 square(Vec3 v) { return {v.x*v.x,v.y*v.y,v.z*v.z}; }
// Jacobi rotations diagonalize a real symmetric matrix without SVD or a library.
template<size_t N> void jacobi(double (&a)[N][N],double (&v)[N][N]) {
    for(size_t i=0;i<N;++i) for(size_t j=0;j<N;++j) v[i][j]=i==j ? 1.0 : 0.0;
    for(int sweep=0;sweep<80;++sweep) {
        size_t p=0,q=1; double largest=0;
        for(size_t i=0;i<N;++i) for(size_t j=i+1;j<N;++j) if(std::abs(a[i][j])>largest) { largest=std::abs(a[i][j]); p=i;q=j; }
        if(largest<1e-12) break;
        double angle=0.5*std::atan2(2*a[p][q],a[q][q]-a[p][p]),c=std::cos(angle),s=std::sin(angle);
        double ap=a[p][p],aq=a[q][q],off=a[p][q];
        a[p][p]=c*c*ap-2*c*s*off+s*s*aq; a[q][q]=s*s*ap+2*c*s*off+c*c*aq; a[p][q]=a[q][p]=0;
        for(size_t k=0;k<N;++k) {
            if(k!=p && k!=q) { double kp=a[k][p],kq=a[k][q]; a[k][p]=a[p][k]=c*kp-s*kq; a[k][q]=a[q][k]=s*kp+c*kq; }
            double vp=v[k][p],vq=v[k][q]; v[k][p]=c*vp-s*vq; v[k][q]=s*vp+c*vq;
        }
    }
}
Quat align(const std::vector<Vec3>& a,const std::vector<Vec3>& b) {
    double s[3][3]={};
    for(size_t k=0;k<a.size();++k) for(int i=0;i<3;++i) for(int j=0;j<3;++j) s[i][j]+=a[k][i]*b[k][j];
    double tr=s[0][0]+s[1][1]+s[2][2];
    double n[4][4]={{tr,s[1][2]-s[2][1],s[2][0]-s[0][2],s[0][1]-s[1][0]},
        {s[1][2]-s[2][1],2*s[0][0]-tr,s[0][1]+s[1][0],s[0][2]+s[2][0]},
        {s[2][0]-s[0][2],s[0][1]+s[1][0],2*s[1][1]-tr,s[1][2]+s[2][1]},
        {s[0][1]-s[1][0],s[0][2]+s[2][0],s[1][2]+s[2][1],2*s[2][2]-tr}};
    double v[4][4]; jacobi(n,v); int best=0; for(int i=1;i<4;++i) if(n[i][i]>n[best][best]) best=i;
    return Quat{v[0][best],v[1][best],v[2][best],v[3][best]}.normalized();
}
}
std::string CalibrationResult::serialize() const {
    std::ostringstream out; out.imbue(std::locale::classic()); out<<std::setprecision(17)<<"HTC1 "
        <<bias.x<<' '<<bias.y<<' '<<bias.z<<' '<<imuToHead.w<<' '<<imuToHead.x<<' '<<imuToHead.y<<' '<<imuToHead.z<<' '
        <<latencyMs<<' '<<quality<<' '<<residualDegPerSec<<' '<<excitationDegPerSec.x<<' '<<excitationDegPerSec.y<<' '<<excitationDegPerSec.z;
    return out.str();
}
bool CalibrationResult::deserialize(const std::string& text,CalibrationResult& result) {
    std::istringstream in(text); in.imbue(std::locale::classic()); std::string tag; CalibrationResult r;
    if(!(in>>tag>>r.bias.x>>r.bias.y>>r.bias.z>>r.imuToHead.w>>r.imuToHead.x>>r.imuToHead.y>>r.imuToHead.z
        >>r.latencyMs>>r.quality>>r.residualDegPerSec>>r.excitationDegPerSec.x>>r.excitationDegPerSec.y>>r.excitationDegPerSec.z) || tag!="HTC1") return false;
    in>>std::ws; if(!in.eof()) return false;
    if(!r.bias.finite() || !r.imuToHead.finite() || std::abs(r.imuToHead.dot(r.imuToHead)-1)>0.001 ||
        !std::isfinite(r.latencyMs) || r.latencyMs<0 || r.latencyMs>250 || !std::isfinite(r.quality) || r.quality<0 || r.quality>1 ||
        !std::isfinite(r.residualDegPerSec) || r.residualDegPerSec<0 || !r.excitationDegPerSec.finite()) return false;
    r.imuToHead=r.imuToHead.normalized(); result=r; return true;
}
void Calibration::begin(int64_t t) { *this=Calibration{}; start_=t; phase_=Phase::Still; }
bool Calibration::fail(const std::string& reason) { reason_=reason; phase_=Phase::Failed; return false; }
bool Calibration::endStill() {
    if(stillCount_<200) return fail("Too few IMU samples during the 2 s still phase");
    double n=double(stillCount_); Vec3 w=sumW_/n,a=sumA_/n;
    Vec3 vw=sumW2_/n-square(w),va=sumA2_/n-square(a);
    double wg=std::sqrt(std::max(0.0,vw.x+vw.y+vw.z)),ag=std::sqrt(std::max(0.0,va.x+va.y+va.z));
    if(w.norm()>3*radians || wg>0.8*radians || ag>0.20 || std::abs(a.norm()-9.80665)>0.8)
        return fail("Movement or invalid gravity during still phase; hold still and retry");
    result_.bias=w; phase_=Phase::Motion; return true;
}
void Calibration::addImuSample(int64_t t,const SampleSI& s) {
    if(phase_!=Phase::Still && phase_!=Phase::Motion) return;
    if(!s.gyro.finite() || !s.accel.finite() || (s.flags&7) || s.discontinuity) { fail("Invalid, saturated or restarted IMU during calibration"); return; }
    if(!gyro_.empty() && t<=gyro_.back().t) return;
    if(phase_==Phase::Still && t-start_>=2000000 && !endStill()) return;
    if(phase_==Phase::Still) { sumW_+=s.gyro; sumW2_+=square(s.gyro); sumA_+=s.accel; sumA2_+=square(s.accel); ++stillCount_; }
    gyro_.push_back({t,s.gyro});
}
void Calibration::addOptical(int64_t t,const OpticalPose& p) {
    if(phase_!=Phase::Motion || !p.valid || !std::isfinite(p.yaw) || !std::isfinite(p.pitch) || !std::isfinite(p.roll)) return;
    if(!optical_.empty() && t<=optical_.back().t) return;
    optical_.push_back({t,poseQuaternion(p.yaw,p.pitch,p.roll)});
}
bool Calibration::meanGyro(int64_t a,int64_t b,Vec3& w) const {
    if(gyro_.empty() || a<gyro_.front().t || b>gyro_.back().t || b<=a) return false;
    auto it=std::upper_bound(gyro_.begin(),gyro_.end(),a,[](int64_t t,const Gyro& g){return t<g.t;});
    size_t i=size_t(it-gyro_.begin())-1; Vec3 integral;
    while(i+1<gyro_.size() && gyro_[i].t<b) {
        auto g=gyro_[i],h=gyro_[i+1]; if(h.t-g.t>30000) return false;
        int64_t lo=std::max(a,g.t),hi=std::min(b,h.t); double span=double(h.t-g.t);
        Vec3 vl=g.w+(h.w-g.w)*(double(lo-g.t)/span),vh=g.w+(h.w-g.w)*(double(hi-g.t)/span);
        integral+=(vl+vh)*(0.5*double(hi-lo)); ++i;
    }
    w=integral/double(b-a)-result_.bias; return true;
}
bool Calibration::finish() {
    if(phase_!=Phase::Motion) return false;
    if(gyro_.empty() || gyro_.back().t-start_<10000000 || optical_.size()<180) return fail("Need about 10 s of motion with the face visible");
    struct Rate { int64_t a,b; Vec3 w; };
    std::vector<Rate> rates;
    // Four-frame differences suppress optical noise. Match the same integrated
    // gyro interval at each candidate delay (midpoint timestamps, no phase lag).
    for(size_t i=4;i<optical_.size();++i) {
        auto a=optical_[i-4],b=optical_[i]; double dt=double(b.t-a.t)*1e-6;
        if(dt<0.09 || dt>0.18) continue;
        Vec3 w=(a.q.conj()*b.q).log()/dt;
        if(w.norm()<5*radians || w.norm()>350*radians) continue;
        rates.push_back({a.t,b.t,w});
    }
    double best=-2; int delay=0;
    for(int ms=0;ms<=250;++ms) {
        double sx=0,sy=0,sxx=0,syy=0,sxy=0; size_t n=0;
        for(auto r:rates) { Vec3 w; if(!meanGyro(r.a-ms*1000,r.b-ms*1000,w)) continue;
            double x=w.norm(),y=r.w.norm(); sx+=x;sy+=y;sxx+=x*x;syy+=y*y;sxy+=x*y;++n; }
        if(n<100) continue;
        double score=(sxy-sx*sy/n)/std::sqrt(std::max(1e-15,(sxx-sx*sx/n)*(syy-sy*sy/n)));
        if(score>best) { best=score; delay=ms; }
    }
    if(best<0.75) return fail("Rates do not correlate; vary head speed with the face visible");
    std::vector<Vec3> a,b;
    for(auto r:rates) { Vec3 w; if(meanGyro(r.a-delay*1000,r.b-delay*1000,w)) { a.push_back(w); b.push_back(r.w); } }
    if(a.size()<100) return fail("Insufficient aligned motion samples");
    Quat rotation=align(a,b); double residual=0,energy=0,cov[3][3]={}; Vec3 rms;
    for(size_t i=0;i<a.size();++i) {
        Vec3 e=rotation.rotate(a[i])-b[i]; residual+=e.dot(e);energy+=b[i].dot(b[i]);rms+=square(b[i]);
        for(int j=0;j<3;++j) for(int k=0;k<3;++k) cov[j][k]+=b[i][j]*b[i][k]/a.size();
    }
    double eigenvectors[3][3]; jacobi(cov,eigenvectors);
    double minExcitation=std::sqrt(std::max(0.0,std::min({cov[0][0],cov[1][1],cov[2][2]})));
    if(minExcitation<6*radians) return fail("Insufficient excitation: move left-right, up-down AND tilt");
    double relative=std::sqrt(residual/std::max(1e-12,energy));
    if(relative>0.30) return fail("Alignment residual too high; check rigid mounting and optical tracking");
    result_.imuToHead=rotation;result_.latencyMs=delay;result_.quality=std::clamp(best*(1-relative),0.0,1.0);
    result_.residualDegPerSec=std::sqrt(residual/a.size())*degrees;
    rms=rms/double(a.size()); result_.excitationDegPerSec={std::sqrt(rms.x)*degrees,std::sqrt(rms.y)*degrees,std::sqrt(rms.z)*degrees};
    phase_=Phase::Complete; reason_.clear(); return true;
}
}
