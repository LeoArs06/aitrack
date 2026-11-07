#include "ClockSync.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
namespace hybrid {
void ClockSync::reset() { points_.clear(); ready_=false; slope_=1; offset_=0; ++resets_; }
int64_t ClockSync::map(uint64_t t) const {
    double x=t>=origin_ ? double(t-origin_) : -double(origin_-t);
    return hostOrigin_+int64_t(std::llround(offset_+slope_*x));
}
int64_t ClockSync::observe(uint64_t t,int64_t host,bool discontinuity) {
    if(ready_ && (discontinuity || t<=lastMcu_ || host<lastHost_ ||
        std::abs(double(host-lastHost_)-double(t-lastMcu_))>2000000)) reset();
    if(!ready_) { ready_=true; origin_=t; hostOrigin_=host; }
    double dt=double(t-lastMcu_)*1e-6;
    lastMcu_=t; lastHost_=host;
    double x=double(t-origin_),y=double(host-hostOrigin_);
    points_.push_back({x,y});
    while(!points_.empty() && x-points_.front().x>20000000) points_.pop_front();
    // Select the lowest residual in each 250 ms bucket, then regress the
    // envelope. Centred coordinates avoid precision loss on long-running clocks.
    std::vector<Point> low;
    for(const auto& p:points_) {
        if(low.empty() || std::floor(p.x/250000)!=std::floor(low.back().x/250000)) low.push_back(p);
        else if(p.y-slope_*p.x < low.back().y-slope_*low.back().x) low.back()=p;
    }
    if(low.size()>=8) {
        double mx=0,my=0; for(auto p:low) { mx+=p.x; my+=p.y; } mx/=low.size(); my/=low.size();
        double xx=0,xy=0; for(auto p:low) { xx+=(p.x-mx)*(p.x-mx); xy+=(p.x-mx)*(p.y-my); }
        double target=std::clamp(xy/std::max(1.0,xx),0.998,1.002);
        double a=1-std::exp(-std::clamp(dt,0.0,0.1)/3.0); slope_+=a*(target-slope_);
    }
    double floorOffset=std::numeric_limits<double>::infinity();
    for(auto p:low) floorOffset=std::min(floorOffset,p.y-slope_*p.x);
    offset_=floorOffset;
    return map(t);
}
}
