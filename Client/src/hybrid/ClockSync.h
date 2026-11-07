#pragma once
#include <cstdint>
#include <deque>
namespace hybrid {
// Arrival clocks must be steady, in microseconds. Unknown constant transport
// delay is unobservable; the fit follows the minimum-delay envelope instead.
class ClockSync {
public:
    int64_t observe(uint64_t mcuUs,int64_t arrivalHostUs,bool discontinuity=false);
    int64_t map(uint64_t mcuUs) const;
    void reset();
    double driftPpm() const { return (slope_-1)*1e6; }
    uint64_t resets() const { return resets_; }
private:
    struct Point { double x,y; };
    std::deque<Point> points_;
    uint64_t origin_=0,lastMcu_=0,resets_=0;
    int64_t hostOrigin_=0,lastHost_=0;
    double slope_=1,offset_=0;
    bool ready_=false;
};
}
