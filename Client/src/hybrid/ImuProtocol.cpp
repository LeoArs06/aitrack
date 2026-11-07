#include "ImuProtocol.h"
#include <algorithm>
namespace hybrid {
namespace {
uint16_t u16(const uint8_t* p) { return uint16_t(p[0])|uint16_t(p[1])<<8; }
uint32_t u32(const uint8_t* p) { return uint32_t(u16(p))|uint32_t(u16(p+2))<<16; }
void put(std::vector<uint8_t>& p,uint32_t v,int n) { for(int i=0;i<n;++i) p.push_back(uint8_t(v>>(8*i))); }
std::vector<uint8_t> frame(uint8_t type,const std::vector<uint8_t>& p) {
    std::vector<uint8_t> b{0xA5,0x5A,type,uint8_t(p.size())}; b.insert(b.end(),p.begin(),p.end());
    auto crc=crc16(b.data()+2,b.size()-2); put(b,crc,2); return b;
}
}
uint16_t crc16(const uint8_t* data,size_t size) {
    uint16_t c=0xFFFF;
    for(size_t i=0;i<size;++i) { c^=uint16_t(data[i])<<8; for(int j=0;j<8;++j) c=uint16_t(c&0x8000 ? (c<<1)^0x1021 : c<<1); }
    return c;
}
std::vector<uint8_t> encode(const SampleRaw& s) {
    std::vector<uint8_t> p; put(p,s.seq,4); put(p,s.t_us,4);
    for(int16_t v : {s.ax,s.ay,s.az,s.temp_raw,s.gx,s.gy,s.gz}) put(p,uint16_t(v),2);
    put(p,s.flags,1); put(p,s.reserved,1); return frame(1,p);
}
std::vector<uint8_t> encode(const Info& s) {
    std::vector<uint8_t> p; put(p,s.magic,4);
    for(uint8_t v : {s.version,s.major,s.minor,s.whoAmI,s.gyroFS,s.accelFS,s.dlpf,s.divider}) put(p,v,1);
    put(p,s.sampleHz,2); put(p,s.i2cKHz,2); put(p,s.i2cErrors,4); put(p,s.reinitializations,4); put(p,s.txDrops,4); return frame(2,p);
}
std::vector<uint8_t> encode(const Status& s) {
    std::vector<uint8_t> p; put(p,s.code,1); put(p,s.whoAmI,1); put(p,s.reserved,2); put(p,s.i2cErrors,4); return frame(3,p);
}
void ImuProtocol::reset() { *this=ImuProtocol{}; }
std::vector<ImuMessage> ImuProtocol::feed(const uint8_t* data,size_t size) {
    std::vector<ImuMessage> out;
    // Byte-wise ingestion bounds pending storage to the maximum frame size (70).
    for(size_t i=0;i<size;++i) consume(data[i],out);
    return out;
}
void ImuProtocol::consume(uint8_t b,std::vector<ImuMessage>& out) {
    buffer_.push_back(b);
    for(;;) {
        if(buffer_.empty()) return;
        if(buffer_[0]!=0xA5 || (buffer_.size()>1 && buffer_[1]!=0x5A)) {
            buffer_.erase(buffer_.begin()); ++counters_.resyncs; continue;
        }
        if(buffer_.size()<4) return;
        size_t n=buffer_[3];
        if(n>64) { ++counters_.malformed; ++counters_.resyncs; buffer_.erase(buffer_.begin()); continue; }
        if(buffer_.size()<n+6) return;
        if(crc16(buffer_.data()+2,n+2)!=u16(buffer_.data()+4+n)) {
            ++counters_.crcErrors; ++counters_.resyncs; buffer_.erase(buffer_.begin()); continue;
        }
        ++counters_.frames;
        const auto* p=buffer_.data()+4; uint8_t type=buffer_[2]; bool valid=true;
        if(type==1 && n==24) {
            SampleRaw s; s.seq=u32(p); s.t_us=u32(p+4);
            // Explicit two's-complement conversion, including -32768, portable C++17.
            auto signed16=[](const uint8_t* a) { int v=u16(a); return int16_t(v>=32768 ? v-65536 : v); };
            s.ax=signed16(p+8); s.ay=signed16(p+10); s.az=signed16(p+12); s.temp_raw=signed16(p+14);
            s.gx=signed16(p+16); s.gy=signed16(p+18); s.gz=signed16(p+20); s.flags=p[22]; s.reserved=p[23];
            uint32_t ds=s.seq-seq_,dt=s.t_us-time_;
            s.discontinuity=haveSample_ && ((s.flags&4)!=0 || ds>=0x80000000u || dt>=0x80000000u);
            if(!haveSample_ || s.discontinuity) {
                sequence_=s.seq; timeUs_=s.t_us;
                if(s.discontinuity) ++counters_.discontinuities;
            } else {
                sequence_+=ds; timeUs_+=dt;
                if(ds>0) counters_.lost+=ds-1;
            }
            seq_=s.seq; time_=s.t_us; haveSample_=true; s.sequence=sequence_; s.timeUs=timeUs_; out.emplace_back(s);
        } else if(type==2 && n==28) {
            Info s; s.magic=u32(p); s.version=p[4]; s.major=p[5]; s.minor=p[6]; s.whoAmI=p[7];
            s.gyroFS=p[8]; s.accelFS=p[9]; s.dlpf=p[10]; s.divider=p[11]; s.sampleHz=u16(p+12); s.i2cKHz=u16(p+14);
            s.i2cErrors=u32(p+16); s.reinitializations=u32(p+20); s.txDrops=u32(p+24);
            if(s.magic==0x31495448 && s.version==1) out.emplace_back(s); else valid=false;
        } else if(type==3 && n==8) { out.emplace_back(Status{p[0],p[1],u16(p+2),u32(p+4)}); }
        else valid=false;
        if(!valid) ++counters_.malformed;
        buffer_.erase(buffer_.begin(),buffer_.begin()+n+6);
    }
}
}
