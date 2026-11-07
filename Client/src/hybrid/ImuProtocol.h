#pragma once
#include <cstddef>
#include <cstdint>
#include <variant>
#include <vector>
namespace hybrid {
struct SampleRaw {
    uint32_t seq=0,t_us=0;
    int16_t ax=0,ay=0,az=0,temp_raw=0,gx=0,gy=0,gz=0;
    uint8_t flags=0,reserved=0;
    uint64_t sequence=0,timeUs=0; // host-side unwrapped values; not encoded
    bool discontinuity=false;
};
struct Info {
    uint32_t magic=0x31495448;
    uint8_t version=1,major=0,minor=0,whoAmI=0x68,gyroFS=2,accelFS=1,dlpf=3,divider=4;
    uint16_t sampleHz=200,i2cKHz=400;
    uint32_t i2cErrors=0,reinitializations=0,txDrops=0;
};
struct Status { uint8_t code=0,whoAmI=0; uint16_t reserved=0; uint32_t i2cErrors=0; };
using ImuMessage=std::variant<SampleRaw,Info,Status>;
struct ProtocolCounters { uint64_t frames=0,crcErrors=0,resyncs=0,lost=0,malformed=0,discontinuities=0; };
uint16_t crc16(const uint8_t* data,size_t size);
std::vector<uint8_t> encode(const SampleRaw& s);
std::vector<uint8_t> encode(const Info& s);
std::vector<uint8_t> encode(const Status& s);
class ImuProtocol {
public:
    std::vector<ImuMessage> feed(const uint8_t* data,size_t size);
    std::vector<ImuMessage> feed(const std::vector<uint8_t>& data) { return feed(data.data(),data.size()); }
    const ProtocolCounters& counters() const { return counters_; }
    void reset();
private:
    void consume(uint8_t b,std::vector<ImuMessage>& out);
    std::vector<uint8_t> buffer_;
    ProtocolCounters counters_;
    bool haveSample_=false;
    uint32_t seq_=0,time_=0;
    uint64_t sequence_=0,timeUs_=0;
};
}
