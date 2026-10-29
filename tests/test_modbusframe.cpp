#include "../embeddedGui/modbusframe.h"
#include <cassert>
#include <cstdint>
#include <vector>
#include <utility>
#include <array>

static std::vector<std::uint8_t> response(std::uint8_t function,
                                          const std::vector<std::uint8_t>& payload)
{
    std::vector<std::uint8_t> bytes{1,function};
    bytes.insert(bytes.end(), payload.begin(), payload.end());
    const auto crc=modbus::crc16(bytes.data(),bytes.size());
    bytes.push_back(static_cast<std::uint8_t>(crc&0xff));
    bytes.push_back(static_cast<std::uint8_t>(crc>>8));
    return bytes;
}
int main()
{
    using modbus::FrameStatus;
    // Known CRC16/Modbus vector, 01 03 00 00 00 02 -> 0x0BC4.
    const std::uint8_t request[]{0x01,0x03,0,0,0,2};
    assert(modbus::crc16(request,6)==0x0BC4);
    const auto one=response(3,{2,0x00,0x01});
    const auto two=response(3,{4,0,1,0,2});
    const auto all=response(3,{10,0,1,0,2,0,3,0,4,0,5});
    const auto exception=response(0x83,{2});
    for(const auto &p: std::array<std::pair<std::vector<std::uint8_t>, unsigned>, 4>{{{one,1U},{two,2U},{all,5U},{exception,5U}}}) {
        for(std::size_t n=0;n<p.first.size();++n)
            assert(modbus::inspectResponse(p.first.data(),n,1,p.second)==FrameStatus::NeedMore);
        assert(modbus::inspectResponse(p.first.data(),p.first.size(),1,p.second)==FrameStatus::Complete);
        auto damaged=p.first;damaged.back()^=0x10;
        assert(modbus::inspectResponse(damaged.data(),damaged.size(),1,p.second)==FrameStatus::Invalid);
        auto extra=p.first;extra.push_back(0x00);
        assert(modbus::inspectResponse(extra.data(),extra.size(),1,p.second)==FrameStatus::Invalid);
    }
    assert(modbus::inspectResponse(one.data(),one.size(),2,1)==FrameStatus::Invalid);
    assert(modbus::inspectResponse(two.data(),two.size(),1,1)==FrameStatus::Invalid);
    const auto wrongFunction=response(0x04,{2,0,1});
    assert(modbus::inspectResponse(wrongFunction.data(),wrongFunction.size(),1,1)==FrameStatus::Invalid);
    assert(modbus::inspectResponse(one.data(),one.size(),1,0)==FrameStatus::Invalid);
}
