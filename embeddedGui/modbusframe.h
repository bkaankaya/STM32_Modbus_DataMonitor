#ifndef MODBUSFRAME_H
#define MODBUSFRAME_H

#include <cstddef>
#include <cstdint>

// Small, Qt-independent validator for a response to one Function 0x03 request.
// The caller keeps incoming bytes until Complete or Invalid is returned.
namespace modbus {
enum class FrameStatus { NeedMore, Complete, Invalid };

inline std::uint16_t crc16(const std::uint8_t *data, std::size_t size)
{
    std::uint16_t crc = 0xFFFF;
    for (std::size_t i = 0; i < size; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 1U) ? static_cast<std::uint16_t>((crc >> 1) ^ 0xA001U)
                             : static_cast<std::uint16_t>(crc >> 1);
        }
    }
    return crc;
}

inline FrameStatus inspectResponse(const std::uint8_t *data, std::size_t size,
                                   std::uint8_t expectedSlave, std::uint16_t requestedRegisters)
{
    if (requestedRegisters == 0 || requestedRegisters > 125 || size > 256 || (!data && size))
        return FrameStatus::Invalid;
    if (size == 0)
        return FrameStatus::NeedMore;
    if (data[0] != expectedSlave)
        return FrameStatus::Invalid;
    if (size < 2)
        return FrameStatus::NeedMore;
    const std::uint8_t function = data[1];
    if (function != 0x03 && function != 0x83)
        return FrameStatus::Invalid;
    if (size < 3)
        return FrameStatus::NeedMore;

    const std::size_t fullSize = function == 0x83 ? 5U : 5U + requestedRegisters * 2U;
    if (function == 0x03 && data[2] != requestedRegisters * 2U)
        return FrameStatus::Invalid;
    if (size > fullSize)
        return FrameStatus::Invalid;
    if (size < fullSize)
        return FrameStatus::NeedMore;

    const std::uint16_t expectedCrc = crc16(data, size - 2);
    const std::uint16_t receivedCrc = static_cast<std::uint16_t>(data[size-2]) |
                            (static_cast<std::uint16_t>(data[size-1]) << 8);
    return expectedCrc == receivedCrc ? FrameStatus::Complete : FrameStatus::Invalid;
}
} // namespace modbus

#endif // MODBUSFRAME_H
