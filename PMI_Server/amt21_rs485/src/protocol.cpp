#include "amt21/protocol.hpp"

namespace amt21 {

bool verifyResponseChecksum(uint16_t w)
{
    unsigned xorOdd = 0;
    unsigned xorEven = 0;
    for (int i = 1; i < 16; i += 2)
        xorOdd ^= (w >> i) & 1u;
    for (int i = 0; i < 16; i += 2)
        xorEven ^= (w >> i) & 1u;
    const unsigned k1 = (w >> 15) & 1u;
    const unsigned k0 = (w >> 14) & 1u;
    return k1 == (static_cast<unsigned>(!xorOdd) & 1u) && k0 == (static_cast<unsigned>(!xorEven) & 1u);
}

std::optional<uint16_t> decodePosition14(uint16_t rawLittleEndian)
{
    if (!verifyResponseChecksum(rawLittleEndian))
        return std::nullopt;
    return static_cast<uint16_t>(rawLittleEndian & 0x3FFFu);
}

std::optional<uint16_t> decodePosition12(uint16_t rawLittleEndian)
{
    if (!verifyResponseChecksum(rawLittleEndian))
        return std::nullopt;
    return static_cast<uint16_t>((rawLittleEndian & 0x3FFFu) >> 2);
}

} // namespace amt21
