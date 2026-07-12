#ifndef PACEJKA_H
#define PACEJKA_H

#include <cstdint>
#include <math.h>

constexpr float pi = 3.14159265359f;

// Pacejka general formula coefficients
struct TireConfig
{
    float D, C, B, E;
};

// Unpadded steering command to be sent to MCU
#pragma pack(push,1)
struct SteerCommandPacket
{
    uint16_t header;
    uint32_t sequence_id;
    float target_angle;
    uint8_t checksum;
};
#pragma pack(pop)

#endif // PACEJKA_H
