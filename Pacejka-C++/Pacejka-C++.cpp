#include <iostream>
#include <math.h>
#include <conio.h>
#include <limits>
#include <iomanip>
#include <chrono>

using namespace std;

const float pi = acos(-1);

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

// Calculate and assign packet checksum
void assignChecksum(SteerCommandPacket& p)
{
    // Reinterpret the packet as bytes
    const uint8_t* byte_ptr = reinterpret_cast<const uint8_t*>(&p);
    size_t num_bytes = sizeof(p);
    num_bytes--;  // Exclude the checksum itself from the calculation

    uint8_t checksum = 0;
    for(size_t i=0; i< num_bytes; ++i)
    {
        checksum ^= byte_ptr[i];
    }

    // Assign checksum
    p.checksum = checksum;
}

// Generate a linear space between a and b (a<b)
void linspace(float* arr, int num_elements, float a, float b)
{
    // Calculate linear step
    float step = fabs(a-b)/(num_elements-1);

    for(int i=0; i<num_elements; ++i)
    {
        arr[i] = a + i*step;
    }

    return;
}

inline float toRadian(float degree)
{
    return pi * degree/180.0;
}

void PacejkaForce(const TireConfig& config, const float* slipAngles, float* output_forces, int num_elements)
{
    float rad = 0;
    for(int i=0; i<num_elements; ++i)
    {

        rad = toRadian(slipAngles[i]);
        output_forces[i] = config.D * sin( config.C * atan( config.B*rad - config.E*( config.B*rad - atan( config.B*rad ))));
    }

    return ;
}

// Find smallest angle with maximum force
float OptimalAngle(const float* forces, const float* angles, int num_elements)
{
    float maxForce = std::numeric_limits<float>::lowest();
    float minAngle = std::numeric_limits<float>::max();

    float* optimalAngles = new float[num_elements] {0};
    int anglesCount = 0;

    // Find maximum force
    for(int i=0; i <num_elements; ++i)
    {
        if(maxForce < forces[i])
        {
            maxForce = forces[i];
        }
    }

    // look for multiples maximums
    for(int i=0; i <num_elements; ++i)
    {
        if(maxForce == forces[i])
        {
            optimalAngles[anglesCount] = angles[i];
            ++anglesCount;
        }
    }

    // Choose the smallest angle with maximum force
    for(int i=0; i <anglesCount; ++i)
    {
        if(minAngle > optimalAngles[i])
        {
            minAngle = optimalAngles[i];
        }
    }

    delete[] optimalAngles;

    return minAngle;
}

const char Esc = 0x1B;

int main()
{
    int num_elements = 5000;
    // Slip angle range
    float a = 0.0, b =20.0;
    float angles[num_elements] = {0};
    float forces[num_elements] = {0};
    float optAngle = 0;
    SteerCommandPacket p = {0xAA55, 0x00000001, 0x0,0x0};
    // Configuration of a tire on dry asphalt
    TireConfig config = {1.00, 1.90, 10.00, 0.97};

    linspace(angles, num_elements, a, b);

    chrono::time_point<chrono::steady_clock> Start = chrono::steady_clock::now();
    PacejkaForce(config, angles, forces, num_elements);
    optAngle = OptimalAngle(forces, angles, num_elements);
    chrono::time_point<chrono::steady_clock> End = chrono::steady_clock::now();

    chrono::duration<double, micro> elapsed = End - Start;
    cout << "Time: " << elapsed.count() << "us\n";

    p.target_angle = optAngle;
    assignChecksum(p);
    const uint8_t* byte_ptr = reinterpret_cast<const uint8_t*>(&p);
    size_t num_bytes = sizeof(p);

    cout << "Optimal Steering Angle: " << optAngle << '\n';
    cout << "Steering Packet: ";
    for(size_t i=0; i<num_bytes; ++i)
    {
        cout << hex << static_cast<int>(byte_ptr[i]) << " ";
    }

    return 0;
}
