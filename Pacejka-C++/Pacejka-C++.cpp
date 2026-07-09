#include <iostream>
#include <math.h>
#include <conio.h>
#include <limits>

using namespace std;

const float pi = acos(-1);

// Pacejka general formula coefficients
struct TireConfig
{
    float D, C, B, E;
};

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
    int num_elements = 50;
    // Slip angle range
    float a = 0.0, b =20.0;
    float angles[num_elements] = {0};
    float forces[num_elements] = {0};
    // Configuration of a tire on dry asphalt
    TireConfig config = {1.00, 1.90, 10.00, 0.97};

    linspace(angles, num_elements, a, b);
    PacejkaForce(config, angles, forces, num_elements);
    cout<<OptimalAngle(forces, angles, num_elements);

    return 0;
}
