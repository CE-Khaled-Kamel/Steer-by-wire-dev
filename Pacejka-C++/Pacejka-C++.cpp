#include <iostream>
#include <math.h>
#include <conio.h>

using namespace std;

struct TireConfig
{
    float D, C, B, E;
};

float toRadian(float degree)
{
    float pi = acos(-1);
    return pi * degree/180.0;
}

float PacejkaForce(const TireConfig& config, const float slipDegree)
{
    float rad = toRadian(slipDegree);
    return config.D * sin( config.C * atan( config.B*rad - config.E*( config.B*rad - atan( config.B*rad ))));
}

const char Esc = 0x1B;

int main()
{
    // Flag to repeat or end process
    char repeat = '0';
    float slipDeg = 0, force = 0;
    // Configuration of a tire on dry asphalt
    TireConfig config = {1.00, 1.90, 10.00, 0.97};

    do
    {
        cout << "Enter Steering Slip Angle (degrees): ";
        cin >> slipDeg;

        force = PacejkaForce(config, slipDeg);

        cout << "Generated Lateral Tire Force: " << force << '\n';
        cout << '\n' << "Press any key to repeat, Esc to exit" << "\n\n";
        getchar();
        repeat = _getch();

    }while(repeat!=Esc);

    return 0;
}
