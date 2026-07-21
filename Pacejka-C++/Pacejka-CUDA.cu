#include "Pacejka.cuh"
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <conio.h>
#include <limits>
#include <iomanip>
#include <chrono>
#include <thread>
#include <winsock2.h>

#pragma comment(lib, "ws2_32.lib")

using namespace std;

#ifdef DEBUG
    #define DEBUG_PRINT(x) std::cout << "[DEBUG] " << x
#else
    #define DEBUG_PRINT(x)
#endif // DEBUG

__global__ void Pacejka_Kernel(const TireConfig* config, const float* slipAngles, float* output_forces, int num_elements)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;

    float rad;
    if(i < num_elements)
    {
        rad = pi * slipAngles[i]/180.0f;
        output_forces[i] = config->D * sinf( config->C * atanf(config->B*rad - config->E*( config->B*rad - atanf( config->B*rad ))));
    }

    return;
}

cudaError_t PacejkaForce_Cuda(const TireConfig& config, const float* slipAngles, float* output_forces, DeviceBuffer<TireConfig>* d_config, DeviceBuffer<float>* d_Angles, DeviceBuffer<float>* d_forces, int num_elements)
{
    // Copy data from CPU to GPU
    CUDA_CALL( cudaMemcpy(d_Angles->get(), slipAngles, num_elements * sizeof(float), cudaMemcpyHostToDevice));
    CUDA_CALL( cudaMemcpy(d_config->get(), &config, sizeof(TireConfig), cudaMemcpyHostToDevice));

    // Grid dimensions
    int nThreads = 256;
    int nBlocks = (num_elements + nThreads - 1)/nThreads;
    // Kernel launch
    Pacejka_Kernel<<< nBlocks, nThreads >>> (d_config->get(), d_Angles->get(), d_forces->get(), num_elements);

    // Check for launching errors
    CUDA_CALL( cudaGetLastError() );

    // Wait for kernel to finish
    CUDA_CALL( cudaDeviceSynchronize() );

    // Copy data back to CPU
    CUDA_CALL( cudaMemcpy(output_forces, d_forces->get(), num_elements * sizeof(float), cudaMemcpyDeviceToHost));

    // Wrapper class automatically deallocates device memory

    return cudaSuccess;
}

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

// Find smallest angle with maximum force
float OptimalAngle(const float* forces, const float* angles, int num_elements)
{
    float maxForce = std::numeric_limits<float>::lowest();
    float peakAngle = std::numeric_limits<float>::max();

    // Find maximum force
    for(int i=0; i <num_elements; ++i)
    {
        if(maxForce < fabs(forces[i]))
        {
            maxForce = fabs(forces[i]);
            peakAngle = angles[i];
        }
    }

    return peakAngle;
}

const char Esc = 0x1B;
const bool Headless = true;
const uint32_t printRate = 100; // cycles
const chrono::duration<int, micro> targetFrameTime(10000); // Total frame time
const chrono::duration<int, micro> dropLimit (500); // Time tolerance for droped frames
int cycles = 2000;
const int wsaVersionLO = 2;
const int wsaVersionHi = 2;


int main()
{
    const int num_elements = 5000;
    uint8_t* byte_ptr = nullptr;
    size_t num_bytes = 0;
    size_t i = 0;
    uint32_t droppedFrames = 0;

    chrono::steady_clock::time_point Start;
    chrono::steady_clock::time_point End;
    chrono::steady_clock::time_point prevStart;
    chrono::duration<int, micro> elapsed;
    chrono::duration<int, micro> frameTime;

    char RecvBuf[1024];
    int BufLen = 1024;
    sockaddr_in senderAddr;
    int senderAddrSize = sizeof(senderAddr);
    IncomingTelemetry telemetry = {0.0f, 0.0f, 0.0f};
    size_t telemetrySize = sizeof(telemetry);

    // Slip angle range
    float a = -10.0f, b = 10.0f;
    float angles[num_elements] = {0};
    float forces[num_elements] = {0};
    float peakAngle = 0;
    SteerCommandPacket p = {0xAA55, 0x00000001, 0x0,0x0};

    float Ca = 50000.0f; // Cornering Stiffness(N/rad)
    float FzStatic = 3924.0f; // Vertical static (weight) downforce on one tire
    float airDensity =  1.225f;
    float CL = -0.20f; // Lift Coefficient
    float A = 2.0f; // Car Frontal Area
    float FzAero = 0; // Vertical aerodynamic downforce on one tire
    float Fztotal = FzStatic; // Total vertical downforce on one tire
    // Configuration of a tire on dry asphalt
    TireConfig config = {Fztotal, 1.30f, Ca / (1.30*Fztotal), 0.1f};

    // Choose which GPU to run on
    CUDA_CALL( cudaSetDevice(0) );

    // Allocate data memory on GPU
    DeviceBuffer<float> d_Angles(num_elements);
    DeviceBuffer<float> d_forces(num_elements);
    DeviceBuffer<TireConfig> d_config(1);

    WORD wVersionRequested;
    WSADATA wsaData;
    int err = 0;
    // Make a word out of requeted version
    wVersionRequested = MAKEWORD(wsaVersionLO,wsaVersionHi);
    // Initiate use of the Winsock DLL
    err = WSAStartup(wVersionRequested, &wsaData);
    if(err != 0)
    {
        cout << "WSASTARUP failed with error: " << err << endl;
        return 1;
    }
    // Confirm that the WinSock DLL supports requested version
    if(LOBYTE(wsaData.wVersion) != wsaVersionLO || HIBYTE(wsaData.wVersion) != wsaVersionHi)
    {
        cout << "Could not find requested version of Winsock.dll \n";
        WSACleanup();
        return 1;
    }
    else
        cout << "Winsock ready.....\n";

    SOCKET sock = INVALID_SOCKET;
    int iFamily = AF_INET; // Address family specification
    int iType = SOCK_DGRAM; // Type specification for the new socket
    int iProtocol = IPPROTO_UDP; // The protocol to be used
    //  create a UDP socket
    sock = socket(iFamily, iType, iProtocol);
    if(sock == INVALID_SOCKET)
    {
        cout << "socket function failed with error: " << WSAGetLastError() << endl;
        return 1;
    }
    else
        cout << "socket acquired..... \n";

    sockaddr_in service;
    service.sin_family = AF_INET;
    service.sin_addr.s_addr = inet_addr("127.0.0.1");
    service.sin_port = htons(8080);
    // Associates a local address with the socket
    err = bind(sock, (sockaddr*)&service, sizeof(service));
    if(err !=0)
    {
        cout << "bind function failed with error: " << WSAGetLastError();
        return 1;
    }
    else
        cout << "socket listening..... \n";

    u_long mode = 1;
    // Set I/O mode of the socket to non-blocking
    err = ioctlsocket(sock, FIONBIO, &mode);

    p.sequence_id = -1;

    while(cycles--)
    {
        Start = chrono::steady_clock::now();
        frameTime = chrono::duration_cast<chrono::duration<int, micro>>(Start-prevStart);
        // Check for dropped frames
        if(p.sequence_id != -1 && (frameTime > targetFrameTime+dropLimit || frameTime < targetFrameTime-dropLimit))
        {
            // Dropped a frame
            ++droppedFrames;
            cout << dec << "WARN: Frame Drop Detected! Loop took " << frameTime.count() << " microseconds.\n" << endl;
        }
        // New frame
        ++p.sequence_id;

        // Receive a datagram, and store the source address
        err = recvfrom(sock, RecvBuf, BufLen, 0, (sockaddr*) &senderAddr, &senderAddrSize);
        if(err == 0) // Connection closed
        {
            cout << "UDP connection has been gracefully closed";
            return 1;
        }
        else if(err == SOCKET_ERROR) // Connection error
        {
            if(WSAEWOULDBLOCK != WSAGetLastError()) // Fatal connection error
            {
                cout << "Fatal UDP connection error";
                return 1;
            }
        }
        else if (err > 0) // Data recieved
        {
            memcpy(&telemetry, RecvBuf, telemetrySize);
            FzAero = 0.5f * airDensity*CL*A * pow(telemetry.vehicle_speed_kph / 3.6f  ,2);
            Fztotal = FzStatic - 0.25 * FzAero;
            // Update tire configuration
            config.D = Fztotal * telemetry.surface_friction_estimate;
            config.B = Ca / (config.C*config.D);
            a = telemetry.driver_steering_wheel_angle - 10.0f;
            b = telemetry.driver_steering_wheel_angle + 10.0f;
        }

        // Generate intput angles
        linspace(angles, num_elements, a, b);

        // GPU Version
        PacejkaForce_Cuda(config, angles, forces, &d_config, &d_Angles, &d_forces, num_elements);
        if(!Headless || p.sequence_id%printRate == 0)
        {
            for(int i=0; i<num_elements; ++i)
            {
                DEBUG_PRINT(forces[i] << " ");
            }
        }
        // Find target angle
        peakAngle = OptimalAngle(forces, angles, num_elements);
        p.target_angle = (fabs(telemetry.driver_steering_wheel_angle)>fabs(peakAngle))? peakAngle: telemetry.driver_steering_wheel_angle;

        assignChecksum(p);

//        byte_ptr = reinterpret_cast<uint8_t*>(&p);
//        num_bytes = sizeof(p);
//        for(i=0; i<num_bytes; ++i)
//        {
//            if(!Headless || p.sequence_id%printRate == 0)
//                cout << hex << static_cast<int>(byte_ptr[i]) << " ";
//        }
//        if(!Headless || p.sequence_id%printRate == 0)
//            cout << endl;

        if(!Headless || p.sequence_id%printRate == 0)
            cout << dec << std::fixed << std::setprecision(2)
                 << "SEQ: " << p.sequence_id << " | "
                 << "UDP: " << telemetry.vehicle_speed_kph << "kph, Mu:"
                            << telemetry.surface_friction_estimate << ", Drv: "
                            << telemetry.driver_steering_wheel_angle << " | "
                 << "PHYS: "<< "Fz: " << Fztotal << "N, D: " << config.D << ", B: " << config.B << " | "
                 << "OUT: " << "Tgt: " << p.target_angle << endl;

        prevStart = Start;

        End = Start + targetFrameTime;
        // Spin-lock due to non-RTOS OS
        while(chrono::high_resolution_clock::now() < End)
            ;
    }

    cout << dec << "Total dropped frames: " << droppedFrames;

    // Disable sends or receives on socket
    err = shutdown(sock, SD_BOTH);
    if(err !=0)
    {
        cout << "shutdown function failed with error: " << WSAGetLastError();
        return 1;
    }
    // Close existing socket
    err = closesocket(sock);
    if(err !=0 )
    {
        cout << "closesocket function failed with error: " << WSAGetLastError();
        return 1;
    }
    // Terminate use of the Winsock 2 DLL
    WSACleanup();

    if(droppedFrames > 0)
        return 1;
    else
        return 0;
}
