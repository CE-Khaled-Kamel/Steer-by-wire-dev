#include "Pacejka.h"
#include "Pacejka.cuh"
#include <stdio.h>

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

cudaError_t PacejkaForce_Cuda(const TireConfig& config, const float* slipAngles, float* output_forces, int num_elements)
{
    // Choose which GPU to run on
    CUDA_CALL( cudaSetDevice(0) );

    // Allocate data memory on GPU
    DeviceBuffer<float> d_Angles(num_elements);
    DeviceBuffer<float> d_forces(num_elements);
    DeviceBuffer<TireConfig> d_config(1);

    // Copy data from CPU to GPU
    CUDA_CALL( cudaMemcpy(d_Angles.get(), slipAngles, num_elements * sizeof(float), cudaMemcpyHostToDevice));
    CUDA_CALL( cudaMemcpy(d_config.get(), &config, sizeof(TireConfig), cudaMemcpyHostToDevice));

    // Grid dimensions
    int nThreads = 256;
    int nBlocks = (num_elements + nThreads - 1)/nThreads;
    // Kernel launch
    Pacejka_Kernel<<< nBlocks, nThreads >>> (d_config.get(), d_Angles.get(), d_forces.get(), num_elements);

    // Check for launching errors
    CUDA_CALL( cudaGetLastError() );

    // Wait for kernel to finish
    CUDA_CALL( cudaDeviceSynchronize() );

    // Copy data back to CPU
    CUDA_CALL( cudaMemcpy(output_forces, d_forces.get(), num_elements * sizeof(float), cudaMemcpyDeviceToHost));

    // Wrapper class automatically deallocates device memory

    return cudaSuccess;
}
