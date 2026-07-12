#ifndef PACEJKA_CUH
#define PACEJKA_CUH

#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <stdexcept>
#include <string>

class CudaException : public std::runtime_error
{
public:
    CudaException(cudaError_t err, const char* file, int line)
        : std::runtime_error(
            std::string("[CUDA Error] ") + cudaGetErrorName(err) + " - "
            + cudaGetErrorString(err) + "\n at" + file + ":" + std::to_string(line)
        ) {}
};

inline void checkCudaThrow(cudaError_t err, const char* file, int line)
{
    if(err != cudaSuccess)
    {
        throw CudaException(err, file, line);
    }

    return;
}

#define CUDA_CALL(call) checkCudaThrow((call), __FILE__, __LINE__)

// Wrapper class for device allocated memory, to ensure proper memory deallocation
template <typename T>
class DeviceBuffer
{
private:
    T* d_ptr = nullptr;
    size_t num_elements = 0;

public:
    DeviceBuffer(size_t elements) : num_elements(elements)
    {
        CUDA_CALL( cudaMalloc((void**)&d_ptr, num_elements * sizeof(T)));
    }
    // automatically deallocates device memory
    ~DeviceBuffer()
    {
        if(d_ptr)
        {
            cudaFree(d_ptr);
            d_ptr = nullptr;
        }
    }

    T* get() const
    {
        return d_ptr;
    }

    size_t Size() const
    {
        return num_elements;
    }
};

cudaError_t PacejkaForce_Cuda(const TireConfig& config, const float* slipAngles, float* output_forces, int num_elements);

#endif // PACEJKA_CUH
