#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include "device_functions.h"

#include <stdio.h>
#include <stdlib.h>

const int thread_per_block = 256;

__global__ void Dot(float* a, float* b, float* c, int size)
{
	int tid = threadIdx.x + blockIdx.x * blockDim.x;
	__shared__ float sh[thread_per_block];
	float temp = 0;
	while (tid < size)
	{
		temp += a[tid] * b[tid];
		tid += gridDim.x * blockDim.x;
	}

	sh[threadIdx.x] = temp;
	__syncthreads();

	int i = threadIdx.x / 2;
	while (i > 0)
	{
		if (threadIdx.x < i)
		{
			sh[threadIdx.x] += sh[threadIdx.x + i];
		}
		__syncthreads();
		i /= 2;
	}

	if (threadIdx.x == 0)
	{
		c[blockIdx.x] = sh[0];
	}
}

extern "C"
float CUDADot(float* a, float* b, int size)
{
	cudaError_t cudaStatus;
	int block_per_grid = (size + thread_per_block - 1) / thread_per_block;

	float* partial_c = (float*)(malloc(block_per_grid * sizeof(float)));
	
	float* dev_a, * dev_b, * dev_c, c;
	cudaMalloc((void**)&dev_a, size * sizeof(float));
	cudaMalloc((void**)&dev_b, size * sizeof(float));
	cudaMalloc((void**)&dev_c, block_per_grid * sizeof(float));

	for (int i = 0; i < size; ++i)
	{
		a[i] = i;
		b[i] = i * 2;
	}

	cudaMemcpy(dev_a, a, size * sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(dev_b, b, size * sizeof(float), cudaMemcpyHostToDevice);

	Dot << <block_per_grid, thread_per_block >> > (dev_a, dev_b, dev_c, size);

	cudaStatus = cudaMemcpy(partial_c, dev_c, block_per_grid * sizeof(float), cudaMemcpyDeviceToHost);
	c = 0;
	for (int i = 0; i < block_per_grid; ++i)
	{
		c += partial_c[i];
	}

	cudaFree(dev_a);
	cudaFree(dev_b);
	cudaFree(dev_c);
	free(partial_c);

	return c;
}