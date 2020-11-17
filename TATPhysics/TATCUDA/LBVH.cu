#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include "device_functions.h"
#include "../TATCommon/TATVector3.h"
#include <stdio.h>
#include <iostream>

const int thread_per_block = 256;

#if(0)
struct LBVNode
{
public:
	LBVNode() {}

	TATVector3 m_Center;
	LBVNode* m_Children[2];
	LBVNode* m_Parent;

};

unsigned int ExpandBits(unsigned int v)
{
	v = (v * 0x00010001u) & 0xFF0000FFu;
	v = (v * 0x00000101u) & 0x0F00F00Fu;
	v = (v * 0x00000011u) & 0xC30C30C3u;
	v = (v * 0x00000005u) & 0x49249249u;
	return v;
}

// Calculates a 30-bit Morton code for the
// given 3D point located within the unit cube [0,1].
__global__ unsigned int Morton3D(float x, float y, float z)
{
	x = _Min(_Max(x * 1024.0f, 0.0f), 1023.0f);
	y = _Min(_Max(y * 1024.0f, 0.0f), 1023.0f);
	z = _Min(_Max(z * 1024.0f, 0.0f), 1023.0f);
	unsigned int xx = ExpandBits((unsigned int)x);
	unsigned int yy = ExpandBits((unsigned int)y);
	unsigned int zz = ExpandBits((unsigned int)z);
	return xx * 4 + yy * 2 + zz;
}

__global__ void GenLBVH(float* x, float* y, float* z, int size)
{

}

__global__ void GenMorton(TATVector3* c, int size, UINT* mortons)
{
	int idx = threadIdx.x + blockIdx.x * blockDim.x;
	while (idx < size)
	{
		mortons[idx] = Morton3D(c[idx].X, c[idx].Y, c[idx].Z);

		idx += blockIdx.x * gridDim.x;
	}
}



extern "C"
void CUDAGenLBVH(TATVector3 * centers, int size)
{
	float* host_buffer = (float*)malloc(3 * size * sizeof(float));
	for (int i = 0; i < size; ++i)
	{
		//memcpy(host_buffer + i * 3 * sizeof(float), centers[i].m_Datas, 3 * sizeof(float));
	}

	int block_num = size / thread_per_block;

	float* res_buffer = (float*)malloc(block_num * 6);
	
	for (int i = 1; i < block_num; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			if (res_buffer[i * 6 + j] < res_buffer[j])
				res_buffer[j] = res_buffer[i * 6 + j];
			if (res_buffer[j + 3] < res_buffer[i * 6 + j + 3])
				res_buffer[j + 3] = res_buffer[i * 6 + j + 3];
		}
	}


	TATVector3* bvh = (TATVector3*)malloc(2 * size * sizeof(TATVector3));
	LBVNode* testnode = (LBVNode*)malloc(sizeof(LBVNode));

	free(bvh);
}

#endif
//=================
//x0 x1 x2 y1 y2 y3 z1 z2 z3
__global__ void DevGetMinMax(float* c, int size, float* res)
{
	__shared__ float buffer[thread_per_block * 2 * 3];

	int idx = threadIdx.x + blockIdx.x * blockDim.x;
	int index = threadIdx.x;

	for (int i = 0; i < 3; ++i)
	{
		buffer[index + i * thread_per_block] = FLT_MAX;
		buffer[index + (i + 3) * thread_per_block] = -FLT_MAX;
	}

	__syncthreads();

	while (idx < size)
	{
		for (int i = 0; i < 3; ++i)
		{
			if (c[i * size + idx] < buffer[i * thread_per_block + index])
			{
				buffer[i * thread_per_block + index] = c[i * size + idx];
			}
			if (buffer[(i + 3) * thread_per_block + index] < c[i * size + idx])
			{
				buffer[(i + 3) * thread_per_block + index] = c[i * size + idx];
			}
		}

		idx += blockDim.x * gridDim.x;
	}

	__syncthreads();

	int offset = thread_per_block / 2;
	while (offset > 0)
	{
		if (index < offset)
		{
			for (int i = 0; i < 3; ++i)
			{
				if (buffer[index + offset + thread_per_block * i] < buffer[index + thread_per_block * i])
					buffer[index + thread_per_block * i] = buffer[index + offset + thread_per_block * i];

				if (buffer[index + (3 + i) * thread_per_block] < buffer[index + (3 + i) * thread_per_block + offset])
					buffer[index + (3 + i) * thread_per_block] = buffer[index + (3 + i) * thread_per_block + offset];
			}
		}

		__syncthreads();
		offset /= 2;
	}

	res[blockIdx.x] = buffer[0];
	res[blockIdx.x + 1] = buffer[0 + thread_per_block];
	res[blockIdx.x + 2] = buffer[0 + 2 * thread_per_block];

	for (int i = 0; i < 3; ++i)
	{
		res[blockIdx.x * 6 + i] = buffer[thread_per_block * i];
		res[blockIdx.x * 6 + i + 3] = buffer[thread_per_block * (i + 3)];
	}
}

extern "C"
void GetMinMax(TATVector3 * datas, int size, TATVector3 & min, TATVector3 & max)
{
	float* host_buffer = (float*)malloc(3 * size * sizeof(float));
	for (int i = 0; i < size; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			host_buffer[i + size * j] = datas[i].m_Datas[j];
		}
	}

	int block_num = (size + thread_per_block - 1) / thread_per_block;

	float* res_buffer = (float*)malloc(block_num * 6 * sizeof(float));

	float* dev_buffer, * dev_res_buffer;

	cudaMalloc((void**)&dev_buffer, 3 * size * sizeof(float));
	cudaMalloc((void**)&dev_res_buffer, block_num * 6 * sizeof(float));
	cudaMemcpy(dev_buffer, host_buffer, 3 * size * sizeof(float), cudaMemcpyHostToDevice);

	DevGetMinMax << <block_num, thread_per_block >> > (dev_buffer, size, dev_res_buffer);

	cudaMemcpy(res_buffer, dev_res_buffer, block_num * 6 * sizeof(float), cudaMemcpyDeviceToHost);

	for (int i = 1; i < block_num; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			if (res_buffer[i * 6 + j] < res_buffer[j])
				res_buffer[j] = res_buffer[i * 6 + j];
			if (res_buffer[j + 3] < res_buffer[i * 6 + j + 3])
				res_buffer[j + 3] = res_buffer[i * 6 + j + 3];
		}
	}

	memcpy(min.m_Datas, res_buffer, 3 * sizeof(float));
	memcpy(max.m_Datas, res_buffer + 3, 3 * sizeof(float));

	free(host_buffer);
	free(res_buffer);
	cudaFree(dev_buffer);
	cudaFree(dev_res_buffer);

	int res = 1;
}