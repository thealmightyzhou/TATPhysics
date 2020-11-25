#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include "device_functions.h"
#include "../TATBasis/TATErrorReporter.h"
#include "../TATBroadPhase/LBVH.h"
#include <stdio.h>
#include <iostream>
#include <algorithm>
#include <map>
#include <stack>
#include <queue>

const int thread_per_block = 256;

//====================
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

	for (int i = 0; i < 3; ++i)
	{
		res[blockIdx.x * 6 + i] = buffer[thread_per_block * i];
		res[blockIdx.x * 6 + i + 3] = buffer[thread_per_block * (i + 3)];
	}
}

#define USE_GPUASSIGN 0

#if(USE_GPUASSIGN)
__global__ void Assign(TATVector3* buffer, int size, float* datas)
{
	int idx = threadIdx.x + blockIdx.x * blockDim.x;
	if (idx < size)
	{
		for (int i = 0; i < 3; ++i)
		{
			datas[i + size * idx] = buffer[idx].m_Datas[i];
		}
	}

	__syncthreads();
}
#endif

extern "C"
void GetMinMax(TATVector3 * datas, int size, TATVector3 & min, TATVector3 & max)
{
	//x0 x1 x2 y0 y1 y2 z0 z1 z2 (3 * size)
	int block_num = (size + thread_per_block - 1) / thread_per_block;

#if(!USE_GPUASSIGN)
	float* host_buffer = (float*)malloc(3 * size * sizeof(float));
#endif
	float* res_buffer = (float*)malloc(block_num * 6 * sizeof(float));
	float* dev_buffer, * dev_res_buffer;
	cudaMalloc((void**)&dev_buffer, 3 * size * sizeof(float));
	cudaMalloc((void**)&dev_res_buffer, block_num * 6 * sizeof(float));

#if(USE_GPUASSIGN)
	TATVector3* dev_vec;
	cudaMalloc((void**)&dev_vec, size * sizeof(TATVector3));
	cudaMemcpy(dev_vec, datas, size * sizeof(TATVector3), cudaMemcpyHostToDevice);
	Assign < << block_num, thread_per_block >> > (dev_vec, size, dev_buffer);
#else
	for (int i = 0; i < size; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			host_buffer[i + size * j] = datas[i].m_Datas[j];
		}
	}

	//Xmin Ymin Zmin Xmax Ymax Zmax 
	cudaMemcpy(dev_buffer, host_buffer, 3 * size * sizeof(float), cudaMemcpyHostToDevice);
#endif

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
#if(!USE_GPUASSIGN)
	free(host_buffer);
#endif
	free(res_buffer);
	cudaFree(dev_buffer);
	cudaFree(dev_res_buffer);
}

__device__ int Sign(int i)
{
	if (i >= 0)
		return 1;
	if (i < 0)
		return -1;
}

int Host_Sign(int i)
{
	if (i > 0)
		return 1;
	if (i < 0)
		return -1;
	return 0;
}

__device__ int Prefix(UINT i, UINT j, UINT* buffer, int max)
{
	if (j >= max || j < 0)
		return -1;

	return __clz(buffer[i] ^ buffer[j]);
}

int Host_Prefix(UINT i, UINT j, UINT* buffer, int max)
{
	if (j >= max || j < 0)
		return -1;

	UINT same = buffer[i] ^ buffer[j];

	int count = 0;
	while (same > 0)
	{
		same = same >> 1;
		count++;
	}
	return 32 - count;
}

//									 leaf      internal left    internal right
//total size 3*size-2 [0,3*size-3] [0,size-1] [size,2*size-2] [2*size-1,3*size-3]
__global__ void DevGenLBVH(UINT* buffer, bool* internal, int size)
{
	int i = threadIdx.x + blockIdx.x * blockDim.x;
	if (i < size - 1)
	{
		int d = Sign(Prefix(i, i + 1, buffer, size) - Prefix(i, i - 1, buffer, size));
		int min_pre = Prefix(i, i - d, buffer, size);
		int max_step = 1;
		int pre;
		do
		{
			max_step *= 2;
			pre = Prefix(i, i + d * max_step, buffer, size);

		} while (pre > min_pre);

		int step = max_step / 2;
		int curr_offset = 0;

		while (step > 0)
		{
			pre = Prefix(i, i + (curr_offset + step) * d, buffer, size);
			if (pre > min_pre)
			{
				curr_offset += step;
			}

			step /= 2;
		}

		int j = i + curr_offset * d;
		int prefix_node = Prefix(i, j, buffer, size);

		//int npow = 0;
		step = 1;
		while (curr_offset > 0)
		{
			curr_offset = curr_offset >> 1;
			//npow++;
			step *= 2;
		}

		//step = (int)pow(2.0, (double)npow);

		int split = 0;

		while (step > 0)
		{
			pre = Prefix(i, i + (split + step) * d, buffer, size);
			if (pre > prefix_node)
			{
				split += step;
			}
			step /= 2;
		}

		int cut = (i + split * d + (d <= 0 ? d : 0));

		if (cut == (i < j ? i : j))
		{
			internal[i] = false;
			buffer[size + i] = cut;
		}
		else
		{
			internal[i] = true;
			buffer[size + i] = cut;
		}

		if ((cut + 1) == (i > j ? i : j))
		{
			internal[size - 1 + i] = false;
			buffer[2 * size - 1 + i] = (cut + 1);
		}
		else
		{
			internal[size - 1 + i] = true;
			buffer[2 * size - 1 + i] = (cut + 1);
		}

	}
}

void HostGenLBVH(UINT* buffer, bool* internal, int size)
{
	for (int i = 0; i < size - 1; ++i)
	{
		int d = Host_Sign(Host_Prefix(i, i + 1, buffer, size) - Host_Prefix(i, i - 1, buffer, size));
		int min_pre = Host_Prefix(i, i - d, buffer, size);
		int max_step = 1;
		int pre;
		do
		{
			max_step *= 2;
			pre = Host_Prefix(i, i + d * max_step, buffer, size);

		} while (pre > min_pre);

		int step = max_step / 2;
		int curr_offset = 0;

		while (step > 0)
		{
			pre = Host_Prefix(i, i + (curr_offset + step) * d, buffer, size);
			if (pre > min_pre)
			{
				curr_offset += step;
			}

			step /= 2;
		}

		int j = i + curr_offset * d;
		int prefix_node = Host_Prefix(i, j, buffer, size);

		int npow = 0;
		while (curr_offset > 0)
		{
			curr_offset = curr_offset >> 1;
			npow++;
		}

		step = pow(2, npow);

		int split = 0;

		while (step > 0)
		{
			pre = Host_Prefix(i, i + (split + step) * d, buffer, size);
			if (pre > prefix_node)
			{
				split += step;
			}
			step /= 2;
		} 

		int cut = (i + split * d + (d <= 0 ? d : 0));

		UINT pr = Host_Prefix(i, cut, buffer, size);
		UINT aft = Host_Prefix(i, cut + 1, buffer, size);

		if (cut == (i < j ? i : j))
		{
			internal[i] = false;
			buffer[size + i] = cut;
		}
		else
		{
			internal[i] = true;
			buffer[size + i] = cut;
		}

		if ((cut + 1) == (i > j ? i : j))
		{
			internal[size - 1 + i] = false;
			buffer[2 * size - 1 + i] = (cut + 1);
		}
		else
		{
			internal[size - 1 + i] = true;
			buffer[2 * size - 1 + i] = (cut + 1);
		}
	}
}

extern "C"
void BuildLBVH(std::vector<LBVNode>& nodes, TATVector3* pos, std::vector<LBVNode>& internal_nodes, int num)
{
	int block_num = (num + thread_per_block - 1) / thread_per_block;

	TATVector3 min, max;
	GetMinMax(pos, num, min, max);
	TATVector3 unit = max - min;

	for (int i = 0; i < num; ++i)//parallel
	{
		nodes[i].m_UnitCenter = (nodes[i].m_Center - min) / unit;
		nodes[i].GenMorton();
	}

	sort(nodes.begin(), nodes.end());

	UINT* host_buffer = (UINT*)malloc((3 * num - 2) * sizeof(UINT));

	for (int i = 0; i < num; ++i)
	{
		host_buffer[i] = nodes[i].m_MortonCode;
	}

	UINT* dev_buffer;
	cudaMalloc((void**)&dev_buffer, (3 * num - 2) * sizeof(UINT));
	cudaMemcpy(dev_buffer, host_buffer, (3 * num - 2) * sizeof(UINT), cudaMemcpyHostToDevice);

	bool* dev_internalbuffer;
	cudaMalloc((void**)&dev_internalbuffer, sizeof(bool) * (2 * num - 2));
	bool* host_internalbuffer = (bool*)malloc(sizeof(bool) * (2 * num - 2));

	//HostGenLBVH(host_buffer, host_internalbuffer, num);
	//UINT* host_bufferTemp = (UINT*)malloc((3 * num - 2) * sizeof(UINT));
	//bool* host_internalbufferTemp = (bool*)malloc(sizeof(bool) * (2 * num - 2));

	DevGenLBVH << <block_num, thread_per_block >> > (dev_buffer, dev_internalbuffer, num);
	cudaDeviceSynchronize();
	cudaMemcpy(host_internalbuffer, dev_internalbuffer, (2 * num - 2) * sizeof(bool), cudaMemcpyDeviceToHost);
	cudaMemcpy(host_buffer, dev_buffer, (3 * num - 2) * sizeof(UINT), cudaMemcpyDeviceToHost);
	//cudaMemcpy(host_internalbufferTemp, dev_internalbuffer, (2 * num - 2) * sizeof(bool), cudaMemcpyDeviceToHost);
	//cudaMemcpy(host_bufferTemp, dev_buffer, (3 * num - 2) * sizeof(UINT), cudaMemcpyDeviceToHost);

	/*for (int i = 0; i < (2 * num - 2); ++i)
	{
		if (host_internalbufferTemp[i] != host_internalbuffer[i])
		{
			int stop = 1;
		}
	}
	for (int i = 0; i < (3 * num - 2); ++i)
	{
		if (host_bufferTemp[i] != host_buffer[i])
		{
			int stop = 1;
		}
	}*/

	internal_nodes.resize(num - 1);

	LBVNode* lchild, * rchild;
	for (int i = 0; i < num - 1; ++i) //parallel
	{
		internal_nodes[i].m_IsInternal = true;

		bool linternal = host_internalbuffer[i];
		bool rinternal = host_internalbuffer[num - 1 + i];

		if (!linternal)
			internal_nodes[i].m_Children[0] = &nodes[host_buffer[num + i]];
		else
			internal_nodes[i].m_Children[0] = &internal_nodes[host_buffer[num + i]];
		if (!rinternal)
			internal_nodes[i].m_Children[1] = &nodes[host_buffer[2 * num - 1 + i]];
		else
			internal_nodes[i].m_Children[1] = &internal_nodes[host_buffer[2 * num - 1 + i]];

		lchild = internal_nodes[i].m_Children[0];
		rchild = internal_nodes[i].m_Children[1];
		lchild->m_Parent = &internal_nodes[i];
		rchild->m_Parent = &internal_nodes[i];
	}

	free(host_buffer);
	free(host_internalbuffer);
	cudaFree(dev_buffer);
	cudaFree(dev_internalbuffer);
}