#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include "device_functions.h"
#include "../TATCommon/TATVector3.h"
#include "../TATBasis/TATErrorReporter.h"
#include <stdio.h>
#include <iostream>
#include <algorithm>
#include <map>

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
	float time;
	cudaEvent_t begin, end;
	cudaEventCreate(&begin);
	cudaEventCreate(&end);
	cudaEventRecord(begin, 0);

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

	cudaEventRecord(end, 0);
	cudaEventSynchronize(begin);
	cudaEventSynchronize(end);
	cudaEventElapsedTime(&time, begin, end);

	TATErrorReporter::Instance()->ReportErr("gpu used time: " + TString::ConvertFloat(time) + " ms");

	cudaEventDestroy(begin);
	cudaEventDestroy(end);
}


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
unsigned int Morton3D(float x, float y, float z)
{
	x = _Min(_Max(x * 1024.0f, 0.0f), 1023.0f);
	y = _Min(_Max(y * 1024.0f, 0.0f), 1023.0f);
	z = _Min(_Max(z * 1024.0f, 0.0f), 1023.0f);
	unsigned int xx = ExpandBits((unsigned int)x);
	unsigned int yy = ExpandBits((unsigned int)y);
	unsigned int zz = ExpandBits((unsigned int)z);
	return xx * 4 + yy * 2 + zz;
}

__device__ int Sign(int i)
{
	if (i > 0)
		return 1;
	if (i < 0)
		return -1;
	return 0;
}

__device__ int Prefix(UINT i, UINT j, int max)
{
	if (j >= max || j < 0)
		return -1;

	return __clz(i ^ j);
}

//									 leaf      internal left    internal right
//total size 3*size-2 [0,3*size-3] [0,size-1] [size,2*size-2] [2*size-1,3*size-3]
__global__ void DevGenLBVH(UINT* buffer, int size)
{
	int i = threadIdx.x + blockIdx.x * blockDim.x;
	if (i < size - 1)
	{
		int d = Sign(Prefix(buffer[i], buffer[i + 1], size - 1) - Prefix(buffer[i], buffer[i - 1], size - 1));
		int min_pre = Prefix(buffer[i], buffer[i - d], size - 1);
		int max_step = 1;
		int pre;
		do
		{
			max_step *= 2;
			pre = Prefix(buffer[i], buffer[i + d * max_step], size - 1);

		} while (pre > min_pre);

		int step = max_step / 2;
		int curr_offset = 0;

		do
		{
			pre = Prefix(buffer[i], buffer[i + (curr_offset + step) * d], size - 1);
			if (pre > min_pre)
			{
				curr_offset += step;
				step /= 2;
			}
		} while (step > 0);

		int j = i + curr_offset * d;
		int prefix_node = Prefix(buffer[i], buffer[j], size - 1);

		step = curr_offset / 2;
		int split = 0;
		do
		{
			pre = Prefix(buffer[i], buffer[i + (split + step) * d], size - 1);
			if (pre > prefix_node)
			{
				split += step;
				step /= 2;
			}
		} while (step > 0);

		int cut = i + split * d + d >= 0 ? d : 0;
		if (cut == i < j ? i : j)
			buffer[size + i] = buffer[cut];
		else
			buffer[size + i] = buffer[size + cut];

		if (cut + 1 == i > j ? i : j)
			buffer[2 * size - 1 + i] = buffer[size + cut + 1];
		else
			buffer[2 * size - 1 + i] = buffer[2 * size - 1 + cut + 1];
	}
}

struct LBVNode
{
public:
	LBVNode(const TATVector3& min, const TATVector3& max) :m_Min(min), m_Max(max)
	{
		m_Center = (min + max) / 2;
		m_IsInternal = false;
		m_Children[0] = m_Children[1] = m_Parent = 0;
		m_UserData = 0;
	}

	LBVNode()
	{
		m_IsInternal = false;
		m_Children[0] = m_Children[1] = m_Parent = 0;
		m_UserData = 0;
	}

	bool operator<(const LBVNode& c)
	{
		return m_MortonCode < c.m_MortonCode;
	}

	bool m_IsInternal;
	TATVector3 m_Min, m_Max;
	TATVector3 m_Center;
	TATVector3 m_UnitCenter;
	LBVNode* m_Children[2];
	LBVNode* m_Parent;
	void* m_UserData;
	UINT m_MortonCode;
};

class LBVH
{
public:
	LBVH()
	{
		m_Root = 0;
	}

	LBVNode* InsertAABB(const TATVector3& min, const TATVector3& max)
	{
		m_StoreNodes.push_back(LBVNode(min, max));
		m_Positions.push_back((min + max) / 2);
		return &m_StoreNodes[m_StoreNodes.size() - 1];
	}

	void Build()
	{	
		int num = m_StoreNodes.size();

		TATVector3 min, max;
		GetMinMax(&m_Positions[0], m_Positions.size(), min, max);
		TATVector3 unit = max - min;
		for (int i = 0; i < num; ++i)
		{
			m_StoreNodes[i].m_UnitCenter = (m_StoreNodes[i].m_Center - min) / unit;
			m_StoreNodes[i].m_MortonCode = Morton3D(m_StoreNodes[i].m_UnitCenter.X, m_StoreNodes[i].m_UnitCenter.Y, m_StoreNodes[i].m_UnitCenter.Z);
		}

		sort(m_StoreNodes.begin(), m_StoreNodes.end());
		UINT* host_buffer = (UINT*)malloc((3 * num - 2) * sizeof(UINT));
		for (int i = 0; i < num; ++i) // parallel
		{
			m_MapNodes[m_StoreNodes[i].m_MortonCode] = &m_StoreNodes[i];
			host_buffer[i] = m_StoreNodes[i].m_MortonCode;
		}

		UINT* dev_buffer;
		cudaMalloc((void**)&dev_buffer, (3 * num - 2) * sizeof(UINT));
		cudaMemcpy(dev_buffer, host_buffer, (3 * num - 2) * sizeof(UINT), cudaMemcpyHostToDevice);

		int block_num = (num + thread_per_block - 1) / thread_per_block;
		DevGenLBVH << <block_num, thread_per_block >> > (dev_buffer, num);

		cudaMemcpy(host_buffer, dev_buffer, (3 * num - 2) * sizeof(UINT), cudaMemcpyDeviceToHost);
		m_InternalNodes.resize(num - 1);

		LBVNode* lchild, * rchild;
		for (int i = 0; i < num - 1; ++i) //parallel
		{
			m_InternalNodes[i].m_IsInternal = true;
			m_InternalNodes[i].m_Children[0] = m_MapNodes[host_buffer[host_buffer[num + i]]];
			m_InternalNodes[i].m_Children[1] = m_MapNodes[host_buffer[host_buffer[2 * num - 1 + i]]];

			lchild = m_InternalNodes[i].m_Children[0];
			rchild = m_InternalNodes[i].m_Children[1];
			lchild->m_Parent = &m_InternalNodes[i];
			rchild->m_Parent = &m_InternalNodes[i];

			m_InternalNodes[i].m_Min = TATVector3::MakeMin(lchild->m_Min, rchild->m_Min);
			m_InternalNodes[i].m_Max = TATVector3::MakeMax(lchild->m_Max, rchild->m_Max);

		}

		m_Root = &m_InternalNodes[0];
		
		free(host_buffer);
		cudaFree(dev_buffer);
	}

	std::vector<LBVNode> m_InternalNodes;
	std::vector<LBVNode> m_StoreNodes;
	std::vector<TATVector3> m_Positions;
	std::map<UINT, LBVNode*> m_MapNodes;

	LBVNode* m_Root;
};
