#ifndef THEALMIGHTY_MODELLODER
#define THEALMIGHTY_MODELLODER
#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include "../TATCommon/TATCore.h"
#include "../TATBasis/TString.h"

using namespace std;

enum TATRenderPrimitiveMask
{
	TATModelVertexMask,
	TATModelFaceMask,
	TATModelNormalMask,
	TATModelTexCoordinateMask,
	TATModelTangentMask,
	TATModelColourMask
};

class TATModelElementMask
{
public:
	struct BufferOffset
	{
	public:
		BufferOffset(int index, int size, int beforeSize) :m_Index(index), m_Size(size), m_BeforeSize(beforeSize)
		{}
		int m_Index;
		int m_Size;
		int m_BeforeSize;
	};

	TATModelElementMask()
	{
		m_Mask = 0;
		m_TexCount = 0;
		m_TotalSize = 0;
	}

	inline void UseNormal()
	{
		m_Mask |= (1 << TATModelNormalMask);
	}

	inline void UseTexCoordinate()
	{
		m_Mask |= (1 << TATModelTexCoordinateMask);
	}

	inline void UseTangent()
	{
		m_Mask |= (1 << TATModelTangentMask);
	}

	inline void UseFace()
	{
		m_Mask |= (1 << TATModelFaceMask);
	}

	inline void UseColour()
	{
		m_Mask |= (1 << TATModelColourMask);
	}

	inline bool IsUseNormal()
	{
		return (m_Mask & (1 << TATModelNormalMask));
	}

	inline bool IsUseTexCoordinate()
	{
		return (m_Mask & (1 << TATModelTexCoordinateMask));
	}

	inline bool IsUseTangent()
	{
		return (m_Mask & (1 << TATModelTangentMask));
	}

	inline bool IsUseFace()
	{
		return (m_Mask & (1 << TATModelFaceMask));
	}

	inline bool IsUseColour()
	{
		return (m_Mask & (1 << TATModelColourMask));
	}

	void SetTexNum(int n)
	{
		m_TexCount = n;
	}

	int GetElementMask()
	{
		return m_Mask;
	}

	//call once is enough
	int ComputeSize()
	{
		m_BufferOffsets.clear();

		m_TotalSize = 3 + 3 * IsUseNormal() + 3 * IsUseTangent() + 3 * IsUseColour() + 2 * IsUseTexCoordinate() * m_TexCount;

		int index = 0;
		int beforeSize = 0;
		m_BufferOffsets.push_back(BufferOffset(index++, 3, beforeSize));
		beforeSize += 3;
		if (IsUseNormal())
		{
			m_BufferOffsets.push_back(BufferOffset(index++, 3, beforeSize));
			beforeSize += 3;
		}

		if (IsUseTangent())
		{
			m_BufferOffsets.push_back(BufferOffset(index++, 3, beforeSize));
			beforeSize += 3;
		}

		if (IsUseColour())
		{
			m_BufferOffsets.push_back(BufferOffset(index++, 3, beforeSize));
			beforeSize += 3;
		}

		if (IsUseTexCoordinate())
		{
			for (int i = 0; i < m_TexCount; i++)
			{
				m_BufferOffsets.push_back(BufferOffset(index++, 2, beforeSize));
				beforeSize += 2;
			}
		}
		return m_TotalSize;
	}

	void Clear()
	{
		m_TexCount = 0;
		m_BufferOffsets.clear();
		m_TotalSize = 0;
		m_Mask = 0;
	}

public:
	int m_TexCount;
	std::vector<BufferOffset> m_BufferOffsets;
	int m_TotalSize;
protected:
	int m_Mask;
};

struct TATFaceBuffer
{
public:
	TATFaceBuffer()
	{}
	TATFaceBuffer(int vv1, int vv2, int vv3) :v1(vv1), v2(vv2), v3(vv3)
	{}
	TATFaceBuffer(int vv1, int vv2, int vv3, int nn1, int nn2, int nn3) :v1(vv1), v2(vv2), v3(vv3), n1(nn1), n2(nn2), n3(nn3)
	{}
	TATFaceBuffer(int vv1, int vv2, int vv3,int uvv1,int uvv2,int uvv3, int nn1, int nn2, int nn3) :v1(vv1), v2(vv2), v3(vv3),uv1(uvv1),uv2(uvv2),uv3(uvv3), n1(nn1), n2(nn2), n3(nn3)
	{}
	int v1, v2, v3;
	int n1, n2, n3;
	int uv1, uv2, uv3;
};

struct TATVertexBuffer
{
public:
	TATVertexBuffer() 
	{
		m_Data = 0;
		TAT_MEMSET(m_Position, 0);
		TAT_MEMSET(m_Normal, 0);
		TAT_MEMSET(m_Tangent, 0);
	}

	TATVertexBuffer(float x, float y, float z)
	{
		m_Position[0] = x;
		m_Position[1] = y;
		m_Position[2] = z;
		m_CurrentTex = 0;
		m_TexCount = 0;
		m_Data = 0;
	}

	TATVertexBuffer(float x, float y, float z, float nx, float ny, float nz)
	{
		m_Position[0] = x;
		m_Position[1] = y;
		m_Position[2] = z;
		m_Normal[0] = nx;
		m_Normal[1] = ny;
		m_Normal[2] = nz;
		m_CurrentTex = 0;
		m_TexCount = 0;
		m_Data = 0;
	}

	float m_Position[3];
	float m_TexCoordinates[MAX_TEXCOORDINATE_COUNT][2];
	float m_Normal[3];
	float m_Tangent[3];
	int m_TexCount;
	int m_CurrentTex;
	bool m_IsRendVert;
	void* m_Data;
};

struct TATNormalBuffer 
{
public:
	TATNormalBuffer(float nx, float ny, float nz)
	{
		m_Normal[0] = nx;
		m_Normal[1] = ny;
		m_Normal[2] = nz;
	}

	float m_Normal[3];
};

struct TATexCoordinateBuffer
{
public:
	TATexCoordinateBuffer(float x, float y, float z)
	{
		m_TexCoordinate[0] = x;
		m_TexCoordinate[1] = y;
		m_TexCoordinate[2] = z;
	}
	float m_TexCoordinate[3];
};

struct TATetraBuffer
{
public:
	TATetraBuffer() {}

	TATetraBuffer(int vv1, int vv2, int vv3, int vv4) :v1(vv1), v2(vv2), v3(vv3), v4(vv4)
	{}
	int v1, v2, v3, v4;
};

class TATModelBuffer
{
public:
	std::vector<TATVertexBuffer> vertexBuffer;
	std::vector<TATNormalBuffer> normalBuffer;
	std::vector<TATexCoordinateBuffer> texCoordinateBuffer;
	std::vector<TATFaceBuffer> faceBuffer;
	std::vector<TATetraBuffer> tetraBuffer;
};

class TATModelLoader
{
public:
	TATModelLoader(const TString& filePath)
	{
	}

	virtual void Load(const TString& str, TATModelBuffer& buffer) {}

	virtual ~TATModelLoader() {}

	TATModelBuffer m_Buffer;

	TATModelElementMask m_ModelElementMask;

};

#endif // !THEALMIGHTY_MODELLODER
