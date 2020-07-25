#ifndef THEALMIGHTY_MODELLODER
#define THEALMIGHTY_MODELLODER
#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include "TATPaths.h"
#include "../TATApplication/TATApplication.h"

using namespace std;

struct TATFaceBuffer
{
public:
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
	TATVertexBuffer(float xx, float yy, float zz) :x(xx), y(yy), z(zz)
	{}
	TATVertexBuffer(float xx, float yy, float zz, float nxx, float nyy, float nzz) :x(xx), y(yy), z(zz), nx(nxx),ny(nyy), nz(nzz)
	{}
	TATVertexBuffer(float xx, float yy, float zz, float ux, float vx) :x(xx), y(yy), z(zz), u(ux), v(vx)
	{}
	TATVertexBuffer(float xx, float yy, float zz, float ux, float vx, float nxx, float nyy, float nzz) :x(xx), y(yy), z(zz), u(ux), v(vx), nx(nxx), ny(nyy), nz(nzz)
	{}
	float x,y,z;
	float u,v;
	float nx,ny,nz;
	bool isRendVert;
};

struct TATNormalBuffer 
{
public:
	TATNormalBuffer(float nxx, float nyy, float nzz) :nx(nxx), ny(nyy), nz(nzz)
	{}
	float nx, ny, nz;

};

struct TATetraBuffer
{
public:
	TATetraBuffer(int vv1, int vv2, int vv3, int vv4) :v1(vv1), v2(vv2), v3(vv3), v4(vv4)
	{}
	int v1, v2, v3, v4;
};

class TATModelBuffer
{
public:
	std::vector<TATVertexBuffer> vertexBuffer;
	std::vector<TATNormalBuffer> normalBuffer;
	std::vector<TATFaceBuffer> faceBuffer;
	std::vector<TATetraBuffer> tetraBuffer;
};

class TATModelLoader
{
public:
	TATModelLoader(const TString& filePath)
	{

	}

	virtual void Load(const TString&, TATModelBuffer& buffer)
	{
		
	}

	virtual ~TATModelLoader(){}

	TATModelBuffer m_Buffer;
};

#endif // !THEALMIGHTY_MODELLODER
