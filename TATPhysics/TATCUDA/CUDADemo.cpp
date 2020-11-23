#include "CUDADemo.h"
#include "../TATBasis/TATErrorReporter.h"
#include "../TATCommon/TATVector3.h"
#include "time.h"

extern "C"
cudaError_t addWithCuda(int* c, const int* a, const int* b, unsigned int size);


int CUDADemo::CUDAHelloWorld()
{
	const int arraySize = 5;
	const int a[arraySize] = { 1, 2, 3, 4, 5 };
	const int b[arraySize] = { 10, 20, 30, 40, 50 };
	int c[arraySize] = { 0 };

	// Add vectors in parallel.
	cudaError_t cudaStatus = addWithCuda(c, a, b, arraySize);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "addWithCuda failed!");
		return 1;
	}

	printf("{1,2,3,4,5} + {10,20,30,40,50} = {%d,%d,%d,%d,%d}\n",
		c[0], c[1], c[2], c[3], c[4]);

	// cudaDeviceReset must be called before exiting in order for profiling and
	// tracing tools such as Nsight and Visual Profiler to show complete traces.
	cudaStatus = cudaDeviceReset();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaDeviceReset failed!");
		return 1;
	}
	return 0;
}

extern "C"
void VectorAdd(int size, int* a, int* b, int* c);
void CUDADemo::CUDAVectorAdd()
{
	int* a = new int[33 * 1024];
	int* b = new int[33 * 1024];
	int* c = new int[33 * 1024];

	VectorAdd(33 * 1024, a, b, c);
}

extern "C"
void CUDADimNum(int* cnt);
void CUDADemo::DimNum()
{
	int* n = new int[6];
	CUDADimNum(n);
	for (int i = 0; i < 6; ++i)
	{
		TATErrorReporter::Instance()->ReportErr(std::to_string(n[i]));
	}

}

extern "C"
float CUDADot(float* a, float* b, int size);
void CUDADemo::CUDAVectorDot()
{
	int size = 665533;
	float* a, * b;
	a = (float*)malloc(size * sizeof(float));
	b = (float*)malloc(size * sizeof(float));

	float c = CUDADot(a, b, size);
	TATErrorReporter::Instance()->ReportErr(std::to_string(c));

	free(a);
	free(b);
}

//extern "C"
//void CUDAGenLBVH(TATVector3 * centers, int size);
//void CUDADemo::BuildLBVH()
//{
//	int size = 100;
//
//	TATVector3* centers = (TATVector3*)malloc(size * sizeof(TATVector3));
//	CUDAGenLBVH(centers, size);
//
//}

extern "C"
void GetMinMax(TATVector3 * datas, int size, TATVector3 & min, TATVector3 & max);
void CUDADemo::CUDAGetMinMax()
{
	int size = 1000000;
	std::vector<TATVector3> datas;
	datas.resize(size);
	srand(0);
	for (int i = 0; i < size; ++i)
	{
		datas[i].X = rand() * 53 % 100000;
		datas[i].Y = rand() * 13 % 10000;
		datas[i].Z = rand() * 41 % 10000;
	}

	datas[rand() % size].X = -2;
	datas[rand() % size].Y = -3;
	datas[rand() % size].Z = -12213;
	datas[rand() % size].X = 1000011;
	datas[rand() % size].Y = 123123;
	datas[rand() % size].Z = 4545445;

	TATVector3 min, max;
	GetMinMax(&datas[0], size, min, max);

	TATErrorReporter::Instance()->ReportErr(TString::ConvertInt(min.X) + " " + TString::ConvertInt(min.Y) + " " + TString::ConvertInt(min.Z));
	TATErrorReporter::Instance()->ReportErr(TString::ConvertInt(max.X) + " " + TString::ConvertInt(max.Y) + " " + TString::ConvertInt(max.Z));

	min = TAT_MAXVECTOR3;
	max = -TAT_MAXVECTOR3;

	float begin = clock();
	for (int i = 0; i < size; ++i)
	{
		if (datas[i].X < min.X)
			min.X = datas[i].X;
		if (datas[i].Y < min.Y)
			min.Y = datas[i].Y;
		if (datas[i].Z < min.Z)
			min.Z = datas[i].Z;
		if (datas[i].X > max.X)
			max.X = datas[i].X;
		if (datas[i].Y > max.Y)
			max.Y = datas[i].Y;
		if (datas[i].Z > max.Z)
			max.Z = datas[i].Z;
	}
	float end = clock();
	float dt = end - begin;
	TATErrorReporter::Instance()->ReportErr("cpu used time: " + TString::ConvertFloat(dt) + " ms");
	TATErrorReporter::Instance()->ReportErr(TString::ConvertInt(min.X) + " " + TString::ConvertInt(min.Y) + " " + TString::ConvertInt(min.Z));
	TATErrorReporter::Instance()->ReportErr(TString::ConvertInt(max.X) + " " + TString::ConvertInt(max.Y) + " " + TString::ConvertInt(max.Z));

	int res = 1;
}