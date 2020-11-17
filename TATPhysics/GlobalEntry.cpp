#include "TATApplication/TestApp.h"
#include "TATApplication/SoftBodyTestApp.h"
#include "TATApplication/EdgeDistAlgoApp.h"
#include "TATCUDA/CUDADemo.h"

int main()
{
	CUDADemo demo;
	//demo.CUDAHelloWorld();
	//demo.DimNum();
	//demo.CUDAVectorAdd();
	//demo.CUDAVectorDot();
	//demo.BuildLBVH();
	demo.CUDAGetMinMax();
	//TestApp* app = new TestApp("TestApp");
	//SoftBodyTestApp* app = new SoftBodyTestApp("SoftBodyTest");

	//EdgeDistAlgoApp* app = new EdgeDistAlgoApp("EdgeDistAlgo");
	//app->Run();

	system("pause");
}