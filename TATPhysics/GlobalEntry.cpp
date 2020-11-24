#include "TATApplication/TestApp.h"
#include "TATApplication/SoftBodyTestApp.h"
#include "TATApplication/EdgeDistAlgoApp.h"
#include "TATCUDA/CUDADemo.h"

int main()
{
	CUDADemo demo;
	//demo.DimNum();
	//demo.CUDAVectorAdd();
	//demo.CUDAVectorDot();
	//demo.BuildLBVH();
	//demo.CUDAGetMinMax();
	demo.LBVHTest();

	//TestApp* app = new TestApp("TestApp");
	//SoftBodyTestApp* app = new SoftBodyTestApp("SoftBodyTest");

	//EdgeDistAlgoApp* app = new EdgeDistAlgoApp("EdgeDistAlgo");
	//app->Run();

	system("pause");
}