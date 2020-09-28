#include "TATApplication/TestApp.h"
#include "TATApplication/SoftBodyTestApp.h"
#include "TATApplication/EdgeDistAlgoApp.h"

int main()
{
	//TestApp* app = new TestApp("TestApp");
	//SoftBodyTestApp* app = new SoftBodyTestApp("SoftBodyTest");

	EdgeDistAlgoApp* app = new EdgeDistAlgoApp("EdgeDistAlgo");
	app->Run();
}