#include "TATApplication/TestApp.h"
#include "TATApplication/SoftBodyTestApp.h"

int main()
{
	//TestApp* app = new TestApp("TestApp");
	SoftBodyTestApp* app = new SoftBodyTestApp("SoftBodyTest");
	app->Run();
}