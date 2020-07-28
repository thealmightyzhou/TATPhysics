#include "TATApplication/TestApp.h"

int main()
{
	TestApp* app = new TestApp("TestApp");
	TATApplicationEntry::SetApplication(app);
	app->Run();
}