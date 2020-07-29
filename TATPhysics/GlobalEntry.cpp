#include "TATApplication/TestApp.h"

int main()
{
	TestApp* app = new TestApp("TestApp");
	TATApplicationEntry::Instance()->SetApplication(app);
	app->Run();
}