#include <utils.h>
#include <memory>
#include "window.h"

#pragma comment(lib, "glfw3.lib")

int main(int argc, char* argv[])
{
	syslog("We are all alone on life's journey, held captive by the limitations of human consciousness.");
	std::unique_ptr<CWindow> pApp = std::make_unique<CWindow>();

	pApp->SetWindowType(EWindowMode::WINDOWED_MODE);

	if (pApp->InitializeWindow() == false)
	{
		return (EXIT_FAILURE);
	}

	pApp->Update();

	return (EXIT_SUCCESS);
}