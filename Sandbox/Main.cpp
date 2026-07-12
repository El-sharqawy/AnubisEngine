#include "Stdafx.h"
#include "AnubisEngine.h"
#include <windows.h>

int main()
{
	CAnubisEngine engine; // First Declaration of singleton

	if (!engine.Initialize(EGraphicsAPI::API_VULKAN))
	{
		printf("Failed to Initialize The Engine\n");
		return (EXIT_FAILURE);
	}

	while (engine.IsRunning())
	{
		auto& timer = CServiceLocator::Get<CTimerManager>();
		auto& window = CServiceLocator::Get<CWindowManager>();

		timer.Update();
		float dt = timer.GetDeltaTimeF();

		// if OS says the window should close, request shutdown
		if (window.ShouldClose())
		{
			engine.RequestShutdown();
		}

		engine.Tick(dt);
	}

	engine.Destroy();

	return (EXIT_SUCCESS);
}