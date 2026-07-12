#pragma once

#include "Memory/MemoryManager.h"
#include "Logging/LogManager.h"
#include "Time/TimerManager.h"
#include "Config/ConfigManager.h"
#include "Input/InputManager.h"
#include "Window/WindowManager.h"
#include "API/RenderDevice.h"
#include "Events/EventBus.h"
#include "Textures/TexturesManager.h"
#include "Device/PipelinesManager.h"

#define AnubisInstance(T)				CServiceLocator::Instance().Get<T>()
#define AnubisInstancePtr(T)			CServiceLocator::Instance().GetPtr<T>()
#define AnubisHasService(T)				CServiceLocator::Instance().IsRegistered<T>()

class CAnubisEngine : public CSingleton<CAnubisEngine>
{
public:
	CAnubisEngine() = default;
	~CAnubisEngine() = default;

	// Disable copying - You don't want two managers managing the same list!
	CAnubisEngine(const CAnubisEngine&) = delete;
	CAnubisEngine& operator=(const CAnubisEngine&) = delete;

	bool Initialize(const EGraphicsAPI& api);
	void Destroy();

	void Run();                    // contains the main loop

	// Subsystem accessors (via ServiceLocator — don't store them here)
	bool IsRunning() const;
	void RequestShutdown();

	void ProcessInput(float deltaTime);

	void InitializeEvents();
	void Tick(float deltaTime);                    // one frame

	void HandleInput(float deltaTime);
	void Update(float deltaTime);
	void Render(float deltaTime);
	void UpdateFPS() const;

	std::unique_ptr<CIRenderDevice> CreateRenderDevice(const EGraphicsAPI& graphicsAPI);

	// Service Locators
protected:

	// Core
	CServiceLocator service_locator;
	CEventBus event_bus;

	// Managers are plain stack objects — no singleton
	CMemoryManager memory_manager;
	CLogManager log_manager;

	// Platform
	CTimerManager timer_manager;
	CConfigManager config_manager;
	CInputManager input_manager;
	CWindowManager window_manager;
	CTexturesManager textures_mgr;
	CPipelinesManager pipelines_manager;

private:
	bool m_bIsRunning = true;
	bool m_bIsWireframe = false;
	bool m_bLimitFPS = false;

	int32_t m_iTargetFPS = 60;

	EGraphicsAPI m_eGraphicsAPI = EGraphicsAPI::API_OPENGL;
};