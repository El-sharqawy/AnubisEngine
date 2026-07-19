#include "Stdafx.h"
#include "AnubisEngine.h"
#include "Events/Events.hpp"
#include "Device/VulkanRenderDevice.h"
#include "Device/OpenGLRenderDevice.h"
#include "API/ActorData.h"

bool CAnubisEngine::Initialize(const EGraphicsAPI& api)
{
	m_eGraphicsAPI = api;
	m_bIsRunning = true;
	m_bIsWireframe = false;
	m_bLimitFPS = false;

	m_iTargetFPS = 240;

	// Register the services
	// Core
	CServiceLocator::Register(&memory_manager);
	CServiceLocator::Register(&log_manager);

	// Platform
	CServiceLocator::Register(&timer_manager);
	CServiceLocator::Register(&config_manager);
	CServiceLocator::Register(&input_manager);
	CServiceLocator::Register(&window_manager);
	CServiceLocator::Register(&assimp_model_importer);
	CServiceLocator::Register(&actors_manager);
	CServiceLocator::Register(&animations_manager);
	CServiceLocator::Register(&render_queue);
	CServiceLocator::Register(&scene_manager);
	CServiceLocator::Register(&skin_palette_manager);

	CIRenderDevice* pRenderDevice = CreateRenderDevice(api).release();

	CServiceLocator::Register<CIRenderDevice>(pRenderDevice);

	// 1. Initialize Log Manager
	CServiceLocator::Get<CLogManager>().Initialize();

	WriteLog("We are all alone on life's journey, held captive by the limitations of human consciousness.");

	// 2. Initialize Config Manager
	CServiceLocator::Get<CConfigManager>().Initialize();

	// 3. Initialize Timer Manager
	CServiceLocator::Get<CTimerManager>().Initialize();

	// 4. Initialize Window Manager
	auto& window = CServiceLocator::Get<CWindowManager>();
	window.Initialize(EWindowMode::MODE_WINDOWED, api);

	// 5. Initialize Input Manager
	CServiceLocator::Get<CInputManager>().Initialize();
	
	// 7. Initialize Render Device
	if (!CServiceLocator::Get<CIRenderDevice>().Initialize(window.GetGLFWWindow()))
	{
	 	return false;
	}

	CServiceLocator::Get<CSceneManager>().Initialize();

	// Initialize Events
	InitializeEvents();

	return (true);
}

void CAnubisEngine::Destroy()
{
	CServiceLocator::Get<CSceneManager>().Destroy();

	// Destroy CPU Systems in reverse order
	CServiceLocator::Get<CIRenderDevice>().Shutdown();

	// Platform
	CServiceLocator::Get<CWindowManager>().Destroy();
	CServiceLocator::Get<CInputManager>().Destroy();
	CServiceLocator::Get<CTimerManager>().Destroy();

	// Core
	CServiceLocator::Get<CMemoryManager>().DumpLeaks();
	CServiceLocator::Get<CConfigManager>().Destroy();
	CServiceLocator::Get<CLogManager>().Destroy();

	CServiceLocator::Clear();
}

void CAnubisEngine::Run()
{
	auto& timer = CServiceLocator::Get<CTimerManager>();
	auto& window = CServiceLocator::Get<CWindowManager>();

	while (IsRunning())
	{
		timer.Update();
		float dt = timer.GetDeltaTimeF();

		// if OS says the window should close, request shutdown
		if (window.ShouldClose())
		{
			RequestShutdown();
		}

		Tick(dt);
	}
}

bool CAnubisEngine::IsRunning() const
{
	return (m_bIsRunning);
}

void CAnubisEngine::RequestShutdown()
{
	m_bIsRunning = false;
	CServiceLocator::Get<CWindowManager>().Close();   // tells GLFW to close window
}

void CAnubisEngine::ProcessInput(float deltaTime)
{
	auto& input = CServiceLocator::Get<CInputManager>();
	auto& window = CServiceLocator::Get<CWindowManager>();

	if (input.IsKeyPressed(EInputKey::KEY_F11))
	{
		if (window.GetWindowMode() == EWindowMode::MODE_WINDOWED)
		{
			window.SetWindowMode(EWindowMode::MODE_FULLSCREEN);
		}
		else
		{
			window.SetWindowMode(EWindowMode::MODE_WINDOWED);
		}
	}

	if (input.IsKeyDown(EInputKey::KEY_ESCAPE))
	{
		syslog("Attemp to Shutdown the Engine...");
		RequestShutdown();
	}
}

void CAnubisEngine::InitializeEvents()
{
	auto& bus = CEventBus::Instance();

	// Resize Window Event
	bus.Subscribe(EEventType::WindowResize,
		[](SIEvent& event)
		{
			CEventDispatcher dispatcher(event);
			dispatcher.Dispatch<SIWindowResizeEvent>([](SIWindowResizeEvent& ev) -> bool
				{
					auto& device = CServiceLocator::Get<CIRenderDevice>();

					device.Resize(ev.iWidth, ev.iHeight);
					return true; // handled
				});
		}
	);

	// Close Window
	bus.Subscribe(EEventType::WindowClose, [](SIEvent& event)
		{
			CEventDispatcher dispatcher(event);
			dispatcher.Dispatch<SIWindowCloseEvent>([](SIWindowCloseEvent&) -> bool
				{
					CAnubisEngine::Instance().RequestShutdown();
					return true;
				});
		});

	// Pressing Keyboard Key
	CEventBus::Instance().Subscribe(EEventType::KeyPressed,
		[](SIEvent& event)
		{
			CEventDispatcher dispatcher(event);
			dispatcher.Dispatch<SIKeyPressedEvent>([](SIKeyPressedEvent& ev) -> bool
				{
					auto& input = CServiceLocator::Get<CInputManager>();
					input.OnKey(ev.iKeyCode, true);
					return true; // handled
				});
		}
	);

	// Releasing Keyboard Key
	CEventBus::Instance().Subscribe(EEventType::KeyReleased,
		[](SIEvent& event)
		{
			CEventDispatcher dispatcher(event);
			dispatcher.Dispatch<SIKeyReleasedEvent>([](SIKeyReleasedEvent& ev) -> bool
				{
					auto& input = CServiceLocator::Get<CInputManager>();
					input.OnKey(ev.iKeyCode, false);
					return true; // handled
				});
		}
	);

	// Pressing Mouse Key
	CEventBus::Instance().Subscribe(EEventType::MouseButtonPressed,
		[](SIEvent& event)
		{
			auto& input = CServiceLocator::Get<CInputManager>();

			CEventDispatcher dispatcher(event);
			dispatcher.Dispatch<SIMouseKeyPressedEvent>([&](SIMouseKeyPressedEvent& ev) -> bool
				{
					input.OnMouseButton(ev.iKeyCode, true);
					return true; // handled
				});
		}
	);

	// Releasing Mouse Key
	CEventBus::Instance().Subscribe(EEventType::MouseButtonReleased,
		[](SIEvent& event)
		{
			auto& input = CServiceLocator::Get<CInputManager>();

			CEventDispatcher dispatcher(event);
			dispatcher.Dispatch<SIMouseKeyReleasedEvent>([&](SIMouseKeyReleasedEvent& ev) -> bool
				{
					input.OnMouseButton(ev.iKeyCode, false);
					return true; // handled
				});
		}
	);

	// Mouse Movement
	CEventBus::Instance().Subscribe(EEventType::MouseMoved,
		[](SIEvent& event)
		{
			auto& input = CServiceLocator::Get<CInputManager>();

			CEventDispatcher dispatcher(event);
			dispatcher.Dispatch<SIMouseMovedEvent>([&](SIMouseMovedEvent& ev) -> bool
				{
					input.OnMouseMove(ev.fX, ev.fY);
					return true;
				});
		});

	// Mouse Scroll
	CEventBus::Instance().Subscribe(EEventType::MouseScrolled,
		[](SIEvent& event)
		{
			CEventDispatcher dispatcher(event);
			dispatcher.Dispatch<SIMouseScrolledEvent>([](SIMouseScrolledEvent& ev) -> bool
				{
					auto& input = CServiceLocator::Get<CInputManager>();
					input.OnMouseScroll(ev.fOffsetY);
					return true; // handled
				});
		}
	);
}

void CAnubisEngine::Tick(float deltaTime)
{
	HandleInput(deltaTime);

	// Update UI
	Update(deltaTime);

	// Render Scene
	Render(deltaTime);

	// 4. Frame limiting
	UpdateFPS();
}

void CAnubisEngine::HandleInput(float deltaTime)
{
	auto& window = CServiceLocator::Get<CWindowManager>();
	auto& input = CServiceLocator::Get<CInputManager>();

	// 0. OS events
	window.HandleOsInput();        // glfwPollEvents + callbacks

	// 1. Gameplay / high-level input processing
	ProcessInput(deltaTime);              // your own engine/game code

	// 2. Input state, place all the Pressed buttons before this.
	input.Update(deltaTime);              // finalize per-frame key/mouse state
}

void CAnubisEngine::Update(float deltaTime)
{
	auto& window = CServiceLocator::Get<CWindowManager>();
	auto& sceneMgr = CServiceLocator::Get<CSceneManager>();
	window.Update(deltaTime);
	sceneMgr.Update(deltaTime);
}

void CAnubisEngine::Render(float deltaTime)
{
	CServiceLocator::Get<CIRenderDevice>().BeginFrame();
	CServiceLocator::Get<CIRenderDevice>().Present();
	CServiceLocator::Get<CIRenderDevice>().EndFrame();
}

void CAnubisEngine::UpdateFPS() const
{
	if (m_bLimitFPS)
	{
		auto& timer = CServiceLocator::Get<CTimerManager>();
		timer.LimitFrameRate(m_iTargetFPS);
	}
}

std::unique_ptr<CIRenderDevice> CAnubisEngine::CreateRenderDevice(const EGraphicsAPI& graphicsAPI)
{
	std::unique_ptr<CIRenderDevice> pRenderDevice = nullptr;
	if (graphicsAPI == EGraphicsAPI::API_VULKAN)
	{
		pRenderDevice = std::make_unique<CVulkanRenderDevice>();
	}
	else if (graphicsAPI == EGraphicsAPI::API_OPENGL)
	{
		pRenderDevice = std::make_unique<COpenGLRenderDevice>();
	}

	return (pRenderDevice);
}
