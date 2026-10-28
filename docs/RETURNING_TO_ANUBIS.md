# Returning to AnubisEngine

A short guide for getting back into the code after time away. This describes the repository as it is now; it is a map, not a promise that every subsystem is finished.

## The 60-second map

- `Sandbox/Main.cpp` is the sample executable. It initializes the engine, updates the timer, and ticks until shutdown.
- `Engine/AnubisEngine.cpp` wires services together and owns the frame order: input/events → window and scene update → render device frame → FPS update.
- `Engine/Core` contains shared interfaces and services: `CIRenderDevice`, window/input/timer, events, logging, configuration, memory, and the service locator.
- `Engine/Renderer` contains the backend implementations. Shared render API types are in `Engine/Core/include/API`; Vulkan code is in `Engine/Renderer/{include,src}/Vulkan` and `Device/VulkanRenderDevice.*`.
- `Engine/Model` loads and represents static/skeletal models, animation, skin palettes, actors, and the render queue. `Engine/Scene` connects entities/components to those models. `Engine/Terrain` is a separate subsystem.
- `Assets/Config` configures actors and shaders. `shaders/vulkan` has Vulkan GLSL and checked-in SPIR-V; `shaders/opengl` has the OpenGL shaders.
- `Engine/Math` is the custom math library. `Extern` holds third-party headers and libraries; skip it on an initial reread.

## The Vulkan route through one frame

1. The caller chooses the backend in `Initialize(EGraphicsAPI::API_VULKAN)`. **The current sandbox chooses OpenGL** in `Sandbox/Main.cpp`; F1 switches to Vulkan and F2 switches back at runtime (`Engine/AnubisEngine.cpp`).
2. `CAnubisEngine::Initialize` registers core services, creates a backend through `CreateRenderDevice`, initializes the GLFW window for that API, then initializes the render device and scene.
3. `CVulkanRenderDevice::Initialize` creates the Vulkan instance and surface, selects a physical/logical device and queues, creates the command pool and swapchain/frame resources, loads actors and pipelines, then initializes `CVulkanRenderer`.
4. Each engine tick calls `CVulkanRenderDevice::BeginFrame`, `Present`, and `EndFrame`. The renderer waits/acquires and begins recording; opens the swapchain render pass; `Present` updates per-frame data, asks the scene to submit render items, and flushes sorted batches; then `EndFrame` submits work and presents. The name `Present` here is a little counterintuitive: the actual `vkQueuePresentKHR` call is in `CVulkanRenderer::EndFrame`.
5. The scene currently creates a `Warrior_Male` skeletal entity when that actor was loaded from `Assets/Config/EngineActors.json`. Model components contribute render instances to `CRenderQueue`; Vulkan binds pipeline, buffers, material and frame/skin descriptor sets, pushes model data, and issues indexed draws.

Useful first files to follow, in order:

1. `Sandbox/Main.cpp`
2. `Engine/AnubisEngine.cpp` (`Initialize`, `Tick`, `Render`, `SwitchGraphicsAPI`)
3. `Engine/Renderer/src/Device/VulkanRenderDevice.cpp` (`Initialize`, `BeginFrame`, `Present`, `EndFrame`)
4. `Engine/Renderer/src/Vulkan/VulkanRenderer.cpp` (`BeginFrame`, `FlushRenderItems`, `EndFrame`)
5. `Engine/Scene/src/Core/SceneManager.cpp`, `Engine/Scene/src/Entity/SkeletalMeshComponent.cpp`, `Engine/Model/src/Services/RenderQueue.cpp`

## A low-pressure review loop

Aim for 10–15 minutes when you have time. One session is enough; there is no catch-up requirement.

1. Before opening the code, write what you remember about one item below.
2. Read just the linked area and correct your explanation in a few sentences.
3. End by writing one question or the next file you would inspect. Stop there.

Suggested rotation:

- **Session 1 — engine startup:** `CAnubisEngine::Initialize`; name the services registered and the initialization order.
- **Session 2 — a Vulkan frame:** trace acquire → render pass → submit → present. Explain why the frame index and swapchain image index are different.
- **Session 3 — a draw:** trace `SceneManager::Render` through the render queue to `FlushRenderItems` and `DrawIndexed`.
- **Session 4 — resources:** follow one buffer or texture from the render API through its Vulkan implementation and destruction path.
- **Session 5 — shader/pipeline:** follow the shader config/path resolution through `PipelinesManager` and Vulkan pipeline creation.
- **Session 6 — resize and lifetime:** follow the resize event, swapchain recreation, and shutdown order.

When you return after a longer gap, start at the top of this guide and pick one session. You can ask an assistant to quiz you on one session, explain a file you have open, or update this note when the architecture changes.

## Re-entry notes and questions to verify

- The sample starts in OpenGL despite having a Vulkan backend; select Vulkan with F1 or change the initial API in `Sandbox/Main.cpp` if that is what you intend to run.
- The Vulkan device chooser currently accepts only a discrete GPU with geometry shader support and swapchain extension support (`CVulkanDevice::IsDeviceSuitable`). A system without that combination may not initialize.
- Vulkan instance creation enables `VK_LAYER_KHRONOS_validation` and targets Vulkan 1.4 (`CVulkanDevice::Initialize`); confirm those assumptions on the machine you use when you resume.
- The checked-in SPIR-V files in `shaders/vulkan` are runtime artifacts alongside GLSL sources. `shaders/vulkan/compile.bat` is the nearby shader compile helper.
- Startup loads actor configuration and the scene looks specifically for `Warrior_Male`; if no actor appears, inspect the config/asset paths before beginning with renderer internals.
- The guide does not imply the project was built or run as part of this documentation pass.

## Scratchpad

Copy this small block after a review session if you want to keep a dated trail:

```text
Date:
Area reviewed:
What I remembered before reading:
What the code actually does:
Question / next file:
```
