# AnubisEngine

A modular C++20 game engine built from scratch, featuring dual **OpenGL** and **Vulkan** rendering backends behind a unified render API, a custom tracked memory allocator, and a skeletal animation pipeline.

> ⚠️ **Status:** Active development / early stage. APIs, module boundaries, and build setup are still evolving.

---

## Overview

AnubisEngine is organized as a set of independent static libraries linked together into a single `Engine` target. Each subsystem owns its own public/private headers and `CMakeLists.txt`, making it straightforward to build, test, or replace modules in isolation.

```
Engine/
├── Core/       # Memory, logging, windowing, input, config, events, render API interfaces
├── Math/       # Vectors, matrices, quaternions, transforms, frustum/bounding volumes
├── Model/      # Static & skeletal meshes, bones, animation, Assimp import pipeline
├── Renderer/   # OpenGL and Vulkan backend implementations
└── Scene/      # Scene graph, entities, and components
```

## Features

### Rendering
- **Dual-backend renderer** — OpenGL and Vulkan implementations behind a single `CIRenderDevice` interface (`CreateBuffer`, `CreateTexture2D`, `CreateShaderProgram`, `CreatePipeline`, `CreateMaterial`, etc.)
- Frame-in-flight management (`MAX_FRAMES_IN_FLIGHT`), swapchain handling, command lists, descriptor sets, and pipeline objects for the Vulkan path
- Shader, buffer, texture, and vertex array abstractions for the OpenGL path

### Core Systems
- **Custom tracked memory allocator** (`CMemoryManager`) — tagged allocations, per-block debug headers, corruption detection via magic numbers, and alignment tuned per use case (SIMD/AVX, cache-line, GPU UBO offset alignment)
- **Service Locator** pattern for subsystem access (`AnubisInstance(T)`, `AnubisInstancePtr(T)`, `AnubisHasService(T)`)
- **Type-indexed event bus** (`CEventBus`) with subscribe/fire/emit semantics and event consumption
- Window, input, timer, config, and logging managers

### Math
- Hand-rolled Vector2/3/4, Matrix2/3/4, Quaternion, Transform, Ray, Frustum, and BoundingBox types
- Perspective/orthographic projection utilities

### Models & Animation
- Static and skeletal mesh/model types
- Bone hierarchies, animators, and animation playback
- Assimp-based model import pipeline
- Static and skeletal actor abstractions with a render queue

### Scene
- Entity/component scene representation (`Scene`, `Entity`, `TransformComponent`, `LightComponent`, `IComponent`)

## Requirements

- C++20-compliant compiler (MSVC or GCC/Clang)
- CMake 3.23+
- [GLFW](https://www.glfw.org/) — windowing
- [Vulkan SDK](https://vulkan.lunarg.com/) — Vulkan backend
- [Assimp](https://github.com/assimp/assimp) — model import
- [GLAD](https://glad.dav1d.de/) — OpenGL loader
- [stb_image](https://github.com/nothings/stb) — texture loading
- FreeType / FreeType-GL — text rendering
- Python 3.14 — scripting support

External dependencies are expected under `Extern/Include` and `Extern/lib` at the repository root (see `Engine/CMakeLists.txt`). Prebuilt Windows libraries (MSVC toolset) are referenced directly; Linux builds link against system packages (`GL`, `dl`, `m`, `pthread`) plus a prebuilt GLFW `.so`.

## Building

```bash
git clone <repo-url>
cd GameEngine
mkdir build && cd build
cmake ..
cmake --build .
```

> On Windows, ensure the `Extern/lib/Windows/{Debug,Release}` libraries (GLFW, Assimp, zlib, Python, FreeType/FreeType-GL) and the Vulkan SDK are available at the paths referenced in `Engine/CMakeLists.txt`.
> On Linux, ensure `Extern/lib/Linux/libglfw*.so` is present alongside system OpenGL, Vulkan, and pthread libraries.

## Project Structure Notes

- `AnubisEngine.h` / `AnubisEngine.cpp` at the `Engine/` root wire up all subsystems through the Service Locator and drive the main loop (`Initialize` → `Run` → `Tick` → `HandleInput` / `Update` / `Render`).
- `Stdafx.h` is used as a precompiled header across the `Engine` target.
- Graphics API selection happens at `CAnubisEngine::Initialize(const EGraphicsAPI& api)`, which constructs either an `OpenGLRenderDevice` or `VulkanRenderDevice` via `CreateRenderDevice`.

## Roadmap

- [ ] Expand the Scene module (broader ECS support, hierarchy management)
- [ ] Add automated tests, particularly for the Math library
- [ ] CI build coverage for Windows and Linux
- [ ] Reduce hardcoded MSVC-specific library paths in favor of a package manager (vcpkg/Conan) or `FetchContent`
- [ ] Scripting integration (Python bindings referenced in build, integration TBD)

## License

MIT License

## Author

Osama Elsharqawy