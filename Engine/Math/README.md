# Anubis Engine — Math Library

A comprehensive, hand-rolled C++ mathematics library built from scratch for the **Anubis Engine**. It covers every mathematical primitive required for 3D real-time rendering, physics, animation, and spatial queries — with zero dependency on GLM or any external math library.

---

## Overview

The Math library is structured as its own CMake module (`Engine/Math`) with clean `include/` and `src/` separation. It is designed for direct integration into the engine's rendering pipeline, physics simulation, and animation systems.

---

## Features at a Glance

| Category | Types / Utilities |
|---|---|
| Vectors | `Vector2`, `Vector3`, `Vector4` (templated) |
| Matrices | `Matrix2x2`, `Matrix3x3`, `Matrix4x4` |
| Quaternions | `Quaternion` (full SLERP, axis-angle, Euler support) |
| Transforms | `Transform` (TRS decomposition) |
| Projections | `PerspectiveProjection`, `OrthographicProjection` |
| Spatial | `BoundingBox`, `Frustum`, `Ray`, `Grid` |
| Utilities | `MathUtils`, `EngineMath` helpers |
| Vertex | `Vertex` (position, normal, UV, tangent, bone data) |

---

## Module Breakdown

### Vectors

Three fully-featured vector types — `Vector2`, `Vector3`, and `Vector4` — each implemented as templated classes with operator overloading for all arithmetic operations.

**Supported operations:**
- Addition, subtraction, scalar multiplication/division
- Dot product and cross product (Vector3)
- Length, squared length, normalization
- Linear interpolation (LERP)
- Component-wise min/max/clamp
- Comparison operators and equality checks

```cpp
Vector3f a(1.0f, 0.0f, 0.0f);
Vector3f b(0.0f, 1.0f, 0.0f);
Vector3f c = a.Cross(b);      // (0, 0, 1)
float    d = a.Dot(b);        // 0.0
Vector3f n = a.Normalized();  // unit vector
```

---

### Matrices

Three matrix types designed for both 2D and 3D use cases.

- **`Matrix2x2`** — 2D linear transformations, rotation, scale
- **`Matrix3x3`** — 3D rotations, normal matrix (inverse-transpose)
- **`Matrix4x4`** — Full 4×4 homogeneous transform matrix; primary type for MVP pipeline

**`Matrix4x4` highlights:**
- Translation, rotation (X/Y/Z axis and arbitrary axis), scale
- `LookAt` view matrix construction
- `Inverse` and `Transpose`
- Row-major / column-major layout compatible with OpenGL

```cpp
Matrix4x4 model;
model.SetTranslation(Vector3f(5.0f, 0.0f, 0.0f));
model.SetRotationY(45.0f);  // degrees
```

---

### Quaternions

A full quaternion implementation in `Quaternion.h` (~27 KB), covering all rotation representations needed for skeletal animation and camera systems.

**Key features:**
- Construction from axis-angle, Euler angles (XYZ order), and rotation matrices
- Spherical linear interpolation (`SLERP`) for smooth animation blending
- Normalized linear interpolation (`NLERP`)
- Quaternion multiplication (composition of rotations)
- Conversion to/from `Matrix4x4` and `Matrix3x3`
- Conjugate, inverse, and normalization

```cpp
Quaternion q1 = Quaternion::FromAxisAngle(Vector3f(0,1,0), 90.0f);
Quaternion q2 = Quaternion::FromEuler(0.0f, 180.0f, 0.0f);
Quaternion q  = Quaternion::Slerp(q1, q2, 0.5f);  // halfway blend
Matrix4x4 rot = q.ToMatrix4x4();
```

---

### Transform

The `Transform` class encapsulates a full TRS (Translation-Rotation-Scale) node, composing them into a single `Matrix4x4` world matrix.

```cpp
Transform t;
t.SetPosition(Vector3f(0.0f, 1.0f, 0.0f));
t.SetRotation(Quaternion::FromEuler(0.0f, 45.0f, 0.0f));
t.SetScale(Vector3f(2.0f, 2.0f, 2.0f));

Matrix4x4 world = t.GetWorldMatrix();
```

---

### Projection Matrices

Two dedicated projection types, decoupled from the camera system for flexibility.

- **`PerspectiveProjection`** — FOV, aspect ratio, near/far planes
- **`OrthographicProjection`** — Left/right/top/bottom/near/far bounds

Both produce OpenGL-compatible NDC clip-space matrices (depth range \([-1, 1]\)).

```cpp
PerspectiveProjection  proj(45.0f, 16.0f / 9.0f, 0.1f, 1000.0f);
OrthographicProjection ortho(-10.0f, 10.0f, -10.0f, 10.0f, 0.1f, 100.0f);
Matrix4x4 P = proj.GetMatrix();
```

---

### Spatial Primitives

#### BoundingBox (AABB)
Axis-aligned bounding box with min/max extents. Used for broad-phase collision and frustum culling.

```cpp
BoundingBox aabb(Vector3f(-1,-1,-1), Vector3f(1,1,1));
bool hit = aabb.Intersects(ray);
```

#### Frustum
View frustum extracted from the view-projection matrix. Contains 6 planes for per-plane point/sphere/AABB intersection tests — the backbone of the engine's culling pipeline.

```cpp
Frustum frustum;
frustum.ExtractPlanes(viewProjectionMatrix);
bool visible = frustum.IsBoxInside(aabb);
```

#### Ray
A ray defined by origin and direction, supporting intersection tests against planes, AABBs, and triangles.

```cpp
Ray ray(cameraPos, rayDir);
float t;
bool hit = ray.IntersectsAABB(aabb, t);
```

#### Grid
A spatial grid structure used for terrain tiling, spatial partitioning, and brush-based editing operations.

---

### MathUtils & EngineMath

General-purpose scalar math utilities:

- `Clamp`, `Lerp`, `SmoothStep`, `Step`
- `ToRadians`, `ToDegrees`
- `IsPowerOfTwo`, `NextPowerOfTwo`
- Trigonometric helpers wrapping `<cmath>` with engine-specific conventions

---

### Vertex

The `Vertex` struct defines the engine's standard vertex layout used across mesh rendering, terrain, and skeletal animation:

```cpp
struct Vertex {
    Vector3f Position;
    Vector3f Normal;
    Vector2f TexCoords;
    Vector3f Tangent;
    Vector3f Bitangent;
    int      BoneIDs[MAX_BONE_INFLUENCE];
    float    BoneWeights[MAX_BONE_INFLUENCE];
};
```

---

## Build Integration

The Math library is compiled as a static library via CMake and linked into the Engine core.

```cmake
# In Engine/Math/CMakeLists.txt
add_library(AnubisMath STATIC ${MATH_SOURCES})
target_include_directories(AnubisMath PUBLIC include/)
```

To use in the engine:
```cpp
#include "Math/include/Vector3.h"
#include "Math/include/Matrix4x4.h"
#include "Math/include/Quaternion.h"
```

Or via the precompiled header `MathPCH.h` which aggregates all math types.

---

## Design Philosophy

- **No external dependencies** — everything is written from scratch in C++17.
- **Engine-native types** — all types use `float` (or `double` via templates) and are layout-compatible with OpenGL uniform uploads.
- **Operator overloading** — math reads naturally in shader-like syntax without verbosity.
- **Header-heavy design** — most types are fully defined in headers for inlining and cross-TU optimization.
- **Modular** — each type is independently includable; you are not forced to pull in the whole library.

---

## License

This library is part of the Anubis Engine project. See [LICENSE.txt](../../LICENSE.txt) at the repository root.
