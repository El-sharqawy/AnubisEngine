#pragma once

#include "Matrix4x4.h"

/**
 * @brief Represents a 4x4 matrix of single-precision floating-point numbers (GLfloat).
 *
 * This structure is primarily used for complex 3D transformations (translation,
 * rotation, scale) and projection (perspective/orthographic) in graphics programming.
 * It follows the column-major storage convention, ensuring compatibility with
 * standard APIs like OpenGL and libraries like GLM.
 */
using Matrix4x4 = SMatrix4x4;

/**
 * @brief Represents a 4x4 matrix of single-precision floating-point numbers (GLfloat).
 *
 * This structure is primarily used for complex 3D transformations (translation,
 * rotation, scale) and projection (perspective/orthographic) in graphics programming.
 * It follows the column-major storage convention, ensuring compatibility with
 * standard APIs like OpenGL and libraries like GLM.
 */
using Matrix4 = SMatrix4x4;
