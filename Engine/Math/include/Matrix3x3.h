#pragma once

#include <glm/detail/type_mat3x3.hpp>

struct SVector3Df;
struct SMatrix4x4;

struct SMatrix3x3
{
	/**
	 * @brief Defines the type for a column vector of the 3x3 matrix.
	 *
	 * In a column-major matrix organization, each column is a vector.
	 * For SMatrix3x3, the column type is a 3D float vector (SVector3Df).
	 */
	typedef SVector3Df col_type;

	/**
	 * @brief Defines the type for a row vector of the 3x3 matrix.
	 *
	 * In a row-major matrix organization, each row is a vector.
	 * For SMatrix3x3, the row type is a 3D float vector (SVector3Df).
	 */
	typedef SVector3Df row_type;

	/**
	 * @brief Default constructor.
	 *
	 * Initializes the matrix using default member initialization (typically zeroing out
	 * the internal storage if the members are primitive types or have a default constructor).
	 * Using '= default' allows the compiler to generate the most efficient default constructor.
	 */
	SMatrix3x3() = default;

	/**
	 * @brief Component-wise constructor for SMatrix3x3 (Column-Major).
	 *
	 * Constructs a 3x3 matrix using nine individual float values.
	 * The input values are ordered column-by-column (OpenGL/GLM style).
	 *
	 * @param x0 The x-component of the first column (Column 0, Row 0).
	 * @param y0 The y-component of the first column (Column 0, Row 1).
	 * @param z0 The z-component of the first column (Column 0, Row 2).
	 * @param x1 The x-component of the second column (Column 1, Row 0).
	 * @param y1 The y-component of the second column (Column 1, Row 1).
	 * @param z1 The z-component of the second column (Column 1, Row 2).
	 * @param x2 The x-component of the third column (Column 2, Row 0).
	 * @param y2 The y-component of the third column (Column 2, Row 1).
	 * @param z2 The z-component of the third column (Column 2, Row 2).
	 */
	SMatrix3x3(float x0, float y0, float z0, float x1, float y1, float z1, float x2, float y2, float z2);

	/**
	 * @brief Column vector constructor for SMatrix3x3.
	 *
	 * Constructs a 3x3 matrix using three existing 3D column vectors.
	 *
	 * @param col1 The constant reference to the SVector3Df for the first column (value[0]).
	 * @param col2 The constant reference to the SVector3Df for the second column (value[1]).
	 * @param col3 The constant reference to the SVector3Df for the third column (value[2]).
	 */
	SMatrix3x3(const SVector3Df& col1, const SVector3Df& col2, const SVector3Df& col3);

	/**
	 * @brief Matrix3 matrix conversion constructor.
	 *
	 * Constructs an SMatrix3x3 from a standard Matrix3 object, copying components.
	 *
	 * @param glmMat The constant reference to the Matrix3 source matrix.
	 */
	SMatrix3x3(const SMatrix3x3& mat3);

	/**
	 * @brief Matrix4 matrix conversion constructor.
	 *
	 * Constructs an SMatrix3x3 from a standard Matrix4 object, copying components.
	 *
	 * @param mat4 The constant reference to the Matrix3 source matrix.
	 */
	SMatrix3x3(const SMatrix4x4& mat4);

	/**
	 * @brief GLM matrix conversion constructor.
	 *
	 * Constructs an SMatrix3x3 from a standard GLM mat3 object, copying components.
	 * This is useful for interoperability with GLM-based libraries.
	 *
	 * @param glmMat The constant reference to the glm::mat3 source matrix.
	 */
	SMatrix3x3(const glm::mat3& glmMat);

	/**
	 * @brief GLM matrix conversion constructor.
	 *
	 * Constructs an SMatrix3x3 from a standard GLM mat4 object, copying components.
	 * This is useful for interoperability with GLM-based libraries.
	 *
	 * @param glmMat The constant reference to the glm::mat4 source matrix.
	 */
	SMatrix3x3(const glm::mat4& glmMat);

	/**
	 * @brief Non-constant column access operator (L-value).
	 *
	 * Provides mutable access to the column vector at the specified index.
	 * Includes bounds checking, returning the first column if the index is out of range.
	 *
	 * @param i The column index (0 or 1 or 2).
	 * @return SMatrix3x3::col_type& A mutable reference to the column vector (SVector3Df).
	 */
	col_type& operator[](size_t i);

	/**
	 * @brief Constant column access operator (R-value).
	 *
	 * Provides read-only access to the column vector at the specified index.
	 * Includes bounds checking, returning the first column if the index is out of range.
	 *
	 * @param i The column index (0 or 1 or 2).
	 * @return SMatrix3x3::col_type& A mutable reference to the column vector (SVector3Df).
	 */
	const col_type& operator[](size_t i) const;

	/**
	 * @brief Implicit conversion operator to const float pointer.
	 *
	 * Allows the matrix data to be accessed as a contiguous C-style array of float values,
	 * which is necessary for passing matrix uniforms to graphics APIs (like glUniformMatrix3fv in OpenGL).
	 *
	 * @return const float* A pointer to the first element of the matrix data (read-only).
	 */
	operator const float* () const;

	/**
	 * @brief Implicit conversion operator to float pointer.
	 *
	 * Allows the matrix data to be accessed as a contiguous C-style array of float values,
	 * which is necessary for passing matrix uniforms to graphics APIs (like glUniformMatrix3fv in OpenGL).
	 *
	 * @return float* A pointer to the first element of the matrix data.
	 */
	operator float* ();

private:
	col_type value[3];
};
