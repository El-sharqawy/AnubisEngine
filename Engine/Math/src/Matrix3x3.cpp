#include "MathPCH.h"
#include "Matrix3x3.h"

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
SMatrix3x3::SMatrix3x3(float x0, float y0, float z0, float x1, float y1, float z1, float x2, float y2, float z2)
{
	this->value[0] = col_type(x0, y0, z0);
	this->value[1] = col_type(x1, y1, z1);
	this->value[2] = col_type(x2, y2, z2);
}

/**
 * @brief Column vector constructor for SMatrix3x3.
 *
 * Constructs a 3x3 matrix using three existing 3D column vectors.
 *
 * @param col1 The constant reference to the SVector3Df for the first column (value[0]).
 * @param col2 The constant reference to the SVector3Df for the second column (value[1]).
 * @param col3 The constant reference to the SVector3Df for the third column (value[2]).
 */
SMatrix3x3::SMatrix3x3(const SVector3Df& col1, const SVector3Df& col2, const SVector3Df& col3)
{
	this->value[0] = col_type(col1);
	this->value[1] = col_type(col2);
	this->value[2] = col_type(col3);
}

/**
 * @brief Matrix3 matrix conversion constructor.
 *
 * Constructs an SMatrix3x3 from a standard Matrix3 object, copying components.
 *
 * @param mat3 The constant reference to the Matrix3 source matrix.
 */
SMatrix3x3::SMatrix3x3(const SMatrix3x3& mat3)
{
	this->value[0] = col_type(mat3.value[0]);
	this->value[1] = col_type(mat3.value[1]);
	this->value[2] = col_type(mat3.value[2]);
}

/**
 * @brief Matrix4 matrix conversion constructor.
 *
 * Constructs an SMatrix3x3 from a standard Matrix4 object, copying components.
 *
 * @param mat4 The constant reference to the Matrix3 source matrix.
 */
SMatrix3x3::SMatrix3x3(const Matrix4& mat4)
{
	value[0][0] = mat4[0][0]; value[0][1] = mat4[0][1]; value[0][2] = mat4[0][2];
	value[1][0] = mat4[1][0]; value[1][1] = mat4[1][1]; value[1][2] = mat4[1][2];
	value[2][0] = mat4[2][0]; value[2][1] = mat4[2][1]; value[2][2] = mat4[2][2];
}

/**
 * @brief GLM matrix conversion constructor.
 *
 * Constructs an SMatrix3x3 from a standard GLM mat3 object, copying components.
 * This is useful for interoperability with GLM-based libraries.
 *
 * @param glmMat The constant reference to the glm::mat3 source matrix.
 */
SMatrix3x3::SMatrix3x3(const glm::mat3& glmMat)
{
	// Copy components, assuming both SMatrix3x3 and GLM::mat3 are column-major:
	value[0][0] = glmMat[0][0]; value[0][1] = glmMat[0][1]; value[0][2] = glmMat[0][2];
	value[1][0] = glmMat[1][0]; value[1][1] = glmMat[1][1]; value[1][2] = glmMat[1][2];
	value[2][0] = glmMat[2][0]; value[2][1] = glmMat[2][1]; value[2][2] = glmMat[2][2];
}

/**
 * @brief GLM matrix conversion constructor.
 *
 * Constructs an SMatrix3x3 from a standard GLM mat4 object, copying components.
 * This is useful for interoperability with GLM-based libraries.
 *
 * @param glmMat The constant reference to the glm::mat4 source matrix.
 */
SMatrix3x3::SMatrix3x3(const glm::mat4& glmMat)
{
	// Copy components, assuming both SMatrix3x3 and GLM::mat3 are column-major:
	value[0][0] = glmMat[0][0]; value[0][1] = glmMat[0][1]; value[0][2] = glmMat[0][2];
	value[1][0] = glmMat[1][0]; value[1][1] = glmMat[1][1]; value[1][2] = glmMat[1][2];
	value[2][0] = glmMat[2][0]; value[2][1] = glmMat[2][1]; value[2][2] = glmMat[2][2];
}

/**
 * @brief Non-constant column access operator (L-value).
 *
 * Provides mutable access to the column vector at the specified index.
 * Includes bounds checking, returning the first column if the index is out of range.
 *
 * @param i The column index (0 or 1 or 2).
 * @return SMatrix3x3::col_type& A mutable reference to the column vector (SVector3Df).
 */
SMatrix3x3::col_type& SMatrix3x3::operator[](size_t i)
{
	if (i >= 3)
	{
		// Default to the first column for safety in case of out-of-bounds access.
		return this->value[0];
	}
	return this->value[i];
}

/**
 * @brief Constant column access operator (R-value).
 *
 * Provides read-only access to the column vector at the specified index.
 * Includes bounds checking, returning the first column if the index is out of range.
 *
 * @param i The column index (0 or 1 or 2).
 * @return SMatrix3x3::col_type& A mutable reference to the column vector (SVector3Df).
 */
const SMatrix3x3::col_type& SMatrix3x3::operator[](size_t i) const
{
	if (i >= 3)
	{
		// Default to the first column for safety in case of out-of-bounds access.
		return this->value[0];
	}
	return this->value[i];
}

/**
 * @brief Implicit conversion operator to const float pointer.
 *
 * Allows the matrix data to be accessed as a contiguous C-style array of float values,
 * which is necessary for passing matrix uniforms to graphics APIs (like glUniformMatrix3fv in OpenGL).
 *
 * @return const float* A pointer to the first element of the matrix data (read-only).
 */
SMatrix3x3::operator const float* () const
{
	// Casts the internal array of column vectors (value) to a pointer to the underlying float data.
	return (const float*)value;
}

/**
 * @brief Implicit conversion operator to float pointer.
 *
 * Allows the matrix data to be accessed as a contiguous C-style array of float values,
 * which is necessary for passing matrix uniforms to graphics APIs (like glUniformMatrix3fv in OpenGL).
 *
 * @return float* A pointer to the first element of the matrix data.
 */
SMatrix3x3::operator float* ()
{
	return (float*)value;
}
