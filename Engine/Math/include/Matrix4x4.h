#pragma once

#include <glm/detail/type_mat4x4.hpp>
#include "TypeVector4.h"
#include "TypeVector3.h"
#include "TypeMatrix3.h"

/**
 * @brief Represents a 4x4 matrix of single-precision floating-point numbers (float).
 *
 * This structure is primarily used for complex 3D transformations (translation,
 * rotation, scale) and projection (perspective/orthographic) in graphics programming.
 * It follows the column-major storage convention, ensuring compatibility with
 * standard APIs like OpenGL and libraries like GLM.
 */
struct SMatrix4x4
{
	/**
	 * @brief Defines the type for a column vector of the 4x4 matrix.
	 *
	 * In a column-major matrix organization, each column is a vector.
	 * For SMatrix4x4, the column type is a 2D float vector (SVector4Df).
	 */
	typedef SVector4Df col_type;

	/**
	 * @brief Defines the type for a row vector of the 4x4 matrix.
	 *
	 * In a row-major matrix organization, each row is a vector.
	 * For SMatrix4x4, the row type is a 2D float vector (SVector4Df).
	 */
	typedef SVector4Df row_type;

	/**
	 * @brief Default constructor.
	 * * Initializes the matrix components using the default member initialization.
	 * This typically results in an uninitialized or zero-initialized state based on how
	 * the member array 'value' is defined.
	 */
	SMatrix4x4() = default;

	/**
	 * @brief Identity constructor.
	 * * Initializes the matrix as an Identity matrix, setting all diagonal elements
	 * (value[i][i]) to fValue and all off-diagonal elements to 0.0f.
	 *
	 * @param fValue The value to set the main diagonal elements (e.g., 1.0f for a standard identity matrix).
	 */
	SMatrix4x4(float fValue);

	/**
	 * @brief Component-wise constructor (Column-Major order).
	 *
	 * Initializes the matrix using 16 individual float values.
	 * The values are supplied column by column (x0..w0 form Column 0, x1..w1 form Column 1, etc.).
	 * 
	 * @param x0 The x-component of the first column (Column 0, Row 0).
	 * @param y0 The y-component of the first column (Column 0, Row 1).
	 * @param z0 The z-component of the first column (Column 0, Row 2).
	 * @param w0 The w-component of the first column (Column 0, Row 3).
	 * @param x1 The x-component of the second column (Column 1, Row 0).
	 * @param y1 The y-component of the second column (Column 1, Row 1).
	 * @param z1 The z-component of the second column (Column 1, Row 2).
	 * @param w1 The w-component of the second column (Column 1, Row 3).
	 * @param x2 The x-component of the third column (Column 2, Row 0).
	 * @param y2 The y-component of the third column (Column 2, Row 1).
	 * @param z2 The z-component of the third column (Column 2, Row 2).
	 * @param w2 The w-component of the third column (Column 2, Row 3).
	 * @param x3 The x-component of the fourth column (Column 3, Row 0).
	 * @param y3 The y-component of the fourth column (Column 3, Row 1).
	 * @param z3 The z-component of the fourth column (Column 3, Row 2).
	 * @param w3 The w-component of the fourth column (Column 3, Row 3).
	 */
	SMatrix4x4(float x0, float y0, float z0, float w0
		, float x1, float y1, float z1, float w1
		, float x2, float y2, float z2, float w2
		, float x3, float y3, float z3, float w3);

	/**
	 * @brief Column-vector constructor.
	 *
	 * Initializes the matrix using four SVector4Df objects, where each vector
	 * represents a column of the resulting matrix (Column-Major format).
	 *
	 * @param col1 The first column vector.
	 * @param col2 The second column vector.
	 * @param col3 The third column vector.
	 * @param col4 The fourth column vector.
	 */
	SMatrix4x4(const SVector4Df& col1, const SVector4Df& col2, const SVector4Df& col3, const SVector4Df& col4);

	/**
	 * @brief matrix3 conversion constructor.
	 *
	 * Initializes the SMatrix4x4 by copying component values directly from a mat3.
	 * This assumes both matrices share the same internal memory layout (typically Column-Major).
	 *
	 * @param mat3 The source SMatrix3x3 object.
	 */
	SMatrix4x4(const SMatrix3x3& mat3);

	/**
	 * @brief GLM matrix conversion constructor.
	 *
	 * Initializes the SMatrix4x4 by copying component values directly from a GLM mat4.
	 * This assumes both matrices share the same internal memory layout (typically Column-Major).
	 *
	 * @param glmMat The source glm::mat4 object.
	 */
	SMatrix4x4(const glm::mat4& glmMat);

	/**
	 * @brief Provides mutable access to a column vector (col_type) in the matrix.
	 *
	 * This operator allows modification of the vector at the specified column index.
	 *
	 * @param i The zero-based index of the column vector (0 to 3).
	 * @return A mutable reference to the column vector (col_type) at index i.
	 */
	col_type& operator[](size_t i);

	/**
	 * @brief Provides read-only access to a column vector (col_type) in the matrix.
	 *
	 * This const operator returns a reference to the column vector at the specified index,
	 * suitable for constant objects.
	 *
	 * @param i The zero-based index of the column vector (0 to 3).
	 * @return A constant reference to the column vector (col_type) at index i.
	 */
	const col_type& operator[](size_t i) const;

	/**
	 * @brief Copies the content of another SMatrix4x4 into this matrix.
	 *
	 * Performs a deep copy of the matrix components.
	 *
	 * @param mat4 The source SMatrix4x4 to copy from.
	 * @return A mutable reference to the current matrix (*this).
	 */
	SMatrix4x4& operator=(const SMatrix4x4& mat4);

	/**
	 * @brief Performs component-wise matrix addition and assigns the result.
	 *
	 * Adds the components of the input matrix to the corresponding components of this matrix.
	 *
	 * @param mat4 The matrix to add.
	 * @return A mutable reference to the current matrix (*this).
	 */
	SMatrix4x4& operator+=(const SMatrix4x4& mat4);

	/**
	 * @brief Performs scalar addition component-wise and assigns the result.
	 *
	 * Adds the scalar value to every component of this matrix.
	 *
	 * @param fVal The scalar value to add.
	 * @return A mutable reference to the current matrix (*this).
	 */
	SMatrix4x4& operator+=(float fVal);

	/**
	 * @brief Performs component-wise matrix subtraction and assigns the result.
	 *
	 * Subtracts the components of the input matrix from the corresponding components of this matrix.
	 *
	 * @param mat4 The matrix to subtract.
	 * @return A mutable reference to the current matrix (*this).
	 */
	SMatrix4x4& operator-=(const SMatrix4x4& mat4);

	/**
	 * @brief Performs scalar subtraction component-wise and assigns the result.
	 *
	 * Subtracts the scalar value from every component of this matrix.
	 *
	 * @param fVal The scalar value to subtract.
	 * @return A mutable reference to the current matrix (*this).
	 */
	SMatrix4x4& operator-=(float fVal);

	/**
	 * @brief Performs component-wise matrix multiplication and assigns the result.
	 *
	 * Note: This is *not* standard matrix-matrix multiplication (dot product).
	 * It multiplies corresponding components (Hadamard product).
	 *
	 * @param mat4 The matrix to multiply.
	 * @return A mutable reference to the current matrix (*this).
	 */
	SMatrix4x4& operator*=(const SMatrix4x4& mat4);

	/**
	 * @brief Performs scalar multiplication component-wise and assigns the result.
	 *
	 * Multiplies every component of this matrix by the scalar value.
	 *
	 * @param fVal The scalar value to multiply by.
	 * @return A mutable reference to the current matrix (*this).
	 */
	SMatrix4x4& operator*=(float fVal);

	/**
	 * @brief Performs matrix division by multiplying by the inverse of the divisor.
	 *
	 * Divides this matrix by multiplying it with the inverse of the input matrix (mat4).
	 * Requires the existence of an 'InverseSub' function.
	 *
	 * @param mat4 The matrix to divide by.
	 * @return A mutable reference to the current matrix (*this).
	 */
	SMatrix4x4& operator/=(const SMatrix4x4& mat4);

	/**
	 * @brief Performs scalar division component-wise and assigns the result.
	 *
	 * Divides every component of this matrix by the scalar value.
	 *
	 * @param fVal The scalar value to divide by.
	 * @return A mutable reference to the current matrix (*this).
	 */
	SMatrix4x4& operator/=(float fVal);

	/**
	 * @brief Pre-increments every component of the matrix by one.
	 *
	 * @return A mutable reference to the incremented matrix (*this).
	 */
	SMatrix4x4& operator++();

	/**
	 * @brief Pre-decrements every component of the matrix by one.
	 *
	 * @return A mutable reference to the decremented matrix (*this).
	 */
	SMatrix4x4& operator--();

	/**
	 * @brief Post-increments every component of the matrix by one.
	 *
	 * Note: Returns a reference to the incremented value, which is non-standard
	 * for post-increment, but common in custom math libraries for efficiency.
	 *
	 * @param int (int) Dummy argument to denote post-increment.
	 * @return A mutable reference to the incremented matrix (*this).
	 */
	SMatrix4x4& operator++(int32_t);

	/**
	 * @brief Post-decrements every component of the matrix by one.
	 *
	 * Note: Returns a reference to the decremented value, which is non-standard
	 * for post-decrement, but common in custom math libraries for efficiency.
	 *
	 * @param int Dummy argument to denote post-decrement.
	 * @return A mutable reference to the decremented matrix (*this).
	 */
	SMatrix4x4& operator--(int32_t);

	/**
	 * @brief Unary plus operator (Returns the matrix unchanged).
	 *
	 * Provides the identity operation, returning a copy of the current matrix.
	 *
	 * @return A new SMatrix4x4 object identical to the current object.
	 */
	SMatrix4x4 operator+() const;

	/**
	 * @brief Unary negation operator.
	 *
	 * Calculates the additive inverse of the matrix by negating every component.
	 *
	 * @return A new SMatrix4x4 object representing the negated matrix.
	 */
	SMatrix4x4 operator-() const;

	/**
	 * @brief Checks for exact component-wise equality between two matrices.
	 *
	 * @param mat4 The matrix to compare against.
	 * @return True if all components are equal; otherwise, false.
	 */
	bool operator==(const SMatrix4x4& mat4);

	/**
	 * @brief Checks for inequality between two matrices.
	 *
	 * @param mat4 The matrix to compare against.
	 * @return True if at least one component is unequal; otherwise, false.
	 */
	bool operator!=(const SMatrix4x4& mat4);

	/**
	 * @brief Initializes the matrix to the 4x4 Identity Matrix.
	 *
	 * Sets all diagonal components (M[i][i]) to 1.0f and all off-diagonal components to 0.0f.
	 */
	void InitIdentity();

	/**
	 * Provides a const pointer to the underlying float array.
	 *
	 * This allows direct access to the matrix vectors components as a float array.
	 *
	 * @return A const pointer to the float array.
	 */
	const float* value_ptr() const;

	/**
	 * @brief Implicit conversion operator to const float pointer.
	 *
	 * Allows the matrix data to be accessed as a contiguous C-style array of float values,
	 * which is necessary for passing matrix uniforms to graphics APIs (like glUniformMatrix4fv in OpenGL).
	 *
	 * @return const float* A pointer to the first element of the matrix data (read-only).
	 */
	operator const float* () const;

	/**
	 * @brief Implicit conversion operator to float pointer.
	 *
	 * Allows the matrix data to be accessed as a contiguous C-style array of float values,
	 * which is necessary for passing matrix uniforms to graphics APIs (like glUniformMatrix4fv in OpenGL).
	 *
	 * @return float* A pointer to the first element of the matrix data.
	 */
	operator float* ();

	/**
	 * @brief Calculates and returns the transpose of the matrix.
	 *
	 * The transpose operation swaps the rows and columns (value[i][j] becomes value[j][i]).
	 * Note: The signature is corrected to return by value (SMatrix4x4) because the function
	 * is declared const and should not modify the current object.
	 *
	 * @return A new SMatrix4x4 object representing the transposed matrix.
	 */
	SMatrix4x4 Transpose();

	/**
	 * @brief Calculates the determinant of the 4x4 matrix.
	 *
	 * This function computes the determinant of the full 4x4 matrix using a highly
	 * optimized cofactor expansion (Laplace expansion) along the first row. The
	 * implementation is unrolled and optimized to minimize overhead, similar to
	 * high-performance math libraries (like GLM).
	 *
	 * @return The determinant of the 4x4 matrix.
	 */
	float Determinant() const;

	/**
	 * @brief Calculates the inverse of the current matrix using the cofactor method.
	 *
	 * This non-member helper function computes the matrix inverse by finding cofactors,
	 * calculating the adjugate, and multiplying by the inverse of the determinant.
	 * It is structured to return the inverse of *this* matrix.
	 *
	 * @return A new SMatrix4x4 object representing the inverse of the current matrix.
	 */
	SMatrix4x4 Inverse();

	/**
	 * @brief Transforms a 3D point using the matrix.
	 *
	 * This function applies the full 4x4 transformation to a 3D point,
	 * including translation, rotation, and scaling. The input point is treated
	 * as a homogeneous coordinate with w=1.
	 *
	 * @param point The 3D point to transform.
	 * @return The transformed 3D point.
	 */
	Vector3D TransformDirection(const Vector3D& dir) const;

	/**
	 * @brief Transforms a 3D point using the matrix, including translation.
	 *
	 * This function applies the full 4x4 transformation to a 3D point,
	 * treating the point as a homogeneous coordinate with w=1. This includes
	 * translation, rotation, and scaling.
	 *
	 * @param point The 3D point to transform.
	 * @return The transformed 3D point.
	 */
	Vector3D TransformPoint(const Vector3D& point) const;

	/**
	 * @brief Transforms a 3D point using the matrix with perspective division.
	 *
	 * This function applies the full 4x4 transformation to a 3D point,
	 * treating the point as a homogeneous coordinate with w=1. After the
	 * transformation, it performs perspective division by the computed W component.
	 *
	 * @param point The 3D point to transform.
	 * @return The transformed 3D point after perspective division.
	 */
	Vector3D TransformPointPerspective(const Vector3D& point) const;

private:
	SVector4Df value[4];
};

/**
 * @brief Performs Matrix * Vector multiplication. (Standard M * v).
 *
 * This operation calculates a **Linear Combination** of the matrix columns.
 * The result is the transformed column vector, which is the standard
 * convention for transforming points/vectors in **OpenGL/GLSL**.
 * The implementation uses a component-wise approach to facilitate SIMD optimization (like SSE).
 *
 * @param mat4 The matrix operand (Left-Hand Side).
 * @param rowVector The vector operand (Right-Hand Side).
 * @return A new Column Vector (col_type) representing the transformed vector.
 */
SMatrix4x4::col_type operator*(const SMatrix4x4& mat4, const SMatrix4x4::row_type& rowVector);

/**
 * @brief Performs Vector * Matrix multiplication. (v * M).
 *
 * This operation calculates the dot product of the input vector against the
 * columns of the matrix. The result is a Row Vector, which is the standard
 * convention for transforming points/vectors in **Row-Major** systems (like DirectX).
 * This is consistent with GLM's definition for v * M.
 *
 * @param colVector The vector operand (Left-Hand Side).
 * @param mat4 The matrix operand (Right-Hand Side).
 * @return A new Row Vector (row_type) representing the transformed vector.
 */
SMatrix4x4::row_type operator*(const SMatrix4x4::col_type& colVector, const SMatrix4x4& mat4);

/**
 * @brief Performs component-wise scalar addition (Matrix + Scalar).
 *
 * Adds the scalar value (fVal) to every component of the matrix (mat).
 *
 * @param mat The matrix operand.
 * @param fVal The scalar operand.
 * @return A new SMatrix4x4 object representing the sum.
 */
SMatrix4x4 operator+(const SMatrix4x4& mat, float fVal);


/**
 * @brief Performs component-wise scalar addition (Scalar + Matrix).
 *
 * Adds the scalar value (fVal) to every component of the matrix (mat).
 *
 * @param fVal The scalar operand.
 * @param mat The matrix operand.
 * @return A new SMatrix4x4 object representing the sum.
 */
SMatrix4x4 operator+(float fVal, const SMatrix4x4& mat);

/**
 * @brief Performs component-wise matrix addition (Matrix + Matrix).
 *
 * Adds the corresponding components of the two input matrices (mat1 and mat2).
 *
 * @param mat1 The left-hand matrix operand.
 * @param mat2 The right-hand matrix operand.
 * @return A new SMatrix4x4 object representing the sum.
 */
SMatrix4x4 operator+(const SMatrix4x4& mat1, const SMatrix4x4& mat2);

/**
 * @brief Performs component-wise scalar subtraction (Matrix - Scalar).
 *
 * Subtracts the scalar value (fVal) from every component of the matrix (mat).
 *
 * @param mat The matrix operand.
 * @param fVal The scalar operand.
 * @return A new SMatrix4x4 object representing the difference.
 */
SMatrix4x4 operator-(const SMatrix4x4& mat, float fVal);

/**
 * @brief Performs component-wise scalar subtraction (Scalar - Matrix).
 *
 * Subtracts every component of the matrix (mat) from the scalar value (fVal).
 *
 * @param fVal The scalar operand.
 * @param mat The matrix operand.
 * @return A new SMatrix4x4 object representing the difference.
 */
SMatrix4x4 operator-(float fVal, const SMatrix4x4& mat);

/**
 * @brief Performs component-wise matrix subtraction (Matrix - Matrix).
 *
 * Subtracts the components of mat2 from the corresponding components of mat1.
 *
 * @param mat1 The left-hand matrix operand.
 * @param mat2 The right-hand matrix operand.
 * @return A new SMatrix4x4 object representing the difference.
 */
SMatrix4x4 operator-(const SMatrix4x4& mat1, const SMatrix4x4& mat2);

/**
 * @brief Performs component-wise scalar multiplication (Matrix * Scalar).
 *
 * Multiplies every component of the matrix (mat) by the scalar value (fVal).
 *
 * @param mat The matrix operand.
 * @param fVal The scalar operand.
 * @return A new SMatrix4x4 object representing the product.
 */
SMatrix4x4 operator*(const SMatrix4x4& mat, float fVal);

/**
 * @brief Performs component-wise scalar multiplication (Scalar * Matrix).
 *
 * Multiplies every component of the matrix (mat) by the scalar value (fVal).
 *
 * @param fVal The scalar operand.
 * @param mat The matrix operand.
 * @return A new SMatrix4x4 object representing the product.
 */
SMatrix4x4 operator*(float fVal, const SMatrix4x4& mat);

/**
 * @brief Performs component-wise scalar division (Matrix / Scalar).
 *
 * Divides every component of the matrix (mat) by the scalar value (fVal).
 *
 * @param mat The matrix operand.
 * @param fVal The scalar operand.
 * @return A new SMatrix4x4 object representing the quotient.
 */
SMatrix4x4 operator/(const SMatrix4x4& mat, float fVal);

/**
 * @brief Performs component-wise scalar division (Scalar / Matrix).
 *
 * Divides the scalar value (fVal) by every component of the matrix (mat).
 * NOTE: This implementation performs mat[i] / fVal, which is mathematically
 * component-wise scalar division of the matrix. This signature is typically
 * discouraged in matrix math.
 *
 * @param fVal The scalar operand.
 * @param mat The matrix operand.
 * @return A new SMatrix4x4 object representing the quotient.
 */
SMatrix4x4 operator/(float fVal, const SMatrix4x4& mat);

/**
 * @brief Performs Matrix / Vector division, defined as M-inverse * v.
 *
 * Calculates the product of the inverse of 'mat' with the vector 'rowVec'.
 * This is consistent with GLM's definition of M / v.
 *
 * @param mat The matrix operand (Left-Hand Side).
 * @param rowVec The vector operand (Right-Hand Side).
 * @return A new Column Vector (col_type) transformed by the inverse matrix.
 */
SMatrix4x4::col_type operator/(const SMatrix4x4& mat, const SMatrix4x4::row_type& rowVec);

/**
 * @brief Performs Vector / Matrix division, defined as v * M-inverse.
 *
 * Calculates the product of the vector 'colVec' with the inverse of the matrix 'mat'.
 * This is the standard Row-Major inverse transformation (v * M^-1) and is consistent
 * with GLM's definition of v / M.
 *
 * @param colVec The vector operand (Left-Hand Side).
 * @param mat The matrix operand (Right-Hand Side).
 * @return A new Row Vector (row_type) transformed by the inverse matrix.
 */
SMatrix4x4::row_type operator/(const SMatrix4x4::col_type& colVec, const SMatrix4x4& mat);

/**
 * @brief Performs Matrix / Matrix division, defined as mat1 * mat2-inverse.
 *
 * This operation calculates the product of the first matrix (mat1) and the
 * inverse of the second matrix (mat2).
 *
 * @param mat1 The left-hand matrix operand.
 * @param mat2 The right-hand matrix operand.
 * @return A new SMatrix4x4 object representing the quotient (mat1 * mat2^-1).
 */
SMatrix4x4 operator/(const SMatrix4x4& mat1, const SMatrix4x4& mat2);

/**
 * @brief Performs standard matrix-matrix multiplication (mat1 * mat2).
 *
 * Computes the matrix product, where mat1 multiplies mat2.
 * This implementation is optimized for vector types where the matrix's
 * internal vectors are columns (Column-Major storage).
 *
 *
 * @param mat1 The left-hand matrix operand.
 * @param mat2 The right-hand matrix operand.
 * @return A new SMatrix4x4 object representing the matrix product.
 */
SMatrix4x4 operator*(const SMatrix4x4& mat1, const SMatrix4x4& mat2);