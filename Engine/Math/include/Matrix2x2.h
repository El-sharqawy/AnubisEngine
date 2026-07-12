#pragma once

#include <glm/detail/type_mat2x2.hpp>

struct SVector2Df;
struct SVector3Df;

typedef struct SMatrix2x2
{
	/**
	 * @brief Defines the type for a column vector of the 2x2 matrix.
	 *
	 * In a column-major matrix organization, each column is a vector.
	 * For SMatrix2x2, the column type is a 2D float vector (SVector2Df).
	 */
	typedef SVector2Df col_type;

	/**
	 * @brief Defines the type for a row vector of the 2x2 matrix.
	 *
	 * In a row-major matrix organization, each row is a vector.
	 * For SMatrix2x2, the row type is a 2D float vector (SVector2Df).
	 */
	typedef SVector2Df row_type;

	/**
	 * @brief Default constructor.
	 *
	 * Initializes the matrix using default member initialization (typically zeroing out
	 * the internal storage if the members are primitive types or have a default constructor).
	 * Using '= default' allows the compiler to generate the most efficient default constructor.
	 */
	SMatrix2x2() = default;

	/**
	 * @brief Component-wise constructor for SMatrix2x2.
	 *
	 * Constructs a 2x2 matrix using four individual float values.
	 * The input values are typically organized column-major (OpenGL/GLM style):
	 * [x0, y0] for the first column (value[0]), and [x1, y1] for the second column (value[1]).
	 *
	 * @param x0 The x-component of the first column (Column 0, Row 0).
	 * @param y0 The y-component of the first column (Column 0, Row 1).
	 * @param x1 The x-component of the second column (Column 1, Row 0).
	 * @param y1 The y-component of the second column (Column 1, Row 1).
	 */
	SMatrix2x2(float x0, float y0, float x1, float y1);

	/**
	 * @brief Column vector constructor for SMatrix2x2.
	 *
	 * Constructs a 2x2 matrix using two existing 2D column vectors.
	 *
	 * @param col1 The constant reference to the SVector2Df for the first column (value[0]).
	 * @param col2 The constant reference to the SVector2Df for the second column (value[1]).
	 */
	SMatrix2x2(const SVector2Df& col1, const SVector2Df& col2);

	/**
	 * @brief GLM matrix conversion constructor.
	 *
	 * Constructs an SMatrix2x2 from a standard GLM mat2 object, copying components.
	 *
	 * @param glmMat The constant reference to the glm::mat2 source matrix.
	 */
	SMatrix2x2(const glm::mat2& glmMat);

	/**
	 * @brief Non-constant column access operator (L-value).
	 *
	 * Provides mutable access to the column vector at the specified index.
	 * Includes bounds checking, returning the first column if the index is out of range.
	 *
	 * @param i The column index (0 or 1).
	 * @return SMatrix2x2::col_type& A mutable reference to the column vector (SVector2Df).
	 */
	col_type& operator[](size_t i);

	/**
	 * @brief Constant column access operator (R-value).
	 *
	 * Provides read-only access to the column vector at the specified index.
	 * Includes bounds checking, returning the first column if the index is out of range.
	 *
	 * @param i The column index (0 or 1).
	 * @return const SMatrix2x2::col_type& A constant reference to the column vector (SVector2Df).
	 */
	const col_type& operator[](size_t i) const;

	/**
	 * @brief Assignment operator (in-place modification).
	 *
	 * Copies the contents of the source matrix (mat2) into this matrix (*this).
	 *
	 * @param mat2 The constant reference to the source SMatrix2x2 object.
	 * @return SMatrix2x2& A reference to the modified SMatrix2x2 object (*this).
	 */
	SMatrix2x2& operator=(const SMatrix2x2& mat2);

	/**
	 * @brief Compound matrix addition assignment operator (in-place modification).
	 *
	 * Adds the components of the input matrix (mat2) to the corresponding components
	 * of this matrix (*this) and stores the result back in *this.
	 *
	 * @param mat2 The constant reference to the source SMatrix2x2 object to add.
	 * @return SMatrix2x2& A reference to the modified SMatrix2x2 object (*this).
	 */
	SMatrix2x2& operator+=(const SMatrix2x2& mat2);

	/**
	 * @brief Compound scalar addition assignment operator (in-place modification).
	 *
	 * Adds the scalar value (fVal) to every component of this matrix (*this).
	 *
	 * @param fVal The constant reference to the float scalar value to add.
	 * @return SMatrix2x2& A reference to the modified SMatrix2x2 object (*this).
	 */
	SMatrix2x2& operator+=(const float& fVal);

	/**
	 * @brief Compound matrix subtraction assignment operator (in-place modification).
	 *
	 * Subtracts the components of the input matrix (mat2) from the corresponding components
	 * of this matrix (*this) and stores the result back in *this.
	 *
	 * @param mat2 The constant reference to the source SMatrix2x2 object to subtract.
	 * @return SMatrix2x2& A reference to the modified SMatrix2x2 object (*this).
	 */
	SMatrix2x2& operator-=(const SMatrix2x2& mat2);

	/**
	 * @brief Compound scalar subtraction assignment operator (in-place modification).
	 *
	 * Subtracts the scalar value (fVal) from every component of this matrix (*this).
	 *
	 * @param fVal The constant reference to the float scalar value to subtract.
	 * @return SMatrix2x2& A reference to the modified SMatrix2x2 object (*this).
	 */
	SMatrix2x2& operator-=(const float& fVal);

	/**
	 * @brief Implicit conversion operator to const float pointer.
	 *
	 * Allows the matrix data to be accessed as a contiguous C-style array of float values,
	 * which is necessary for passing matrix uniforms to graphics APIs (like glUniformMatrix2fv in OpenGL).
	 *
	 * @return const float* A pointer to the first element of the matrix data (read-only).
	 */
	operator const float* () const;

	/**
	 * @brief Implicit conversion operator to float pointer.
	 *
	 * Allows the matrix to be treated as a C-style array of float values,
	 * which is essential for passing data to graphics APIs like OpenGL (column-major order).
	 *
	 * @return float* A pointer to the first element of the matrix data.
	 */
	operator float* ();

private:
	col_type value[2];

} Matrix2x2;
