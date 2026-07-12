#include "MathPCH.h"
#include "Matrix4x4.h"

/**
 * @brief Identity constructor.
 * * Initializes the matrix as an Identity matrix, setting all diagonal elements
 * (value[i][i]) to fValue and all off-diagonal elements to 0.0f.
 *
 * @param fValue The value to set the main diagonal elements (e.g., 1.0f for a standard identity matrix).
 */
SMatrix4x4::SMatrix4x4(float fValue)
{
	this->value[0] = col_type(fValue, 0.0f, 0.0f, 0.0f);
	this->value[1] = col_type(0.0f, fValue, 0.0f, 0.0f);
	this->value[2] = col_type(0.0f, 0.0f, fValue, 0.0f);
	this->value[3] = col_type(0.0f, 0.0f, 0.0f, fValue);
}

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
SMatrix4x4::SMatrix4x4(float x0, float y0, float z0, float w0
	, float x1, float y1, float z1, float w1
	, float x2, float y2, float z2, float w2
	, float x3, float y3, float z3, float w3)
{
	this->value[0] = col_type(x0, y0, z0, w0);
	this->value[1] = col_type(x1, y1, z1, w1);
	this->value[2] = col_type(x2, y2, z2, w2);
	this->value[3] = col_type(x3, y3, z3, w3);
}

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
SMatrix4x4::SMatrix4x4(const SVector4Df& col1, const SVector4Df& col2, const SVector4Df& col3, const SVector4Df& col4)
{
	this->value[0] = col_type(col1);
	this->value[1] = col_type(col2);
	this->value[2] = col_type(col3);
	this->value[3] = col_type(col4);
}

/**
 * @brief matrix3 conversion constructor.
 *
 * Initializes the SMatrix4x4 by copying component values directly from a mat3.
 * This assumes both matrices share the same internal memory layout (typically Column-Major).
 *
 * @param mat3 The source SMatrix3x3 object.
 */
SMatrix4x4::SMatrix4x4(const SMatrix3x3& mat3)
{
	value[0] = col_type(mat3[0], 0.0f);  // col0, w=0
	value[1] = col_type(mat3[1], 0.0f);  // col1, w=0
	value[2] = col_type(mat3[2], 0.0f);  // col2, w=0
	value[3] = col_type(0, 0, 0, 1.0f);  // translation = 0, w=1
}

/**
 * @brief GLM matrix conversion constructor.
 *
 * Initializes the SMatrix4x4 by copying component values directly from a GLM mat4.
 * This assumes both matrices share the same internal memory layout (typically Column-Major).
 *
 * @param glmMat The source glm::mat4 object.
 */
SMatrix4x4::SMatrix4x4(const glm::mat4& glmMat)
{
	value[0][0] = glmMat[0][0]; value[0][1] = glmMat[0][1]; value[0][2] = glmMat[0][2]; value[0][3] = glmMat[0][3];
	value[1][0] = glmMat[1][0]; value[1][1] = glmMat[1][1]; value[1][2] = glmMat[1][2]; value[1][3] = glmMat[1][3];
	value[2][0] = glmMat[2][0]; value[2][1] = glmMat[2][1]; value[2][2] = glmMat[2][2]; value[2][3] = glmMat[2][3];
	value[3][0] = glmMat[3][0]; value[3][1] = glmMat[3][1]; value[3][2] = glmMat[3][2]; value[3][3] = glmMat[3][3];
}

/**
 * @brief Provides mutable access to a column vector (col_type) in the matrix.
 *
 * This operator allows modification of the vector at the specified column index.
 *
 * @param i The zero-based index of the column vector (0 to 3).
 * @return A mutable reference to the column vector (col_type) at index i.
 */
SMatrix4x4::col_type& SMatrix4x4::operator[](size_t i)
{
	if (i >= 4)
	{
		// Default to the first column for safety in case of out-of-bounds access.
		return this->value[0];
	}
	return this->value[i];
}

/**
 * @brief Provides read-only access to a column vector (col_type) in the matrix.
 *
 * This const operator returns a reference to the column vector at the specified index,
 * suitable for constant objects.
 *
 * @param i The zero-based index of the column vector (0 to 3).
 * @return A constant reference to the column vector (col_type) at index i.
 */
const SMatrix4x4::col_type& SMatrix4x4::operator[](size_t i) const
{
	if (i >= 4)
	{
		// Default to the first column for safety in case of out-of-bounds access.
		return this->value[0];
	}
	return this->value[i];
}

/**
 * @brief Copies the content of another SMatrix4x4 into this matrix.
 *
 * Performs a deep copy of the matrix components.
 *
 * @param mat4 The source SMatrix4x4 to copy from.
 * @return A mutable reference to the current matrix (*this).
 */
SMatrix4x4& SMatrix4x4::operator=(const SMatrix4x4& mat4)
{
	//memcpy could be faster
	//memcpy(&this->value, &m.value, 16 * sizeof(valType));
	this->value[0] = mat4[0];
	this->value[1] = mat4[1];
	this->value[2] = mat4[2];
	this->value[3] = mat4[3];
	return *this;
}

/**
 * @brief Performs component-wise matrix addition and assigns the result.
 *
 * Adds the components of the input matrix to the corresponding components of this matrix.
 *
 * @param mat4 The matrix to add.
 * @return A mutable reference to the current matrix (*this).
 */
SMatrix4x4& SMatrix4x4::operator+=(const SMatrix4x4& mat4)
{
	this->value[0] += mat4[0];
	this->value[1] += mat4[1];
	this->value[2] += mat4[2];
	this->value[3] += mat4[3];
	return *this;
}

/**
 * @brief Performs scalar addition component-wise and assigns the result.
 *
 * Adds the scalar value to every component of this matrix.
 *
 * @param fVal The scalar value to add.
 * @return A mutable reference to the current matrix (*this).
 */
SMatrix4x4& SMatrix4x4::operator+=(float fVal)
{
	this->value[0] += fVal;
	this->value[1] += fVal;
	this->value[2] += fVal;
	this->value[3] += fVal;
	return *this;
}

/**
 * @brief Performs component-wise matrix subtraction and assigns the result.
 *
 * Subtracts the components of the input matrix from the corresponding components of this matrix.
 *
 * @param mat4 The matrix to subtract.
 * @return A mutable reference to the current matrix (*this).
 */
SMatrix4x4& SMatrix4x4::operator-=(const SMatrix4x4& mat4)
{
	this->value[0] -= mat4[0];
	this->value[1] -= mat4[1];
	this->value[2] -= mat4[2];
	this->value[3] -= mat4[3];
	return *this;
}

/**
 * @brief Performs scalar subtraction component-wise and assigns the result.
 *
 * Subtracts the scalar value from every component of this matrix.
 *
 * @param fVal The scalar value to subtract.
 * @return A mutable reference to the current matrix (*this).
 */
SMatrix4x4& SMatrix4x4::operator-=(float fVal)
{
	this->value[0] -= fVal;
	this->value[1] -= fVal;
	this->value[2] -= fVal;
	this->value[3] -= fVal;
	return *this;
}

/**
 * @brief Performs component-wise matrix multiplication and assigns the result.
 *
 * Note: This is *not* standard matrix-matrix multiplication (dot product).
 * It multiplies corresponding components (Hadamard product).
 *
 * @param mat4 The matrix to multiply.
 * @return A mutable reference to the current matrix (*this).
 */
SMatrix4x4& SMatrix4x4::operator*=(const SMatrix4x4& mat4)
{
	this->value[0] *= mat4[0];
	this->value[1] *= mat4[1];
	this->value[2] *= mat4[2];
	this->value[3] *= mat4[3];
	return *this;
}

/**
 * @brief Performs scalar multiplication component-wise and assigns the result.
 *
 * Multiplies every component of this matrix by the scalar value.
 *
 * @param fVal The scalar value to multiply by.
 * @return A mutable reference to the current matrix (*this).
 */
SMatrix4x4& SMatrix4x4::operator*=(float fVal)
{
	this->value[0] *= fVal;
	this->value[1] *= fVal;
	this->value[2] *= fVal;
	this->value[3] *= fVal;
	return *this;
}

/**
 * @brief Performs matrix division by multiplying by the inverse of the divisor.
 *
 * Divides this matrix by multiplying it with the inverse of the input matrix (mat4).
 * Requires the existence of an 'InverseSub' function.
 *
 * @param mat4 The matrix to divide by.
 * @return A mutable reference to the current matrix (*this).
 */
SMatrix4x4& SMatrix4x4::operator/=(const SMatrix4x4& mat4)
{
	SMatrix4x4 InversedMat = mat4;
	InversedMat = InversedMat.Inverse();
	return *this *= InversedMat;
}

/**
 * @brief Performs scalar division component-wise and assigns the result.
 *
 * Divides every component of this matrix by the scalar value.
 *
 * @param fVal The scalar value to divide by.
 * @return A mutable reference to the current matrix (*this).
 */
SMatrix4x4& SMatrix4x4::operator/=(float fVal)
{
	if (fVal != 0.0f)
	{
		this->value[0] /= fVal;
		this->value[1] /= fVal;
		this->value[2] /= fVal;
		this->value[3] /= fVal;
	}
	return *this;
}

/**
 * @brief Pre-increments every component of the matrix by one.
 *
 * @return A mutable reference to the incremented matrix (*this).
 */
SMatrix4x4& SMatrix4x4::operator++()
{
	this->value[0]++;
	this->value[1]++;
	this->value[2]++;
	this->value[3]++;
	return (*this);
}

/**
 * @brief Pre-decrements every component of the matrix by one.
 *
 * @return A mutable reference to the decremented matrix (*this).
 */
SMatrix4x4& SMatrix4x4::operator--()
{
	this->value[0]--;
	this->value[1]--;
	this->value[2]--;
	this->value[3]--;
	return (*this);
}

/**
 * @brief Post-increments every component of the matrix by one.
 *
 * Note: Returns a reference to the incremented value, which is non-standard
 * for post-increment, but common in custom math libraries for efficiency.
 *
 * @param int (int) Dummy argument to denote post-increment.
 * @return A mutable reference to the incremented matrix (*this).
 */
SMatrix4x4& SMatrix4x4::operator++(int32_t)
{
	this->value[0]++;
	this->value[1]++;
	this->value[2]++;
	this->value[3]++;
	return (*this);
}

/**
 * @brief Post-decrements every component of the matrix by one.
 *
 * Note: Returns a reference to the decremented value, which is non-standard
 * for post-decrement, but common in custom math libraries for efficiency.
 *
 * @param int Dummy argument to denote post-decrement.
 * @return A mutable reference to the decremented matrix (*this).
 */
SMatrix4x4& SMatrix4x4::operator--(int32_t)
{
	this->value[0]--;
	this->value[1]--;
	this->value[2]--;
	this->value[3]--;
	return (*this);
}

/**
 * @brief Unary plus operator (Returns the matrix unchanged).
 *
 * Provides the identity operation, returning a copy of the current matrix.
 *
 * @return A new SMatrix4x4 object identical to the current object.
 */
SMatrix4x4 SMatrix4x4::operator+() const
{
	return (*this);
}

/**
 * @brief Unary negation operator.
 *
 * Calculates the additive inverse of the matrix by negating every component.
 *
 * @return A new SMatrix4x4 object representing the negated matrix.
 */
SMatrix4x4 SMatrix4x4::operator-() const
{
	SMatrix4x4 Result(-value[0], -value[1], -value[2], -value[3]);
	return (Result);
}

/**
 * @brief Checks for exact component-wise equality between two matrices.
 *
 * @param mat4 The matrix to compare against.
 * @return True if all components are equal; otherwise, false.
 */
bool SMatrix4x4::operator==(const SMatrix4x4& mat4)
{
	return (value[0] == mat4[0] && value[1] == mat4[1] && value[2] == mat4[2] && value[3] == mat4[3]);
}

/**
 * @brief Checks for inequality between two matrices.
 *
 * @param mat4 The matrix to compare against.
 * @return True if at least one component is unequal; otherwise, false.
 */
bool SMatrix4x4::operator!=(const SMatrix4x4& mat4)
{
	return (value[0] != mat4[0]) || (value[1] != mat4[1]) || (value[2] != mat4[2]) || (value[3] != mat4[3]);
}

/**
 * @brief Initializes the matrix to the 4x4 Identity Matrix.
 *
 * Sets all diagonal components (M[i][i]) to 1.0f and all off-diagonal components to 0.0f.
 */
void SMatrix4x4::InitIdentity()
{
	value[0][0] = 1.0f; value[0][1] = 0.0f; value[0][2] = 0.0f; value[0][3] = 0.0f;
	value[1][0] = 0.0f; value[1][1] = 1.0f; value[1][2] = 0.0f; value[1][3] = 0.0f;
	value[2][0] = 0.0f; value[2][1] = 0.0f; value[2][2] = 1.0f; value[2][3] = 0.0f;
	value[3][0] = 0.0f; value[3][1] = 0.0f; value[3][2] = 0.0f; value[3][3] = 1.0f;
}

/**
 * Provides a const pointer to the underlying float array.
 *
 * This allows direct access to the matrix vectors components as a float array.
 *
 * @return A const pointer to the float array.
 */
const float* SMatrix4x4::value_ptr() const
{
	// Safely cast the address of the first vector to a pointer to float.
	// This is safe because SVector4Df is an array of float or a struct 
	// with standard layout whose first member is float.
	return (const float*)value;
}

/**
 * @brief Implicit conversion operator to const float pointer.
 *
 * Allows the matrix data to be accessed as a contiguous C-style array of float values,
 * which is necessary for passing matrix uniforms to graphics APIs (like glUniformMatrix4fv in OpenGL).
 *
 * @return const float* A pointer to the first element of the matrix data (read-only).
 */
SMatrix4x4::operator const float* () const
{
	return (const float*)value;
}

/**
 * @brief Implicit conversion operator to float pointer.
 *
 * Allows the matrix data to be accessed as a contiguous C-style array of float values,
 * which is necessary for passing matrix uniforms to graphics APIs (like glUniformMatrix4fv in OpenGL).
 *
 * @return float* A pointer to the first element of the matrix data.
 */
SMatrix4x4::operator float* ()
{
	return (float*)value;
}

/**
 * @brief Calculates and returns the transpose of the matrix.
 *
 * The transpose operation swaps the rows and columns (value[i][j] becomes value[j][i]).
 * Note: The signature is corrected to return by value (SMatrix4x4) because the function
 * is declared const and should not modify the current object.
 *
 * @return A new SMatrix4x4 object representing the transposed matrix.
 */
SMatrix4x4 SMatrix4x4::Transpose()
{
	SMatrix4x4 transposedMat{ 0.0f };
	transposedMat[0][0] = value[0][0];
	transposedMat[0][1] = value[1][0];
	transposedMat[0][2] = value[2][0];
	transposedMat[0][3] = value[3][0];

	transposedMat[1][0] = value[0][1];
	transposedMat[1][1] = value[1][1];
	transposedMat[1][2] = value[2][1];
	transposedMat[1][3] = value[3][1];

	transposedMat[2][0] = value[0][2];
	transposedMat[2][1] = value[1][2];
	transposedMat[2][2] = value[2][2];
	transposedMat[2][3] = value[3][2];

	transposedMat[3][0] = value[0][3];
	transposedMat[3][1] = value[1][3];
	transposedMat[3][2] = value[2][3];
	transposedMat[3][3] = value[3][3];

	*this = transposedMat;
	return (*this);
}

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
float SMatrix4x4::Determinant() const
{
	const float SubFactor00 = value[2][2] * value[3][3] - value[3][2] * value[2][3];
	const float SubFactor01 = value[2][1] * value[3][3] - value[3][1] * value[2][3];
	const float SubFactor02 = value[2][1] * value[3][2] - value[3][1] * value[2][2];
	const float SubFactor03 = value[2][0] * value[3][3] - value[3][0] * value[2][3];
	const float SubFactor04 = value[2][0] * value[3][2] - value[3][0] * value[2][2];
	const float SubFactor05 = value[2][0] * value[3][1] - value[3][0] * value[2][1];

	SVector4Df DetCof(
		+(value[1][1] * SubFactor00 - value[1][2] * SubFactor01 + value[1][3] * SubFactor02),
		-(value[1][0] * SubFactor00 - value[1][2] * SubFactor03 + value[1][3] * SubFactor04),
		+(value[1][0] * SubFactor01 - value[1][1] * SubFactor03 + value[1][3] * SubFactor05),
		-(value[1][0] * SubFactor02 - value[1][1] * SubFactor04 + value[1][2] * SubFactor05)
	);

	return value[0][0] * DetCof[0] + value[0][1] * DetCof[1] + value[0][2] * DetCof[2] + value[0][3] * DetCof[3];
}

/**
 * @brief Calculates the inverse of the current matrix using the cofactor method.
 *
 * This non-member helper function computes the matrix inverse by finding cofactors,
 * calculating the adjugate, and multiplying by the inverse of the determinant.
 * It is structured to return the inverse of *this* matrix.
 *
 * @return A new SMatrix4x4 object representing the inverse of the current matrix.
 */
inline SMatrix4x4 SMatrix4x4::Inverse()
{
	const float Coef00 = value[2][2] * value[3][3] - value[3][2] * value[2][3];
	const float Coef02 = value[1][2] * value[3][3] - value[3][2] * value[1][3];
	const float Coef03 = value[1][2] * value[2][3] - value[2][2] * value[1][3];

	const float Coef04 = value[2][1] * value[3][3] - value[3][1] * value[2][3];
	const float Coef06 = value[1][1] * value[3][3] - value[3][1] * value[1][3];
	const float Coef07 = value[1][1] * value[2][3] - value[2][1] * value[1][3];

	const float Coef08 = value[2][1] * value[3][2] - value[3][1] * value[2][3];
	const float Coef10 = value[1][1] * value[3][2] - value[3][1] * value[1][2];
	const float Coef11 = value[1][1] * value[2][2] - value[2][1] * value[1][2];

	const float Coef12 = value[2][0] * value[3][3] - value[3][0] * value[2][3];
	const float Coef14 = value[1][0] * value[3][3] - value[3][0] * value[1][3];
	const float Coef15 = value[1][0] * value[2][3] - value[2][0] * value[1][3];

	const float Coef16 = value[2][0] * value[3][2] - value[3][0] * value[2][2];
	const float Coef18 = value[1][0] * value[3][2] - value[3][0] * value[1][2];
	const float Coef19 = value[1][0] * value[2][2] - value[2][0] * value[1][2];

	const float Coef20 = value[2][0] * value[3][1] - value[3][0] * value[2][1];
	const float Coef22 = value[1][0] * value[3][1] - value[3][0] * value[1][1];
	const float Coef23 = value[1][0] * value[2][1] - value[2][0] * value[1][1];

	const SVector4Df Fac0(Coef00, Coef00, Coef02, Coef03);
	const SVector4Df Fac1(Coef04, Coef04, Coef06, Coef07);
	const SVector4Df Fac2(Coef08, Coef08, Coef10, Coef11);
	const SVector4Df Fac3(Coef12, Coef12, Coef14, Coef15);
	const SVector4Df Fac4(Coef16, Coef16, Coef18, Coef19);
	const SVector4Df Fac5(Coef20, Coef20, Coef22, Coef23);

	const SVector4Df Vec0(value[1][0], value[0][0], value[0][0], value[0][0]);
	const SVector4Df Vec1(value[1][1], value[0][1], value[0][1], value[0][1]);
	const SVector4Df Vec2(value[1][2], value[0][2], value[0][2], value[0][2]);
	const SVector4Df Vec3(value[1][3], value[0][3], value[0][3], value[0][3]);

	const SVector4Df Inv0(Vec1 * Fac0 - Vec2 * Fac1 + Vec3 * Fac2);
	const SVector4Df Inv1(Vec0 * Fac0 - Vec2 * Fac3 + Vec3 * Fac4);
	const SVector4Df Inv2(Vec0 * Fac1 - Vec1 * Fac3 + Vec3 * Fac5);
	const SVector4Df Inv3(Vec0 * Fac2 - Vec1 * Fac4 + Vec2 * Fac5);

	const SVector4Df SignA(+1.0f, -1.0f, +1.0f, -1.0f);
	const SVector4Df SignB(-1.0f, +1.0f, -1.0f, +1.0f);

	SMatrix4x4 Inverse(Inv0 * SignA, Inv1 * SignB, Inv2 * SignA, Inv3 * SignB);

	const SVector4Df Row0(Inverse.value[0][0], Inverse.value[1][0], Inverse.value[2][0], Inverse.value[3][0]);

	const SVector4Df Dot0 = SVector4Df(value[0]) * Row0;

	const float Dot1 = (Dot0.x + Dot0.y) + (Dot0.z + Dot0.w);

	const float OneOverDeterminant = 1.0f / Dot1;

	return Inverse * OneOverDeterminant;
}

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
Vector3D Matrix4x4::TransformDirection(const Vector3D& dir) const
{
	// Column0 * dir.x + Column1 * dir.y + Column2 * dir.z
	return Vector3D(
		value[0][0] * dir.x + value[1][0] * dir.y + value[2][0] * dir.z,
		value[0][1] * dir.x + value[1][1] * dir.y + value[2][1] * dir.z,
		value[0][2] * dir.x + value[1][2] * dir.y + value[2][2] * dir.z
	);
	// Notice we ignore m[3][0], m[3][1], m[3][2] (the translation)
}

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
Vector3D Matrix4x4::TransformPoint(const Vector3D& point) const
{
	return Vector3D(
		(value[0][0] * point.x + value[1][0] * point.y + value[2][0] * point.z) + value[3][0],
		(value[0][1] * point.x + value[1][1] * point.y + value[2][1] * point.z) + value[3][1],
		(value[0][2] * point.x + value[1][2] * point.y + value[2][2] * point.z) + value[3][2]
	);
}

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
Vector3D Matrix4x4::TransformPointPerspective(const Vector3D& point) const
{
	// Calculate W component
	float w = value[0][3] * point.x + value[1][3] * point.y + value[2][3] * point.z + value[3][3];

	// Transform the point
	Vector3D result = TransformPoint(point);

	// Perform perspective divide if W is not 1
	if (w != 1.0f && std::abs(w) > 0.000001f)
	{
		result /= w; // Divide by W to get back to Normalized Device Coordinates
	}
	return (result);
}

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
SMatrix4x4::col_type operator*(const SMatrix4x4& mat4, const SMatrix4x4::row_type& rowVector)
{
	const SMatrix4x4::col_type Mov0 = (rowVector[0]);
	const SMatrix4x4::col_type Mov1 = (rowVector[1]);

	const SMatrix4x4::col_type Mul0 = (mat4[0] * Mov0);
	const SMatrix4x4::col_type Mul1 = (mat4[1] * Mov1);

	const SMatrix4x4::col_type Add0 = Mul0 + Mul1;

	const SMatrix4x4::col_type Mov2 = (rowVector[2]);
	const SMatrix4x4::col_type Mov3 = (rowVector[3]);

	const SMatrix4x4::col_type Mul2 = (mat4[2] * Mov2);
	const SMatrix4x4::col_type Mul3 = (mat4[3] * Mov3);

	const SMatrix4x4::col_type Add1 = Mul2 + Mul3;
	const SMatrix4x4::col_type Add2 = Add0 + Add1;

	return (Add2);
}

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
SMatrix4x4::row_type operator*(const SMatrix4x4::col_type& colVector, const SMatrix4x4& mat4)
{
	const float row0 = mat4[0][0] * colVector[0] + mat4[0][1] * colVector[1] + mat4[0][2] * colVector[2] + mat4[0][3] * colVector[3];
	const float row1 = mat4[1][0] * colVector[0] + mat4[1][1] * colVector[1] + mat4[1][2] * colVector[2] + mat4[1][3] * colVector[3];
	const float row2 = mat4[2][0] * colVector[0] + mat4[2][1] * colVector[1] + mat4[2][2] * colVector[2] + mat4[2][3] * colVector[3];
	const float row3 = mat4[3][0] * colVector[0] + mat4[3][1] * colVector[1] + mat4[3][2] * colVector[2] + mat4[3][3] * colVector[3];

	const SMatrix4x4::row_type resultRow(row0, row1, row2, row3);
	return resultRow;
}

/**
 * @brief Performs component-wise scalar addition (Matrix + Scalar).
 *
 * Adds the scalar value (fVal) to every component of the matrix (mat).
 *
 * @param mat The matrix operand.
 * @param fVal The scalar operand.
 * @return A new SMatrix4x4 object representing the sum.
 */
SMatrix4x4 operator+(const SMatrix4x4& mat, float fVal)
{
	SMatrix4x4 Result(mat[0] + fVal, mat[1] + fVal, mat[2] + fVal, mat[3] + fVal);
	return (Result);
}

/**
 * @brief Performs component-wise scalar addition (Scalar + Matrix).
 *
 * Adds the scalar value (fVal) to every component of the matrix (mat).
 *
 * @param fVal The scalar operand.
 * @param mat The matrix operand.
 * @return A new SMatrix4x4 object representing the sum.
 */
SMatrix4x4 operator+(float fVal, const SMatrix4x4& mat)
{
	SMatrix4x4 Result(fVal + mat[0], fVal + mat[1], fVal + mat[2], fVal + mat[3]);
	return (Result);
}

/**
 * @brief Performs component-wise matrix addition (Matrix + Matrix).
 *
 * Adds the corresponding components of the two input matrices (mat1 and mat2).
 *
 * @param mat1 The left-hand matrix operand.
 * @param mat2 The right-hand matrix operand.
 * @return A new SMatrix4x4 object representing the sum.
 */
SMatrix4x4 operator+(const SMatrix4x4& mat1, const SMatrix4x4& mat2)
{
	SMatrix4x4 Result(mat1[0] + mat2[0], mat1[1] + mat2[1], mat1[2] + mat2[2], mat1[3] + mat2[3]);
	return (Result);
}

/**
 * @brief Performs component-wise scalar subtraction (Matrix - Scalar).
 *
 * Subtracts the scalar value (fVal) from every component of the matrix (mat).
 *
 * @param mat The matrix operand.
 * @param fVal The scalar operand.
 * @return A new SMatrix4x4 object representing the difference.
 */
SMatrix4x4 operator-(const SMatrix4x4& mat, float fVal)
{
	SMatrix4x4 Result(mat[0] - fVal, mat[1] - fVal, mat[2] - fVal, mat[3] - fVal);
	return (Result);
}

/**
 * @brief Performs component-wise scalar subtraction (Scalar - Matrix).
 *
 * Subtracts every component of the matrix (mat) from the scalar value (fVal).
 *
 * @param fVal The scalar operand.
 * @param mat The matrix operand.
 * @return A new SMatrix4x4 object representing the difference.
 */
SMatrix4x4 operator-(float fVal, const SMatrix4x4& mat)
{
	SMatrix4x4 Result(fVal - mat[0], fVal - mat[1], fVal - mat[2], fVal - mat[3]);
	return (Result);
}

/**
 * @brief Performs component-wise matrix subtraction (Matrix - Matrix).
 *
 * Subtracts the components of mat2 from the corresponding components of mat1.
 *
 * @param mat1 The left-hand matrix operand.
 * @param mat2 The right-hand matrix operand.
 * @return A new SMatrix4x4 object representing the difference.
 */
SMatrix4x4 operator-(const SMatrix4x4& mat1, const SMatrix4x4& mat2)
{
	SMatrix4x4 Result(mat1[0] - mat2[0], mat1[1] - mat2[1], mat1[2] - mat2[2], mat1[3] - mat2[3]);
	return (Result);
}

/**
 * @brief Performs component-wise scalar multiplication (Matrix * Scalar).
 *
 * Multiplies every component of the matrix (mat) by the scalar value (fVal).
 *
 * @param mat The matrix operand.
 * @param fVal The scalar operand.
 * @return A new SMatrix4x4 object representing the product.
 */
SMatrix4x4 operator*(const SMatrix4x4& mat, float fVal)
{
	SMatrix4x4 Result(mat[0] * fVal, mat[1] * fVal, mat[2] * fVal, mat[3] * fVal);
	return (Result);
}

/**
 * @brief Performs component-wise scalar multiplication (Scalar * Matrix).
 *
 * Multiplies every component of the matrix (mat) by the scalar value (fVal).
 *
 * @param fVal The scalar operand.
 * @param mat The matrix operand.
 * @return A new SMatrix4x4 object representing the product.
 */
SMatrix4x4 operator*(float fVal, const SMatrix4x4& mat)
{
	SMatrix4x4 Result(mat[0] * fVal, mat[1] * fVal, mat[2] * fVal, mat[3] * fVal);
	return (Result);
}

/**
 * @brief Performs component-wise scalar division (Matrix / Scalar).
 *
 * Divides every component of the matrix (mat) by the scalar value (fVal).
 *
 * @param mat The matrix operand.
 * @param fVal The scalar operand.
 * @return A new SMatrix4x4 object representing the quotient.
 */
SMatrix4x4 operator/(const SMatrix4x4& mat, float fVal)
{
	SMatrix4x4 Result(mat[0] / fVal, mat[1] / fVal, mat[2] / fVal, mat[3] / fVal);
	return (Result);
}

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
SMatrix4x4 operator/(float fVal, const SMatrix4x4& mat)
{
	SMatrix4x4 Result(mat[0] / fVal, mat[1] / fVal, mat[2] / fVal, mat[3] / fVal);
	return (Result);
}

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
SMatrix4x4::col_type operator/(const SMatrix4x4& mat, const SMatrix4x4::row_type& rowVec)
{
	// NOTE: This calculates the inverse which is computationally very expensive.
	// For performance, consider using a non-inversion solver (e.g., Gaussian elimination).
	SMatrix4x4 InversedMat = mat;
	InversedMat = InversedMat.Inverse();
	SMatrix4x4::col_type Result = InversedMat * rowVec;
	return (Result);
}

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
SMatrix4x4::row_type operator/(const SMatrix4x4::col_type& colVec, const SMatrix4x4& mat)
{
	SMatrix4x4 InversedMat = mat;
	InversedMat = InversedMat.Inverse();
	SMatrix4x4::row_type Result = colVec * InversedMat;
	return (Result);
}

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
SMatrix4x4 operator/(const SMatrix4x4& mat1, const SMatrix4x4& mat2)
{
	SMatrix4x4 mat1Copy(mat1);
	return (mat1Copy /= mat2);
}

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
SMatrix4x4 operator*(const SMatrix4x4& mat1, const SMatrix4x4& mat2)
{
	const SMatrix4x4::col_type SrcA0 = mat1[0];
	const SMatrix4x4::col_type SrcA1 = mat1[1];
	const SMatrix4x4::col_type SrcA2 = mat1[2];
	const SMatrix4x4::col_type SrcA3 = mat1[3];

	const SMatrix4x4::col_type SrcB0 = mat2[0];
	const SMatrix4x4::col_type SrcB1 = mat2[1];
	const SMatrix4x4::col_type SrcB2 = mat2[2];
	const SMatrix4x4::col_type SrcB3 = mat2[3];

	SMatrix4x4 Result{};
	Result[0] = SrcA0 * SrcB0[0] + SrcA1 * SrcB0[1] + SrcA2 * SrcB0[2] + SrcA3 * SrcB0[3];
	Result[1] = SrcA0 * SrcB1[0] + SrcA1 * SrcB1[1] + SrcA2 * SrcB1[2] + SrcA3 * SrcB1[3];
	Result[2] = SrcA0 * SrcB2[0] + SrcA1 * SrcB2[1] + SrcA2 * SrcB2[2] + SrcA3 * SrcB2[3];
	Result[3] = SrcA0 * SrcB3[0] + SrcA1 * SrcB3[1] + SrcA2 * SrcB3[2] + SrcA3 * SrcB3[3];

	return (Result);
}