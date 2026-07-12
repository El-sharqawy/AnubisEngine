#include "MathPCH.h"
#include "Vector4.h"

/**
 * @brief Constructs an SVector4Df object with all components set to a single value.
 *
 * This is useful for initializing identity vectors or zero vectors.
 *
 * @param fVal The value to set all components (x, y, z, w) to.
 */
SVector4Df::SVector4Df(float fVal)
{
	x = y = z = w = fVal;
}

/**
 * @brief Constructs an SVector4Df from a 2D vector, setting z and w to zero.
 *
 * Useful for mapping 2D texture coordinates or UI positions into a 4D context.
 *
 * @param fX The value for the x component.
 * @param fY The value for the y component.
 */
SVector4Df::SVector4Df(float fX, float fY)
{
	x = fX;
	y = fY;
	z = 0.0f;
	w = 0.0f;
}

/**
 * @brief Constructs an SVector4Df from a 3D vector, setting the w component to zero.
 *
 * This representation is typically used for 3D **vectors** (direction, normal)
 * in homogeneous coordinates, as the w=0 value denotes no translation.
 *
 * @param fX The value for the x component.
 * @param fY The value for the y component.
 * @param fZ The value for the z component.
 */
SVector4Df::SVector4Df(float fX, float fY, float fZ)
{
	x = fX;
	y = fY;
	z = fZ;
	w = 0.0f;
}

/**
 * @brief Constructs an SVector4Df object with specified values for all four components.
 *
 * This representation is commonly used for 3D **points** (position) in
 * homogeneous coordinates, where the w=1 value denotes a position vector.
 *
 * @param fX The value for the x component.
 * @param fY The value for the y component.
 * @param fZ The value for the z component.
 * @param fW The value for the w (homogeneous) component.
 */
SVector4Df::SVector4Df(float fX, float fY, float fZ, float fW)
{
	x = fX;
	y = fY;
	z = fZ;
	w = fW;
}

/**
 * @brief Constructs an SVector4Df from a 2D vector, padding z and w with zero.
 *
 * Useful for mapping 2D vectors or texture coordinates into a 4D context
 * while maintaining the vector's original magnitude.
 *
 * @param vec The source SVector2Df object.
 * @param fZ The value for the z component (default is 0.0f).
 * @param fW The value for the w (homogeneous) component (default is 0.0f).
 */
SVector4Df::SVector4Df(const SVector2Df& vec, float fZ, float fW)
{
	x = vec.x;
	y = vec.y;
	z = fZ;
	w = fW;
}

/**
 * @brief Constructs an SVector4Df from a glm::vec2, padding z and w with zero.
 *
 * @param vec The source glm::vec2 object.
 * @param fZ The value for the z component (default is 0.0f).
 * @param fW The value for the w (homogeneous) component (default is 0.0f).
 */
SVector4Df::SVector4Df(const glm::vec2& vec, float fZ, float fW)
{
	x = vec.x;
	y = vec.y;
	z = fZ;
	w = fW;
}

/**
 * @brief Constructs an SVector4Df from a 3D vector, setting the homogeneous (w) component to zero.
 *
 * This representation is typically used for 3D **direction vectors** or **normals**
 * in homogeneous coordinates, as the w=0 value denotes no translation.
 *
 * @param vec The source SVector3Df object.
 * @param fW The value for the w (homogeneous) component (default is 0.0f).
 */
SVector4Df::SVector4Df(const SVector3Df& vec, float fW)
{
	x = vec.x;
	y = vec.y;
	z = vec.z;
	w = fW;
}

/**
 * @brief Constructs an SVector4Df from a glm::vec3, setting the homogeneous (w) component to zero.
 *
 * This allows treating the glm::vec3 as a 3D direction vector within the 4D context.
 *
 * @param vec The source glm::vec3 object.
 * @param fW The value for the w (homogeneous) component (default is 0.0f).
 */
SVector4Df::SVector4Df(const glm::vec3& vec, float fW)
{
	x = vec.x;
	y = vec.y;
	z = vec.z;
	w = fW;
}

/**
 * @brief Copy constructor.
 *
 * Constructs an SVector4Df by copying the component values from another SVector4Df.
 *
 * @param vec The source SVector4Df object.
 */
SVector4Df::SVector4Df(const SVector4Df& vec)
{
	x = vec.x;
	y = vec.y;
	z = vec.z;
	w = vec.w;
}

/**
 * @brief Constructs an SVector4Df by copying components from a glm::vec4.
 *
 * This is the direct conversion between the two 4D vector types.
 *
 * @param vec The source glm::vec4 object.
 */
SVector4Df::SVector4Df(const glm::vec4& vec)
{
	x = vec.x;
	y = vec.y;
	z = vec.z;
	w = vec.w;
}

/**
 * @brief Constructs an SVector4Df from a pointer to an array of four float values.
 *
 * Components are read in order: pVec[0] -> x, pVec[1] -> y, etc.
 * Provides a null-check to safely initialize to the zero vector if the pointer is invalid.
 *
 * @param pVec A pointer to an array of at least four float values.
 */
SVector4Df::SVector4Df(const float* pVec)
{
	if (!pVec)
	{
		x = y = z = w = 0.0f;
		return;
	}

	x = pVec[0];
	y = pVec[1];
	z = pVec[2];
	w = pVec[3];
}

/**
 * @brief Adds two SVector4Df objects component-wise.
 *
 * Implements vector addition: result.x = this->x + vec.x, etc.
 *
 * @param vec The SVector4Df object to add.
 * @return The resulting SVector4Df object (a new vector).
 */
SVector4Df SVector4Df::operator+(const SVector4Df& vec) const
{
	return SVector4Df(x + vec.x, y + vec.y, z + vec.z, w + vec.w);
}

/**
 * @brief Subtracts one SVector4Df object from another component-wise.
 *
 * Implements vector subtraction: result.x = this->x - vec.x, etc.
 *
 * @param vec The SVector4Df object to subtract.
 * @return The resulting SVector4Df object.
 */
SVector4Df SVector4Df::operator-(const SVector4Df& vec) const
{
	return SVector4Df(x - vec.x, y - vec.y, z - vec.z, w - vec.w);
}

/**
 * @brief Divides one SVector4Df object by another component-wise.
 *
 * Implements component-wise division: result.x = this->x / vec.x, etc.
 * Caution: Does not check for division by zero.
 *
 * @param vec The SVector4Df object to divide by.
 * @return The resulting SVector4Df object.
 */
SVector4Df SVector4Df::operator*(const SVector4Df& vec) const
{
	return SVector4Df(x * vec.x, y * vec.y, z * vec.z, w * vec.w);
}

/**
 * @brief Divides one SVector4Df object by another component-wise.
 *
 * Implements component-wise division: result.x = this->x / vec.x, etc.
 * Caution: Does not check for division by zero.
 *
 * @param vec The SVector4Df object to divide by.
 * @return The resulting SVector4Df object.
 */
SVector4Df SVector4Df::operator/(const SVector4Df& vec) const
{
	// Check and calculate X component
	float resX = (vec.x != 0.0f) ? (this->x / vec.x) : this->x;

	// Check and calculate Y component
	float resY = (vec.y != 0.0f) ? (this->y / vec.y) : this->y;

	// Check and calculate Z component
	float resZ = (vec.z != 0.0f) ? (this->z / vec.z) : this->z;

	// Check and calculate W component
	float resW = (vec.w != 0.0f) ? (this->w / vec.w) : this->w;

	// Return the new vector with calculated components.
	return SVector4Df(resX, resY, resZ, resW);

}

/**
 * @brief Adds a scalar value to every component of the SVector4Df object.
 *
 * Implements vector-scalar addition: result.x = this->x + fVal, etc.
 * This operation effectively translates the point/vector uniformly along all axes.
 *
 * @param fVal The scalar floating-point value to add.
 * @return The resulting SVector4Df object (a new vector).
 */
SVector4Df SVector4Df::operator+(float fVal) const
{
	return SVector4Df(x + fVal, y + fVal, z + fVal, w + fVal);
}

/**
 * @brief Subtracts a scalar value from every component of the SVector4Df object.
 *
 * Implements vector-scalar subtraction: result.x = this->x - fVal, etc.
 *
 * @param fVal The scalar floating-point value to subtract.
 * @return The resulting SVector4Df object.
 */
SVector4Df SVector4Df::operator-(float fVal) const
{
	return SVector4Df(x - fVal, y - fVal, z - fVal, w - fVal);
}

/**
 * @brief Multiplies every component of the SVector4Df object by a scalar value (scaling).
 *
 * Implements vector-scalar multiplication: result.x = this->x * fVal, etc.
 * This operation is used to uniformly scale the vector's magnitude.
 *
 * @param fVal The scalar floating-point value to multiply by.
 * @return The resulting SVector4Df object.
 */
SVector4Df SVector4Df::operator*(float fVal) const
{
	return SVector4Df(x * fVal, y * fVal, z * fVal, w * fVal);
}

/**
 * @brief Vector division by a scalar (non-modifying).
 *
 * Calculates a new vector where each component is divided by the scalar fVal.
 * Performs a zero-check: if fVal is 0, the original vector is returned unchanged
 * to prevent division by zero errors.
 *
 * @param fVal The scalar float value to divide by (passed by value for efficiency).
 * @return SVector4Df A new vector instance resulting from the division.
 */
SVector4Df SVector4Df::operator/(float fVal) const
{
	// 1. Check for division by zero.
	if (fVal != 0.0f)
	{
		// 2. Return the new vector initialized with the divided components.
		return SVector4Df(x / fVal, y / fVal, z / fVal, w / fVal);
	}

	// 3. If division by zero, return a copy of the original vector (safe fallback).
	return *this;
}

/**
 * @brief Compound assignment operator for vector addition (in-place modification).
 *
 * Adds the components of the input vector 'vec' to the corresponding components
 * of this vector (*this), modifying this object directly.
 *
 * @param vec The constant reference to the SVector4Df object to add.
 * @return SVector4Df& A reference to the modified SVector4Df object (*this).
 */
SVector4Df& SVector4Df::operator+=(const SVector4Df& vec)
{
	x += vec.x;
	y += vec.y;
	z += vec.z;
	w += vec.w;
	return (*this);
}

/**
 * @brief Compound assignment operator for vector subtraction (in-place modification).
 *
 * Subtracts the components of the input vector 'vec' from the corresponding components
 * of this vector (*this), modifying this object directly.
 *
 * @param vec The constant reference to the SVector4Df object to subtract.
 * @return SVector4Df& A reference to the modified SVector4Df object (*this).
 */
SVector4Df& SVector4Df::operator-=(const SVector4Df & vec)
{
	x -= vec.x;
	y -= vec.y;
	z -= vec.z;
	w -= vec.w;
	return (*this);
}

/**
 * @brief Compound assignment operator for component-wise multiplication (in-place modification).
 *
 * Multiplies the components of this vector (*this) by the corresponding components
 * of the input vector 'vec' (Hadamard product), modifying this object directly.
 *
 * @param vec The constant reference to the SVector4Df object to multiply by.
 * @return SVector4Df& A reference to this modified SVector4Df object.
 */
SVector4Df& SVector4Df::operator*=(const SVector4Df& vec)
{
	x *= vec.x;
	y *= vec.y;
	z *= vec.z;
	w *= vec.w;
	return (*this);
}

/**
 * @brief Compound assignment operator for component-wise division (in-place modification).
 *
 * Divides the components of this vector (*this) by the corresponding components
 * of the input vector 'vec' (Hadamard division), modifying this object directly.
 * Includes a safety check for division by zero on each component.
 *
 * @param vec The constant reference to the SVector4Df object to divide by.
 * @return SVector4Df& A reference to this modified SVector4Df object.
 */
SVector4Df& SVector4Df::operator/=(const SVector4Df& vec)
{
	if (vec.x != 0.0f)
	{
		x /= vec.x;
	}
	if (vec.y != 0.0f)
	{
		y /= vec.y;
	}
	if (vec.z != 0.0f)
	{
		z /= vec.z;
	}
	if (vec.w != 0.0f)
	{
		w /= vec.w;
	}
	return (*this);
}

/**
 * @brief Compound assignment operator for scalar addition (in-place modification).
 *
 * Adds the scalar float value 'fVal' to all components (x, y, z, w) of this vector
 * (*this), modifying the vector directly.
 *
 * @param fVal The scalar float value to add (passed by value for efficiency).
 * @return SVector4Df& A reference to the modified SVector4Df object (*this).
 */
SVector4Df& SVector4Df::operator+=(float fVal)
{
	x += fVal;
	y += fVal;
	z += fVal;
	w += fVal;
	return (*this);
}

/**
 * @brief Compound assignment operator for scalar subtraction (in-place modification).
 *
 * Subtracts the scalar float value 'fVal' from all components (x, y, z, w) of this vector
 * (*this), modifying the vector directly.
 *
 * @param fVal The scalar float value to subtract (passed by value for efficiency).
 * @return SVector4Df& A reference to the modified SVector4Df object (*this).
 */
SVector4Df& SVector4Df::operator-=(float fVal)
{
	x -= fVal;
	y -= fVal;
	z -= fVal;
	w -= fVal;
	return (*this);
}

/**
 * @brief Compound assignment operator for scalar multiplication (in-place modification).
 *
 * Multiplies all components (x, y, z) of this vector (*this) by the scalar float value
 * 'fVal', modifying the vector directly (vector scaling).
 *
 * @param fVal The scalar float value to multiply by (passed by value for efficiency).
 * @return SVector4Df& A reference to the modified SVector4Df object (*this).
 */
SVector4Df& SVector4Df::operator*=(float fVal)
{
	x *= fVal;
	y *= fVal;
	z *= fVal;
	w *= fVal;
	return (*this);
}

/**
 * @brief Compound assignment operator for scalar division (in-place modification).
 *
 * Divides all components (x, y, z) of this vector (*this) by the scalar float value
 * 'fVal', modifying the vector directly. Includes a safety check to prevent
 * division by zero.
 *
 * @param fVal The scalar float value to divide by (passed by value for efficiency).
 * @return SVector4Df& A reference to the modified SVector4Df object (*this).
 */
SVector4Df& SVector4Df::operator/=(float fVal)
{
	if (fVal != 0.0f)
	{
		x /= fVal;
		y /= fVal;
		z /= fVal;
		w /= fVal;
	}
	return (*this);
}

/**
 * @brief Component-wise equality comparison operator.
 *
 * Compares this vector (*this) with the input vector 'vec'.
 * Returns true if and only if all corresponding components (x, y, z, and w) are equal.
 * This operator is marked 'const' as it does not modify the vector.
 *
 * @param vec The constant reference to the SVector4Df vector to compare against.
 * @return bool True if the vectors are equal component-wise, false otherwise.
 */
bool SVector4Df::operator == (const SVector4Df& vec) const
{
	bool bEqualX = std::fabs(x - vec.x) <= VECTOR4_EPS;
	bool bEqualY = std::fabs(y - vec.y) <= VECTOR4_EPS;
	bool bEqualZ = std::fabs(z - vec.z) <= VECTOR4_EPS;
	bool bEqualW = std::fabs(w - vec.w) <= VECTOR4_EPS;

	return (bEqualX && bEqualY && bEqualZ && bEqualW);
}

/**
 * @brief Component-wise inequality comparison operator.
 *
 * Compares this vector (*this) with the input vector 'vec'.
 * Returns true if any corresponding component (x, y, z, or w) is not equal.
 * This operator is marked 'const' as it does not modify the vector.
 *
 * @param vec The constant reference to the SVector4Df vector to compare against.
 * @return bool True if the vectors are not equal component-wise, false otherwise.
 */
bool SVector4Df::operator != (const SVector4Df& vec) const
{
	return !(*this == vec);
}

/**
 * @brief Component-wise inequality comparison operator.
 *
 * Compares this vector (*this) with the input vector 'vec'.
 * Returns true if any corresponding component (x, y, z, or w) is not equal.
 * This operator is marked 'const' as it does not modify the vector.
 *
 * @param vec The constant reference to the glm::vec4 vector to compare against.
 * @return bool True if the vectors are not equal component-wise, false otherwise.
 */
bool SVector4Df::operator == (const glm::vec4& vec) const
{
	bool bEqualX = std::fabs(x - vec.x) <= VECTOR4_EPS;
	bool bEqualY = std::fabs(y - vec.y) <= VECTOR4_EPS;
	bool bEqualZ = std::fabs(z - vec.z) <= VECTOR4_EPS;
	bool bEqualW = std::fabs(w - vec.w) <= VECTOR4_EPS;

	return (bEqualX && bEqualY && bEqualZ && bEqualW);
}

/**
 * @brief Component-wise inequality comparison operator.
 *
 * Compares this vector (*this) with the input vector 'vec'.
 * Returns true if any corresponding component (x, y, z, or w) is not equal.
 * This operator is marked 'const' as it does not modify the vector.
 *
 * @param vec The constant reference to the glm::vec4 vector to compare against.
 * @return bool True if the vectors are not equal component-wise, false otherwise.
 */
bool SVector4Df::operator != (const glm::vec4& vec) const
{
	return !(*this == vec);
}

/**
 * @brief Subscript operator (non-const version).
 *
 * Provides read/write access to the vector components using an index (0=x, 1=y, 2=z, 3=w).
 * Returns a reference to the component, allowing it to be modified.
 * If the index is out of bounds, it defaults to returning a reference to the x component (index 0).
 *
 * @param index The zero-based index of the component to access (0, 1, 2, or 3).
 * @return float& A reference to the specified component (x, y, z, or w).
 */
float& SVector4Df::operator[](size_t index)
{
	switch (index)
	{
	case 0:
		return x;
	case 1:
		return y;
	case 2:
		return z;
	case 3:
		return w;
	default:
		return x; // Fallback for invalid index
	}
}

/**
 * @brief Subscript operator (const version).
 *
 * Provides read-only access to the vector components using an index  (0=x, 1=y, 2=z, 3=w).
 * Returns the value of the component.
 * If the index is out of bounds, it defaults to returning the value of the x component (index 0).
 * This operator is marked 'const' as it does not modify the vector.
 *
 * @param index The zero-based index of the component to access (0, 1, 2, or 3).
 * @return const float The value of the specified component (x, y, z, or w).
 */
const float SVector4Df::operator[](size_t index) const
{
	switch (index)
	{
	case 0:
		return x;
	case 1:
		return y;
	case 2:
		return z;
	case 3:
		return w;
	default:
		return x; // Fallback for invalid index
	}
}

/**
 * @brief Prefix increment operator (e.g., ++v).
 *
 * Increments both the x, y and z components of the vector before the result of
 * the operation is used. This is generally more efficient than the postfix form.
 *
 * @return SVector4Df& A reference to the modified vector instance (*this).
 */
SVector4Df& SVector4Df::operator++()
{
	++this->x;
	++this->y;
	++this->z;
	++this->w;
	return (*this);
}

/**
 * @brief Postfix increment operator (e.g., v++).
 *
 * Increments both the x, y and z components of the vector after the original
 * value of the vector is retrieved and used.
 * This function returns a copy of the vector's state *before* modification,
 * ensuring standard C++ behavior for expressions like v2 = v1++.
 *
 * @param int The dummy parameter used to distinguish this as the postfix operator.
 * @return SVector4Df A copy of the vector instance before the increment.
 */
SVector4Df SVector4Df::operator++(int32_t)
{
	SVector4Df temp = *this; // 1. Save the original state (required for postfix).
	++this->x;              // 2. Increment the actual object.
	++this->y;
	++this->z;
	++this->w;
	return temp;            // 3. Return the saved original state.
}

/**
 * @brief Prefix decrement operator (e.g., --v).
 *
 * Decrements both the x, y and z components of the vector before the result of
 * the operation is used. This is generally more efficient than the postfix form.
 *
 * @return SVector4Df& A reference to the modified vector instance (*this).
 */
SVector4Df& SVector4Df::operator--()
{
	--this->x;
	--this->y;
	--this->z;
	--this->w;
	return (*this);
}

/**
 * @brief Postfix decrement operator (e.g., v--).
 *
 * Decrements both the x, y and z components of the vector after the original
 * value of the vector is retrieved and used.
 * This function returns a copy of the vector's state *before* modification,
 * ensuring standard C++ behavior for expressions like v2 = v1--.
 *
 * @param int The dummy parameter used to distinguish this as the postfix operator.
 * @return SVector4Df A copy of the vector instance before the decrement.
 */
SVector4Df SVector4Df::operator--(int32_t)
{
	SVector4Df temp = *this; // 1. Save the original state (required for postfix).
	--this->x;              // 2. Decrement the actual object.
	--this->y;
	--this->z;
	--this->w;
	return temp;            // 3. Return the saved original state.
}

/**
 * @brief Unary negation operator (non-modifying).
 *
 * Calculates a new vector where every component is negated (multiplied by -1).
 *
 * @return SVector4Df A new vector instance with all components negated.
 */
SVector4Df SVector4Df::operator-() const
{
	return SVector4Df(-x, -y, -z, -w);
}

/**
 * @brief Unary plus operator (non-modifying).
 *
 * Returns a copy of the vector unchanged. This operator is primarily
 * included for symmetry with the unary minus operator.
 *
 * @return SVector4Df A copy of the SVector4Df instance (*this).
 */
SVector4Df SVector4Df::operator+() const
{
	return (*this);
}

/**
 * @brief Assignment operator overload for glm::vec4.
 *
 * Allows direct assignment from a GLM vector to this SVector4Df instance,
 * facilitating smooth interoperability between the two math libraries.
 * e.g., SVector4Df myVec; myVec = glm::vec4(5.0f, 10.0f, 5.0f, 7.0f);
 *
 * @param vec4 The constant reference to the glm::vec4 source vector.
 * @return SVector4Df& A reference to the modified SVector4Df instance (*this).
 */
SVector4Df& SVector4Df::operator=(const glm::vec4& vec4)
{
	this->x = vec4.x;
	this->y = vec4.y;
	this->z = vec4.z;
	this->w = vec4.w;
	return (*this);
}

/**
 * @brief Implicit conversion operator to glm::vec4.
 *
 * Allows an SVector4Df instance to be implicitly converted to a glm::vec4 type,
 * enabling direct assignment to GLM variables: glm::vec4 myGlmVec = myVec4;
 * It is defined as 'const' because it does not modify the SVector4Df object.
 *
 * @return glm::vec4 A glm::vec4 instance with the same x, y, z and w values.
 */
SVector4Df::operator glm::vec4() const
{
	return glm::vec4(this->x, this->y, this->z, this->w);
}

/**
 * @brief Const float* conversion operator.
 *
 * Allows the SVector4Df object to be implicitly converted to a constant
 * pointer to its underlying float array (starting at the 'x' component).
 * This is useful for passing vector data directly to C-style functions,
 * such as OpenGL API calls.
 * This operator is marked 'const' as it provides read-only access.
 *
 * @return const float* A constant pointer to the first component (x).
 */
SVector4Df::operator const float* () const
{
	return (&(x));
}

/**
 * @brief float* conversion operator.
 *
 * Allows the SVector3Df object to be implicitly converted to a pointer
 * to its underlying float array (starting at the 'x' component).
 * This is useful for passing vector data directly to C-style functions,
 * such as OpenGL API calls.
 *
 * @return float* A pointer to the first component (x).
 */
SVector4Df::operator float* ()
{
	return (&(x));
}

/**
 * @brief Calculates the squared length of the vector.
 *
 * Computes the squared Euclidean length of the vector, defined as
 * the sum of the squares of its components: x^2 + y^2 + z^2 + w^2.
 * This operation is non-modifying and marked 'const'.
 *
 * @return float The squared length of the vector.
 */
float SVector4Df::lengthSquared() const
{
	return (x * x + y * y + z * z + w * w);
}

/**
 * @brief Calculates the vector magnitude (length).
 *
 * Computes the Euclidean length (or magnitude) of the vector, defined as
 * the square root of the sum of the squares of its components: sqrt{x^2 + y^2 + z^2 + w^2}.
 * This operation is non-modifying and marked 'const'.
 *
 * @return float The magnitude (length) of the vector.
 */
float SVector4Df::length() const
{
	return (std::sqrt(x * x + y * y + z * z + w * w)); // Calculates the length (magnitude) of the vector
}

/**
 * @brief Normalizes the vector (in-place modification).
 *
 * Modifies the vector such that its length becomes 1.0, preserving its direction.
 * If the current vector length is zero, the vector is left unchanged for safety.
 *
 * @return SVector4Df& A reference to the modified (normalized) SVector4Df object (*this).
 */
SVector4Df& SVector4Df::normalize()
{
	float fLen = length();
	if (fLen != 0.0f)
	{
		x /= fLen;
		y /= fLen;
		z /= fLen;
		w /= fLen;
	}
	return (*this);

}

/**
 * @brief Calculates the Dot Product.
 *
 * Computes the scalar dot product of this vector (*this) and another vector 'vec'.
 * The result is a scalar value equal to x_1x_2 + y_1y_2 + z_1z_2 + w_1w_2.
 * This operation is non-modifying and marked 'const'.
 *
 * @param vec The constant reference to the other SVector4Df vector.
 * @return float The resulting scalar dot product.
 */
float SVector4Df::dot(const SVector4Df& vec) const
{
	float fRet = x * vec.x + y * vec.y + z * vec.z + w * vec.w;
	return (fRet);
}

/**
 * @brief Calculates the distance to another vector.
 *
 * Computes the Euclidean distance between the point represented by this vector
 * (*this) and the point represented by the input vector 'vec'.
 * The distance is calculated as the length of the difference vector:
 * sqrt{(x_1-x_2)^2 + (y_1-y_2)^2 + (z_1-z_2)^2 + (w_1-w_2)^2}.
 * This operation is non-modifying and marked 'const'.
 *
 * @param vec The constant reference to the other SVector4Df vector (the target point).
 * @return float The scalar distance between the two points.
 */
float SVector4Df::distance(const SVector4Df& vec) const
{
	float fDeltaX = x - vec.x;
	float fDeltaY = y - vec.y;
	float fDeltaZ = z - vec.z;
	float fDeltaW = w - vec.w;

	float fDistance = std::sqrt(fDeltaX * fDeltaX + fDeltaY * fDeltaY + fDeltaZ * fDeltaZ + fDeltaW * fDeltaW);
	return (fDistance);
}

/**
 * @brief Converts the SVector4Df object to a glm::vec4 object (non-modifying).
 *
 * Creates and returns an equivalent `glm::vec4` object, facilitating
 * interoperability with the GLM math library.
 *
 * @return glm::vec4 The equivalent glm::vec4 object.
 */
glm::vec4 SVector4Df::ToGLM() const
{
	return (glm::vec4(x, y, z, w));
}

/**
 * @brief Commutative non-member vector-scalar addition operator.
 *
 * Allows for commutative addition where the scalar is the left operand (e.g., fVal + vec).
 * Implemented by calling the standard (vec + fVal) operator.
 *
 * @param fVal The scalar float value to add.
 * @param vec The constant reference to the SVector4Df object.
 * @return SVector4Df The resulting SVector4Df object from the operation.
 */
SVector4Df operator+(float fVal, const SVector4Df& vec)
{
	// Solution: Directly access components to perform the addition, 
	// avoiding a potentially complex or ambiguous call to the member operator.
	return SVector4Df(vec.x + fVal, vec.y + fVal, vec.z + fVal, vec.w + fVal);
}

/**
 * @brief Non-member vector-scalar reverse subtraction operator.
 *
 * Allows for reverse subtraction where the vector is subtracted from the scalar (e.g., fVal - vec).
 * This operation negates the result of the standard subtraction: fVal - vec = -(vec - fVal).
 *
 * @param fVal The scalar float value (minuend).
 * @param vec The constant reference to the SVector4Df object (subtrahend).
 * @return SVector4Df The resulting SVector4Df object from the operation.
 */
SVector4Df operator-(float fVal, const SVector4Df& vec)
{
	SVector4Df result(fVal - vec.x, fVal - vec.y, fVal - vec.z, fVal - vec.w);
	return (result);
}

/**
 * @brief Commutative non-member vector-scalar multiplication operator.
 *
 * Allows for commutative multiplication where the scalar is the left operand (e.g., fVal * vec).
 * Implemented by calling the standard (vec * fVal) operator.
 *
 * @param fVal The scalar float value to multiply by.
 * @param vec The constant reference to the SVector4Df object.
 * @return SVector4Df The resulting SVector4Df object from the operation.
 */
SVector4Df operator*(float fVal, const SVector4Df& vec)
{
	return SVector4Df(vec.x * fVal, vec.y * fVal, vec.z * fVal, vec.w * fVal);
}

/**
 * @brief Constructs an SVector4Di object with all components set to a single value.
 *
 * This is useful for initializing identity vectors or zero vectors.
 *
 * @param iVal The value to set all components (x, y, z, w) to.
 */
SVector4Di::SVector4Di(int32_t iVal)
{
	x = y = z = w = iVal;
}

/**
 * @brief Constructs an SVector4Di from a 2D vector, setting z and w to zero.
 *
 * Useful for mapping 2D texture coordinates or UI positions into a 4D context.
 *
 * @param fX The value for the x component.
 * @param fY The value for the y component.
 */
SVector4Di::SVector4Di(int32_t iX, int32_t iY)
{
	x = iX;
	y = iY;
	z = 0;
	w = 0;
}

/**
 * @brief Constructs an SVector4Di from a 3D vector, setting the w component to zero.
 *
 * This representation is typically used for 3D **vectors** (direction, normal)
 * in homogeneous coordinates, as the w=0 value denotes no translation.
 *
 * @param iX The value for the x component.
 * @param iY The value for the y component.
 * @param iZ The value for the z component.
 */
SVector4Di::SVector4Di(int32_t iX, int32_t iY, int32_t iZ)
{
	x = iX;
	y = iY;
	z = iZ;
	w = 0;
}

/**
 * @brief Constructs an SVector4Di object with specified values for all four components.
 *
 * This representation is commonly used for 3D **points** (position) in
 * homogeneous coordinates, where the w=1 value denotes a position vector.
 *
 * @param iX The value for the x component.
 * @param iY The value for the y component.
 * @param iZ The value for the z component.
 * @param iW The value for the w (homogeneous) component.
 */
SVector4Di::SVector4Di(int32_t iX, int32_t iY, int32_t iZ, int32_t iW)
{
	x = iX;
	y = iY;
	z = iZ;
	w = iW;
}

/**
 * @brief Constructs an SVector4Df from a 2D vector, padding z and w with zero.
 *
 * Useful for mapping 2D vectors or texture coordinates into a 4D context
 * while maintaining the vector's original magnitude.
 *
 * @param vec The source SVector2Df object.
 * @param iZ The value for the z component (default is 0.0f).
 * @param iW The value for the w (homogeneous) component (default is 0.0f).
 */
SVector4Di::SVector4Di(const SVector2Di& vec, int32_t iZ, int32_t iW)
{
	x = vec.x;
	y = vec.y;
	z = iZ;
	w = iW;
}
