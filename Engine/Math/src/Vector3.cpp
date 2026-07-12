#include "MathPCH.h"
#include "Vector3.h"

/**
 * Constructs an SVector3Df object with all components set to the same value.
 *
 * @param fVal The value to set all components to.
 */
SVector3Df::SVector3Df(float fVal)
{
	x = y = z = fVal;
}

/**
 * Constructs an SVector3Df object with specified values for each component.
 *
 * @param fX The value for the x component.
 * @param fY The value for the y component.
 */
SVector3Df::SVector3Df(const SVector2Df& vec2, float fZ)
{
	x = vec2.x;
	y = vec2.y;
	z = fZ;
}

/**
 * Constructs an SVector3Df object with specified values for each component.
 *
 * @param fX The value for the x component.
 * @param fY The value for the y component.
 * @param fZ The value for the z component.  
 */
SVector3Df::SVector3Df(float fX, float fY, float fZ)
{
	x = fX;
	y = fY;
	z = fZ;
}

/**
 * Constructs an SVector3Df object from a SVector3Df object.
 *
 * @param vec The SVector3Df object to construct from.
 */
SVector3Df::SVector3Df(const SVector3Df& vec)
{
	x = vec.x;
	y = vec.y;
	z = vec.z;
}

/**
 * Constructs an SVector3Df object from a glm::vec3 object.
 *
 * @param vec The glm::vec3 object to construct from.
 */
SVector3Df::SVector3Df(const glm::vec3& vec)
{
	x = vec.x;
	y = vec.y;
	z = vec.z;
}

/**
 * Constructs an SVector3Df object from a pointer to a float array.
 *
 * @param pVec The pointer to the float array.
 */
SVector3Df::SVector3Df(const float* pVec)
{
	if (!pVec)
	{
		x = y = z = 0.0f;
		return;
	}

	x = pVec[0];
	y = pVec[1];
	z = pVec[2];
}

/**
 * @brief Vector addition with another vector (non-modifying).
 *
 * Calculates a new vector by adding the components of the input vector 'vec'
 * to the components of this vector (*this).
 *
 * @param vec The constant reference to the vector to add.
 * @return SVector3Df A new vector instance resulting from the component-wise addition.
 */
SVector3Df SVector3Df::operator+(const SVector3Df& vec) const
{
	return SVector3Df(x + vec.x, y + vec.y, z + vec.z);
}

/**
 * @brief Vector subtraction with another vector (non-modifying).
 *
 * Calculates a new vector by subtracting the components of the input vector 'vec'
 * from the components of this vector (*this).
 *
 * @param vec The constant reference to the vector to subtract.
 * @return SVector3Df A new vector instance resulting from the component-wise subtraction.
 */
SVector3Df SVector3Df::operator-(const SVector3Df& vec) const
{
	return SVector3Df(x - vec.x, y - vec.y, z - vec.z);
}

/**
 * @brief Component-wise (Hadamard) vector multiplication (non-modifying).
 *
 * Calculates a new vector by multiplying each component of this vector by the
 * corresponding component of the input vector 'vec'. This is NOT the dot product or cross product.
 *
 * @param vec The constant reference to the vector to multiply by.
 * @return SVector3Df A new vector instance with component-wise results.
 */
SVector3Df SVector3Df::operator*(const SVector3Df& vec) const
{
	return SVector3Df(x * vec.x, y * vec.y, z * vec.z);
}

/**
 * @brief Component-wise (Hadamard) vector division (non-modifying).
 *
 * Divides each component of this vector by the corresponding component of the
 * input vector 'vec', resulting in a new vector.
 *
 * For safety, if any component of 'vec' is 0, the corresponding component
 * from the original vector (*this) is retained in the result.
 *
 * @param vec The constant reference to the divisor vector.
 * @return SVector3Df A new vector instance with component-wise results.
 */
SVector3Df SVector3Df::operator/(const SVector3Df& vec) const
{
	// Check and calculate X component
	float resX = (vec.x != 0.0f) ? (this->x / vec.x) : this->x;

	// Check and calculate Y component
	float resY = (vec.y != 0.0f) ? (this->y / vec.y) : this->y;

	// Check and calculate Z component
	float resZ = (vec.z != 0.0f) ? (this->z / vec.z) : this->z;

	// Return the new vector with calculated components.
	return SVector3Df(resX, resY, resZ);
}

/**
 * @brief Vector addition with a scalar value (non-modifying).
 *
 * Calculates a new vector by adding the scalar value 'fVal' to all components
 * of this vector (*this).
 *
 * @param fVal The scalar float value to add (passed by value for efficiency).
 * @return SVector3Df A new vector instance resulting from the scalar addition.
 */
SVector3Df SVector3Df::operator+(float fVal) const
{
	return SVector3Df(x + fVal, y + fVal, z + fVal);
}

/**
 * @brief Vector subtraction with a scalar value (non-modifying).
 *
 * Calculates a new vector by subtracting the scalar value 'fVal' from all
 * components of this vector (*this).
 *
 * @param fVal The scalar float value to subtract (passed by value for efficiency).
 * @return SVector3Df A new vector instance resulting from the scalar subtraction.
 */
SVector3Df SVector3Df::operator-(float fVal) const
{
	return SVector3Df(x - fVal, y - fVal, z - fVal);
}

/**
 * @brief Vector multiplication by a scalar value (non-modifying).
 *
 * Calculates a new vector by multiplying all components of this vector by the
 * scalar value 'fVal'. This is standard vector scaling.
 *
 * @param fVal The scalar float value to multiply by (passed by value for efficiency).
 * @return SVector3Df A new vector instance resulting from the scalar multiplication.
 */
SVector3Df SVector3Df::operator*(float fVal) const
{
	return SVector3Df(x * fVal, y * fVal, z * fVal);
}

/**
 * @brief Vector division by a scalar (non-modifying).
 *
 * Calculates a new vector where each component is divided by the scalar fVal.
 * Performs a zero-check: if fVal is 0, the original vector is returned unchanged
 * to prevent division by zero errors.
 *
 * @param fVal The scalar float value to divide by (passed by value for efficiency).
 * @return SVector3Df A new vector instance resulting from the division.
 */
SVector3Df SVector3Df::operator/(float fVal) const
{
	// 1. Check for division by zero.
	if (fVal != 0.0f)
	{
		// 2. Return the new vector initialized with the divided components.
		return SVector3Df(x / fVal, y / fVal, z / fVal);
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
 * @param vec The constant reference to the SVector3Df object to add.
 * @return SVector3Df& A reference to the modified SVector3Df object (*this).
 */
SVector3Df& SVector3Df::operator+=(const SVector3Df& vec)
{
	x += vec.x;
	y += vec.y;
	z += vec.z;
	return (*this);
}

/**
 * @brief Compound assignment operator for vector subtraction (in-place modification).
 *
 * Subtracts the components of the input vector 'vec' from the corresponding components
 * of this vector (*this), modifying this object directly.
 *
 * @param vec The constant reference to the SVector3Df object to subtract.
 * @return SVector3Df& A reference to the modified SVector3Df object (*this).
 */
SVector3Df& SVector3Df::operator-=(const SVector3Df& vec)
{
	x -= vec.x;
	y -= vec.y;
	z -= vec.z;
	return (*this);
}

/**
 * @brief Compound assignment operator for component-wise multiplication (in-place modification).
 *
 * Multiplies the components of this vector (*this) by the corresponding components
 * of the input vector 'vec' (Hadamard product), modifying this object directly.
 *
 * @param vec The constant reference to the SVector3Df object to multiply by.
 * @return SVector3Df& A reference to this modified SVector3Df object.
 */
SVector3Df& SVector3Df::operator*=(const SVector3Df& vec)
{
	x *= vec.x;
	y *= vec.y;
	z *= vec.z;
	return (*this);
}

/**
 * @brief Compound assignment operator for component-wise division (in-place modification).
 *
 * Divides the components of this vector (*this) by the corresponding components
 * of the input vector 'vec' (Hadamard division), modifying this object directly.
 * Includes a safety check for division by zero on each component.
 *
 * @param vec The constant reference to the SVector3Df object to divide by.
 * @return SVector3Df& A reference to this modified SVector3Df object.
 */
SVector3Df& SVector3Df::operator/=(const SVector3Df& vec)
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
	return (*this);
}

/**
 * @brief Compound assignment operator for scalar addition (in-place modification).
 *
 * Adds the scalar float value 'fVal' to all components (x, y, z) of this vector
 * (*this), modifying the vector directly.
 *
 * @param fVal The scalar float value to add (passed by value for efficiency).
 * @return SVector3Df& A reference to the modified SVector3Df object (*this).
 */
SVector3Df& SVector3Df::operator+=(float fVal)
{
	x += fVal;
	y += fVal;
	z += fVal;
	return (*this);
}

/**
 * @brief Compound assignment operator for scalar subtraction (in-place modification).
 *
 * Subtracts the scalar float value 'fVal' from all components (x, y, z) of this vector
 * (*this), modifying the vector directly.
 *
 * @param fVal The scalar float value to subtract (passed by value for efficiency).
 * @return SVector3Df& A reference to the modified SVector3Df object (*this).
 */
SVector3Df& SVector3Df::operator-=(float fVal)
{
	x -= fVal;
	y -= fVal;
	z -= fVal;
	return (*this);
}

/**
 * @brief Compound assignment operator for scalar multiplication (in-place modification).
 *
 * Multiplies all components (x, y, z) of this vector (*this) by the scalar float value
 * 'fVal', modifying the vector directly (vector scaling).
 *
 * @param fVal The scalar float value to multiply by (passed by value for efficiency).
 * @return SVector3Df& A reference to the modified SVector3Df object (*this).
 */
SVector3Df& SVector3Df::operator*=(float fVal)
{
	x *= fVal;
	y *= fVal;
	z *= fVal;
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
 * @return SVector3Df& A reference to the modified SVector3Df object (*this).
 */
SVector3Df& SVector3Df::operator/=(float fVal)
{
	if (fVal != 0.0f)
	{
		x /= fVal;
		y /= fVal;
		z /= fVal;
	}
	return (*this);
}

/**
 * @brief Component-wise equality comparison operator.
 *
 * Compares this vector (*this) with the input vector 'vec'.
 * Returns true if and only if all corresponding components (x, y, and z) are equal.
 * This operator is marked 'const' as it does not modify the vector.
 *
 * @param vec The constant reference to the SVector3Df vector to compare against.
 * @return bool True if the vectors are equal component-wise, false otherwise.
 */
bool SVector3Df::operator == (const SVector3Df& vec) const
{
	bool bEqualX = std::fabs(x - vec.x) <= VECTOR3_EPS;
	bool bEqualY = std::fabs(y - vec.y) <= VECTOR3_EPS;
	bool bEqualZ = std::fabs(z - vec.z) <= VECTOR3_EPS;

	return (bEqualX && bEqualY && bEqualZ);
}

/**
 * @brief Component-wise inequality comparison operator.
 *
 * Compares this vector (*this) with the input vector 'vec'.
 * Returns true if any corresponding component (x, y, or z) is not equal.
 * This operator is marked 'const' as it does not modify the vector.
 *
 * @param vec The constant reference to the SVector3Df vector to compare against.
 * @return bool True if the vectors are not equal component-wise, false otherwise.
 */
bool SVector3Df::operator != (const SVector3Df& vec) const
{
	return !(*this == vec);
}

/**
 * @brief Component-wise inequality comparison operator.
 *
 * Compares this vector (*this) with the input vector 'vec'.
 * Returns true if any corresponding component (x, y, or z) is not equal.
 * This operator is marked 'const' as it does not modify the vector.
 *
 * @param vec The constant reference to the glm::vec3 vector to compare against.
 * @return bool True if the vectors are not equal component-wise, false otherwise.
 */
bool SVector3Df::operator == (const glm::vec3& vec) const
{
	bool bEqualX = std::fabs(x - vec.x) <= VECTOR3_EPS;
	bool bEqualY = std::fabs(y - vec.y) <= VECTOR3_EPS;
	bool bEqualZ = std::fabs(z - vec.z) <= VECTOR3_EPS;

	return (bEqualX && bEqualY && bEqualZ);
}

/**
 * @brief Component-wise inequality comparison operator.
 *
 * Compares this vector (*this) with the input vector 'vec'.
 * Returns true if any corresponding component (x, y, or z) is not equal.
 * This operator is marked 'const' as it does not modify the vector.
 *
 * @param vec The constant reference to the glm::vec3 vector to compare against.
 * @return bool True if the vectors are not equal component-wise, false otherwise.
 */
bool SVector3Df::operator != (const glm::vec3& vec) const
{
	return !(*this == vec);
}

/**
 * @brief Subscript operator (non-const version).
 *
 * Provides read/write access to the vector components using an index (0=x, 1=y, 2=z).
 * Returns a reference to the component, allowing it to be modified.
 * If the index is out of bounds, it defaults to returning a reference to the x component (index 0).
 *
 * @param index The zero-based index of the component to access (0, 1, or 2).
 * @return float& A reference to the specified component (x, y, or z).
 */
float& SVector3Df::operator[](size_t index)
{
	switch (index)
	{
	case 0:
		return x;
	case 1:
		return y;
	case 2:
		return z;
	default:
		return x; // Fallback for invalid index
	}
}

/**
 * @brief Subscript operator (const version).
 *
 * Provides read-only access to the vector components using an index (0=x, 1=y, 2=z).
 * Returns the value of the component.
 * If the index is out of bounds, it defaults to returning the value of the x component (index 0).
 * This operator is marked 'const' as it does not modify the vector.
 *
 * @param index The zero-based index of the component to access (0, 1, or 2).
 * @return const float The value of the specified component (x, y, or z).
 */
const float SVector3Df::operator[](size_t index) const
{
	switch (index)
	{
	case 0:
		return x;
	case 1:
		return y;
	case 2:
		return z;
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
 * @return SVector3Df& A reference to the modified vector instance (*this).
 */
SVector3Df& SVector3Df::operator++()
{
	++this->x;
	++this->y;
	++this->z;
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
 * @return SVector3Df A copy of the vector instance before the increment.
 */
SVector3Df SVector3Df::operator++(int32_t)
{
	SVector3Df temp = *this; // 1. Save the original state (required for postfix).
	++this->x;              // 2. Increment the actual object.
	++this->y;
	++this->z;
	return temp;            // 3. Return the saved original state.
}

/**
 * @brief Prefix decrement operator (e.g., --v).
 *
 * Decrements both the x, y and z components of the vector before the result of
 * the operation is used. This is generally more efficient than the postfix form.
 *
 * @return SVector3Df& A reference to the modified vector instance (*this).
 */
SVector3Df& SVector3Df::operator--()
{
	--this->x;
	--this->y;
	--this->z;
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
 * @return SVector3Df A copy of the vector instance before the decrement.
 */
SVector3Df SVector3Df::operator--(int32_t)
{
	SVector3Df temp = *this; // 1. Save the original state (required for postfix).
	--this->x;              // 2. Decrement the actual object.
	--this->y;
	--this->z;
	return temp;            // 3. Return the saved original state.
}

/**
 * @brief Unary negation operator (non-modifying).
 *
 * Calculates a new vector where every component is negated (multiplied by -1).
 *
 * @return SVector3Df A new vector instance with all components negated.
 */
SVector3Df SVector3Df::operator-() const
{
	return SVector3Df(-x, -y, -z);
}

/**
 * @brief Unary plus operator (non-modifying).
 *
 * Returns a copy of the vector unchanged. This operator is primarily
 * included for symmetry with the unary minus operator.
 *
 * @return SVector3Df A copy of the SVector3Df instance (*this).
 */
SVector3Df SVector3Df::operator+() const
{
	return *this;
}

/**
 * @brief Assignment operator overload for glm::vec3.
 *
 * Allows direct assignment from a GLM vector to this SVector3Df instance,
 * facilitating smooth interoperability between the two math libraries.
 * e.g., SVector3Df myVec; myVec = glm::vec3(5.0f, 10.0f, 5.0f);
 *
 * @param vec3 The constant reference to the glm::vec3 source vector.
 * @return SVector3Df& A reference to the modified SVector3Df instance (*this).
 */
SVector3Df& SVector3Df::operator=(const glm::vec3& vec3)
{
	this->x = vec3.x;
	this->y = vec3.y;
	this->z = vec3.z;
	return (*this);

}

/**
 * @brief Implicit conversion operator to glm::vec3.
 *
 * Allows an SVector3Df instance to be implicitly converted to a glm::vec3 type,
 * enabling direct assignment to GLM variables: glm::vec3 myGlmVec = myVec3;
 * It is defined as 'const' because it does not modify the SVector3Df object.
 *
 * @return glm::vec3 A glm::vec3 instance with the same x, y and z values.
 */
SVector3Df::operator glm::vec3() const
{
	return glm::vec3(this->x, this->y, this->z);
}

/**
 * @brief Const float* conversion operator.
 *
 * Allows the SVector3Df object to be implicitly converted to a constant
 * pointer to its underlying float array (starting at the 'x' component).
 * This is useful for passing vector data directly to C-style functions,
 * such as OpenGL API calls.
 * This operator is marked 'const' as it provides read-only access.
 *
 * @return const float* A constant pointer to the first component (x).
 */
SVector3Df::operator const float* () const
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
SVector3Df::operator float* ()
{
	return (&(x));
}

/**
 * @brief Calculates the squared length of the vector.
 *
 * Computes the squared Euclidean length of the vector, defined as
 * the sum of the squares of its components: x^2 + y^2 + z^2.
 * This operation is non-modifying and marked 'const'.
 *
 * @return float The squared length of the vector.
 */
float SVector3Df::lengthSquared() const
{
	 return (x * x + y * y + z * z);
}

/**
 * @brief Calculates the vector magnitude (length).
 *
 * Computes the Euclidean length (or magnitude) of the vector, defined as
 * the square root of the sum of the squares of its components: sqrt{x^2 + y^2 + z^2}.
 * This operation is non-modifying and marked 'const'.
 *
 * @return float The magnitude (length) of the vector.
 */
float SVector3Df::length() const
{
	return (std::sqrt(x * x + y * y + z * z)); // Calculates the length (magnitude) of the vector
}

/**
 * @brief Normalizes the vector (in-place modification).
 *
 * Modifies the vector such that its length becomes 1.0, preserving its direction.
 * If the current vector length is zero, the vector is left unchanged for safety.
 *
 * @return SVector3Df& A reference to the modified (normalized) SVector3Df object (*this).
 */
SVector3Df& SVector3Df::normalize()
{
	float fLen = length();
	if (fLen != 0.0f)
	{
		x /= fLen;
		y /= fLen;
		z /= fLen;
	}
	return (*this);
}

/**
 * @brief Calculates the Dot Product.
 *
 * Computes the scalar dot product of this vector (*this) and another vector 'vec'.
 * The result is a scalar value equal to $x_1x_2 + y_1y_2 + z_1z_2$.
 * This operation is non-modifying and marked 'const'.
 *
 * @param vec The constant reference to the other SVector3Df vector.
 * @return float The resulting scalar dot product.
 */
float SVector3Df::dot(const SVector3Df& vec) const
{
	float fRet = x * vec.x + y * vec.y + z * vec.z;
	return (fRet);
}

/**
 * @brief Calculates the Cross Product.
 *
 * Computes the vector cross product of this vector (*this) and another vector 'vec'.
 * The resulting vector is perpendicular to both input vectors.
 * This operation is non-modifying and marked 'const'.
 *
 * @param vec The constant reference to the other SVector3Df vector.
 * @return SVector3Df A new vector instance resulting from the cross product.
 */
SVector3Df SVector3Df::cross(const SVector3Df& vec) const
{
	// Cross Product formula:
	// R.x = (y * vec.z) - (z * vec.y)
	// R.y = (z * vec.x) - (x * vec.z)
	// R.z = (x * vec.y) - (y * vec.x)
	float fX = y * vec.z - z * vec.y;
	float fY = z * vec.x - x * vec.z;
	float fZ = x * vec.y - y * vec.x;

	return (SVector3Df(fX, fY, fZ));
}

/**
 * @brief Calculates the distance to another vector.
 *
 * Computes the Euclidean distance between the point represented by this vector
 * (*this) and the point represented by the input vector 'vec'.
 * The distance is calculated as the length of the difference vector:
 * sqrt{(x_1-x_2)^2 + (y_1-y_2)^2 + (z_1-z_2)^2}.
 * This operation is non-modifying and marked 'const'.
 *
 * @param vec The constant reference to the other SVector3Df vector (the target point).
 * @return float The scalar distance between the two points.
 */
float SVector3Df::distance(const SVector3Df& vec) const
{
	float fDeltaX = x - vec.x;
	float fDeltaY = y - vec.y;
	float fDeltaZ = z - vec.z;

	float fDistance = std::sqrt(fDeltaX * fDeltaX + fDeltaY * fDeltaY + fDeltaZ * fDeltaZ);
	return (fDistance);
}

/**
 * @brief Rotates the vector using quaternion rotation (in-place modification).
 *
 * Rotates this vector around a specified axis by a given angle using quaternion
 * mathematics. The rotation is applied in-place, modifying the vector's x, y, and z components.
 * Note: This implementation assumes SQuaternion has been correctly implemented.
 *
 * @param fAngle The angle of rotation (in radians).
 * @param vec The constant reference to the axis of rotation (SVector3Df).
 * @return void
 */
SVector3Df SVector3Df::rotate(const float fAngle, const SVector3Df& vec)
{
	SQuaternion rotationQuat{};
	rotationQuat  = rotationQuat.FromAxisAngle(vec, fAngle);
	return rotationQuat.Rotate(*this);
}

/**
 * @brief Converts the SVector3Df object to a glm::vec3 object (non-modifying).
 *
 * Creates and returns an equivalent `glm::vec3` object, facilitating
 * interoperability with the GLM math library.
 *
 * @return glm::vec3 The equivalent glm::vec3 object.
 */
glm::vec3 SVector3Df::ToGLM() const
{
	return (glm::vec3(x, y, z));
}

/**
 * @brief Commutative non-member vector-scalar addition operator.
 *
 * Allows for commutative addition where the scalar is the left operand (e.g., fVal + vec).
 * Implemented by calling the standard (vec + fVal) operator.
 *
 * @param fVal The scalar float value to add.
 * @param vec The constant reference to the SVector3Df object.
 * @return SVector3Df The resulting SVector3Df object from the operation.
 */
SVector3Df operator+(float fVal, const SVector3Df& vec)
{
	// Solution: Directly access components to perform the addition, 
	// avoiding a potentially complex or ambiguous call to the member operator.
	return SVector3Df(vec.x + fVal, vec.y + fVal, vec.z + fVal);
}

/**
 * @brief Non-member vector-scalar reverse subtraction operator.
 *
 * Allows for reverse subtraction where the vector is subtracted from the scalar (e.g., fVal - vec).
 * This operation negates the result of the standard subtraction: fVal - vec = -(vec - fVal).
 *
 * @param fVal The scalar float value (minuend).
 * @param vec The constant reference to the SVector3Df object (subtrahend).
 * @return SVector3Df The resulting SVector3Df object from the operation.
 */
SVector3Df operator-(float fVal, const SVector3Df& vec)
{
	SVector3Df result(fVal - vec.x, fVal - vec.y, fVal - vec.z);
	return (result);
}

/**
 * @brief Commutative non-member vector-scalar multiplication operator.
 *
 * Allows for commutative multiplication where the scalar is the left operand (e.g., fVal * vec).
 * Implemented by calling the standard (vec * fVal) operator.
 *
 * @param fVal The scalar float value to multiply by.
 * @param vec The constant reference to the SVector3Df object.
 * @return SVector3Df The resulting SVector3Df object from the operation.
 */
SVector3Df operator*(float fVal, const SVector3Df& vec)
{
	return SVector3Df(vec.x * fVal, vec.y * fVal, vec.z * fVal);
}
