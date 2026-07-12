#pragma once

#include <glm/detail/type_vec3.hpp>

constexpr float VECTOR3_EPS = (1e-6f);	// geometry / vectors

struct SVector2Df;

/**
 * SVector3Df: A 3D float vector struct.
 *
 * This struct represents a 3D vector with float components. It provides
 * basic vector operations such as addition, subtraction, scalar multiplication,
 * dot product, length calculation, and normalization.
 *
 * Key features:
 * - Efficient float-based operations
 * - Accurate length and normalization calculations
 * - Support for dot product
 * - Conversion to and from GLM vectors for interoperability
 * - Clear and concise implementation adhering to Betty coding standards
 */
struct SVector3Df
{
	union
	{
		float x;
		float r;
		float s;
	};
	union
	{
		float y;
		float g;
		float t;
	};
	union
	{
		float z;
		float b;
		float p;
	};

	/**
	 * @brief Default constructor.
	 *
	 * Initializes all vector components (x, y, z) to zero.
	 * Uses the default C++ compiler implementation for efficiency.
	 */
	SVector3Df() = default;

	/**
	 * @brief Uniform value constructor.
	 *
	 * Constructs an SVector3Df object with all components (x, y, z)
	 * set to the same scalar value fVal.
	 *
	 * @param fVal The scalar value to set all components to.
	 */
	SVector3Df(float fVal);

	/**
	 * @brief 2D vector extension constructor.
	 *
	 * Constructs an SVector3Df object by using the components from an SVector2Df
	 * object for x and y, and setting the z component to an optional value (defaulting to 0.0f).
	 *
	 * @param vec2 The SVector2Df object providing the x and y values.
	 * @param fZ The value for the z component (defaults to 0.0f).
	 */
	SVector3Df(const SVector2Df& vec2, float fZ = 0.0f);


	/**
	 * @brief Explicit component constructor.
	 *
	 * Constructs an SVector3Df object using explicitly specified values
	 * for the x, y, and z components.
	 *
	 * @param fX The value for the x component.
	 * @param fY The value for the y component.
	 * @param fZ The value for the z component.
	 */
	SVector3Df(float fX, float fY, float fZ);

	/**
	 * @brief Copy constructor.
	 *
	 * Constructs an SVector3Df object by copying the components from another
	 * SVector3Df object.
	 *
	 * @param vec The constant reference to the SVector3Df object to copy from.
	 */
	SVector3Df(const SVector3Df& vec);

	/**
	 * @brief Copy constructor.
	 *
	 * Constructs an SVector3Df object by copying the components from another
	 * SVector3Df object.
	 *
	 * @param vec The constant reference to the SVector3Df object to copy from.
	 */
	SVector3Df(const glm::vec3& vec);

	/**
	 * @brief Array pointer constructor.
	 *
	 * Constructs an SVector3Df object by reading the x, y, and z components
	 * sequentially from the float array pointed to by pVec.
	 *
	 * @param pVec The constant pointer to the float array containing at least 3 elements.
	 */
	SVector3Df(const float* pVec);

	/**
	 * @brief Vector addition with another vector (non-modifying).
	 *
	 * Calculates a new vector by adding the components of the input vector 'vec'
	 * to the components of this vector (*this).
	 *
	 * @param vec The constant reference to the vector to add.
	 * @return SVector3Df A new vector instance resulting from the component-wise addition.
	 */
	SVector3Df operator+(const SVector3Df& vec) const;

	/**
	 * @brief Vector subtraction with another vector (non-modifying).
	 *
	 * Calculates a new vector by subtracting the components of the input vector 'vec'
	 * from the components of this vector (*this).
	 *
	 * @param vec The constant reference to the vector to subtract.
	 * @return SVector3Df A new vector instance resulting from the component-wise subtraction.
	 */
	SVector3Df operator-(const SVector3Df& vec) const;

	/**
	 * @brief Component-wise (Hadamard) vector multiplication (non-modifying).
	 *
	 * Calculates a new vector by multiplying each component of this vector by the
	 * corresponding component of the input vector 'vec'. This is NOT the dot product or cross product.
	 *
	 * @param vec The constant reference to the vector to multiply by.
	 * @return SVector3Df A new vector instance with component-wise results.
	 */
	SVector3Df operator*(const SVector3Df& vec) const;

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
	SVector3Df operator/(const SVector3Df& vec) const;

	/**
	 * @brief Vector addition with a scalar value (non-modifying).
	 *
	 * Calculates a new vector by adding the scalar value 'fVal' to all components
	 * of this vector (*this).
	 *
	 * @param fVal The scalar float value to add (passed by value for efficiency).
	 * @return SVector3Df A new vector instance resulting from the scalar addition.
	 */
	SVector3Df operator+(float fVal) const;

	/**
	 * @brief Vector subtraction with a scalar value (non-modifying).
	 *
	 * Calculates a new vector by subtracting the scalar value 'fVal' from all
	 * components of this vector (*this).
	 *
	 * @param fVal The scalar float value to subtract (passed by value for efficiency).
	 * @return SVector3Df A new vector instance resulting from the scalar subtraction.
	 */
	SVector3Df operator-(float fVal) const;

	/**
	 * @brief Vector multiplication by a scalar value (non-modifying).
	 *
	 * Calculates a new vector by multiplying all components of this vector by the
	 * scalar value 'fVal'. This is standard vector scaling.
	 *
	 * @param fVal The scalar float value to multiply by (passed by value for efficiency).
	 * @return SVector3Df A new vector instance resulting from the scalar multiplication.
	 */
	SVector3Df operator*(float fVal) const;

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
	SVector3Df operator/(float fVal) const;

	/**
	 * @brief Compound assignment operator for vector addition (in-place modification).
	 *
	 * Adds the components of the input vector 'vec' to the corresponding components
	 * of this vector (*this), modifying this object directly.
	 *
	 * @param vec The constant reference to the SVector3Df object to add.
	 * @return SVector3Df& A reference to the modified SVector3Df object (*this).
	 */
	SVector3Df& operator+=(const SVector3Df& vec);

	/**
	 * @brief Compound assignment operator for vector subtraction (in-place modification).
	 *
	 * Subtracts the components of the input vector 'vec' from the corresponding components
	 * of this vector (*this), modifying this object directly.
	 *
	 * @param vec The constant reference to the SVector3Df object to subtract.
	 * @return SVector3Df& A reference to the modified SVector3Df object (*this).
	 */
	SVector3Df& operator-=(const SVector3Df& vec);

	/**
	 * @brief Compound assignment operator for component-wise multiplication (in-place modification).
	 *
	 * Multiplies the components of this vector (*this) by the corresponding components
	 * of the input vector 'vec' (Hadamard product), modifying this object directly.
	 *
	 * @param vec The constant reference to the SVector3Df object to multiply by.
	 * @return SVector3Df& A reference to this modified SVector3Df object.
	 */
	SVector3Df& operator*=(const SVector3Df& vec);

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
	SVector3Df& operator/=(const SVector3Df& vec);

	/**
	 * @brief Compound assignment operator for scalar addition (in-place modification).
	 *
	 * Adds the scalar float value 'fVal' to all components (x, y, z) of this vector
	 * (*this), modifying the vector directly.
	 *
	 * @param fVal The scalar float value to add (passed by value for efficiency).
	 * @return SVector3Df& A reference to the modified SVector3Df object (*this).
	 */
	SVector3Df& operator+=(float fVal);

	/**
	 * @brief Compound assignment operator for scalar subtraction (in-place modification).
	 *
	 * Subtracts the scalar float value 'fVal' from all components (x, y, z) of this vector
	 * (*this), modifying the vector directly.
	 *
	 * @param fVal The scalar float value to subtract (passed by value for efficiency).
	 * @return SVector3Df& A reference to the modified SVector3Df object (*this).
	 */
	SVector3Df& operator-=(float fVal);

	/**
	 * @brief Compound assignment operator for scalar multiplication (in-place modification).
	 *
	 * Multiplies all components (x, y, z) of this vector (*this) by the scalar float value
	 * 'fVal', modifying the vector directly (vector scaling).
	 *
	 * @param fVal The scalar float value to multiply by (passed by value for efficiency).
	 * @return SVector3Df& A reference to the modified SVector3Df object (*this).
	 */
	SVector3Df& operator*=(float fVal);

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
	SVector3Df& operator/=(float fVal);

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
	bool operator == (const SVector3Df& vec) const;

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
	bool operator != (const SVector3Df& vec) const;

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
	bool operator == (const glm::vec3& vec) const;

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
	bool operator != (const glm::vec3& vec) const;

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
	float& operator[](size_t index);

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
	const float operator[](size_t index) const;

	/**
	 * @brief Prefix increment operator (e.g., ++v).
	 *
	 * Increments both the x, y and z components of the vector before the result of
	 * the operation is used. This is generally more efficient than the postfix form.
	 *
	 * @return SVector3Df& A reference to the modified vector instance (*this).
	 */
	SVector3Df& operator++();

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
	SVector3Df operator++(int32_t);

	/**
	 * @brief Prefix decrement operator (e.g., --v).
	 *
	 * Decrements both the x, y and z components of the vector before the result of
	 * the operation is used. This is generally more efficient than the postfix form.
	 *
	 * @return SVector3Df& A reference to the modified vector instance (*this).
	 */
	SVector3Df& operator--();

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
	SVector3Df operator--(int32_t);

	/**
	 * @brief Unary negation operator (non-modifying).
	 *
	 * Calculates a new vector where every component is negated (multiplied by -1).
	 *
	 * @return SVector3Df A new vector instance with all components negated.
	 */
	SVector3Df operator-() const;

	/**
	 * @brief Unary plus operator (non-modifying).
	 *
	 * Returns a copy of the vector unchanged. This operator is primarily
	 * included for symmetry with the unary minus operator.
	 *
	 * @return SVector3Df A copy of the SVector3Df instance (*this).
	 */
	SVector3Df operator+() const;

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
	SVector3Df& operator=(const glm::vec3& vec3);

	/**
	 * @brief Implicit conversion operator to glm::vec3.
	 *
	 * Allows an SVector3Df instance to be implicitly converted to a glm::vec3 type,
	 * enabling direct assignment to GLM variables: glm::vec3 myGlmVec = myVec3;
	 * It is defined as 'const' because it does not modify the SVector3Df object.
	 *
	 * @return glm::vec3 A glm::vec3 instance with the same x, y and z values.
	 */
	operator glm::vec3() const;

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
	operator const float* () const;

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
	operator float* ();

	/**
	 * @brief Calculates the squared length of the vector.
	 *
	 * Computes the squared Euclidean length of the vector, defined as
	 * the sum of the squares of its components: x^2 + y^2 + z^2.
	 * This operation is non-modifying and marked 'const'.
	 *
	 * @return float The squared length of the vector.
	 */
	float lengthSquared() const;

	/**
	 * @brief Calculates the vector magnitude (length).
	 *
	 * Computes the Euclidean length (or magnitude) of the vector, defined as
	 * the square root of the sum of the squares of its components: sqrt{x^2 + y^2 + z^2}.
	 * This operation is non-modifying and marked 'const'.
	 *
	 * @return float The magnitude (length) of the vector.
	 */
	float length() const;

	/**
	 * @brief Normalizes the vector (in-place modification).
	 *
	 * Modifies the vector such that its length becomes 1.0, preserving its direction.
	 * If the current vector length is zero, the vector is left unchanged for safety.
	 *
	 * @return SVector3Df& A reference to the modified (normalized) SVector3Df object (*this).
	 */
	SVector3Df& normalize();

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
	float dot(const SVector3Df& vec) const;


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
	SVector3Df cross(const SVector3Df& vec) const;

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
	float distance(const SVector3Df& vec) const;

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
	SVector3Df rotate(const float fAngle, const SVector3Df& vec);

	/**
	 * @brief Converts the SVector3Df object to a glm::vec3 object (non-modifying).
	 *
	 * Creates and returns an equivalent `glm::vec3` object, facilitating
	 * interoperability with the GLM math library.
	 *
	 * @return glm::vec3 The equivalent glm::vec3 object.
	 */
	glm::vec3 ToGLM() const;
};

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
SVector3Df operator+(float fVal, const SVector3Df& vec);

/**
 * @brief Non-member vector-scalar reverse subtraction operator.
 *
 * Allows for reverse subtraction where the vector is subtracted from the scalar (e.g., fVal - vec).
 * This operation negates the result of the standard subtraction: fVal - vec = -(vec - fVal).
 *
 * @param fVal The scalar float value (minuend).
 * @param vec The constant reference to the Vector3D object (subtrahend).
 * @return Vector3D The resulting Vector3D object from the operation.
 */
SVector3Df operator-(float fVal, const SVector3Df& vec);

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
SVector3Df operator*(float fVal, const SVector3Df& vec);

/*
typedef struct SVector3D
{
	union
	{
		GLdouble x;
		GLdouble r;
	};
	union
	{
		GLdouble y;
		GLdouble g;
	};
	union
	{
		GLdouble z;
		GLdouble b;
	};

	SVector3D() = default;

	SVector3D(GLdouble dVal)
	{
		x = y = z = dVal;
	}

	SVector3D(GLdouble dX, GLdouble dY)
	{
		x = dX;
		y = dY;
		z = 0.0;
	}

	SVector3D(GLdouble dX, GLdouble dY, GLdouble dZ)
	{
		x = dX;
		y = dY;
		z = dZ;
	}

	SVector3D operator+(const SVector3D& vec)
	{
		return SVector3D(x + vec.x, y + vec.y, z + vec.z);
	}

	SVector3D operator-(const SVector3D& vec)
	{
		return SVector3D(x - vec.x, y - vec.y, z - vec.z);
	}

	SVector3D operator*(const SVector3D& vec)
	{
		return SVector3D(x * vec.x, y * vec.y, z * vec.z);
	}

	SVector3D operator/(const SVector3D& vec)
	{
		GLdouble dX = x;
		if (vec.x != 0)
		{
			dX = x / vec.x;
		}

		GLdouble dY = y;
		if (vec.y != 0)
		{
			dY = y / vec.y;
		}

		GLdouble dZ = z;
		if (vec.z != 0)
		{
			dZ = z / vec.z;
		}

		return SVector3D(dX, dY, dZ);
	}

	SVector3D operator+(const GLdouble& dVal)
	{
		return SVector3D(x + dVal, y + dVal, z + dVal);
	}

	SVector3D operator-(const GLdouble& dVal)
	{
		return SVector3D(x - dVal, y - dVal, z - dVal);
	}

	SVector3D operator*(const GLdouble& dVal)
	{
		return SVector3D(x * dVal, y * dVal, z * dVal);
	}

	SVector3D operator/(const GLdouble& dVal)
	{
		GLdouble dX = x;
		GLdouble dY = y;
		GLdouble dZ = z;

		if (dVal != 0.0)
		{
			dX = x / dVal;
			dY = y / dVal;
			dZ = z / dVal;
		}

		return SVector3D(dX, dY, dZ);
	}

	SVector3D& operator+=(const SVector3D& vec)
	{
		x += vec.x;
		y += vec.y;
		z += vec.z;
		return (*this);
	}

	SVector3D& operator-=(const SVector3D& vec)
	{
		x -= vec.x;
		y -= vec.y;
		z -= vec.z;
		return (*this);
	}

	SVector3D& operator*=(const SVector3D& vec)
	{
		x *= vec.x;
		y *= vec.y;
		z *= vec.z;
		return (*this);
	}

	SVector3D& operator/=(const SVector3D& vec)
	{
		if (vec.x != 0.0)
		{
			x /= vec.x;
		}
		if (vec.y != 0.0)
		{
			y /= vec.y;
		}
		if (vec.z != 0.0)
		{
			z /= vec.z;
		}
		return (*this);
	}

	SVector3D& operator+=(const GLdouble& dVal)
	{
		x += dVal;
		y += dVal;
		z += dVal;
		return (*this);
	}

	SVector3D& operator-=(const GLdouble& dVal)
	{
		x -= dVal;
		y -= dVal;
		z -= dVal;
		return (*this);
	}

	SVector3D& operator*=(const GLdouble& dVal)
	{
		x *= dVal;
		y *= dVal;
		z *= dVal;
		return (*this);
	}

	SVector3D& operator/=(const GLdouble& dVal)
	{
		if (dVal != 0.0)
		{
			x /= dVal;
			y /= dVal;
			z /= dVal;
		}
		return (*this);
	}

	bool operator == (const SVector3D& vec)
	{
		return (x == vec.x && y == vec.y && z == vec.z);
	}

	bool operator != (const SVector3D& vec)
	{
		return (x != vec.x || y != vec.y || z != vec.z);
	}

	GLdouble length() const
	{
		GLdouble dVals = (x * x + y * y + z * z);
		return (std::sqrt(dVals)); // Calculates the length (magnitude) of the vector
	}

	GLdouble dot(const SVector3D& vec) const
	{
		GLdouble dRet = x * vec.x + y * vec.y + z * vec.z;
		return (dRet); // Calculates the dot product of two vector objects
	}

	SVector3D cross(const SVector3D& vec) const
	{
		GLdouble dX = y * vec.z - z * vec.y;
		GLdouble dY = z * vec.x - x * vec.z;
		GLdouble dZ = x * vec.y - y * vec.x;
		return (SVector3D(dX, dY, dZ));
	}

	GLdouble distance(const SVector3D& vec) const
	{
		GLdouble dDeltaX = x - vec.x;
		GLdouble dDeltaY = y - vec.y;
		GLdouble dDeltaZ = z - vec.z;

		GLdouble dDistance = std::sqrt(dDeltaX * dDeltaX + dDeltaY * dDeltaY + dDeltaZ * dDeltaZ);
		return (dDistance);
	}
} TVector3D;*/
