#pragma once

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/ext.hpp>
#include <cmath>
#include <stdexcept>

#if defined(_WIN32) || defined(_WIN64)
#define M_E        2.71828182845904523536   // e
#define M_LOG2E    1.44269504088896340736   // log2(e)
#define M_LOG10E   0.434294481903251827651  // log10(e)
#define M_LN2      0.693147180559945309417  // ln(2)
#define M_LN10     2.30258509299404568402   // ln(10)
#define M_PI       3.14159265358979323846   // pi
#define M_PI_2     1.57079632679489661923   // pi/2
#define M_PI_4     0.785398163397448309616  // pi/4
#define M_1_PI     0.318309886183790671538  // 1/pi
#define M_2_PI     0.636619772367581343076  // 2/pi
#define M_2_SQRTPI 1.12837916709551257390   // 2/sqrt(pi)
#define M_SQRT2    1.41421356237309504880   // sqrt(2)
#define M_SQRT1_2  0.707106781186547524401  // 1/sqrt(2)
#endif

#define powi(base,exp) (int)powf((float)(base), (float)(exp))

#define ToRadian(x) (float)(((x) * M_PI / 180.0f))
#define ToDegree(x) (float)(((x) * 180.0f / M_PI))

struct SVector3Df;

typedef struct SQuaternion
{
	GLfloat x;           /**< Vector part */
	GLfloat y;           /**< Vector part */
	GLfloat z;           /**< Vector part */
	GLfloat w;           /**< Scalar part */

	SQuaternion() = default;

	SQuaternion(GLfloat fX, GLfloat fY, GLfloat fZ, GLfloat fW)
	{
		x = fX;
		y = fY;
		z = fZ;
		w = fY;
	}

	SQuaternion operator+(const SQuaternion& quat)
	{
		SQuaternion result{};
		result.x = x + quat.x;
		result.y = y + quat.y;
		result.z = z + quat.z;
		result.w = w + quat.w;
		return (result);
	}

	SQuaternion operator-(const SQuaternion& quat)
	{
		SQuaternion result{};
		result.x = x - quat.x;
		result.y = y - quat.y;
		result.z = z - quat.z;
		result.w = w - quat.w;
		return (result);
	}

	SQuaternion operator*(const SQuaternion& quat)
	{
		SQuaternion result{};

		/*
		Formula from http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/arithmetic/index.htm
				a*e - b*f - c*g - d*h
			+ i (b*e + a*f + c*h- d*g)
			+ j (a*g - b*h + c*e + d*f)
			+ k (a*h + b*g - c*f + d*e)
		*/

		result.w = w * quat.w - x * quat.x - y * quat.y - z * quat.z;
		result.x = x * quat.w + w * quat.x + y * quat.z - z * quat.y;
		result.y = w * quat.y - x * quat.z + y * quat.w + z * quat.x;
		result.z = w * quat.z + x * y - y * quat.x + z * quat.w;

		return (result);
	}

	void set(GLfloat fX, GLfloat fY, GLfloat fZ, GLfloat fW)
	{
		x = fX;
		y = fY;
		z = fZ;
		w = fY;
	}

	void FromAxisAngle(const SVector3Df& vAxis, GLfloat fAngle, bool bRadian = true);

	void conjugate()
	{
		x = -x;
		y = -y;
		z = -z;
		w = w;
	}

	SQuaternion& conjugate(const SQuaternion& quat)
	{
		w = quat.w;
		x = -quat.x;
		y = -quat.y;
		z = -quat.z;

		return (*this);
	}

	SQuaternion& multiply(const SQuaternion& q1, const SQuaternion& q2)
	{
		/*
		Formula from http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/arithmetic/index.htm
				a*e - b*f - c*g - d*h
			+ i (b*e + a*f + c*h- d*g)
			+ j (a*g - b*h + c*e + d*f)
			+ k (a*h + b*g - c*f + d*e)
		*/

		w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
		x = q1.x * q2.w + q1.w * q2.x + q1.y * q2.z - q1.z * q2.y;
		y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
		z = q1.w * q2.z + q1.x * q1.y - q1.y * q2.x + q1.z * q2.w;

		return (*this);
	}

	SQuaternion multiply(const SQuaternion& q1, const SQuaternion& q2) const
	{
		SQuaternion result{};

		/*
		Formula from http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/arithmetic/index.htm
				a*e - b*f - c*g - d*h
			+ i (b*e + a*f + c*h- d*g)
			+ j (a*g - b*h + c*e + d*f)
			+ k (a*h + b*g - c*f + d*e)
		*/

		result.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
		result.x = q1.x * q2.w + q1.w * q2.x + q1.y * q2.z - q1.z * q2.y;
		result.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
		result.z = q1.w * q2.z + q1.x * q1.y - q1.y * q2.x + q1.z * q2.w;

		return (result);
	}

} Quaternion;

/**
 * Adds two Quaternion objects component-wise.
 *
 * @param vec1 The first Quaternion object.
 * @param vec2 The second Quaternion object.
 * @return The resulting Quaternion object.
 */
 inline Quaternion operator+(const Quaternion& quat1, const Quaternion& quat2)
 {
	 Quaternion Result(quat1.x + quat2.x, quat1.y + quat2.y, quat1.z + quat2.z, quat1.w + quat2.w);
	 return (Result);
 }

 /**
  * Subtracts one Quaternion object from another component-wise.
  *
  * @param vec1 The first Quaternion object.
  * @param vec2 The second Quaternion object to subtract.
  * @return The resulting Quaternion object.
  */
 inline Quaternion operator-(const Quaternion& quat1, const Quaternion& quat2)
 {
	 Quaternion Result(quat1.x - quat2.x, quat1.y - quat2.y, quat1.z - quat2.z, quat1.w - quat2.w);
	 return (Result);
 }

 inline Quaternion operator*(const Quaternion& quat1, const Quaternion& quat2)
 {
	 SQuaternion result{};

	 /*
	 Formula from http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/arithmetic/index.htm
			 a*e - b*f - c*g - d*h
		 + i (b*e + a*f + c*h- d*g)
		 + j (a*g - b*h + c*e + d*f)
		 + k (a*h + b*g - c*f + d*e)
	 */

	 result.w = quat1.w * quat2.w - quat1.x * quat2.x - quat1.y * quat2.y - quat1.z * quat2.z;
	 result.x = quat1.x * quat2.w + quat1.w * quat2.x + quat1.y * quat2.z - quat1.z * quat2.y;
	 result.y = quat1.w * quat2.y - quat1.x * quat2.z + quat1.y * quat2.w + quat1.z * quat2.x;
	 result.z = quat1.w * quat2.z + quat1.x * quat1.y - quat1.y * quat2.x + quat1.z * quat2.w;

	 return (result);
 }

void Quaternion_Set(float _w, float _v1, float _v2, float _v3, SQuaternion* qOutput);
void Quaternion_FromAxisAngle(const SVector3Df& vAxis, float fAngle, SQuaternion* qOutput, bool bRadian = true);
void Quaternion_Conjugate(SQuaternion* quat, SQuaternion* qOutput);
void Quaternion_Multiply(SQuaternion* q1, SQuaternion* q2, SQuaternion* qResult);

/**
 * SVector2Df: A 2D float vector struct.
 *
 * This struct represents a 2D vector with float components. It provides
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
typedef struct SVector2Df
{
	union
	{
		GLfloat x;
		GLfloat u;
		GLfloat r;
		GLfloat s;
	};

	union
	{
		GLfloat y;
		GLfloat v;
		GLfloat g;
		GLfloat t;
	};

	/**
	 * Default constructor, initializes all components to zero.
	 */
	SVector2Df() = default;

	/**
	 * Constructs an SVector2Df object with both x and y components initialized to the same value.
	 *
	 * @param fVal The initial value for both x and y components.
	 */
	SVector2Df(GLfloat fVal)
	{
		x = y = fVal;
	}

	/**
	 * Constructs an SVector2Df object with specified x and y components.
	 *
	 * @param _x The initial value for the x component.
	 * @param _y The initial value for the y component.
	 */
	SVector2Df(GLfloat _x, GLfloat _y)
	{
		x = _x;
		y = _y;
	}

	/**
	 * Constructs an SVector2Df object with both x and y components initialized to the same value.
	 *
	 * @param fVal The initial value for both x and y components.
	 */
	SVector2Df(int iVal)
	{
		x = y = static_cast<GLfloat>(iVal);
	}

	/**
	 * Constructs an SVector2Df object with specified x and y components.
	 *
	 * @param _x The initial value for the x component.
	 * @param _y The initial value for the y component.
	 */
	SVector2Df(GLint _x, GLint _y)
	{
		x = static_cast<GLfloat>(_x);
		y = static_cast<GLfloat>(_y);
	}

	/**
	 * Constructs an SVector2Df object by copying the components from a glm::vec2 vector.
	 *
	 * @param vec The glm::vec2 vector to copy from.
	 */
	SVector2Df(const glm::vec2& vec)
	{
		x = vec.x;
		y = vec.y;
	}

	/**
	 * Adds two SVector2Df objects component-wise.
	 *
	 * @param vec The SVector2Df object to add.
	 * @return The resulting SVector2Df object.
	 */
	SVector2Df operator+(const SVector2Df& vec)
	{
		return (SVector2Df(x + vec.x, y + vec.y));
	}

	/**
	 * Subtracts one SVector2Df object from another component-wise.
	 *
	 * @param vec The SVector2Df object to subtract.
	 * @return The resulting SVector2Df object.
	 */
	SVector2Df operator-(const SVector2Df& vec)
	{
		return (SVector2Df(x - vec.x, y - vec.y));
	}

	/**
	 * Multiplies two SVector2Df objects component-wise.
	 *
	 * @param vec The SVector2Df object to multiply.
	 * @return The resulting SVector2Df object.
	 */
	SVector2Df operator*(const SVector2Df& vec)
	{
		return (SVector2Df(x * vec.x, y * vec.y));
	}

	/**
	 * Divides one SVector2Df object by another component-wise.
	 *
	 * @param vec The SVector2Df object to divide by.
	 * @return The resulting SVector2Df object.
	 * @throws std::domain_error If either component of the divisor is zero.
	 */
	SVector2Df operator/(const SVector2Df& vec)
	{
		GLfloat fX = x;
		GLfloat fY = y;
		if (vec.x != 0.0f)
		{
			fX = x / vec.x;
		}
		if (vec.y != 0.0f)
		{
			fY = y / vec.y;
		}

		return SVector2Df(fX, fY);
	}

	/**
	 * Adds a scalar value to both components of an SVector2Df object.
	 *
	 * @param fVal The scalar value to add.
	 * @return The resulting SVector2Df object.
	 */
	SVector2Df operator+(const GLfloat fVal)
	{
		return (SVector2Df(x + fVal, y + fVal));
	}

	/**
	 * Subtracts a scalar value from both components of an SVector2Df object.
	 *
	 * @param fVal The scalar value to subtract.
	 * @return The resulting SVector2Df object.
	 */
	SVector2Df operator-(const GLfloat fVal)
	{
		return (SVector2Df(x - fVal, y - fVal));
	}

	/**
	 * Multiplies both components of an SVector2Df object by a scalar value.
	 *
	 * @param fVal The scalar value to multiply by.
	 * @return The resulting SVector2Df object.
	 */
	SVector2Df operator*(const GLfloat fVal)
	{
		return (SVector2Df(x * fVal, y * fVal));
	}

	/**
	 * Divides both components of an SVector2Df object by a scalar value.
	 *
	 * @param fVal The scalar value to divide by.
	 * @return The resulting SVector2Df object.
	 * @throws std::domain_error If the divisor is zero.
	 */
	SVector2Df operator/(const GLfloat fVal)
	{
		GLfloat fX = x;
		GLfloat fY = y;
		if (fVal != 0.0f)
		{
			fX = x / fVal;
			fY = y / fVal;
		}

		return SVector2Df(fX, fY);
	}

	/**
	 * Adds another SVector2Df object to this one, modifying this object in-place.
	 *
	 * @param vec The SVector2Df object to add.
	 * @return A reference to this modified SVector2Df object.
	 */
	SVector2Df& operator+=(const SVector2Df& vec)
	{
		x += vec.x;
		y += vec.y;
		return (*this);
	}

	/**
	 * Subtracts another SVector2Df object from this one, modifying this object in-place.
	 *
	 * @param vec The SVector2Df object to subtract.
	 * @return A reference to this modified SVector2Df object.
	 */
	SVector2Df& operator-=(const SVector2Df& vec)
	{
		x -= vec.x;
		y -= vec.y;
		return (*this);
	}

	/**
	 * Multiplies this SVector2Df object by another one, modifying this object in-place.
	 *
	 * @param vec The SVector2Df object to multiply by.
	 * @return A reference to this modified SVector2Df object.
	 */
	SVector2Df& operator*=(const SVector2Df& vec)
	{
		x *= vec.x;
		y *= vec.y;
		return (*this);
	}

	/**
	 * Divides this SVector2Df object by another one, modifying this object in-place.
	 *
	 * @param vec The SVector2Df object to divide by.
	 * @return A reference to this modified SVector2Df object.
	 * @throws std::domain_error If either component of the divisor is zero.
	 */
	SVector2Df& operator/=(const SVector2Df& vec)
	{
		if (vec.x != 0.0f)
		{
			x /= vec.x;
		}
		if (vec.y != 0.0f)
		{
			y /= vec.y;
		}
		return (*this);
	}

	/**
	 * Adds a scalar value to both components of this SVector2Df object, modifying it in-place.
	 *
	 * @param fVal The scalar value to add.
	 * @return A reference to this modified SVector2Df object.
	 */
	SVector2Df& operator+=(const GLfloat fVal)
	{
		x += fVal;
		y += fVal;
		return (*this);
	}

	/**
	 * Subtracts a scalar value from both components of this SVector2Df object, modifying it in-place.
	 *
	 * @param fVal The scalar value to subtract.
	 * @return A reference to this modified SVector2Df object.
	 */
	SVector2Df& operator-=(const GLfloat fVal)
	{
		x -= fVal;
		y -= fVal;
		return (*this);
	}

	/**
	 * Multiplies both components of this SVector2Df object by a scalar value, modifying it in-place.
	 *
	 * @param fVal The scalar value to multiply by.
	 * @return A reference to this modified SVector2Df object.
	 */
	SVector2Df& operator*=(const GLfloat fVal)
	{
		x *= fVal;
		y *= fVal;
		return (*this);
	}

	/**
	 * Divides both components of this SVector2Df object by a scalar value, modifying it in-place.
	 *
	 * @param fVal The scalar value to divide by.
	 * @return A reference to this modified SVector2Df object.
	 * @throws std::domain_error If the divisor is zero.
	 */
	SVector2Df& operator/=(const GLfloat fVal)
	{
		if (fVal != 0.0f)
		{
			x /= fVal;
			y /= fVal;
		}
		return (*this);
	}

	/**
	 * Compares two SVector2Df objects for equality.
	 *
	 * @param vec The SVector2Df object to compare to.
	 * @return true if the two objects are equal, false otherwise.
	 */
	bool operator == (const SVector2Df& vec)
	{
		return (x == vec.x && y == vec.y);
	}

	/**
	 * Compares two SVector2Df objects for inequality.
	 *
	 * @param vec The SVector2Df object to compare to.
	 * @return true if the two objects are not equal, false otherwise.
	 */
	bool operator != (const SVector2Df& vec)
	{
		return (x != vec.x || y != vec.y);
	}

	/**
	 * Allow to access the vector in array like style []
	 *
	 * @param index access param index
	 * @return actual index value
	 * @throws std::out_of_range If the index is out of 0-2 range.
	 */
	GLfloat& operator[](size_t index)
	{
		switch (index)
		{
		case 0:
			return x;
		case 1:
			return y;
		default:
			throw std::out_of_range("Invalid index");
		}
	}

	/**
	 * Calculates the length (magnitude) of the SVector2Df object.
	 *
	 * @return The length of the vector.
	 */
	GLfloat length() const
	{
		GLfloat fLen = std::sqrt(x * x + y * y); // Corrected the calculation
		return (fLen);
	}

	/**
	 * Normalizes the SVector2Df object, making its length 1.
	 *
	 * @return A reference to this modified SVector2Df object.
	 * @throws std::domain_error If the vector is zero.
	 */
	SVector2Df& normalize()
	{
		GLfloat fLen = length();
		if (fLen != 0.0f)
		{
			x /= fLen;
			y /= fLen;
		}
		return (*this);
	}

	/**
	 * Provides a const pointer to the underlying float array.
	 *
	 * This allows direct access to the x and y components as a float array.
	 *
	 * @return A const pointer to the float array.
	 */
	operator const GLfloat* () const
	{
		return (&(x));
	}

} Vector2D;

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
} TVector3D;

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
typedef struct SVector3Df
{
	union
	{
		GLfloat x;
		GLfloat r;
	};
	union
	{
		GLfloat y;
		GLfloat g;
	};
	union
	{
		GLfloat z;
		GLfloat b;
	};

	/**
	 * Default constructor, initializes all components to zero.
	 */
	SVector3Df() = default;

	/**
	 * Constructs an SVector3Df object with all components set to the same value.
	 *
	 * @param fVal The value to set all components to.
	 */
	SVector3Df(GLfloat fVal)
	{
		x = y = z = fVal;
	}

	/**
	 * Constructs an SVector3Df object with specified values for each component.
	 *
	 * @param fX The value for the x component.
	 * @param fY The value for the y component.
	 */
	SVector3Df(GLfloat fX, GLfloat fY)
	{
		x = fX;
		y = fY;
		z = 0.0f;
	}

	/**
	 * Constructs an SVector3Df object with specified values for each component.
	 *
	 * @param fX The value for the x component.
	 * @param fY The value for the y component.
	 * @param fZ The value for the z component.  
	 */
	SVector3Df(GLfloat fX, GLfloat fY, GLfloat fZ)
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
	SVector3Df(const SVector3Df& vec)
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
	SVector3Df(const glm::vec3& vec)
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
	SVector3Df(const GLfloat* pVec)
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
	 * Negates all components of the SVector3Df object.
	 *
	 * @return The negated SVector3Df object.
	 */
	SVector3Df operator-() const
	{
		return SVector3Df(-x, -y, -z);
	}

	/**
	 * Adds two SVector3Df objects component-wise.
	 *
	 * @param vec The SVector3Df object to add.
	 * @return The resulting SVector3Df object.
	 */
	SVector3Df operator+(const SVector3Df& vec)
	{
		return SVector3Df(x + vec.x, y + vec.y, z + vec.z);
	}

	/**
	 * Subtracts one SVector3Df object from another component-wise.
	 *
	 * @param vec The SVector3Df object to subtract.
	 * @return The resulting SVector3Df object.
	 */
	SVector3Df operator-(const SVector3Df& vec)
	{
		return SVector3Df(x - vec.x, y - vec.y, z - vec.z);
	}

	/**
	 * Multiplies two SVector3Df objects component-wise.
	 *
	 * @param vec The SVector3Df object to multiply.
	 * @return The resulting SVector3Df object.
	 */
	SVector3Df operator*(const SVector3Df& vec)
	{
		return SVector3Df(x * vec.x, y * vec.y, z * vec.z);
	}

	/**
	 * Divides one SVector3Df object by another component-wise.
	 *
	 * @param vec The SVector3Df object to divide by.
	 * @return The resulting SVector3Df object.
	 */
	SVector3Df operator/(const SVector3Df& vec)
	{
		GLfloat fX = x;
		if (vec.x != 0.0f)
		{
			fX = x / vec.x;
		}

		GLfloat fY = y;
		if (vec.y != 0.0f)
		{
			fY = y / vec.y;
		}

		GLfloat fZ = z;
		if (vec.z != 0.0f)
		{
			fZ = z / vec.z;
		}

		return SVector3Df(fX, fY, fZ);
	}

	/**
	 * Adds a scalar value to all components of an SVector3Df object.
	 *
	 * @param fVal The scalar value to add.
	 * @return The resulting SVector3Df object.
	 */
	SVector3Df operator+(const GLfloat& fVal)
	{
		return SVector3Df(x + fVal, y + fVal, z + fVal);
	}

	/**
	 * Subtracts a scalar value from all components of an SVector3Df object.
	 *
	 * @param fVal The scalar value to subtract.
	 * @return The resulting SVector3Df object.
	 */
	SVector3Df operator-(const GLfloat& fVal)
	{
		return SVector3Df(x - fVal, y - fVal, z - fVal);
	}

	/**
	 * Multiplies all components of an SVector3Df object by a scalar value.
	 *
	 * @param fVal The scalar value to multiply by.
	 * @return The resulting SVector3Df object.
	 */
	SVector3Df operator*(const GLfloat& fVal)
	{
		return SVector3Df(x * fVal, y * fVal, z * fVal);
	}

	/**
	 * Divides all components of an SVector3Df object by a scalar value.
	 *
	 * @param fVal The scalar value to divide by.
	 * @return The resulting SVector3Df object.
	 */
	SVector3Df operator/(const GLfloat& fVal)
	{
		GLfloat fX = x;
		GLfloat fY = y;
		GLfloat fZ = z;

		if (fVal != 0.0f)
		{
			fX = x / fVal;
			fY = y / fVal;
			fZ = z / fVal;
		}

		return SVector3Df(fX, fY, fZ);
	}

	/**
	 * Adds another SVector3Df object to this one, modifying this object in-place.
	 *
	 * @param vec The SVector3Df object to add.
	 * @return A reference to this modified SVector3Df object.
	 */
	SVector3Df& operator+=(const SVector3Df& vec)
	{
		x += vec.x;
		y += vec.y;
		z += vec.z;
		return (*this);
	}

	/**
	 * Subtracts another SVector3Df object from this one, modifying this object in-place.
	 *
	 * @param vec The SVector3Df object to subtract.
	 * @return A reference to this modified SVector3Df object.
	 */
	SVector3Df& operator-=(const SVector3Df& vec)
	{
		x -= vec.x;
		y -= vec.y;
		z -= vec.z;
		return (*this);
	}

	/**
	 * Multiplies this SVector3Df object by another one, modifying this object in-place.
	 *
	 * @param vec The SVector3Df object to multiply by.
	 * @return A reference to this modified SVector3Df object.
	 */
	SVector3Df& operator*=(const SVector3Df& vec)
	{
		x *= vec.x;
		y *= vec.y;
		z *= vec.z;
		return (*this);
	}

	/**
	 * Divides this SVector3Df object by another one, modifying this object in-place.
	 *
	 * @param vec The SVector3Df object to divide by.
	 * @return A reference to this modified SVector3Df object.
	 */
	SVector3Df& operator/=(const SVector3Df& vec)
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
	 * Adds a scalar value to all components of this SVector3Df object, modifying it in-place.
	 *
	 * @param fVal The scalar value to add.
	 * @return A reference to this modified SVector3Df object.
	 */
	SVector3Df& operator+=(const GLfloat& fVal)
	{
		x += fVal;
		y += fVal;
		z += fVal;
		return (*this);
	}

	/**
	 * Subtracts a scalar value from all components of this SVector3Df object, modifying it in-place.
	 *
	 * @param fVal The scalar value to subtract.
	 * @return A reference to this modified SVector3Df object.
	 */
	SVector3Df& operator-=(const GLfloat& fVal)
	{
		x -= fVal;
		y -= fVal;
		z -= fVal;
		return (*this);
	}

	/**
	 * Multiplies all components of this SVector3Df object by a scalar value, modifying it in-place.
	 *
	 * @param fVal The scalar value to multiply by.
	 * @return A reference to this modified SVector3Df object.
	 */
	SVector3Df& operator*=(const GLfloat& fVal)
	{
		x *= fVal;
		y *= fVal;
		z *= fVal;
		return (*this);
	}

	/**
	 * Divides all components of this SVector3Df object by a scalar value, modifying it in-place.
	 *
	 * @param fVal The scalar value to divide by.
	 * @return A reference to this modified SVector3Df object.
	 */
	SVector3Df& operator/=(const GLfloat& fVal)
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
	 * Compares two SVector3Df objects for equality.
	 *
	 * @param vec The SVector3Df object to compare to.
	 * @return true if the two objects are equal, false otherwise.
	 */
	bool operator == (const SVector3Df& vec)
	{
		return (x == vec.x && y == vec.y && z == vec.z);
	}

	/**
	 * Compares two SVector3Df objects for inequality.
	 *
	 * @param vec The SVector3Df object to compare to.
	 * @return true if the two objects are not equal, false otherwise.
	 */
	bool operator != (const SVector3Df& vec)
	{
		return (x != vec.x || y != vec.y || z != vec.z);
	}

	/**
	 * Allow to access the vector in array like style []
	 *
	 * @param index access param index
	 * @return actual index value
	 * @throws std::out_of_range If the index is out of 0-2 range.
	 */
	GLfloat& operator[](size_t index)
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
			throw std::out_of_range("Invalid index");
		}
	}

	/**
	 * Calculates the length (magnitude) of the SVector3Df object.
	 *
	 * @return The length of the vector.
	 */
	GLfloat length() const
	{
		GLfloat fVals = (x * x + y * y + z * z);
		return (std::sqrt(fVals)); // Calculates the length (magnitude) of the vector
	}

	/**
	 * Normalizes the SVector3Df object, making its length 1.
	 *
	 * @return A reference to this modified SVector3Df object.
	 */
	SVector3Df& normalize()
	{
		GLfloat fLen = length();
		if (fLen != 0.0f)
		{
			x /= fLen;
			y /= fLen;
			z /= fLen;
		}
		return (*this);
	}

	/**
	 * Calculates the dot product of two SVector3Df objects.
	 *
	 * @param vec The SVector3Df object to calculate the dot product with.
	 * @return The dot product of the two vectors.
	 */
	GLfloat dot(const SVector3Df& vec) const
	{
		GLfloat dRet = x * vec.x + y * vec.y + z * vec.z;
		return (dRet); // Calculates the dot product of two vector objects
	}

	/**
	 * Calculates the cross product of two SVector3Df objects.
	 *
	 * @param vec The SVector3Df object to calculate the cross product with.
	 * @return The cross product of the two vectors.
	 */
	SVector3Df cross(const SVector3Df& vec) const
	{
		GLfloat fX = y * vec.z - z * vec.y;
		GLfloat fY = z * vec.x - x * vec.z;
		GLfloat fZ = x * vec.y - y * vec.x;

		return (SVector3Df(fX, fY, fZ));
	}

	/**
	 * Calculates the Euclidean distance between two SVector3Df objects.
	 *
	 * @param vec The SVector3Df object to calculate the distance to.
	 * @return The distance between the two vectors.
	 */
	GLfloat distance(const SVector3Df& vec) const
	{
		GLfloat fDeltaX = x - vec.x;
		GLfloat fDeltaY = y - vec.y;
		GLfloat fDeltaZ = z - vec.z;

		GLfloat fDistance = std::sqrt(fDeltaX * fDeltaX + fDeltaY * fDeltaY + fDeltaZ * fDeltaZ);
		return (fDistance);
	}

	/**
	 * Rotates the vector using quaternion by giving angle and vector.
	 *
	 * @param fAngle The Angle of Rotation.
	 * @param vec The vector to rotate around, could represent an Axis
	 * @return The cross product of the two vectors.
	 */
	void rotate(const GLfloat fAngle, const SVector3Df& vec)
	{
		// Convert the vector to a quaternion with w = 0
		SQuaternion vectorQuat(x, y, z, 0.0f);

		// Create the quaternion from the axis and angle
		SQuaternion rotationQuat{};
		rotationQuat.FromAxisAngle(vec, fAngle); // Convert angle to radians

		// Compute the conjugate of the rotation quaternion
		SQuaternion conjugateQuat{};
		//Quaternion_Conjugate(&rotationQuat, &conjugateQuat);
		conjugateQuat.conjugate(rotationQuat);

		// Perform the quaternion multiplication: q * v * q^-1
		SQuaternion tempQuat{};
		tempQuat = rotationQuat * vectorQuat;   // q * v

		SQuaternion resultQuat{};
		resultQuat = tempQuat * conjugateQuat; // q * v * q^-1

		// Extract the rotated vector from the resulting quaternion
		x = resultQuat.x;
		y = resultQuat.y;
		z = resultQuat.z;
	}

	/**
	 * Provides a const pointer to the underlying float array.
	 *
	 * This allows direct access to the x and y components as a float array.
	 *
	 * @return A const pointer to the float array.
	 */
	operator const GLfloat* () const
	{
		return (&(x));
	}

} Vector3D;

inline void Quaternion::FromAxisAngle(const Vector3D& vAxis, GLfloat fAngle, bool bRadian)
{
	// Formula from http://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToQuaternion/
	GLfloat angleRad = fAngle;

	if (bRadian)
	{
		angleRad = -ToRadian(fAngle);
	}

	//the sine of half the rotation angle
	GLfloat sinHalf = std::sinf(angleRad / 2.0f);
	x = sinHalf * vAxis.x;
	y = sinHalf * vAxis.y;
	z = sinHalf * vAxis.z;

	w = std::cosf(angleRad / 2.0f);
}

inline void Quaternion_Set(float _w, float _v1, float _v2, float _v3, Quaternion* qOutput)
{
	assert(qOutput != nullptr);
	qOutput->w = _w;
	qOutput->x = _v1;
	qOutput->y = _v2;
	qOutput->z = _v3;
}

inline void Quaternion_FromAxisAngle(const SVector3Df& vAxis, float fAngle, Quaternion* qOutput, bool bRadian)
{
	assert(qOutput != nullptr);

	// Formula from http://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToQuaternion/
	float angleRad = fAngle;

	if (bRadian)
		angleRad = -ToRadian(fAngle);

	qOutput->w = std::cosf(angleRad / 2.0f);

	//the sine of half the rotation angle
	float c = std::sinf(angleRad / 2.0f);
	qOutput->x = c * vAxis.x;
	qOutput->y = c * vAxis.y;
	qOutput->z = c * vAxis.z;
}

inline void Quaternion_Conjugate(Quaternion* quat, Quaternion* qOutput)
{
	assert(qOutput != nullptr && quat != nullptr);

	qOutput->w = quat->w;
	qOutput->x = -quat->x;
	qOutput->y = -quat->y;
	qOutput->z = -quat->z;
}

inline void Quaternion_Multiply(Quaternion* q1, Quaternion* q2, Quaternion* qResult)
{
	assert(q1 != nullptr && q2 != nullptr && qResult != nullptr);

	Quaternion result{};

	/*
	Formula from http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/arithmetic/index.htm
			a*e - b*f - c*g - d*h
		+ i (b*e + a*f + c*h- d*g)
		+ j (a*g - b*h + c*e + d*f)
		+ k (a*h + b*g - c*f + d*e)
	*/

	result.w = q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z;
	result.x = q1->x * q2->w + q1->w * q2->x + q1->y * q2->z - q1->z * q2->y;
	result.y = q1->w * q2->y - q1->x * q2->z + q1->y * q2->w + q1->z * q2->x;
	result.z = q1->w * q2->z + q1->x * q1->y - q1->y * q2->x + q1->z * q2->w;

	*qResult = result;
}

/**
 * Adds a scalar value to all components of an Vector3D object.
 *
 * @param vec The Vector3D object to add to.
 * @param fVal The scalar value to add.
 * @return The resulting Vector3D object.
 */
inline Vector3D operator+(const Vector3D& vec, GLfloat fVal)
{
	Vector3D Result(vec.x + fVal, vec.y + fVal, vec.z + fVal);
	return (Result);
}

/**
 * Subtracts a scalar value from all components of an Vector3D object.
 *
 * @param vec The Vector3D object to subtract from.
 * @param fVal The scalar value to subtract.
 * @return The resulting Vector3D object.
 */
inline Vector3D operator-(const Vector3D& vec, GLfloat fVal)
{
	Vector3D Result(vec.x - fVal, vec.y - fVal, vec.z - fVal);
	return (Result);
}

/**
 * Multiplies all components of an Vector3D object by a scalar value.
 *
 * @param vec1 The first Vector3D object.
 * @param vec2 The second Vector3D object to multiply.
 * @return The resulting Vector3D object.
 */
inline Vector3D operator*(const Vector3D& vec, GLfloat fVal)
{
	Vector3D Result(vec.x * fVal, vec.y * fVal, vec.z * fVal);
	return (Result);
}

/**
 * Divides all components of an Vector3D object by a scalar value.
 *
 * @param vec The Vector3D object to divide.
 * @param fVal The scalar value to divide by.
 * @return The resulting Vector3D object.
 */
inline Vector3D operator/(const Vector3D& vec, GLfloat fVal)
{
	Vector3D Result(0.0f);
	if (fVal != 0.0f)
	{
		Result.x = vec.x / fVal;
		Result.y = vec.y / fVal;
		Result.z = vec.z / fVal;
	}
	return (Result);
}

/**
 * Adds two Vector3D objects component-wise.
 *
 * @param vec1 The first Vector3D object.
 * @param vec2 The second Vector3D object.
 * @return The resulting Vector3D object.
 */
inline Vector3D operator+(const Vector3D& vec1, const Vector3D& vec2)
{
	Vector3D Result(vec1.x + vec2.x, vec1.y + vec2.y, vec1.z + vec2.z);
	return (Result);
}

/**
 * Subtracts one Vector3D object from another component-wise.
 *
 * @param vec1 The first Vector3D object.
 * @param vec2 The second Vector3D object to subtract.
 * @return The resulting Vector3D object.
 */
inline Vector3D operator-(const Vector3D& vec1, const Vector3D& vec2)
{
	Vector3D Result(vec1.x - vec2.x, vec1.y - vec2.y, vec1.z - vec2.z);
	return (Result);
}

/**
 * Multiplies two Vector3D objects component-wise.
 *
 * @param vec1 The first Vector3D object.
 * @param vec2 The second Vector3D object to multiply.
 * @return The resulting Vector3D object.
 */
inline Vector3D operator*(const Vector3D& vec1, const Vector3D& vec2)
{
	Vector3D Result(vec1.x * vec2.x, vec1.y * vec2.y, vec1.z * vec2.z);
	return (Result);
}

/**
 * Divides one Vector3D object by another component-wise.
 *
 * @param vec1 The first Vector3D object.
 * @param vec2 The second Vector3D object to divide by.
 * @return The resulting Vector3D object.
 */
inline Vector3D operator/(const Vector3D& vec1, const Vector3D& vec2)
{
	Vector3D Result(0.0f);
	if (vec2.x != 0.0f)
	{
		Result.x = vec1.x / vec2.x;
	}
	if (vec2.y != 0.0f)
	{
		Result.y = vec1.y / vec2.y;
	}
	if (vec2.z != 0.0f)
	{
		Result.z = vec1.z / vec2.z;
	}
	return (Result);
}

/**
 * SVector4Df: A 4D float vector struct.
 *
 * This struct represents a 4D vector with float components. It provides
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
typedef struct SVector4Df
{
	union
	{
		GLfloat x;
		GLfloat r;
	};
	union
	{
		GLfloat y;
		GLfloat g;
	};
	union
	{
		GLfloat z;
		GLfloat b;
	};
	union
	{
		GLfloat w;
		GLfloat a;
	};

	/**
	 * Default constructor, initializes all components to zero.
	 */
	SVector4Df() = default;

	/**
	 * Constructs an SVector4Df object with all components set to the same value.
	 *
	 * @param fVal The value to set all components to.
	 */
	SVector4Df(GLfloat fVal)
	{
		x = y = z = w = fVal;
	}

	/**
	 * Constructs an SVector4Df object with specified values for each component.
	 *
	 * @param fX The value for the x component.
	 * @param fY The value for the y component.
	 */
	SVector4Df(GLfloat fX, GLfloat fY)
	{
		x = fX;
		y = fY;
		z = 0.0f;
		w = 0.0f;
	}

	/**
	 * Constructs an SVector4Df object with specified values for each component.
	 *
	 * @param fX The value for the x component.
	 * @param fY The value for the y component.
	 * @param fZ The value for the z component.
	 */
	SVector4Df(GLfloat fX, GLfloat fY, GLfloat fZ)
	{
		x = fX;
		y = fY;
		z = fZ;
		w = 0.0f;
	}

	/**
	 * Constructs an SVector4Df object with specified values for each component.
	 *
	 * @param fX The value for the x component.
	 * @param fY The value for the y component.
	 * @param fZ The value for the z component.
	 * @param fW The value for the w component.
	 */
	SVector4Df(GLfloat fX, GLfloat fY, GLfloat fZ, GLfloat fW)
	{
		x = fX;
		y = fY;
		z = fZ;
		w = fW;
	}

	/**
	 * Constructs an SVector4Df object from a SVector3Df object.
	 *
	 * @param vec The SVector3Df object to construct from.
	 */
	SVector4Df(const Vector3D& vec)
	{
		x = vec.x;
		y = vec.y;
		z = vec.z;
		w = 0.0f;
	}

	/**
	 * Constructs an SVector4Df object from a SVector4Df object.
	 *
	 * @param vec The SVector4Df object to construct from.
	 */
	SVector4Df(const SVector4Df& vec)
	{
		x = vec.x;
		y = vec.y;
		z = vec.z;
		w = vec.w;
	}

	/**
	 * Constructs an SVector4Df object from a pointer to a float array.
	 *
	 * @param pVec The pointer to the float array.
	 */
	SVector4Df(const GLfloat* pVec)
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
	 * Negates all components of the SVector4Df object.
	 *
	 * @return The negated SVector4Df object.
	 */
	SVector4Df operator-() const
	{
		return SVector4Df(-x, -y, -z, -w);
	}

	/**
	 * Adds two SVector3Df objects component-wise.
	 *
	 * @param vec The SVector3Df object to add.
	 * @return The resulting SVector3Df object.
	 */
	SVector4Df operator+(const SVector4Df& vec)
	{
		return SVector4Df(x + vec.x, y + vec.y, z + vec.z, w + vec.w);
	}

	/**
	 * Subtracts one SVector4Df object from another component-wise.
	 *
	 * @param vec The SVector4Df object to subtract.
	 * @return The resulting SVector4Df object.
	 */
	SVector4Df operator-(const SVector4Df& vec)
	{
		return SVector4Df(x - vec.x, y - vec.y, z - vec.z, w - vec.w);
	}

	/**
	 * Multiplies two SVector4Df objects component-wise.
	 *
	 * @param vec The SVector4Df object to multiply.
	 * @return The resulting SVector4Df object.
	 */
	SVector4Df operator*(const SVector4Df& vec)
	{
		return SVector4Df(x * vec.x, y * vec.y, z * vec.z, w * vec.w);
	}

	/**
	 * Divides one SVector4Df object by another component-wise.
	 *
	 * @param vec The SVector4Df object to divide by.
	 * @return The resulting SVector4Df object.
	 */
	SVector4Df operator/(const SVector4Df& vec)
	{
		GLfloat fX = x;
		if (vec.x != 0.0f)
		{
			fX = x / vec.x;
		}

		GLfloat fY = y;
		if (vec.y != 0.0f)
		{
			fY = y / vec.y;
		}

		GLfloat fZ = z;
		if (vec.z != 0.0f)
		{
			fZ = z / vec.z;
		}

		GLfloat fW = w;
		if (vec.w != 0.0f)
		{
			fW = w / vec.w;
		}

		return SVector4Df(fX, fY, fZ, fW);
	}

	/**
	 * Adds a scalar value to all components of an SVector3Df object.
	 *
	 * @param fVal The scalar value to add.
	 * @return The resulting SVector3Df object.
	 */
	SVector4Df operator+(const GLfloat& fVal)
	{
		return SVector4Df(x + fVal, y + fVal, z + fVal, w + fVal);
	}

	/**
	 * Subtracts a scalar value from all components of an SVector4Df object.
	 *
	 * @param fVal The scalar value to subtract.
	 * @return The resulting SVector4Df object.
	 */
	SVector4Df operator-(const GLfloat& fVal)
	{
		return SVector4Df(x - fVal, y - fVal, z - fVal, w - fVal);
	}

	/**
	 * Multiplies all components of an SVector4Df object by a scalar value.
	 *
	 * @param fVal The scalar value to multiply by.
	 * @return The resulting SVector4Df object.
	 */
	SVector4Df operator*(const GLfloat& fVal)
	{
		return SVector4Df(x * fVal, y * fVal, z * fVal, w * fVal);
	}

	/**
	 * Divides all components of an SVector4Df object by a scalar value.
	 *
	 * @param fVal The scalar value to divide by.
	 * @return The resulting SVector4Df object.
	 */
	SVector4Df operator/(const GLfloat& fVal)
	{
		GLfloat fX = x;
		GLfloat fY = y;
		GLfloat fZ = z;
		GLfloat fW = w;

		if (fVal != 0.0f)
		{
			fX = x / fVal;
			fY = y / fVal;
			fZ = z / fVal;
			fW = w / fVal;
		}

		return SVector4Df(fX, fY, fZ, fW);
	}

	/**
	 * Adds another SVector4Df object to this one, modifying this object in-place.
	 *
	 * @param vec The SVector4Df object to add.
	 * @return A reference to this modified SVector4Df object.
	 */
	SVector4Df& operator+=(const SVector4Df& vec)
	{
		x += vec.x;
		y += vec.y;
		z += vec.z;
		w += vec.w;
		return (*this);
	}

	/**
	 * Subtracts another SVector4Df object from this one, modifying this object in-place.
	 *
	 * @param vec The SVector4Df object to subtract.
	 * @return A reference to this modified SVector4Df object.
	 */
	SVector4Df& operator-=(const SVector4Df& vec)
	{
		x -= vec.x;
		y -= vec.y;
		z -= vec.z;
		w -= vec.w;
		return (*this);
	}

	/**
	 * Multiplies this SVector4Df object by another one, modifying this object in-place.
	 *
	 * @param vec The SVector4Df object to multiply by.
	 * @return A reference to this modified SVector4Df object.
	 */
	SVector4Df& operator*=(const SVector4Df& vec)
	{
		x *= vec.x;
		y *= vec.y;
		z *= vec.z;
		w *= vec.w;
		return (*this);
	}

	/**
	 * Divides this SVector4Df object by another one, modifying this object in-place.
	 *
	 * @param vec The SVector4Df object to divide by.
	 * @return A reference to this modified SVector4Df object.
	 */
	SVector4Df& operator/=(const SVector4Df& vec)
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
	 * Adds a scalar value to all components of this SVector4Df object, modifying it in-place.
	 *
	 * @param fVal The scalar value to add.
	 * @return A reference to this modified SVector4Df object.
	 */
	SVector4Df& operator+=(const GLfloat& fVal)
	{
		x += fVal;
		y += fVal;
		z += fVal;
		w += fVal;
		return (*this);
	}

	/**
	 * Subtracts a scalar value from all components of this SVector4Df object, modifying it in-place.
	 *
	 * @param fVal The scalar value to subtract.
	 * @return A reference to this modified SVector4Df object.
	 */
	SVector4Df& operator-=(const GLfloat& fVal)
	{
		x -= fVal;
		y -= fVal;
		z -= fVal;
		w -= fVal;
		return (*this);
	}

	/**
	 * Multiplies all components of this SVector4Df object by a scalar value, modifying it in-place.
	 *
	 * @param fVal The scalar value to multiply by.
	 * @return A reference to this modified SVector4Df object.
	 */
	SVector4Df& operator*=(const GLfloat& fVal)
	{
		x *= fVal;
		y *= fVal;
		z *= fVal;
		w *= fVal;
		return (*this);
	}

	/**
	 * Divides all components of this SVector4Df object by a scalar value, modifying it in-place.
	 *
	 * @param fVal The scalar value to divide by.
	 * @return A reference to this modified SVector4Df object.
	 */
	SVector4Df& operator/=(const GLfloat& fVal)
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
	 * Compares two SVector4Df objects for equality.
	 *
	 * @param vec The SVector4Df object to compare to.
	 * @return true if the two objects are equal, false otherwise.
	 */
	bool operator == (const SVector4Df& vec)
	{
		return (x == vec.x && y == vec.y && z == vec.z && w == vec.w);
	}

	/**
	 * Compares two SVector4Df objects for inequality.
	 *
	 * @param vec The SVector4Df object to compare to.
	 * @return true if the two objects are not equal, false otherwise.
	 */
	bool operator != (const SVector4Df& vec)
	{
		return (x != vec.x || y != vec.y || z != vec.z || w != vec.w);
	}

	/**
	 * Allow to access the vector in array like style []
	 *
	 * @param index access param index
	 * @return actual index value
	 * @throws std::out_of_range If the index is out of 0-3 range.
	 */
	GLfloat& operator[](size_t index)
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
			throw std::out_of_range("Invalid index");
		}
	}

	/**
	 * Allow to access the vector in array like style []
	 *
	 * @param index access param index
	 * @return actual index value
	 * @throws std::out_of_range If the index is out of 0-3 range.
	 */
	const GLfloat& operator[](size_t index) const
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
			throw std::out_of_range("Invalid index");
		}
	}

	SVector4Df& operator++()
	{
		++this->x;
		++this->y;
		++this->z;
		++this->w;
		return (*this);
	}

	SVector4Df& operator++(GLint)
	{
		++this->x;
		++this->y;
		++this->z;
		++this->w;
		return (*this);
	}

	SVector4Df& operator--()
	{
		--this->x;
		--this->y;
		--this->z;
		--this->w;
		return (*this);
	}

	SVector4Df& operator--(GLint)
	{
		--this->x;
		--this->y;
		--this->z;
		--this->w;
		return (*this);
	}

	/**
	 * Calculates the length (magnitude) of the SVector4Df object.
	 *
	 * @return The length of the vector.
	 */
	GLfloat length() const
	{
		GLfloat fVals = (x * x + y * y + z * z + w * w);
		return (std::sqrt(fVals)); // Calculates the length (magnitude) of the vector
	}

	/**
	 * Calculates the dot product of two SVector4Df objects.
	 *
	 * @param vec The SVector4Df object to calculate the dot product with.
	 * @return The dot product of the two vectors.
	 */
	GLfloat dot(const SVector4Df& vec) const
	{
		GLfloat dRet = x * vec.x + y * vec.y + z * vec.z + w * vec.w;
		return (dRet); // Calculates the dot product of two vector objects
	}

	/**
	 * Normalizes the SVector4Df object, making its length 1.
	 *
	 * @return A reference to this modified SVector4Df object.
	 */
	SVector4Df& normalize()
	{
		GLfloat fLen = length();
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
	 * Calculates the Euclidean distance between two SVector4Df objects.
	 *
	 * @param vec The SVector4Df object to calculate the distance to.
	 * @return The distance between the two vectors.
	 */
	GLfloat distance(const SVector4Df& vec) const
	{
		GLfloat fDeltaX = x - vec.x;
		GLfloat fDeltaY = y - vec.y;
		GLfloat fDeltaZ = z - vec.z;
		GLfloat fDeltaW = w - vec.w;

		GLfloat fDistance = std::sqrt(fDeltaX * fDeltaX + fDeltaY * fDeltaY + fDeltaZ * fDeltaZ + fDeltaW * fDeltaW);
		return (fDistance);
	}

	/**
	 * Provides a const pointer to the underlying float array.
	 *
	 * This allows direct access to the x and y components as a float array.
	 *
	 * @return A const pointer to the float array.
	 */
	operator const GLfloat* () const
	{
		return (&(x));
	}

} Vector4D;

/**
 * Adds a scalar value to all components of an SVector4Df object.
 *
 * @param vec The SVector4Df object to add to.
 * @param fVal The scalar value to add.
 * @return The resulting SVector4Df object.
 */
inline Vector4D operator+(const Vector4D& vec, GLfloat fVal)
{
	Vector4D Result(vec.x + fVal, vec.y + fVal, vec.z + fVal, vec.w + fVal);
	return (Result);
}

/**
 * Subtracts a scalar value from all components of an SVector4Df object.
 *
 * @param vec The SVector4Df object to subtract from.
 * @param fVal The scalar value to subtract.
 * @return The resulting SVector4Df object.
 */
inline Vector4D operator-(const Vector4D& vec, GLfloat fVal)
{
	SVector4Df Result(vec.x - fVal, vec.y - fVal, vec.z - fVal, vec.w - fVal);
	return (Result);
}

/**
 * Multiplies all components of an SVector4Df object by a scalar value.
 *
 * @param vec1 The first SVector4Df object.
 * @param vec2 The second SVector4Df object to multiply.
 * @return The resulting SVector4Df object.
 */
inline Vector4D operator*(const Vector4D& vec, GLfloat fVal)
{
	Vector4D Result(vec.x * fVal, vec.y * fVal, vec.z * fVal, vec.w * fVal);
	return (Result);
}

/**
 * Divides all components of an SVector4Df object by a scalar value.
 *
 * @param vec The SVector4Df object to divide.
 * @param fVal The scalar value to divide by.
 * @return The resulting SVector4Df object.
 */
inline Vector4D operator/(const Vector4D& vec, GLfloat fVal)
{
	Vector4D Result(0.0f);
	if (fVal != 0.0f)
	{
		Result.x = vec.x / fVal;
		Result.y = vec.y / fVal;
		Result.z = vec.z / fVal;
		Result.w = vec.w / fVal;
	}
	return (Result);
}

/**
 * Adds two SVector4Df objects component-wise.
 *
 * @param vec1 The first SVector4Df object.
 * @param vec2 The second SVector4Df object.
 * @return The resulting SVector4Df object.
 */
inline Vector4D operator+(const Vector4D& vec1, const Vector4D& vec2)
{
	Vector4D Result(vec1.x + vec2.x, vec1.y + vec2.y, vec1.z + vec2.z, vec1.w + vec2.w);
	return (Result);
}

/**
 * Subtracts one SVector4Df object from another component-wise.
 *
 * @param vec1 The first SVector4Df object.
 * @param vec2 The second SVector4Df object to subtract.
 * @return The resulting SVector4Df object.
 */
inline Vector4D operator-(const Vector4D& vec1, const Vector4D& vec2)
{
	Vector4D Result(vec1.x - vec2.x, vec1.y - vec2.y, vec1.z - vec2.z, vec1.w - vec2.w);
	return (Result);
}

/**
 * Multiplies two SVector4Df objects component-wise.
 *
 * @param vec1 The first SVector4Df object.
 * @param vec2 The second SVector4Df object to multiply.
 * @return The resulting SVector4Df object.
 */
inline Vector4D operator*(const Vector4D& vec1, const Vector4D& vec2)
{
	Vector4D Result(vec1.x * vec2.x, vec1.y * vec2.y, vec1.z * vec2.z, vec1.w * vec2.w);
	return (Result);
}

/**
 * Divides one SVector4Df object by another component-wise.
 *
 * @param vec1 The first SVector4Df object.
 * @param vec2 The second SVector4Df object to divide by.
 * @return The resulting SVector4Df object.
 */
inline Vector4D operator/(const Vector4D& vec1, const Vector4D& vec2)
{
	Vector4D Result(0.0f);
	if (vec2.x != 0.0f)
	{
		Result.x = vec1.x / vec2.x;
	}
	if (vec2.y != 0.0f)
	{
		Result.y = vec1.y / vec2.y;
	}
	if (vec2.z != 0.0f)
	{
		Result.z = vec1.z / vec2.z;
	}
	if (vec2.w != 0.0f)
	{
		Result.w = vec1.w / vec2.w;
	}
	return (Result);
}

/**
 * Structure to hold parameters for a perspective projection matrix
 *
 * This structure contains all the necessary information to create a perspective projection matrix
 * for rendering 3D scenes. The perspective projection simulates depth, making objects appear smaller
 * as they get farther from the camera, similar to how the human eye perceives depth.
 *
 * Members:
 *   - FOV: Field of View, defines the vertical angle (in degrees or radians) for the camera's view.
 *   - Width: The width of the viewport (screen/window) for rendering.
 *   - Height: The height of the viewport for rendering.
 *   - zNear: The near clipping plane distance. Objects closer than this will not be rendered.
 *   - zFar: The far clipping plane distance. Objects farther than this will not be rendered.
 */
typedef struct SPersProjInfo
{
	GLfloat FOV;      /* Vertical Field of View in degrees or radians */
	GLfloat Width;    /* Width of the viewport (screen/window) */
	GLfloat Height;   /* Height of the viewport */
	GLfloat zNear;    /* Near clipping plane distance */
	GLfloat zFar;     /* Far clipping plane distance */
} TPersProjInfo;

/**
 * Structure to hold parameters for an orthogonal projection matrix
 *
 * This structure contains the parameters needed to define an orthographic projection matrix, which
 * is used when you want to render objects with no perspective distortion. In this projection, objects
 * appear the same size regardless of their distance from the camera, which is ideal for 2D views,
 * technical drawings, or isometric views.
 *
 * Members:
 *   - Right: The right plane of the orthogonal view frustum.
 *   - Left: The left plane of the orthogonal view frustum.
 *   - Bottom: The bottom plane of the orthogonal view frustum.
 *   - Top: The top plane of the orthogonal view frustum.
 *   - NearZ: The near clipping plane distance.
 *   - FarZ: The far clipping plane distance.
 *   - Width: The width of the view in world-space units.
 *   - Height: The height of the view in world-space units.
 */
typedef struct SOrthoProjInfo
{
	GLfloat Left;    /** The left plane of the orthogonal frustum */
	GLfloat Right;   /** The right plane of the orthogonal frustum */
	GLfloat Bottom;  /** The bottom plane of the orthogonal frustum */
	GLfloat Top;     /** The top plane of the orthogonal frustum */
	GLfloat NearZ;   /** The near clipping plane distance */
	GLfloat FarZ;    /** The far clipping plane distance */
	GLfloat Width;   /** The width of the orthogonal view in world-space units */
	GLfloat Height;  /** The height of the orthogonal view in world-space units */
} TOrthoProjInfo;

typedef struct SMatrix2x2
{
	typedef Vector2D col_type;
	typedef Vector2D row_type;

	SMatrix2x2() = default;

	SMatrix2x2(GLfloat x0, GLfloat y0, GLfloat x1, GLfloat y1)
	{
		this->value[0] = col_type(x0, y0);
		this->value[1] = col_type(x1, y1);
	}

	SMatrix2x2(const SVector2Df& col1, const SVector2Df& col2)
	{
		this->value[0] = col_type(col1);
		this->value[1] = col_type(col2);
	}

	/**
	 * Constructor that initializes the matrix from a GLM matrix.
	 *
	 * @param glmMat: A GLM matrix to copy from, correct for column major.
	 */
	SMatrix2x2(const glm::mat2& glmMat)
	{
		value[0][0] = glmMat[0][0]; value[0][1] = glmMat[0][1];
		value[1][0] = glmMat[1][0]; value[1][1] = glmMat[1][1];
	}

	/**
	 * @brief Provides mutable access to a column vector (col_type) in the matrix.
	 *
	 * This operator allows modification of the vector at the specified column index.
	 *
	 * @param i The zero-based index of the column vector (0 to 1).
	 * @return A mutable reference to the column vector (col_type) at index i.
	 */
	col_type& operator[](size_t i)
	{
		if (i >= 2)
		{
			throw std::out_of_range("Column index out of bounds");
		}
		return this->value[i];
	}

	/**
	 * @brief Provides read-only access to a column vector (col_type) in the matrix.
	 *
	 * This const operator returns a reference to the column vector at the specified index,
	 * suitable for constant objects.
	 *
	 * @param i The zero-based index of the column vector (0 to 1).
	 * @return A constant reference to the column vector (col_type) at index i.
	 */
	const col_type& operator[](size_t i) const
	{
		if (i >= 2)
		{
			throw std::out_of_range("Column index out of bounds");
		}
		return this->value[i];
	}

	SMatrix2x2& operator=(const SMatrix2x2& mat2)
	{
		//memcpy could be faster
		//memcpy(&this->value, &m.value, 16 * sizeof(valType));
		this->value[0] = mat2[0];
		this->value[1] = mat2[1];
		return *this;
	}

	SMatrix2x2& operator+=(const SMatrix2x2& mat2)
	{
		this->value[0] += mat2[0];
		this->value[1] += mat2[1];
		return *this;
	}

	SMatrix2x2& operator+=(const GLfloat& fVal)
	{
		this->value[0] += fVal;
		this->value[1] += fVal;
		return *this;
	}


	SMatrix2x2& operator-=(const SMatrix2x2& mat2)
	{
		this->value[0] -= mat2[0];
		this->value[1] -= mat2[1];
		return *this;
	}

	SMatrix2x2& operator-=(const GLfloat& fVal)
	{
		this->value[0] -= fVal;
		this->value[1] -= fVal;
		return *this;
	}

	/**
	 * Provides a const pointer to the underlying float array.
	 *
	 * This allows direct access to the matrix vectors components as a float array.
	 *
	 * @return A const pointer to the float array.
	 */
	operator const GLfloat* () const
	{
		return (const GLfloat*)value;
	}

private:
	col_type value[2];

} Matrix2x2;

typedef struct SMatrix3x3
{
	typedef Vector3D col_type;
	typedef Vector3D row_type;

	SMatrix3x3() = default;

	SMatrix3x3(GLfloat x0, GLfloat y0, GLfloat z0, GLfloat x1, GLfloat y1, GLfloat z1, GLfloat x2, GLfloat y2, GLfloat z2)
	{
		this->value[0] = col_type(x0, y0, z0);
		this->value[1] = col_type(x1, y1, z1);
		this->value[2] = col_type(x1, y1, z1);
	}

	SMatrix3x3(const SVector3Df& col1, const SVector3Df& col2, const SVector3Df& col3)
	{
		this->value[0] = col_type(col1);
		this->value[1] = col_type(col2);
		this->value[2] = col_type(col3);
	}

	/**
	 * Constructor that initializes the matrix from a GLM matrix.
	 *
	 * @param glmMat: A GLM matrix to copy from, correct for column major.
	 */
	SMatrix3x3(const glm::mat3& glmMat)
	{
		value[0][0] = glmMat[0][0]; value[0][1] = glmMat[0][1]; value[0][2] = glmMat[0][2];
		value[1][0] = glmMat[1][0]; value[1][1] = glmMat[1][1]; value[1][2] = glmMat[1][2];
		value[2][0] = glmMat[2][0]; value[2][1] = glmMat[2][1]; value[2][2] = glmMat[2][2];
	}

	/**
	 * Provides a const pointer to the underlying float array.
	 *
	 * This allows direct access to the matrix vectors components as a float array.
	 *
	 * @return A const pointer to the float array.
	 */
	operator const GLfloat* () const
	{
		return (const GLfloat*)value;
	}
private:
	col_type value[3];

} Matrix3x3;

// Column Major Matrix
typedef struct SMatrix4x4
{
	typedef Vector4D col_type;
	typedef Vector4D row_type;

	SMatrix4x4() = default;

	SMatrix4x4(GLfloat x0, GLfloat y0, GLfloat z0, GLfloat w0
		, GLfloat x1, GLfloat y1, GLfloat z1, GLfloat w1
		, GLfloat x2, GLfloat y2, GLfloat z2, GLfloat w2
		, GLfloat x3, GLfloat y3, GLfloat z3, GLfloat w3)
	{
		this->value[0] = col_type(x0, y0, z0, w0);
		this->value[1] = col_type(x1, y1, z1, w1);
		this->value[2] = col_type(x2, y2, z2, w2);
		this->value[3] = col_type(x3, y3, z3, w3);

	}

	SMatrix4x4(const SVector4Df& col1, const SVector4Df& col2, const SVector4Df& col3, const SVector4Df& col4)
	{
		this->value[0] = col_type(col1);
		this->value[1] = col_type(col2);
		this->value[2] = col_type(col3);
		this->value[3] = col_type(col4);
	}

	/**
	 * Constructor that initializes the matrix from a GLM matrix.
	 *
	 * @param glmMat: A GLM matrix to copy from.
	 */
	SMatrix4x4(const glm::mat4& glmMat)
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
	col_type& operator[](size_t i)
	{
		if (i >= 4)
		{
			throw std::out_of_range("Column index out of bounds");
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
	const col_type& operator[](size_t i) const
	{
		if (i >= 4)
		{
			throw std::out_of_range("Column index out of bounds");
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
	SMatrix4x4& operator=(const SMatrix4x4& mat4)
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
	SMatrix4x4& operator+=(const SMatrix4x4& mat4)
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
	SMatrix4x4& operator+=(const GLfloat& fVal)
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
	SMatrix4x4& operator-=(const SMatrix4x4& mat4)
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
	SMatrix4x4& operator-=(const GLfloat& fVal)
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
	SMatrix4x4& operator*=(const SMatrix4x4& mat4)
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
	SMatrix4x4& operator*=(const GLfloat& fVal)
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
	SMatrix4x4& operator/=(const SMatrix4x4& mat4)
	{
		return *this *= InverseSub(mat4);
	}

	/**
	 * @brief Performs scalar division component-wise and assigns the result.
	 *
	 * Divides every component of this matrix by the scalar value.
	 *
	 * @param fVal The scalar value to divide by.
	 * @return A mutable reference to the current matrix (*this).
	 */
	SMatrix4x4& operator/=(const GLfloat& fVal)
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
	SMatrix4x4& operator++()
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
	SMatrix4x4& operator--()
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
	 * @param GLint (int) Dummy argument to denote post-increment.
	 * @return A mutable reference to the incremented matrix (*this).
	 */
	SMatrix4x4& operator++(GLint)
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
	 * @param GLint Dummy argument to denote post-decrement.
	 * @return A mutable reference to the decremented matrix (*this).
	 */
	SMatrix4x4& operator--(GLint)
	{
		this->value[0]--;
		this->value[1]--;
		this->value[2]--;
		this->value[3]--;
		return (*this);
	}

	/**
	 * @brief converts a matrix into positive.
	 *
	 * Converts the given matrix into a positive one.
	 *
	 * @param mat4 The matrix to set.
	 * @return A new SMatrix4x4 object representing the positive one.
	 */
	SMatrix4x4 operator+(const SMatrix4x4& mat4)
	{
		return (mat4);
	}

	/**
	 * @brief converts a matrix into negative.
	 *
	 * Converts the positive given matrix into a negative one.
	 *
	 * @param mat4 The matrix to negate.
	 * @return A new SMatrix4x4 object representing the negated one.
	 */
	SMatrix4x4 operator-(const SMatrix4x4& mat4)
	{
		SMatrix4x4 Result(-mat4[0], -mat4[1], -mat4[2], -mat4[3]);
		return (Result);
	}

	/**
	 * @brief Checks for exact component-wise equality between two matrices.
	 *
	 * @param mat4 The matrix to compare against.
	 * @return True if all components are equal; otherwise, false.
	 */
	bool operator==(const SMatrix4x4& mat4)
	{
		return (value[0] == mat4[0] && value[1] == mat4[1] && value[2] == mat4[2] && value[3] == mat4[3]);
	}

	/**
	 * @brief Checks for inequality between two matrices.
	 *
	 * @param mat4 The matrix to compare against.
	 * @return True if at least one component is unequal; otherwise, false.
	 */
	bool operator!=(const SMatrix4x4& mat4)
	{
		return (value[0] != mat4[0]) || (value[1] != mat4[1]) || (value[2] != mat4[2]) || (value[3] != mat4[3]);
	}

	SMatrix4x4 InverseSub();
	SMatrix4x4 InverseSub(const SMatrix4x4& mat4);

	/**
	 * @brief Initializes the matrix to the 4x4 Identity Matrix.
	 *
	 * Sets all diagonal components (M[i][i]) to 1.0f and all off-diagonal components to 0.0f.
	 */
	void InitIdentity()
	{
		value[0][0] = 1.0f; value[0][1] = 0.0f; value[0][2] = 0.0f; value[0][3] = 0.0f;
		value[1][0] = 0.0f; value[1][1] = 1.0f; value[1][2] = 0.0f; value[1][3] = 0.0f;
		value[2][0] = 0.0f; value[2][1] = 0.0f; value[2][2] = 1.0f; value[2][3] = 0.0f;
		value[3][0] = 0.0f; value[3][1] = 0.0f; value[3][2] = 0.0f; value[3][3] = 1.0f;
	}

	/**
	 * Constructs a view matrix that transforms world coordinates to camera space.
	 *
	 * @param v3Eye    The camera position in world space
	 * @param v3Center The point the camera is looking at
	 * @param v3Up     The up direction vector (typically (0,1,0))
	 * @return         A 4x4 view matrix in column-major order
	 */
	SMatrix4x4 LookAtRH(const Vector3D& v3Eye, const Vector3D& v3Center, const Vector3D& v3Up)
	{
		// Forward vector: f = (v_center - v_eye) / ||v_center - v_eye||
		Vector3D f = v3Center - v3Eye;
		f.normalize();

		// Right vector: s = (f x v_up) / ||f x v_up||
		// where 'x' denotes cross product
		Vector3D s = f.cross(v3Up);
		s.normalize();

		// Up vector: u = s x f
		// where 'x' denotes cross product
		Vector3D u = s.cross(f);

		// Construct the view matrix M:
		//
		//     [  s_x    s_y    s_z   -s.p ]
		// M = [  u_x    u_y    u_z   -u.p ]
		//     [ -f_x   -f_y   -f_z    f.p ]
		//     [   0      0      0      1  ]
		//
		// where s = (s_x, s_y, s_z) is the right vector
		//       u = (u_x, u_y, u_z) is the up vector
		//       f = (f_x, f_y, f_z) is the forward vector
		//       p is the camera position (v3Eye)
		//       '.' denotes dot product
		//
		// Matrix is indexed as M[column][row] for OpenGL compatibility
		SMatrix4x4 viewMatrix{};

		// Rotation components (basis vectors)
		viewMatrix[0][0] = s.x;   viewMatrix[1][0] = s.y;   viewMatrix[2][0] = s.z;		// Row 0: right vector
		viewMatrix[0][1] = u.x;   viewMatrix[1][1] = u.y;   viewMatrix[2][1] = u.z;		// Row 1: up vector
		viewMatrix[0][2] = -f.x;  viewMatrix[1][2] = -f.y;  viewMatrix[2][2] = -f.z;	// Row 2: -forward vector
		viewMatrix[0][3] = 0.0f;  viewMatrix[1][3] = 0.0f;  viewMatrix[2][3] = 0.0f;	// Row 3: homogeneous

		// Translation components (negated dot products with camera position)
		viewMatrix[3][0] = -s.dot(v3Eye);	// -s.p
		viewMatrix[3][1] = -u.dot(v3Eye);	// -u.p
		viewMatrix[3][2] = f.dot(v3Eye);		//  f.p
		viewMatrix[3][3] = 1.0f;

		return (viewMatrix);
	}

	/**
	 * Constructs a perspective projection matrix for 3D rendering.
	 * Maps view space coordinates to normalized device coordinates (NDC).
	 *
	 * Uses a 45-degree field of view and clips geometry between near (0.1)
	 * and far (1000.0) planes. Compatible with OpenGL's coordinate system.
	 *
	 * @return 4x4 perspective projection matrix in column-major order
	 */
	SMatrix4x4 PerspectiveRH(const SPersProjInfo& persProj)
	{
		// Perspective projection parameters
		GLfloat Fov = persProj.FOV;  // Field of view in degrees
		GLfloat HalfTanFOV = std::tan(ToRadian(Fov / 2.0f));
		GLfloat AspectRatio = persProj.Width / persProj.Height;
		GLfloat NearZ = persProj.zNear;    // Near clipping plane
		GLfloat FarZ = persProj.zFar;  // Far clipping plane
		GLfloat ZRange = FarZ - NearZ;

		// Construct the perspective projection matrix P:
		//
		//     [ 1/(t*a)     0           0              0     ]
		// P = [    0      1/t           0              0     ]
		//     [    0       0     -(f+n)/(f-n)   -2fn/(f-n)   ]
		//     [    0       0          -1              0      ]
		//
		// where t = tan(fov/2)  (half tangent of field of view)
		//       a = aspect ratio (width/height)
		//       n = near clipping plane distance
		//       f = far clipping plane distance
		//
		// This maps view space to clip space with:
		//   X: [-aspect*tan(fov/2), aspect*tan(fov/2)] -> [-1, 1]
		//   Y: [-tan(fov/2), tan(fov/2)] -> [-1, 1]
		//   Z: [-n, -f] -> [-1, 1] (OpenGL depth range)

		SMatrix4x4 CameraProjectionMatrix{};

		// Column 0: X-axis scaling (accounts for aspect ratio)
		CameraProjectionMatrix[0][0] = 1.0f / (HalfTanFOV * AspectRatio);
		CameraProjectionMatrix[0][1] = 0.0f;
		CameraProjectionMatrix[0][2] = 0.0f;
		CameraProjectionMatrix[0][3] = 0.0f;

		// Column 1: Y-axis scaling (based on FOV)
		CameraProjectionMatrix[1][0] = 0.0f;
		CameraProjectionMatrix[1][1] = 1.0f / HalfTanFOV;
		CameraProjectionMatrix[1][2] = 0.0f;
		CameraProjectionMatrix[1][3] = 0.0f;

		// Column 2: Z-axis mapping and perspective division trigger
		CameraProjectionMatrix[2][0] = 0.0f;
		CameraProjectionMatrix[2][1] = 0.0f;
		CameraProjectionMatrix[2][2] = -(FarZ + NearZ) / (FarZ - NearZ);  // Z remapping
		CameraProjectionMatrix[2][3] = -1.0f;  // Triggers perspective division (w = -z)

		// Column 3: Z translation component
		CameraProjectionMatrix[3][0] = 0.0f;
		CameraProjectionMatrix[3][1] = 0.0f;
		CameraProjectionMatrix[3][2] = -(2.0f * FarZ * NearZ) / (FarZ - NearZ);  // Z offset
		CameraProjectionMatrix[3][3] = 0.0f;

		return (CameraProjectionMatrix);
	}

	// Getter for use with OpenGL
	const GLfloat* value_ptr() const
	{
		// Safely cast the address of the first vector to a pointer to float.
		// This is safe because SVector4Df is an array of GLfloat or a struct 
		// with standard layout whose first member is GLfloat.
		return (const GLfloat*)value;
	}

	/**
	 * Provides a const pointer to the underlying float array.
	 *
	 * This allows direct access to the matrix vectors components as a float array.
	 *
	 * @return A const pointer to the float array.
	 */
	operator const GLfloat* () const
	{
		return (const GLfloat*)value;
	}

private:
	SVector4Df value[4];

} Matrix4x4;

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
inline Matrix4x4::col_type operator*(const SMatrix4x4& mat4, const Matrix4x4::row_type& rowVector)
{
	const Matrix4x4::col_type Mov0 = (rowVector[0]);
	const Matrix4x4::col_type Mov1 = (rowVector[1]);

	const Matrix4x4::col_type Mul0 = (mat4[0] * Mov0);
	const Matrix4x4::col_type Mul1 = (mat4[1] * Mov1);

	const Matrix4x4::col_type Add0 = Mul0 + Mul1;

	const Matrix4x4::col_type Mov2 = (rowVector[2]);
	const Matrix4x4::col_type Mov3 = (rowVector[3]);

	const Matrix4x4::col_type Mul2 = (mat4[2] * Mov2);
	const Matrix4x4::col_type Mul3 = (mat4[3] * Mov3);

	const Matrix4x4::col_type Add1 = Mul2 + Mul3;
	const Matrix4x4::col_type Add2 = Add0 + Add1;

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
inline Matrix4x4::row_type operator*(const Matrix4x4::col_type& colVector, const SMatrix4x4& mat4)
{
	const GLfloat row0 = mat4[0][0] * colVector[0] + mat4[0][1] * colVector[1] + mat4[0][2] * colVector[2] + mat4[0][3] * colVector[3];
	const GLfloat row1 = mat4[1][0] * colVector[0] + mat4[1][1] * colVector[1] + mat4[1][2] * colVector[2] + mat4[1][3] * colVector[3];
	const GLfloat row2 = mat4[2][0] * colVector[0] + mat4[2][1] * colVector[1] + mat4[2][2] * colVector[2] + mat4[2][3] * colVector[3];
	const GLfloat row3 = mat4[3][0] * colVector[0] + mat4[3][1] * colVector[1] + mat4[3][2] * colVector[2] + mat4[3][3] * colVector[3];

	const Matrix4x4::row_type resultRow(row0, row1, row2, row3);
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
inline SMatrix4x4 operator+(const SMatrix4x4& mat, const GLfloat& fVal)
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
inline SMatrix4x4 operator+(const GLfloat& fVal, const SMatrix4x4& mat)
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
inline SMatrix4x4 operator+(const SMatrix4x4& mat1, const SMatrix4x4& mat2)
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
inline SMatrix4x4 operator-(const SMatrix4x4& mat, const GLfloat& fVal)
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
inline SMatrix4x4 operator-(const GLfloat& fVal, const SMatrix4x4& mat)
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
inline SMatrix4x4 operator-(const SMatrix4x4& mat1, const SMatrix4x4& mat2)
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
inline SMatrix4x4 operator*(const SMatrix4x4& mat, const GLfloat& fVal)
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
inline SMatrix4x4 operator*(const GLfloat& fVal, const SMatrix4x4& mat)
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
inline SMatrix4x4 operator/(const SMatrix4x4& mat, const GLfloat& fVal)
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
inline SMatrix4x4 operator/(const GLfloat& fVal, const SMatrix4x4& mat)
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
inline SMatrix4x4::col_type operator/(const SMatrix4x4& mat, const SMatrix4x4::row_type& rowVec)
{
	SMatrix4x4 InversedMat{};
	InversedMat.InverseSub(mat);
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
inline SMatrix4x4::row_type operator/(const SMatrix4x4::col_type& colVec, const SMatrix4x4& mat)
{
	SMatrix4x4 InversedMat{};
	InversedMat.InverseSub(mat);
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
inline SMatrix4x4 operator/(const SMatrix4x4& mat1, const SMatrix4x4& mat2)
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
inline SMatrix4x4 operator*(const SMatrix4x4& mat1, const SMatrix4x4& mat2)
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
 
/**
 * @brief Calculates the inverse of the current matrix using the cofactor method.
 *
 * This non-member helper function computes the matrix inverse by finding cofactors,
 * calculating the adjugate, and multiplying by the inverse of the determinant.
 * It is structured to return the inverse of *this* matrix.
 *
 * @return A new SMatrix4x4 object representing the inverse of the current matrix.
 */
inline SMatrix4x4 SMatrix4x4::InverseSub()
{
	const GLfloat Coef00 = value[2][2] * value[3][3] - value[3][2] * value[2][3];
	const GLfloat Coef02 = value[1][2] * value[3][3] - value[3][2] * value[1][3];
	const GLfloat Coef03 = value[1][2] * value[2][3] - value[2][2] * value[1][3];

	const GLfloat Coef04 = value[2][1] * value[3][3] - value[3][1] * value[2][3];
	const GLfloat Coef06 = value[1][1] * value[3][3] - value[3][1] * value[1][3];
	const GLfloat Coef07 = value[1][1] * value[2][3] - value[2][1] * value[1][3];

	const GLfloat Coef08 = value[2][1] * value[3][2] - value[3][1] * value[2][3];
	const GLfloat Coef10 = value[1][1] * value[3][2] - value[3][1] * value[1][2];
	const GLfloat Coef11 = value[1][1] * value[2][2] - value[2][1] * value[1][2];

	const GLfloat Coef12 = value[2][0] * value[3][3] - value[3][0] * value[2][3];
	const GLfloat Coef14 = value[1][0] * value[3][3] - value[3][0] * value[1][3];
	const GLfloat Coef15 = value[1][0] * value[2][3] - value[2][0] * value[1][3];

	const GLfloat Coef16 = value[2][0] * value[3][2] - value[3][0] * value[2][2];
	const GLfloat Coef18 = value[1][0] * value[3][2] - value[3][0] * value[1][2];
	const GLfloat Coef19 = value[1][0] * value[2][2] - value[2][0] * value[1][2];

	const GLfloat Coef20 = value[2][0] * value[3][1] - value[3][0] * value[2][1];
	const GLfloat Coef22 = value[1][0] * value[3][1] - value[3][0] * value[1][1];
	const GLfloat Coef23 = value[1][0] * value[2][1] - value[2][0] * value[1][1];

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

	const GLfloat Dot1 = (Dot0.x + Dot0.y) + (Dot0.z + Dot0.w);

	const GLfloat OneOverDeterminant = 1.0f / Dot1;

	return Inverse * OneOverDeterminant;
}

/**
 * @brief Calculates the inverse of an arbitrary matrix using the cofactor method.
 *
 * This non-member helper function computes the matrix inverse by finding cofactors,
 * calculating the adjugate, and multiplying by the inverse of the determinant.
 *
 * @param mat4 The matrix to be inverted.
 * @return A new SMatrix4x4 object representing the inverse of mat4.
 */
inline SMatrix4x4 SMatrix4x4::InverseSub(const SMatrix4x4& mat4)
{
	const GLfloat Coef00 = mat4.value[2][2] * mat4.value[3][3] - mat4.value[3][2] * mat4.value[2][3];
	const GLfloat Coef02 = mat4.value[1][2] * mat4.value[3][3] - mat4.value[3][2] * mat4.value[1][3];
	const GLfloat Coef03 = mat4.value[1][2] * mat4.value[2][3] - mat4.value[2][2] * mat4.value[1][3];

	const GLfloat Coef04 = mat4.value[2][1] * mat4.value[3][3] - mat4.value[3][1] * mat4.value[2][3];
	const GLfloat Coef06 = mat4.value[1][1] * mat4.value[3][3] - mat4.value[3][1] * mat4.value[1][3];
	const GLfloat Coef07 = mat4.value[1][1] * mat4.value[2][3] - mat4.value[2][1] * mat4.value[1][3];

	const GLfloat Coef08 = mat4.value[2][1] * mat4.value[3][2] - mat4.value[3][1] * mat4.value[2][3];
	const GLfloat Coef10 = mat4.value[1][1] * mat4.value[3][2] - mat4.value[3][1] * mat4.value[1][2];
	const GLfloat Coef11 = mat4.value[1][1] * mat4.value[2][2] - mat4.value[2][1] * mat4.value[1][2];

	const GLfloat Coef12 = mat4.value[2][0] * mat4.value[3][3] - mat4.value[3][0] * mat4.value[2][3];
	const GLfloat Coef14 = mat4.value[1][0] * mat4.value[3][3] - mat4.value[3][0] * mat4.value[1][3];
	const GLfloat Coef15 = mat4.value[1][0] * mat4.value[2][3] - mat4.value[2][0] * mat4.value[1][3];

	const GLfloat Coef16 = mat4.value[2][0] * mat4.value[3][2] - mat4.value[3][0] * mat4.value[2][2];
	const GLfloat Coef18 = mat4.value[1][0] * mat4.value[3][2] - mat4.value[3][0] * mat4.value[1][2];
	const GLfloat Coef19 = mat4.value[1][0] * mat4.value[2][2] - mat4.value[2][0] * mat4.value[1][2];

	const GLfloat Coef20 = mat4.value[2][0] * mat4.value[3][1] - mat4.value[3][0] * mat4.value[2][1];
	const GLfloat Coef22 = mat4.value[1][0] * mat4.value[3][1] - mat4.value[3][0] * mat4.value[1][1];
	const GLfloat Coef23 = mat4.value[1][0] * mat4.value[2][1] - mat4.value[2][0] * mat4.value[1][1];

	const SVector4Df Fac0(Coef00, Coef00, Coef02, Coef03);
	const SVector4Df Fac1(Coef04, Coef04, Coef06, Coef07);
	const SVector4Df Fac2(Coef08, Coef08, Coef10, Coef11);
	const SVector4Df Fac3(Coef12, Coef12, Coef14, Coef15);
	const SVector4Df Fac4(Coef16, Coef16, Coef18, Coef19);
	const SVector4Df Fac5(Coef20, Coef20, Coef22, Coef23);

	const SVector4Df Vec0(mat4.value[1][0], mat4.value[0][0], mat4.value[0][0], mat4.value[0][0]);
	const SVector4Df Vec1(mat4.value[1][1], mat4.value[0][1], mat4.value[0][1], mat4.value[0][1]);
	const SVector4Df Vec2(mat4.value[1][2], mat4.value[0][2], mat4.value[0][2], mat4.value[0][2]);
	const SVector4Df Vec3(mat4.value[1][3], mat4.value[0][3], mat4.value[0][3], mat4.value[0][3]);

	const SVector4Df Inv0(Vec1 * Fac0 - Vec2 * Fac1 + Vec3 * Fac2);
	const SVector4Df Inv1(Vec0 * Fac0 - Vec2 * Fac3 + Vec3 * Fac4);
	const SVector4Df Inv2(Vec0 * Fac1 - Vec1 * Fac3 + Vec3 * Fac5);
	const SVector4Df Inv3(Vec0 * Fac2 - Vec1 * Fac4 + Vec2 * Fac5);

	const SVector4Df SignA(+1.0f, -1.0f, +1.0f, -1.0f);
	const SVector4Df SignB(-1.0f, +1.0f, -1.0f, +1.0f);

	SMatrix4x4 Inverse(Inv0 * SignA, Inv1 * SignB, Inv2 * SignA, Inv3 * SignB);

	const SVector4Df Row0(Inverse.value[0][0], Inverse.value[1][0], Inverse.value[2][0], Inverse.value[3][0]);

	const SVector4Df Dot0 = SVector4Df(mat4.value[0]) * Row0;

	const GLfloat Dot1 = (Dot0.x + Dot0.y) + (Dot0.z + Dot0.w);

	const GLfloat OneOverDeterminant = 1.0f / Dot1;

	return Inverse * OneOverDeterminant;
}

typedef Matrix2x2 Matrix2;
typedef Matrix3x3 Matrix3;
typedef Matrix4x4 Matrix4;

typedef struct STerrainVertex
{
	Vector3D m_v3Position;		// World position
	Vector2D m_v2TexCoords;		// UVs (For Texturing)
	Vector3D m_v3Normals;		// Normal

	STerrainVertex()
	{
		m_v3Position = 0.0f;
		m_v2TexCoords = 0.0f;
		m_v3Normals = 0.0f;
	}

	STerrainVertex(GLfloat fX, GLfloat fY, GLfloat fZ)
	{
		m_v3Position.x = fX;
		m_v3Position.y = fY;
		m_v3Position.z = fZ;

		m_v2TexCoords = 0.0f;
		m_v3Normals = 0.0f;
	}
} TerrainVertex;