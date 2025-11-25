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

typedef struct SMatrix4x4
{
	GLfloat mat[4][4];

	/**
	 * Default constructor, initializes all elements to zero.
	 */
	SMatrix4x4() = default;

	/**
	 * Constructor that initializes the matrix with specific values.
	 *
	 * @param a00, a01, ..., a33: Individual matrix elements.
	 */
	SMatrix4x4(const GLfloat a00, const GLfloat a01, const GLfloat a02, const GLfloat a03,
		const GLfloat a10, const GLfloat a11, const GLfloat a12, const GLfloat a13,
		const GLfloat a20, const GLfloat a21, const GLfloat a22, const GLfloat a23,
		const GLfloat a30, const GLfloat a31, const GLfloat a32, const GLfloat a33)
	{
		mat[0][0] = a00; mat[0][1] = a01; mat[0][2] = a02; mat[0][3] = a03;
		mat[1][0] = a10; mat[1][1] = a11; mat[1][2] = a12; mat[1][3] = a13;
		mat[2][0] = a20; mat[2][1] = a21; mat[2][2] = a22; mat[2][3] = a23;
		mat[3][0] = a30; mat[3][1] = a31; mat[3][2] = a32; mat[3][3] = a33;
	}

	/**
	 * Copy constructor.
	 *
	 * @param Mat: The matrix to copy.
	 */
	SMatrix4x4(const SMatrix4x4& Mat)
	{
		mat[0][0] = Mat.mat[0][0]; mat[0][1] = Mat.mat[0][1]; mat[0][2] = Mat.mat[0][2]; mat[0][3] = Mat.mat[0][3];
		mat[1][0] = Mat.mat[1][0]; mat[1][1] = Mat.mat[1][1]; mat[1][2] = Mat.mat[1][2]; mat[1][3] = Mat.mat[1][3];
		mat[2][0] = Mat.mat[2][0]; mat[2][1] = Mat.mat[2][1]; mat[2][2] = Mat.mat[2][2]; mat[2][3] = Mat.mat[2][3];
		mat[3][0] = Mat.mat[3][0]; mat[3][1] = Mat.mat[3][1]; mat[3][2] = Mat.mat[3][2]; mat[3][3] = Mat.mat[3][3];
	}

	/**
	 * Copy constructor.
	 *
	 * @param vec1: 4D Vector to copy for first row.
	 * @param vec2: 4D Vector to copy for second row.
	 * @param vec3: 4D Vector to copy for third row.
	 * @param vec4: 4D Vector to copy for fourth row.
	 */
	SMatrix4x4(const SVector4Df& vec1, const SVector4Df& vec2, const SVector4Df& vec3, const SVector4Df& vec4)
	{
		mat[0][0] = vec1.x; mat[0][1] = vec1.y; mat[0][2] = vec1.z; mat[0][3] = vec1.w;
		mat[1][0] = vec2.x; mat[1][1] = vec2.y; mat[1][2] = vec2.z; mat[1][3] = vec2.w;
		mat[2][0] = vec3.x; mat[2][1] = vec3.y; mat[2][2] = vec3.z; mat[2][3] = vec3.w;
		mat[3][0] = vec4.x; mat[3][1] = vec4.y; mat[3][2] = vec4.z; mat[3][3] = vec4.w;
	}

	/**
	 * Constructor that initializes the matrix from a GLM matrix.
	 *
	 * @param glmMat: A GLM matrix to copy from.
	 */
	SMatrix4x4(const glm::mat4& glmMat)
	{
		mat[0][0] = glmMat[0][0]; mat[0][1] = glmMat[0][1]; mat[0][2] = glmMat[0][2]; mat[0][3] = glmMat[0][3];
		mat[1][0] = glmMat[1][0]; mat[1][1] = glmMat[1][1]; mat[1][2] = glmMat[1][2]; mat[1][3] = glmMat[1][3];
		mat[2][0] = glmMat[2][0]; mat[2][1] = glmMat[2][1]; mat[2][2] = glmMat[2][2]; mat[2][3] = glmMat[2][3];
		mat[3][0] = glmMat[3][0]; mat[3][1] = glmMat[3][1]; mat[3][2] = glmMat[3][2]; mat[3][3] = glmMat[3][3];
	}

	/**
	 * Overload the `[]` operator to access rows of the matrix.
	 *
	 * @param row: Row index (0-3).
	 * @return Reference to the array representing the row.
	 */
	float* operator[](size_t row)
	{
		if (row >= 4)
		{
			throw std::out_of_range("Row index out of bounds");
		}
		return mat[row];
	}

	/**
	 * Multiplies two SMatrix4x4 matrices.
	 *
	 * This operator overloads the multiplication operator for SMatrix4x4 objects,
	 * performing matrix multiplication.
	 *
	 * -Important: the correct order of multiplication is Translation * (Rotation * (Scale * Position)) by order to get the final transform
	 *
	 * @param rightMat: The right-hand side matrix.
	 *
	 * @return The product of the two matrices.
	 */
	SMatrix4x4 operator*(const SMatrix4x4& rightMat)
	{
		SMatrix4x4 resultMat{};
		for (GLsizei i = 0; i < 4; i++)
		{
			for (GLsizei j = 0; j < 4; j++)
			{
				resultMat.mat[i][j] =
					mat[i][0] * rightMat.mat[0][j] +
					mat[i][1] * rightMat.mat[1][j] +
					mat[i][2] * rightMat.mat[2][j] +
					mat[i][3] * rightMat.mat[3][j];
			}
		}
		return (resultMat);
	}

	/**
	 * Multiplies a SMatrix4x4 and an SVector4Df.
	 *
	 * This operator overloads the multiplication operator for SMatrix4x4 and SVector4Df objects,
	 * performing matrix-vector multiplication.
	 *
	 * @param vec: The vector to multiply.
	 *
	 * @return The product of the matrix and the vector.
	 */
	SVector4Df operator*(const SVector4Df& vec)
	{
		SVector4Df newVec{};

		newVec.x = mat[0][0] * vec.x + mat[0][1] * vec.y + mat[0][2] * vec.z + mat[0][3] * vec.w;
		newVec.y = mat[1][0] * vec.x + mat[1][1] * vec.y + mat[1][2] * vec.z + mat[1][3] * vec.w;
		newVec.z = mat[2][0] * vec.x + mat[2][1] * vec.y + mat[2][2] * vec.z + mat[2][3] * vec.w;
		newVec.w = mat[3][0] * vec.x + mat[3][1] * vec.y + mat[3][2] * vec.z + mat[3][3] * vec.w;
		return (newVec);
	}

	/**
	 * Multiplies all components of an SMatrix4x4 object by a scalar value.
	 *
	 * @param fVal The scalar value to multiply by.
	 * @return The resulting SMatrix4x4 object.
	 */
	SMatrix4x4 operator*(const GLfloat fScalar)
	{
		SMatrix4x4 result{};
		for (GLsizei i = 0; i < 4; ++i)
		{
			for (GLsizei j = 0; j < 4; ++j)
			{
				result.mat[i][j] = mat[i][j] * fScalar;
			}
		}
		return result;
	}

	/**
	 * returns a reference to the private `mat4` matrix, It allows read-only access to the 4x4 matrix stored in the class
	 *
	 * @return A constant reference to the `mat4` matrix.
	 */
	const GLfloat(&GetMatrix() const)[4][4]
	{
		return mat;
	}

		/**
		 * Takes a 4x4 array of floats and updates the private `mat4` matrix with the provided values.
		 *
		 * @param values: A 4x4 array containing the new values for the matrix.
		 */
		void SetMatrix(const GLfloat values[4][4])
	{
		for (GLsizei i = 0; i < 4; ++i)
		{
			for (GLsizei j = 0; j < 4; ++j)
			{
				mat[i][j] = values[i][j];
			}
		}
	}

	/**
	 * Transposes the matrix.
	 *
	 * This function returns a new matrix that is the transpose of the current matrix.
	 *
	 * @return The transposed matrix.
	 */
	SMatrix4x4 Transpose() const
	{
		SMatrix4x4 newMat{};

		for (GLsizei i = 0; i < 4; i++)
		{
			for (GLsizei j = 0; j < 4; j++)
			{
				newMat.mat[i][j] = mat[j][i];
			}
		}

		return (newMat);
	}

	/**
	 * Calculates the determinant of the matrix.
	 *
	 * This function calculates the determinant of the 4x4 matrix using a direct formula.
	 *
	 * @return The determinant of the matrix.
	 */
	GLfloat Determinant() const
	{
		const GLfloat fDet
			= mat[0][0] * mat[1][1] * mat[2][2] * mat[3][3] - mat[0][0] * mat[1][1] * mat[2][3] * mat[3][2] + mat[0][0] * mat[1][2] * mat[2][3] * mat[3][1] - mat[0][0] * mat[1][2] * mat[2][1] * mat[3][3]
			+ mat[0][0] * mat[1][3] * mat[2][1] * mat[3][2] - mat[0][0] * mat[1][3] * mat[2][2] * mat[3][1] - mat[0][1] * mat[1][2] * mat[2][3] * mat[3][0] + mat[0][1] * mat[1][2] * mat[2][0] * mat[3][3]
			- mat[0][1] * mat[1][3] * mat[2][0] * mat[3][2] + mat[0][1] * mat[1][3] * mat[2][2] * mat[3][0] - mat[0][1] * mat[1][0] * mat[2][2] * mat[3][3] + mat[0][1] * mat[1][0] * mat[2][3] * mat[3][2]
			+ mat[0][2] * mat[1][3] * mat[2][0] * mat[3][1] - mat[0][2] * mat[1][3] * mat[2][1] * mat[3][0] + mat[0][2] * mat[1][0] * mat[2][1] * mat[3][3] - mat[0][2] * mat[1][0] * mat[2][3] * mat[3][1]
			+ mat[0][2] * mat[1][1] * mat[2][3] * mat[3][0] - mat[0][2] * mat[1][1] * mat[2][0] * mat[3][3] - mat[0][3] * mat[1][0] * mat[2][1] * mat[3][2] + mat[0][3] * mat[1][0] * mat[2][2] * mat[3][1]
			- mat[0][3] * mat[1][1] * mat[2][2] * mat[3][0] + mat[0][3] * mat[1][1] * mat[2][0] * mat[3][2] - mat[0][3] * mat[1][2] * mat[2][0] * mat[3][2] + mat[0][3] * mat[1][2] * mat[2][1] * mat[3][0];

		return (fDet);
	}

	/**
	 * Calculates the determinant of a 3x3 submatrix, Calcualted as Same as GLM Function.
	 *
	 * This function calculates the determinant of a 3x3 submatrix, which is a crucial
	 * step in the cofactor expansion method for computing the determinant of a 4x4 matrix.
	 *
	 * @return The determinant of the 3x3 submatrix.
	 */
	GLfloat DeterminantSub() const
	{
		const GLfloat SubFactor00 = mat[2][2] * mat[3][3] - mat[3][2] * mat[2][3];
		const GLfloat SubFactor01 = mat[2][1] * mat[3][3] - mat[3][1] * mat[2][3];
		const GLfloat SubFactor02 = mat[2][1] * mat[3][2] - mat[3][1] * mat[2][2];
		const GLfloat SubFactor03 = mat[2][0] * mat[3][3] - mat[3][0] * mat[2][3];
		const GLfloat SubFactor04 = mat[2][0] * mat[3][2] - mat[3][0] * mat[2][2];
		const GLfloat SubFactor05 = mat[2][0] * mat[3][1] - mat[3][0] * mat[2][1];

		SVector4Df DetCof(
			+(mat[1][1] * SubFactor00 - mat[1][2] * SubFactor01 + mat[1][3] * SubFactor02),
			-(mat[1][0] * SubFactor00 - mat[1][2] * SubFactor03 + mat[1][3] * SubFactor04),
			+(mat[1][0] * SubFactor01 - mat[1][1] * SubFactor03 + mat[1][3] * SubFactor05),
			-(mat[1][0] * SubFactor02 - mat[1][1] * SubFactor04 + mat[1][2] * SubFactor05)
		);

		return mat[0][0] * DetCof[0] + mat[0][1] * DetCof[1] + mat[0][2] * DetCof[2] + mat[0][3] * DetCof[3];
	}

	/**
	 * Calculates the inverse of the matrix.
	 *
	 * This function calculates the inverse of the 4x4 matrix using the adjugate matrix and the determinant.
	 *
	 * @return The inverse of the matrix.
	 */
	SMatrix4x4 Inverse() const
	{
		// Compute the reciprocal determinant first
		const GLfloat det = Determinant();

		if (det == 0.0f)
		{
			// ASSERT(det == 0.0f, "Matrix Determinant Is 0");
			return (*this);
		}

		// Calculate the inverse determinant
		const GLfloat fInvDet = 1.0f / det;

		// Calculate the adjugate matrix (transpose of the cofactor matrix) (https://byjus.com/maths/inverse-matrix/)
		SMatrix4x4 res{};

		// Calcualte First Element --- Inverse Is Positive
		res.mat[0][0] = fInvDet * (mat[1][1] * (mat[2][2] * mat[3][3] - mat[2][3] * mat[3][2]) + mat[1][2] *
			(mat[2][3] * mat[3][1] - mat[2][1] * mat[3][3]) + mat[1][3] * (mat[2][1] * mat[3][2] - mat[2][2] * mat[3][1]));

		// Same as the previous just switch the [1][x] to [0][x] --- Inverse Is Negative
		res.mat[0][1] = -fInvDet * (mat[0][1] * (mat[2][2] * mat[3][3] - mat[2][3] * mat[3][2]) + mat[0][2] *
			(mat[2][3] * mat[3][1] - mat[2][1] * mat[3][3]) + mat[0][3] * (mat[2][1] * mat[3][2] - mat[2][2] * mat[3][1]));

		// Same as the previous just switch the [2][x] to [1][x] --- Inverse Is Positive
		res.mat[0][2] = fInvDet * (mat[0][1] * (mat[1][2] * mat[3][3] - mat[1][3] * mat[3][2]) + mat[0][2] *
			(mat[1][3] * mat[3][1] - mat[1][1] * mat[3][3]) + mat[0][3] * (mat[1][1] * mat[3][2] - mat[1][2] * mat[3][1]));

		// Same as the previous just switch the [3][x] to [2][x] --- Inverse Is Negative
		res.mat[0][3] = -fInvDet * (mat[0][1] * (mat[1][2] * mat[2][3] - mat[1][3] * mat[2][2]) + mat[0][2] *
			(mat[1][3] * mat[2][1] - mat[1][1] * mat[2][3]) + mat[0][3] * (mat[1][1] * mat[2][2] - mat[1][2] * mat[2][1]));

		// Same as the FIRST just switch the [x][1] to [x][0] --- Inverse Is Negative
		res.mat[1][0] = -fInvDet * (mat[1][0] * (mat[2][2] * mat[3][3] - mat[2][3] * mat[3][2]) + mat[1][2] *
			(mat[2][3] * mat[3][0] - mat[2][0] * mat[3][3]) + mat[1][3] * (mat[2][0] * mat[3][2] - mat[2][2] * mat[3][0]));

		// Same as the previous just switch the [1][x] to [0][x] --- Inverse Is Positive
		res.mat[1][1] = fInvDet * (mat[0][0] * (mat[2][2] * mat[3][3] - mat[2][3] * mat[3][2]) + mat[0][2] *
			(mat[2][3] * mat[3][0] - mat[2][0] * mat[3][3]) + mat[0][3] * (mat[2][0] * mat[3][2] - mat[2][2] * mat[3][0]));

		// Same as the previous just switch the [2][x] to [1][x] --- Inverse Is Negative
		res.mat[1][2] = -fInvDet * (mat[0][0] * (mat[1][2] * mat[3][3] - mat[1][3] * mat[3][2]) + mat[0][2] *
			(mat[1][3] * mat[3][0] - mat[1][0] * mat[3][3]) + mat[0][3] * (mat[1][0] * mat[3][2] - mat[1][2] * mat[3][0]));

		// Same as the previous just switch the [3][x] to [2][x] --- Inverse Is Positive
		res.mat[1][3] = fInvDet * (mat[0][0] * (mat[1][2] * mat[2][3] - mat[1][3] * mat[2][2]) + mat[0][2] *
			(mat[1][3] * mat[2][0] - mat[1][0] * mat[2][3]) + mat[0][3] * (mat[1][0] * mat[2][2] - mat[1][2] * mat[2][0]));

		// Same as [1][0] ELEMENT calculation, just switch the [x][2] to [x][1] --- Inverse Is Positive
		res.mat[2][0] = fInvDet * (mat[1][0] * (mat[2][1] * mat[3][3] - mat[2][3] * mat[3][1]) + mat[1][1] *
			(mat[2][3] * mat[3][0] - mat[2][0] * mat[3][3]) + mat[1][3] * (mat[2][0] * mat[3][1] - mat[2][1] * mat[3][0]));

		// Same as the previous just switch the [1][x] to [0][x] --- Inverse Is Negative
		res.mat[2][1] = -fInvDet * (mat[0][0] * (mat[2][1] * mat[3][3] - mat[2][3] * mat[3][1]) + mat[0][1] *
			(mat[2][3] * mat[3][0] - mat[2][0] * mat[3][3]) + mat[0][3] * (mat[2][0] * mat[3][1] - mat[2][1] * mat[3][0]));

		// Same as the previous just switch the [2][x] to [1][x] --- Inverse Is Positive
		res.mat[2][2] = fInvDet * (mat[0][0] * (mat[1][1] * mat[3][3] - mat[1][3] * mat[3][1]) + mat[0][1] *
			(mat[1][3] * mat[3][0] - mat[1][0] * mat[3][3]) + mat[0][3] * (mat[1][0] * mat[3][1] - mat[1][1] * mat[3][0]));

		// Same as the previous just switch the [3][x] to [2][x] --- Inverse Is Negative
		res.mat[2][3] = -fInvDet * (mat[0][0] * (mat[1][1] * mat[2][3] - mat[1][3] * mat[2][1]) + mat[0][1] *
			(mat[1][3] * mat[2][0] - mat[1][0] * mat[2][3]) + mat[0][3] * (mat[1][0] * mat[2][1] - mat[1][1] * mat[2][0]));

		// Same as [2][0] ELEMENT calculation, just switch the [x][3] to [x][2] --- Inverse Is Negative
		res.mat[3][0] = -fInvDet * (mat[1][0] * (mat[2][1] * mat[3][2] - mat[2][2] * mat[3][1]) + mat[1][1] *
			(mat[2][2] * mat[3][0] - mat[2][0] * mat[3][2]) + mat[1][2] * (mat[2][0] * mat[3][1] - mat[2][1] * mat[3][0]));

		// Same as the previous just switch the [1][x] to [0][x] --- Inverse Is Positive
		res.mat[3][1] = fInvDet * (mat[0][0] * (mat[2][1] * mat[3][2] - mat[2][2] * mat[3][1]) + mat[0][1] *
			(mat[2][2] * mat[3][0] - mat[2][0] * mat[3][2]) + mat[0][2] * (mat[2][0] * mat[3][1] - mat[2][1] * mat[3][0]));

		// Same as the previous just switch the [2][x] to [1][x] --- Inverse Is Negative
		res.mat[3][2] = -fInvDet * (mat[0][0] * (mat[1][1] * mat[3][2] - mat[1][2] * mat[3][1]) + mat[0][1] *
			(mat[1][2] * mat[3][0] - mat[1][0] * mat[3][2]) + mat[0][2] * (mat[2][0] * mat[3][1] - mat[1][1] * mat[3][0]));

		// Same as the previous just switch the [3][x] to [2][x] --- Inverse Is Positive
		res.mat[3][3] = fInvDet * (mat[0][0] * (mat[1][1] * mat[2][2] - mat[1][2] * mat[2][1]) + mat[0][1] *
			(mat[1][2] * mat[3][0] - mat[1][0] * mat[2][2]) + mat[0][2] * (mat[2][0] * mat[2][1] - mat[1][1] * mat[2][0]));

		// The Pattern -> Previous Calculation (make sure of fInvDet sign) and each time make [x][n] = [x-1][n] , ex: [3][2] -> [2][2], [2][2] -> [1][2], [1][2] -> [0][2]
		return (res);
	}

	/**
	 * Calculates the inverse of the matrix, Calcualted as Same as GLM Function.
	 *
	 * This function calculates the inverse of the 4x4 matrix using the adjugate matrix and the determinant.
	 *
	 * @return The inverse of the matrix.
	 */
	SMatrix4x4 InverseSub() const
	{
		const GLfloat Coef00 = mat[2][2] * mat[3][3] - mat[3][2] * mat[2][3];
		const GLfloat Coef02 = mat[1][2] * mat[3][3] - mat[3][2] * mat[1][3];
		const GLfloat Coef03 = mat[1][2] * mat[2][3] - mat[2][2] * mat[1][3];

		const GLfloat Coef04 = mat[2][1] * mat[3][3] - mat[3][1] * mat[2][3];
		const GLfloat Coef06 = mat[1][1] * mat[3][3] - mat[3][1] * mat[1][3];
		const GLfloat Coef07 = mat[1][1] * mat[2][3] - mat[2][1] * mat[1][3];

		const GLfloat Coef08 = mat[2][1] * mat[3][2] - mat[3][1] * mat[2][3];
		const GLfloat Coef10 = mat[1][1] * mat[3][2] - mat[3][1] * mat[1][2];
		const GLfloat Coef11 = mat[1][1] * mat[2][2] - mat[2][1] * mat[1][2];

		const GLfloat Coef12 = mat[2][0] * mat[3][3] - mat[3][0] * mat[2][3];
		const GLfloat Coef14 = mat[1][0] * mat[3][3] - mat[3][0] * mat[1][3];
		const GLfloat Coef15 = mat[1][0] * mat[2][3] - mat[2][0] * mat[1][3];

		const GLfloat Coef16 = mat[2][0] * mat[3][2] - mat[3][0] * mat[2][2];
		const GLfloat Coef18 = mat[1][0] * mat[3][2] - mat[3][0] * mat[1][2];
		const GLfloat Coef19 = mat[1][0] * mat[2][2] - mat[2][0] * mat[1][2];

		const GLfloat Coef20 = mat[2][0] * mat[3][1] - mat[3][0] * mat[2][1];
		const GLfloat Coef22 = mat[1][0] * mat[3][1] - mat[3][0] * mat[1][1];
		const GLfloat Coef23 = mat[1][0] * mat[2][1] - mat[2][0] * mat[1][1];

		const SVector4Df Fac0(Coef00, Coef00, Coef02, Coef03);
		const SVector4Df Fac1(Coef04, Coef04, Coef06, Coef07);
		const SVector4Df Fac2(Coef08, Coef08, Coef10, Coef11);
		const SVector4Df Fac3(Coef12, Coef12, Coef14, Coef15);
		const SVector4Df Fac4(Coef16, Coef16, Coef18, Coef19);
		const SVector4Df Fac5(Coef20, Coef20, Coef22, Coef23);

		const SVector4Df Vec0(mat[1][0], mat[0][0], mat[0][0], mat[0][0]);
		const SVector4Df Vec1(mat[1][1], mat[0][1], mat[0][1], mat[0][1]);
		const SVector4Df Vec2(mat[1][2], mat[0][2], mat[0][2], mat[0][2]);
		const SVector4Df Vec3(mat[1][3], mat[0][3], mat[0][3], mat[0][3]);

		const SVector4Df Inv0(Vec1 * Fac0 - Vec2 * Fac1 + Vec3 * Fac2);
		const SVector4Df Inv1(Vec0 * Fac0 - Vec2 * Fac3 + Vec3 * Fac4);
		const SVector4Df Inv2(Vec0 * Fac1 - Vec1 * Fac3 + Vec3 * Fac5);
		const SVector4Df Inv3(Vec0 * Fac2 - Vec1 * Fac4 + Vec2 * Fac5);

		const SVector4Df SignA(+1, -1, +1, -1);
		const SVector4Df SignB(-1, +1, -1, +1);

		SMatrix4x4 Inverse(Inv0 * SignA, Inv1 * SignB, Inv2 * SignA, Inv3 * SignB);

		const SVector4Df Row0(Inverse.mat[0][0], Inverse.mat[1][0], Inverse.mat[2][0], Inverse.mat[3][0]);

		const SVector4Df Dot0 = SVector4Df(mat[0]) * Row0;

		const GLfloat Dot1 = (Dot0.x + Dot0.y) + (Dot0.z + Dot0.w);

		const GLfloat OneOverDeterminant = 1.0f / Dot1;

		return Inverse * OneOverDeterminant;
	}

	/**
	 * InverseGJ - Calculates the inverse of a matrix using the Gauss-Jordan method.
	 *
	 * This function computes the inverse of the provided 4x4 matrix using the
	 * Gauss-Jordan elimination method. If the matrix is singular (non-invertible),
	 * the behavior is undefined or may result in a failure to calculate the inverse.
	 *
	 * The Gauss-Jordan method involves augmenting the input matrix with the identity
	 * matrix and performing row operations to reduce the input matrix to the identity
	 * matrix, simultaneously transforming the identity matrix into the inverse.
	 *
	 * @param inputMatrix: The matrix to be inverted.
	 * @return A CMatrix4Df object representing the inverse of the input matrix.
	 */
	SMatrix4x4 InverseGJ(const SMatrix4x4& inputMatrix)
	{
		SMatrix4x4 inverseMatrix{};
		GLfloat augmented[4][8] = { 0 };

		// Get the input matrix
		const GLfloat(&mat)[4][4] = inputMatrix.GetMatrix();
		// const GLfloat(* const mat)[4] = inputMatrix.GetMatrix();

		// Create the augmented matrix
		for (GLsizei i = 0; i < 4; i++)
		{
			for (GLsizei j = 0; j < 4; j++)
			{
				augmented[i][j] = mat[i][j]; // Copy the original matrix
			}
			augmented[i][i + 4] = 1; // Add the identity matrix
		}

		// Perform Gaussian elimination
		for (GLsizei i = 0; i < 4; i++)
		{
			// Normalize the current row
			GLfloat diagElement = augmented[i][i];
			for (GLsizei j = 0; j < 8; j++)
			{
				augmented[i][j] /= diagElement;
			}

			// Eliminate other rows
			for (GLsizei k = 0; k < 4; k++)
			{
				if (k != i)
				{
					GLfloat factor = augmented[k][i];
					for (GLsizei j = 0; j < 8; j++)
					{
						augmented[k][j] -= factor * augmented[i][j];
					}
				}
			}
		}

		// Extract the inverse matrix
		GLfloat result[4][4] = { 0 };
		for (GLsizei i = 0; i < 4; i++)
		{
			for (GLsizei j = 0; j < 4; j++)
			{
				result[i][j] = augmented[i][j + 4];
			}
		}

		// Set the result to the inverse matrix object
		inverseMatrix.SetMatrix(result);
		return inverseMatrix;
	}

	// transforms a 3D point (SVector3Df) by the 4x4 matrix (including translation, rotation, and scale):
	SVector3Df TransformPoint(const SVector3Df& v) const
	{
		// Assuming mat4 is a 4x4 float array: mat4[row][col]
		GLfloat x = v.x, y = v.y, z = v.z;
		GLfloat tx = mat[0][0] * x + mat[0][1] * y + mat[0][2] * z + mat[0][3];
		GLfloat ty = mat[1][0] * x + mat[1][1] * y + mat[1][2] * z + mat[1][3];
		GLfloat tz = mat[2][0] * x + mat[2][1] * y + mat[2][2] * z + mat[2][3];
		GLfloat tw = mat[3][0] * x + mat[3][1] * y + mat[3][2] * z + mat[3][3];

		// Homogeneous divide (if w != 1)
		if (tw != 0.0f && tw != 1.0f)
		{
			tx /= tw;
			ty /= tw;
			tz /= tw;
		}
		return SVector3Df(tx, ty, tz);
	}

	/**
	 * Initializes the matrix to zero using memset.
	 *
	 * This function efficiently sets all elements of the matrix to zero using the `memset` function.
	 */
	void InitMemZero()
	{
		memset(mat, 0, sizeof(mat));
	}

	/**
	 * InitZero: Initializes the matrix to zero element-wise.
	 *
	 * This function sets all elements of the matrix to zero using a nested loop.
	 */
	void InitZero()
	{
		for (GLsizei i = 0; i < 4; i++)
		{
			for (GLsizei j = 0; j < 4; j++)
			{
				mat[i][j] = 0.0f;
			}
		}
	}

	/**
	 * Initializes the matrix with a specified number.
	 *
	 * This function initializes the matrix elements by the specified `fNum` value.
	 *
	 * @param fNum: The value for each matrix element.
	 */
	void InitNum(const GLfloat fNum)
	{
		for (GLsizei i = 0; i < 4; i++)
		{
			for (GLsizei j = 0; j < 4; j++)
			{
				mat[i][j] = fNum;
			}
		}
	}

	/**
	 * Initializes the matrix to the identity matrix.
	 *
	 * This function sets the matrix to the identity matrix, which has ones on the
	 * diagonal and zeros elsewhere.
	 */
	void InitIdentity()
	{
		mat[0][0] = 1.0f; mat[0][1] = 0.0f; mat[0][2] = 0.0f; mat[0][3] = 0.0f;
		mat[1][0] = 0.0f; mat[1][1] = 1.0f; mat[1][2] = 0.0f; mat[1][3] = 0.0f;
		mat[2][0] = 0.0f; mat[2][1] = 0.0f; mat[2][2] = 1.0f; mat[2][3] = 0.0f;
		mat[3][0] = 0.0f; mat[3][1] = 0.0f; mat[3][2] = 0.0f; mat[3][3] = 1.0f;
	}

	/**
	 * Initializes the matrix for translation.
	 *
	 * This function sets up a 4x4 matrix that applies a translation
	 * transformation. The resulting matrix moves objects in 3D space along
	 * the specified axes.
	 *
	 * @param x: The translation offset along the X-axis.
	 * @param y: The translation offset along the Y-axis.
	 * @param z: The translation offset along the Z-axis.
	 */
	void InitTranslationTransform(const GLfloat x, const GLfloat y, const GLfloat z)
	{
		mat[0][0] = 1.0f; mat[0][1] = 0.0f; mat[0][2] = 0.0f; mat[0][3] = x;
		mat[1][0] = 0.0f; mat[1][1] = 1.0f; mat[1][2] = 0.0f; mat[1][3] = y;
		mat[2][0] = 0.0f; mat[2][1] = 0.0f; mat[2][2] = 1.0f; mat[2][3] = z;
		mat[3][0] = 0.0f; mat[3][1] = 0.0f; mat[3][2] = 0.0f; mat[3][3] = 1.0f;
	}

	/**
	 * Initializes the matrix for translation.
	 *
	 * This function sets up a 4x4 matrix that applies a translation
	 * transformation. The resulting matrix moves objects in 3D space along
	 * the specified axes.
	 *
	 * @param vTranslate: The translation offset 3D Vector.
	 */
	void InitTranslationTransform(const SVector3Df& vTranslate)
	{
		InitTranslationTransform(vTranslate.x, vTranslate.y, vTranslate.z);
	}

	/**
	 * Initializes the matrix for translation.
	 *
	 * This function sets up a 4x4 matrix that applies a translation
	 * transformation. The resulting matrix moves objects in 3D space along
	 * the specified axes.
	 *
	 * @param vTranslate: The translation offset 3D Vector GLM.
	 */
	void InitTranslationTransform(const glm::vec3& vTranslate)
	{
		InitTranslationTransform(vTranslate.x, vTranslate.y, vTranslate.z);
	}

	/**
	 * Initializes the matrix for a rotation around the X-axis.
	 *
	 * This function sets up a 4x4 transformation matrix for a rotation
	 * around the X-axis. The matrix can be configured for either left-handed
	 * or right-handed coordinate systems, based on the `bLeftHanded` parameter.
	 *
	 * @param fRotX: The angle of rotation in radians.
	 * @param bLeftHanded: A boolean indicating whether to use a left-handed coordinate system if true.
	 */
	void InitRotationX(const GLfloat fRotX, const bool bLeftHanded = false)
	{
		/* Rotate The Matrix in Left Handed Coordinate System Around X Axis */
		if (bLeftHanded)
		{
			mat[0][0] = 1.0f; mat[0][1] = 0.0f; mat[0][2] = 0.0f; mat[0][3] = 0.0f;
			mat[1][0] = 0.0f; mat[1][1] = std::cosf(fRotX); mat[1][2] = std::sinf(fRotX); mat[1][3] = 0.0f;
			mat[2][0] = 0.0f; mat[2][1] = -std::sinf(fRotX); mat[2][2] = std::cosf(fRotX); mat[2][3] = 0.0f;
			mat[3][0] = 0.0f; mat[3][1] = 0.0f; mat[3][2] = 0.0f; mat[3][3] = 1.0f;
			return;
		}

		mat[0][0] = 1.0f; mat[0][1] = 0.0f; mat[0][2] = 0.0f; mat[0][3] = 0.0f;
		mat[1][0] = 0.0f; mat[1][1] = std::cosf(fRotX); mat[1][2] = -std::sinf(fRotX); mat[1][3] = 0.0f;
		mat[2][0] = 0.0f; mat[2][1] = std::sinf(fRotX); mat[2][2] = std::cosf(fRotX); mat[2][3] = 0.0f;
		mat[3][0] = 0.0f; mat[3][1] = 0.0f; mat[3][2] = 0.0f; mat[3][3] = 1.0f;
	}

	/**
	 * Initializes the matrix for a rotation around the Y-axis.
	 *
	 * This function sets up a 4x4 transformation matrix for a rotation
	 * around the Y-axis. The matrix can be configured for either left-handed
	 * or right-handed coordinate systems, based on the `bLeftHanded` parameter.
	 *
	 * @param fRotY: The angle of rotation in radians.
	 * @param bLeftHanded: A boolean indicating whether to use a left-handed coordinate system if true.
	 */
	void InitRotationY(const GLfloat fRotY, const bool bLeftHanded = false)
	{
		/* Rotate The Matrix in Left Handed Coordinate System Around Y Axis */
		if (bLeftHanded)
		{
			mat[0][0] = std::cosf(fRotY); mat[0][1] = 0.0f; mat[0][2] = -std::sinf(fRotY); mat[0][3] = 0.0f;
			mat[1][0] = 0.0f; mat[1][1] = 1.0f; mat[1][2] = 0.0f; mat[1][3] = 0.0f;
			mat[2][0] = std::sinf(fRotY); mat[2][1] = 0.0f; mat[2][2] = std::cosf(fRotY); mat[2][3] = 0.0f;
			mat[3][0] = 0.0f; mat[3][1] = 0.0f; mat[3][2] = 0.0f; mat[3][3] = 1.0f;
			return;
		}

		mat[0][0] = std::cosf(fRotY); mat[0][1] = 0.0f; mat[0][2] = std::sinf(fRotY); mat[0][3] = 0.0f;
		mat[1][0] = 0.0f; mat[1][1] = 1.0f; mat[1][2] = 0.0f; mat[1][3] = 0.0f;
		mat[2][0] = -std::sinf(fRotY); mat[2][1] = 0.0f; mat[2][2] = std::cosf(fRotY); mat[2][3] = 0.0f;
		mat[3][0] = 0.0f; mat[3][1] = 0.0f; mat[3][2] = 0.0f; mat[3][3] = 1.0f;
	}

	/**
	 * Initializes the matrix for a rotation around the Z-axis.
	 *
	 * This function sets up a 4x4 transformation matrix for a rotation
	 * around the Z-axis. The matrix can be configured for either left-handed
	 * or right-handed coordinate systems, based on the `bLeftHanded` parameter.
	 *
	 * @param fRotZ: The angle of rotation in radians.
	 * @param bLeftHanded: A boolean indicating whether to use a left-handed coordinate system if true.
	 */
	void InitRotationZ(const GLfloat fRotZ, const bool bLeftHanded = false)
	{
		/* Rotate The Matrix in Left Handed Coordinate System Around Z Axis */
		if (bLeftHanded)
		{
			mat[0][0] = std::cosf(fRotZ); mat[0][1] = std::sinf(fRotZ); mat[0][2] = 0.0f; mat[0][3] = 0.0f;
			mat[1][0] = -std::sinf(fRotZ); mat[1][1] = std::cosf(fRotZ); mat[1][2] = 0.0f; mat[1][3] = 0.0f;
			mat[2][0] = 0.0f; mat[2][1] = 0.0f; mat[2][2] = 1.0f; mat[2][3] = 0.0f;
			mat[3][0] = 0.0f; mat[3][1] = 0.0f; mat[3][2] = 0.0f; mat[3][3] = 1.0f;
			return;
		}

		mat[0][0] = std::cosf(fRotZ); mat[0][1] = -std::sinf(fRotZ); mat[0][2] = 0.0f; mat[0][3] = 0.0f;
		mat[1][0] = std::sinf(fRotZ); mat[1][1] = std::cosf(fRotZ); mat[1][2] = 0.0f; mat[1][3] = 0.0f;
		mat[2][0] = 0.0f; mat[2][1] = 0.0f; mat[2][2] = 1.0f; mat[2][3] = 0.0f;
		mat[3][0] = 0.0f; mat[3][1] = 0.0f; mat[3][2] = 0.0f; mat[3][3] = 1.0f;
	}

	/**
	 * Initializes a scaling transformation matrix.
	 *
	 * This function sets up a 4x4 transformation matrix that applies scaling
	 * transformations along the X, Y, and Z axes. The scaling values determine
	 * how much the object is enlarged or shrunk in each respective axis.
	 *
	 * @param fScaleX: Scaling factor along the X-axis.
	 * @param fScaleY: Scaling factor along the Y-axis.
	 * @param fScaleZ: Scaling factor along the Z-axis.
	 */
	void InitScaleTransform(const GLfloat fScaleX, const GLfloat fScaleY, const GLfloat fScaleZ)
	{
		mat[0][0] = fScaleX; mat[0][1] = 0.0f; mat[0][2] = 0.0f; mat[0][3] = 0.0f;
		mat[1][0] = 0.0f; mat[1][1] = fScaleY; mat[1][2] = 0.0f; mat[1][3] = 0.0f;
		mat[2][0] = 0.0f; mat[2][1] = 0.0f; mat[2][2] = fScaleZ; mat[2][3] = 0.0f;
		mat[3][0] = 0.0f; mat[3][1] = 0.0f; mat[3][2] = 0.0f; mat[3][3] = 1.0f;
	}

	/**
	 * Initializes a uniform scaling transformation matrix.
	 *
	 * This function sets up a 4x4 transformation matrix that applies uniform
	 * scaling along all three axes (X, Y, and Z) using the specified scale factor.
	 * It delegates the call to the function that accepts individual scaling
	 * factors for each axis.
	 *
	 * @param fScale: Uniform scaling factor applied to all axes.
	 */
	void InitScaleTransform(const GLfloat fScale)
	{
		InitScaleTransform(fScale, fScale, fScale);
	}

	/**
	 * Initializes a scaling transformation matrix using a vector.
	 *
	 * This function sets up a 4x4 transformation matrix that applies scaling
	 * transformations along the X, Y, and Z axes using the values provided
	 * in the input vector. It delegates the call to the function that accepts
	 * individual scaling factors.
	 *
	 * @param vScale: A 3D vector containing scaling factors for the X, Y, and Z axes.
	 */
	void InitScaleTransform(const Vector3D& vScale)
	{
		InitScaleTransform(vScale.x, vScale.y, vScale.z);
	}

	/**
	 * Initializes a scaling transformation matrix using a glm vector.
	 *
	 * This function sets up a 4x4 transformation matrix that applies scaling
	 * transformations along the X, Y, and Z axes using the values provided
	 * in the input vector. It delegates the call to the function that accepts
	 * individual scaling factors.
	 *
	 * @param vScale: A glm::vec3 containing scaling factors for the X, Y, and Z axes.
	 */
	void InitScaleTransform(const glm::vec3& vScale)
	{
		InitScaleTransform(vScale.x, vScale.y, vScale.z);
	}

	void InitRotateTransform(const GLfloat fRotateX, const GLfloat fRotateY, const GLfloat fRotateZ)
	{
		SMatrix4x4 matX{}, matY{}, matZ{};

		const GLfloat x = ToRadian(fRotateX);
		const GLfloat y = ToRadian(fRotateY);
		const GLfloat z = ToRadian(fRotateZ);

		matX.InitRotationX(x);
		matY.InitRotationY(y);
		matZ.InitRotationZ(z);

		/* ZYX Euler rotation sequence */
		*this = matZ * matY * matX;
	}

	/**
	 * Initializes the matrix for a rotation transformation using the XYZ Euler rotation sequence.
	 *
	 * @param RotateX: The angle of rotation around the X-axis in degrees.
	 * @param RotateY: The angle of rotation around the Y-axis in degrees.
	 * @param RotateZ: The angle of rotation around the Z-axis in degrees.
	 *
	 * This function creates individual rotation matrices for the X, Y, and Z axes
	 * and combines them in the order rx * ry * rz. This order applies the
	 * rotations in the sequence: Z-axis, Y-axis, then X-axis, relative to the
	 * global coordinate system.
	 */
	void InitRotateTransformZYX(const GLfloat fRotateX, const GLfloat fRotateY, const GLfloat fRotateZ)
	{
		SMatrix4x4 matX{}, matY{}, matZ{};

		const GLfloat x = ToRadian(fRotateX);
		const GLfloat y = ToRadian(fRotateY);
		const GLfloat z = ToRadian(fRotateZ);

		matX.InitRotationX(x);
		matY.InitRotationY(y);
		matZ.InitRotationZ(z);

		/* XYZ Euler rotation sequence */
		*this = matX * matY * matZ;
	}

	/**
	 * Initializes the matrix for a rotation transformation using the ZYX Euler rotation sequence.
	 *
	 * @param vRotation: The 3D Vector holding rotation degrees.
	 *
	 * This function creates individual rotation matrices for the X, Y, and Z axes
	 * and combines them in the order rz * ry * rx. This order applies the
	 * rotations in the sequence: X-axis, Y-axis, then Z-axis, relative to the
	 * local coordinate system.
	 */
	void InitRotateTransform(const SVector3Df& vRotation)
	{
		InitRotateTransform(vRotation.x, vRotation.y, vRotation.z);
	}

	/**
	 * Initializes the matrix for a rotation transformation using the ZYX Euler rotation sequence.
	 *
	 * @param vRotation: The glm::vec3 holding rotation degrees.
	 *
	 * This function creates individual rotation matrices for the X, Y, and Z axes
	 * and combines them in the order rz * ry * rx. This order applies the
	 * rotations in the sequence: X-axis, Y-axis, then Z-axis, relative to the
	 * local coordinate system.
	 */
	void InitRotateTransform(const glm::vec3& vRotation)
	{
		InitRotateTransform(vRotation.x, vRotation.y, vRotation.z);
	}

	/**
	 * Initializes the matrix for a rotation transformation using a quaternion.
	 *
	 * Quaternions provide a compact and efficient way to represent 3D rotations,
	 * avoiding issues like gimbal lock that occur with Euler angles.
	 *
	 * The resulting matrix applies the rotation described by the quaternion
	 * when multiplied by a vector or another matrix.
	 *
	 * Note: The function assumes the matrix `mat4` is a 4x4 matrix, and it resets
	 *       the translation components to zero, leaving the rotation part intact.
	 *
	 * @param sQuat: The quaternion representing the rotation to be applied from GLM.
	 */
	void InitRotateTransform(const Quaternion& sQuat)
	{
		const GLfloat yy2 = 2.0f * sQuat.y * sQuat.y;
		const GLfloat xy2 = 2.0f * sQuat.x * sQuat.y;
		const GLfloat xz2 = 2.0f * sQuat.x * sQuat.z;
		const GLfloat yz2 = 2.0f * sQuat.y * sQuat.z;
		const GLfloat zz2 = 2.0f * sQuat.z * sQuat.z;
		const GLfloat wz2 = 2.0f * sQuat.w * sQuat.z;
		const GLfloat wy2 = 2.0f * sQuat.w * sQuat.y;
		const GLfloat wx2 = 2.0f * sQuat.w * sQuat.x;
		const GLfloat xx2 = 2.0f * sQuat.x * sQuat.x;

		mat[0][0] = -yy2 - zz2 + 1.0f;
		mat[0][1] = xy2 + wz2;
		mat[0][2] = xz2 - wy2;
		mat[0][3] = 0.0f;

		mat[1][0] = xy2 - wz2;
		mat[1][1] = -xx2 - zz2 + 1.0f;
		mat[1][2] = yz2 + wx2;

		mat[2][0] = xz2 + wy2;
		mat[2][1] = yz2 - wx2;
		mat[2][2] = -xx2 - yy2 + 1.0f;

		mat[3][0] = 0.0f;
		mat[3][1] = 0.0f;
		mat[3][2] = 0.0f;
		mat[3][3] = 1.0f;
	}

	/**
	 * Initializes the matrix for a rotation transformation using a quaternion.
	 * @param sQuat: The quaternion representing the rotation to be applied from GLM.
	 *
	 * This function computes a 4x4 rotation matrix based on the given quaternion.
	 * Quaternions provide a compact and efficient way to represent 3D rotations,
	 * avoiding issues like gimbal lock that occur with Euler angles.
	 *
	 * The matrix is calculated directly from the quaternion's components:
	 * - @quat.x: The X component of the quaternion.
	 * - @quat.y: The Y component of the quaternion.
	 * - @quat.z: The Z component of the quaternion.
	 * - @quat.w: The W (real) component of the quaternion.
	 *
	 * The resulting matrix applies the rotation described by the quaternion
	 * when multiplied by a vector or another matrix.
	 *
	 * Note: The function assumes the matrix `mat4` is a 4x4 matrix, and it resets
	 *       the translation components to zero, leaving the rotation part intact.
	 */
	void InitRotateTransform(const glm::quat& sQuat)
	{
		glm::mat4 mat = glm::mat4_cast(sQuat);

		SMatrix4x4 res(mat);

		*this = res;
	}

	/**
	 * Initializes the rotation transformation matrix based on a specified direction.
	 * @param vDir: Direction vector to align the transformation with.
	 *
	 * This function aligns the camera's forward direction (`vDir`) with the
	 * specified direction while keeping the up direction constant as (0, 1, 0).
	 */
	void InitRotationFromDir(const SVector3Df& vDir)
	{
		SVector3Df vUp(0.0f, 1.0f, 0.0f);
		InitCameraTransform(vDir, vUp);
	}

	/**
	 * Initializes the rotation transformation matrix based on a specified direction.
	 * @param vDir: Direction vector to align the transformation with.
	 *
	 * This function aligns the camera's forward direction (`vDir`) with the
	 * specified direction while keeping the up direction constant as (0, 1, 0).
	 */
	void InitRotationFromDir(const glm::vec3& vDir)
	{
		glm::vec3 vUp(0.0f, 1.0f, 0.0f);
		InitCameraTransform(vDir, vUp);
	}

	/**
	 * Initializes the camera transformation matrix
	 * @param vTarget: Vector representing the target direction (camera's forward direction)
	 * @param vUp: Vector representing the up direction of the camera
	 *
	 * This function sets up the camera transformation matrix based on
	 * the specified target and up direction. The result is a matrix
	 * that transforms coordinates into the camera's coordinate space.
	 */
	void InitCameraTransform(const SVector3Df& vTarget, const SVector3Df& vUp)
	{
		SVector3Df N = vTarget;
		N.normalize();

		SVector3Df UpNorm = vUp;
		UpNorm.normalize();

		SVector3Df U{};
		U = UpNorm.cross(N);
		U.normalize();

		SVector3Df V = N.cross(U);

		mat[0][0] = U.x;
		mat[0][1] = U.y;
		mat[0][2] = U.z;
		mat[0][3] = 0.0f;

		mat[1][0] = V.x;
		mat[1][1] = V.y;
		mat[1][2] = V.z;
		mat[1][3] = 0.0f;

		mat[2][0] = N.x;
		mat[2][1] = N.y;
		mat[2][2] = N.z;
		mat[2][3] = 0.0f;

		mat[3][0] = 0.0f;
		mat[3][1] = 0.0f;
		mat[3][2] = 0.0f;
		mat[3][3] = 1.0f;
	}

	/**
	 * Initializes the camera transformation matrix
	 * @param vTarget: Vector representing the target direction (camera's forward direction)
	 * @param vUp: Vector representing the up direction of the camera
	 *
	 * This function sets up the camera transformation matrix based on
	 * the specified target and up direction. The result is a matrix
	 * that transforms coordinates into the camera's coordinate space.
	 */
	void InitCameraTransform(const glm::vec3& vTarget, const glm::vec3& vUp)
	{
		glm::vec3 N = vTarget;
		glm::normalize(N);

		glm::vec3 UpNorm = vUp;
		glm::normalize(UpNorm);

		glm::vec3 U = glm::cross(UpNorm, N);
		glm::normalize(U);

		glm::vec3 V = glm::cross(N, U);

		mat[0][0] = U.x;
		mat[0][1] = U.y;
		mat[0][2] = U.z;
		mat[0][3] = 0.0f;

		mat[1][0] = V.x;
		mat[1][1] = V.y;
		mat[1][2] = V.z;
		mat[1][3] = 0.0f;

		mat[2][0] = N.x;
		mat[2][1] = N.y;
		mat[2][2] = N.z;
		mat[2][3] = 0.0f;

		mat[3][0] = 0.0f;
		mat[3][1] = 0.0f;
		mat[3][2] = 0.0f;
		mat[3][3] = 1.0f;
	}

	/**
	 * Initializes a camera transformation matrix.
	 *
	 * This function combines a translation and rotation transformation
	 * to position and orient a camera in 3D space. The camera is first
	 * translated to the origin relative to its position (`vPos`), and
	 * then it is rotated to face the target (`vTarget`) with the specified
	 * upward direction (`vUp`). The resulting transformation matrix
	 * is the product of the rotation and translation matrices.
	 *
	 * @param vPos The position of the camera in world space.
	 * @param vTarget The target point the camera should face.
	 * @param vUp The upward direction vector for the camera's orientation.
	 */
	void InitCameraTransform(const SVector3Df& vPos, const SVector3Df& vTarget, const SVector3Df& vUp)
	{
		SMatrix4x4 matCameraTranslation{};
		matCameraTranslation.InitTranslationTransform(-vPos.x, -vPos.y, -vPos.z);

		SMatrix4x4 matCameraRotationTranslation{};
		matCameraRotationTranslation.InitCameraTransform(vTarget, vUp);

		*this = matCameraRotationTranslation * matCameraTranslation;
	}

	/**
	 * Initializes a camera transformation matrix.
	 *
	 * This function combines a translation and rotation transformation
	 * to position and orient a camera in 3D space. The camera is first
	 * translated to the origin relative to its position (`vPos`), and
	 * then it is rotated to face the target (`vTarget`) with the specified
	 * upward direction (`vUp`). The resulting transformation matrix
	 * is the product of the rotation and translation matrices.
	 *
	 * @param vPos The position of the camera in world space as GLM.
	 * @param vTarget The target point the camera should face as GLM.
	 * @param vUp The upward direction vector for the camera's orientation as GLM.
	 */
	void InitCameraTransform(const glm::vec3& vPos, const glm::vec3& vTarget, const glm::vec3& vUp)
	{
		SMatrix4x4 matCameraTranslation{};
		matCameraTranslation.InitTranslationTransform(-vPos.x, -vPos.y, -vPos.z);

		SMatrix4x4 matCameraRotationTranslation{};
		matCameraRotationTranslation.InitCameraTransform(vTarget, vUp);

		*this = matCameraRotationTranslation * matCameraTranslation;
	}

	/**
	 * Initializes a perspective projection transformation matrix.
	 *
	 * This function sets up a perspective projection matrix based on the
	 * given projection parameters. It supports both left-handed and right-handed
	 * coordinate systems.
	 *
	 * @param sPersProj: The projection parameters, encapsulated in an SPersProjInfo structure.
	 * @param bUseGLM A boolean indicating whether to use GLM functions for the transformation.
	 * @param bLeftHanded A boolean indicating whether to use a left-handed coordinate system if true.
	 */
	void InitPersProjTransform(const SPersProjInfo& sPersProj, const bool bUseGLM, const bool bLeftHanded)
	{
		if (bLeftHanded)
		{
			const GLfloat fAspectRatio = sPersProj.Height / sPersProj.Width;
			const GLfloat fZRange = sPersProj.zNear - sPersProj.zFar;
			const GLfloat fTanHalfFOV = std::tanf(ToRadian(sPersProj.FOV / 2.0f));

			mat[0][0] = 1.0f / fTanHalfFOV;
			mat[0][1] = 0.0f;
			mat[0][2] = 0.0f;
			mat[0][3] = 0.0f;

			mat[1][0] = 0.0f;
			mat[1][1] = 1.0f / (fTanHalfFOV * fAspectRatio);
			mat[1][2] = 0.0f;
			mat[1][3] = 0.0f;

			mat[2][0] = 0.0f;
			mat[2][1] = 0.0f;
			mat[2][2] = (-sPersProj.zNear - sPersProj.zFar) / fZRange;
			mat[2][3] = 2.0f * sPersProj.zFar * sPersProj.zNear / fZRange;

			mat[3][0] = 0.0f;
			mat[3][1] = 0.0f;
			mat[3][2] = 1.0f;
			mat[3][3] = 0.0f;

			if (bUseGLM)
			{
				const glm::mat4 mProjection = glm::perspectiveFovLH(glm::radians(sPersProj.FOV), sPersProj.Width, sPersProj.Height, sPersProj.zNear, sPersProj.zFar);

				mat[0][0] = mProjection[0][0]; mat[0][1] = mProjection[0][1]; mat[0][2] = mProjection[0][2]; mat[0][3] = mProjection[0][3];
				mat[1][0] = mProjection[1][0]; mat[1][1] = mProjection[1][1]; mat[1][2] = mProjection[1][2]; mat[1][3] = mProjection[1][3];
				mat[2][0] = mProjection[2][0]; mat[2][1] = mProjection[2][1]; mat[2][2] = mProjection[2][2]; mat[2][3] = mProjection[2][3];
				mat[3][0] = mProjection[3][0]; mat[3][1] = mProjection[3][1]; mat[3][2] = mProjection[3][2]; mat[3][3] = mProjection[3][3];
			}

			return;
		}

		const GLfloat fAspectRatio = sPersProj.Width / sPersProj.Height;
		const GLfloat fZRange = sPersProj.zNear - sPersProj.zFar;
		const GLfloat fTanHalfFOV = std::tanf(ToRadian(sPersProj.FOV / 2.0f));

		mat[0][0] = 1.0f / (fTanHalfFOV * fAspectRatio);
		mat[0][1] = 0.0f;
		mat[0][2] = 0.0f;
		mat[0][3] = 0.0f;

		mat[1][0] = 0.0f;
		mat[1][1] = 1.0f / fTanHalfFOV;
		mat[1][2] = 0.0f;
		mat[1][3] = 0.0f;

		mat[2][0] = 0.0f;
		mat[2][1] = 0.0f;
		mat[2][2] = (-sPersProj.zNear - sPersProj.zFar) / fZRange;
		mat[2][3] = 2.0f * sPersProj.zFar * sPersProj.zNear / fZRange;

		mat[3][0] = 0.0f;
		mat[3][1] = 0.0f;
		mat[3][2] = 1.0f; // Correct for right-handed (negative z-axis points into the screen)
		mat[3][3] = 0.0f;

		if (bUseGLM)
		{
			const glm::mat4 mProjection = glm::perspectiveFovRH(glm::radians(sPersProj.FOV), sPersProj.Width, sPersProj.Height, sPersProj.zNear, sPersProj.zFar);

			mat[0][0] = mProjection[0][0]; mat[0][1] = mProjection[0][1]; mat[0][2] = mProjection[0][2]; mat[0][3] = mProjection[0][3];
			mat[1][0] = mProjection[1][0]; mat[1][1] = mProjection[1][1]; mat[1][2] = mProjection[1][2]; mat[1][3] = mProjection[1][3];
			mat[2][0] = mProjection[2][0]; mat[2][1] = mProjection[2][1]; mat[2][2] = mProjection[2][2]; mat[2][3] = mProjection[2][3];
			mat[3][0] = mProjection[3][0]; mat[3][1] = mProjection[3][1]; mat[3][2] = mProjection[3][2]; mat[3][3] = mProjection[3][3];

		}
	}

	/**
	 * Initializes an orthographic projection matrix.
	 *
	 * This function constructs a 4x4 orthographic projection matrix for use
	 * in right-handed 3D graphics systems. The matrix maps the orthographic
	 * volume specified by the planes (l, r, b, t, n, f) into normalized
	 * device coordinates (NDC), where x and y range from -1 to 1, and z
	 * ranges from -1 to 1.
	 *
	 * @param sOrthoProj: A struct containing the orthographic projection parameters:
	 * @param bLeftHanded: A boolean indicating whether to use a left-handed coordinate system if true.
	 */
	void InitOrthoProjTransform(const SOrthoProjInfo& sOrthoProj, const bool bLeftHanded = false)
	{
		/* Just Make it shorter, less Ugly */
		const GLfloat fL = sOrthoProj.Left;
		const GLfloat fR = sOrthoProj.Right;
		const GLfloat fB = sOrthoProj.Bottom;
		const GLfloat fT = sOrthoProj.Top;
		const GLfloat fN = sOrthoProj.NearZ;
		const GLfloat fF = sOrthoProj.FarZ;

		/* Not Sure if it's better to change make this conditions for mat4[2][2] .. for now it is like that. */
		if (bLeftHanded)
		{
			mat[0][0] = 2.0f / (fR - fL);
			mat[0][1] = 0.0f;
			mat[0][2] = 0.0f;
			mat[0][3] = -(fR + fL) / (fR - fL);

			mat[1][0] = 0.0f;
			mat[1][1] = 2.0f / (fT - fB);
			mat[1][2] = 0.0f;
			mat[1][3] = -(fT + fB) / (fT - fB);

			mat[2][0] = 0.0f;
			mat[2][1] = 0.0f;
			mat[2][2] = 2.0f / (fF - fN);
			mat[2][3] = -(fF + fN) / (fF - fN);

			mat[3][0] = 0.0f;
			mat[3][1] = 0.0f;
			mat[3][2] = 0.0f;
			mat[3][3] = 1.0f;
			return;
		}

		/* Same as bLeftHanded Except mat4[2][2] */
		mat[0][0] = 2.0f / (fR - fL);
		mat[0][1] = 0.0f;
		mat[0][2] = 0.0f;
		mat[0][3] = -(fR + fL) / (fR - fL);

		mat[1][0] = 0.0f;
		mat[1][1] = 2.0f / (fT - fB);
		mat[1][2] = 0.0f;
		mat[1][3] = -(fT + fB) / (fT - fB);

		mat[2][0] = 0.0f;
		mat[2][1] = 0.0f;
		mat[2][2] = -2.0f / (fF - fN); // Change This to Minus
		mat[2][3] = -(fF + fN) / (fF - fN);

		mat[3][0] = 0.0f;
		mat[3][1] = 0.0f;
		mat[3][2] = 0.0f;
		mat[3][3] = 1.0f;
	}

	/**
	 * Extracts clipping planes from the current matrix.
	 *
	 * This function calculates the six frustum planes of a perspective or
	 * orthographic projection matrix. It utilizes the rows of the matrix
	 * to compute the planes by summing or subtracting the respective row
	 * vectors. The resulting vectors define the planes in the form:
	 * Ax + By + Cz + D = 0.
	 *
	 * This is useful for tasks such as frustum culling or collision detection
	 * in 3D graphics.
	 *
	 * @param vLeft: Output vector representing the left clipping plane.
	 * @param vRight: Output vector representing the right clipping plane.
	 * @param vBottom: Output vector representing the bottom clipping plane.
	 * @paramvTop: Output vector representing the top clipping plane.
	 * @param vNear: Output vector representing the near clipping plane.
	 * @param vFar: Output vector representing the far clipping plane.
	 */
	void CalculateClipPlanes(Vector4D& vLeft, Vector4D& vRight, Vector4D& vBottom, Vector4D& vTop, Vector4D& vNear, Vector4D& vFar) const
	{
		Vector4D Row1(mat[0][0], mat[0][1], mat[0][2], mat[0][3]);
		Vector4D Row2(mat[1][0], mat[1][1], mat[1][2], mat[1][3]);
		Vector4D Row3(mat[2][0], mat[2][1], mat[2][2], mat[2][3]);
		Vector4D Row4(mat[3][0], mat[3][1], mat[3][2], mat[3][3]);

		vLeft = Row1 + Row4;
		vRight = Row1 - Row4;
		vBottom = Row2 + Row4;
		vTop = Row2 - Row4;
		vNear = Row3 + Row4;
		vFar = Row3 - Row4;
	}

	/**
	 * Extracts clipping planes from the current matrix.
	 *
	 * This function calculates the six frustum planes of a perspective or
	 * orthographic projection matrix. It utilizes the rows of the matrix
	 * to compute the planes by summing or subtracting the respective row
	 * vectors. The resulting vectors define the planes in the form:
	 * Ax + By + Cz + D = 0.
	 *
	 * This is useful for tasks such as frustum culling or collision detection
	 * in 3D graphics.
	 *
	 * @param vLeft: Output vector representing the left clipping plane GLM version.
	 * @param vRight: Output vector representing the right clipping plane GLM version.
	 * @param vBottom: Output vector representing the bottom clipping plane GLM version.
	 * @paramvTop: Output vector representing the top clipping plane GLM version.
	 * @param vNear: Output vector representing the near clipping plane GLM version.
	 * @param vFar: Output vector representing the far clipping plane GLM version.
	 */
	void CalculateClipPlanes(glm::vec4& vLeft, glm::vec4& vRight, glm::vec4& vBottom, glm::vec4& vTop, glm::vec4& vNear, glm::vec4& vFar)
	{
		glm::vec4 Row1(mat[0][0], mat[0][1], mat[0][2], mat[0][3]);
		glm::vec4 Row2(mat[1][0], mat[1][1], mat[1][2], mat[1][3]);
		glm::vec4 Row3(mat[2][0], mat[2][1], mat[2][2], mat[2][3]);
		glm::vec4 Row4(mat[3][0], mat[3][1], mat[3][2], mat[3][3]);

		vLeft = Row1 + Row4;
		vRight = Row1 - Row4;
		vBottom = Row2 + Row4;
		vTop = Row2 - Row4;
		vNear = Row3 + Row4;
		vFar = Row3 - Row4;
	}

} Matrix4;

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