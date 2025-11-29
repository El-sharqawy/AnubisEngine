#pragma once

#include "maths.h"

typedef struct SMatrix4x4R
{
	GLfloat mat[4][4];

	/**
	 * Default constructor, initializes all elements to zero.
	 */
	SMatrix4x4R() = default;

	/**
	 * Constructor that initializes the matrix with specific values.
	 *
	 * @param a00, a01, ..., a33: Individual matrix elements.
	 */
	SMatrix4x4R(const GLfloat a00, const GLfloat a01, const GLfloat a02, const GLfloat a03,
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
	SMatrix4x4R(const SMatrix4x4R& Mat)
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
	SMatrix4x4R(const SVector4Df& vec1, const SVector4Df& vec2, const SVector4Df& vec3, const SVector4Df& vec4)
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
	SMatrix4x4R(const glm::mat4& glmMat)
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
	SMatrix4x4R operator*(const SMatrix4x4R& rightMat)
	{
		SMatrix4x4R resultMat{};
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
	SMatrix4x4R operator*(const GLfloat fScalar)
	{
		SMatrix4x4R result{};
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
	SMatrix4x4R Transpose() const
	{
		SMatrix4x4R newMat{};

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
	SMatrix4x4R Inverse() const
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
		SMatrix4x4R res{};

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
	SMatrix4x4R InverseSub() const
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

		SMatrix4x4R Inverse(Inv0 * SignA, Inv1 * SignB, Inv2 * SignA, Inv3 * SignB);

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
	SMatrix4x4R InverseGJ(const SMatrix4x4R& inputMatrix)
	{
		SMatrix4x4R inverseMatrix{};
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
		SMatrix4x4R matX{}, matY{}, matZ{};

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
		SMatrix4x4R matX{}, matY{}, matZ{};

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

		SMatrix4x4R res(mat);

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
		SMatrix4x4R matCameraTranslation{};
		matCameraTranslation.InitTranslationTransform(-vPos.x, -vPos.y, -vPos.z);

		SMatrix4x4R matCameraRotationTranslation{};
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
		SMatrix4x4R matCameraTranslation{};
		matCameraTranslation.InitTranslationTransform(-vPos.x, -vPos.y, -vPos.z);

		SMatrix4x4R matCameraRotationTranslation{};
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

} TMatrix4x4R;