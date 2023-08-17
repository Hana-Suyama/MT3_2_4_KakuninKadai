#pragma once
#include <MyMath.h>
#include <Matrix4x4.h>
#include <Vector3.h>
#include <Novice.h>
#include <cassert>

//static const int kRowHeight = 20;
//static const int kColumnWidth = 60;

// void MatrixScreenPrintf(int x, int y, const Matrix4x4& matrix, const char a[]) {
//	Novice::ScreenPrintf(x, y, "%s", a);
//	for (int row = 0; row < 4; ++row) {
//		for (int column = 0; column < 4; ++column) {
//			Novice::ScreenPrintf(x + column * kColumnWidth, 20 + y + row * kRowHeight, "%6.02f",
//matrix.m[row][column]);
//		}
//	}
// }
//

Vector3 Project(const Vector3& v1, const Vector3& v2) {
	Vector3 Result{};
	Result = Vec3FloatMultiplication(Normalize(v2), Dot(v1, Normalize(v2)));
	return Result;
}

Vector3 ClosestPoint(const Vector3& point, const Segment& segment) {
	Vector3 Result{};
	Result = Add(segment.origin, Project(Subtract(point, segment.origin), segment.diff));
	return Result;
}

bool IsCollision(const Sphere& sphere, const Plane& plane) {
	float k{};
	k = fabsf(Dot(plane.normal, sphere.center) - plane.distsnce);
	if (k <= sphere.radius) {
		return true;
	} else {
		return false;
	}
}

bool IsCollision(const Segment& segment, const Plane& plane) {
	float dot = Dot(plane.normal, segment.diff);

	if (dot == 0.0f) {
		return false;
	}

	float t = (plane.distsnce - Dot(segment.origin, plane.normal)) / dot;

	if (t <= 1 && t >= 0) {
		return true;
	} else {
		return false;
	}
}


bool IsCollision(const Line& line, const Plane& plane) {
	float dot = Dot(plane.normal, line.diff);

	if (dot == 0.0f) {
		return false;
	}

	return true;
}


bool IsCollision(const Ray& ray, const Plane& plane) {
	float dot = Dot(plane.normal, ray.diff);

	if (dot == 0.0f) {
		return false;
	}

	float t = (plane.distsnce - Dot(ray.origin, plane.normal)) / dot;

	if (t >= 0) {
		return true;
	} else {
		return false;
	}
}

bool IsCollision(const Triangle& triangle, const Segment& segment) {

	Vector3 v1 = (Subtract(triangle.LocalVertices[1], triangle.LocalVertices[0]));
	Vector3 v2 = (Subtract(triangle.LocalVertices[2], triangle.LocalVertices[1]));
	Vector3 v3 = (Subtract(triangle.LocalVertices[0], triangle.LocalVertices[2]));
	Vector3 n = Normalize(Cross(v1, v2));
	float d = Dot(triangle.LocalVertices[1], n);

	float dot = Dot(n, segment.diff);

	if (dot == 0.0f) {
		return false;
	}

	float t = (d - Dot(segment.origin, n)) / dot;


	if (t <= 1 && t >= 0) {

		Vector3 p = Add(segment.origin, Vec3FloatMultiplication(segment.diff, t));
		Vector3 cross01 = Cross(v1, Subtract(p, triangle.LocalVertices[1]));
		Vector3 cross12 = Cross(v2, Subtract(p, triangle.LocalVertices[2]));
		Vector3 cross20 = Cross(v3, Subtract(p, triangle.LocalVertices[0]));

		if (Dot(cross01, n) >= 0.0f &&
			Dot(cross12, n) >= 0.0f &&
			Dot(cross20, n) >= 0.0f) {
			return true;
		}

	}
	return false;
}

//関数
float DEGtoRAD(float degree) {
	float result;
	result = degree * ((float)M_PI / 180);
	return result;
}

// 行列の加法
Matrix4x4 Add(Matrix4x4 matrix1, Matrix4x4 matrix2) {
	Matrix4x4 Return{};
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			Return.m[i][j] = matrix1.m[i][j] + matrix2.m[i][j];
		}
	}
	return Return;
}

// 行列の減法
Matrix4x4 Subtract(Matrix4x4 matrix1, Matrix4x4 matrix2) {
	Matrix4x4 Return{};
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			Return.m[i][j] = matrix1.m[i][j] - matrix2.m[i][j];
		}
	}
	return Return;
}

// 行列の積
Matrix4x4 Multiply(Matrix4x4 matrix1, Matrix4x4 matrix2) {
	Matrix4x4 Return{};
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			for (int k = 0; k < 4; k++) {
				Return.m[i][j] += matrix1.m[i][k] * matrix2.m[k][j];
			}
		}
	}
	return Return;
}

// ベクトルの加法
Vector3 Add(Vector3 a, Vector3 b) {
	Vector3 result{};
	result.x = a.x + b.x;
	result.y = a.y + b.y;
	result.z = a.z + b.z;
	return result;
}

// ベクトルの減法
Vector3 Subtract(Vector3 a, Vector3 b) {
	Vector3 result{};
	result.x = a.x - b.x;
	result.y = a.y - b.y;
	result.z = a.z - b.z;
	return result;
}

// ベクトルの積
Vector3 Multiply(const Vector3 a, const Vector3 b) {
	Vector3 result{};
	result.x = a.x * b.x;
	result.y = a.y * b.y;
	result.z = a.z * b.z;
	return result;
}

//クロス積
Vector3 Cross(const Vector3& v1, const Vector3& v2) {
	Vector3 result{};
	result.x = (v1.y * v2.z) - (v1.z * v2.y);
	result.y = (v1.z * v2.x) - (v1.x * v2.z);
	result.z = (v1.x * v2.y) - (v1.y * v2.x);
	return result;
}

// 逆行列
Matrix4x4 Inverse(const Matrix4x4& m) {
	Matrix4x4 Return{};
	float A = 0;

	A = (m.m[0][0] * m.m[1][1] * m.m[2][2] * m.m[3][3]) +
	    (m.m[0][0] * m.m[1][2] * m.m[2][3] * m.m[3][1]) +
	    (m.m[0][0] * m.m[1][3] * m.m[2][1] * m.m[3][2]) -
	    (m.m[0][0] * m.m[1][3] * m.m[2][2] * m.m[3][1]) -
	    (m.m[0][0] * m.m[1][2] * m.m[2][1] * m.m[3][3]) -
	    (m.m[0][0] * m.m[1][1] * m.m[2][3] * m.m[3][2]) -
	    (m.m[0][1] * m.m[1][0] * m.m[2][2] * m.m[3][3]) -
	    (m.m[0][2] * m.m[1][0] * m.m[2][3] * m.m[3][1]) -
	    (m.m[0][3] * m.m[1][0] * m.m[2][1] * m.m[3][2]) +
	    (m.m[0][3] * m.m[1][0] * m.m[2][2] * m.m[3][1]) +
	    (m.m[0][2] * m.m[1][0] * m.m[2][1] * m.m[3][3]) +
	    (m.m[0][1] * m.m[1][0] * m.m[2][3] * m.m[3][2]) +
	    (m.m[0][1] * m.m[1][2] * m.m[2][0] * m.m[3][3]) +
	    (m.m[0][2] * m.m[1][3] * m.m[2][0] * m.m[3][1]) +
	    (m.m[0][3] * m.m[1][1] * m.m[2][0] * m.m[3][2]) -
	    (m.m[0][3] * m.m[1][2] * m.m[2][0] * m.m[3][1]) -
	    (m.m[0][2] * m.m[1][1] * m.m[2][0] * m.m[3][3]) -
	    (m.m[0][1] * m.m[1][3] * m.m[2][0] * m.m[3][2]) -
	    (m.m[0][1] * m.m[1][2] * m.m[2][3] * m.m[3][0]) -
	    (m.m[0][2] * m.m[1][3] * m.m[2][1] * m.m[3][0]) -
	    (m.m[0][3] * m.m[1][1] * m.m[2][2] * m.m[3][0]) +
	    (m.m[0][3] * m.m[1][2] * m.m[2][1] * m.m[3][0]) +
	    (m.m[0][2] * m.m[1][1] * m.m[2][3] * m.m[3][0]) +
	    (m.m[0][1] * m.m[1][3] * m.m[2][2] * m.m[3][0]);

	Return.m[0][0] =
	    (1 / A) * ((m.m[1][1] * m.m[2][2] * m.m[3][3]) + (m.m[1][2] * m.m[2][3] * m.m[3][1]) +
	               (m.m[1][3] * m.m[2][1] * m.m[3][2]) - (m.m[1][3] * m.m[2][2] * m.m[3][1]) -
	               (m.m[1][2] * m.m[2][1] * m.m[3][3]) - (m.m[1][1] * m.m[2][3] * m.m[3][2]));

	Return.m[0][1] =
	    (1 / A) * (-(m.m[0][1] * m.m[2][2] * m.m[3][3]) - (m.m[0][2] * m.m[2][3] * m.m[3][1]) -
	               (m.m[0][3] * m.m[2][1] * m.m[3][2]) + (m.m[0][3] * m.m[2][2] * m.m[3][1]) +
	               (m.m[0][2] * m.m[2][1] * m.m[3][3]) + (m.m[0][1] * m.m[2][3] * m.m[3][2]));

	Return.m[0][2] =
	    (1 / A) * ((m.m[0][1] * m.m[1][2] * m.m[3][3]) + (m.m[0][2] * m.m[1][3] * m.m[3][1]) +
	               (m.m[0][3] * m.m[1][1] * m.m[3][2]) - (m.m[0][3] * m.m[1][2] * m.m[3][1]) -
	               (m.m[0][2] * m.m[1][1] * m.m[3][3]) - (m.m[0][1] * m.m[1][3] * m.m[3][2]));

	Return.m[0][3] =
	    (1 / A) * (-(m.m[0][1] * m.m[1][2] * m.m[2][3]) - (m.m[0][2] * m.m[1][3] * m.m[2][1]) -
	               (m.m[0][3] * m.m[1][1] * m.m[2][2]) + (m.m[0][3] * m.m[1][2] * m.m[2][1]) +
	               (m.m[0][2] * m.m[1][1] * m.m[2][3]) + (m.m[0][1] * m.m[1][3] * m.m[2][2]));

	Return.m[1][0] =
	    (1 / A) * (-(m.m[1][0] * m.m[2][2] * m.m[3][3]) - (m.m[1][2] * m.m[2][3] * m.m[3][0]) -
	               (m.m[1][3] * m.m[2][0] * m.m[3][2]) + (m.m[1][3] * m.m[2][2] * m.m[3][0]) +
	               (m.m[1][2] * m.m[2][0] * m.m[3][3]) + (m.m[1][0] * m.m[2][3] * m.m[3][2]));

	Return.m[1][1] =
	    (1 / A) * ((m.m[0][0] * m.m[2][2] * m.m[3][3]) + (m.m[0][2] * m.m[2][3] * m.m[3][0]) +
	               (m.m[0][3] * m.m[2][0] * m.m[3][2]) - (m.m[0][3] * m.m[2][2] * m.m[3][0]) -
	               (m.m[0][2] * m.m[2][0] * m.m[3][3]) - (m.m[0][0] * m.m[2][3] * m.m[3][2]));

	Return.m[1][2] =
	    (1 / A) * (-(m.m[0][0] * m.m[1][2] * m.m[3][3]) - (m.m[0][2] * m.m[1][3] * m.m[3][0]) -
	               (m.m[0][3] * m.m[1][0] * m.m[3][2]) + (m.m[0][3] * m.m[1][2] * m.m[3][0]) +
	               (m.m[0][2] * m.m[1][0] * m.m[3][3]) + (m.m[0][0] * m.m[1][3] * m.m[3][2]));

	Return.m[1][3] =
	    (1 / A) * ((m.m[0][0] * m.m[1][2] * m.m[2][3]) + (m.m[0][2] * m.m[1][3] * m.m[2][0]) +
	               (m.m[0][3] * m.m[1][0] * m.m[2][2]) - (m.m[0][3] * m.m[1][2] * m.m[2][0]) -
	               (m.m[0][2] * m.m[1][0] * m.m[2][3]) - (m.m[0][0] * m.m[1][3] * m.m[2][2]));

	Return.m[2][0] =
	    (1 / A) * ((m.m[1][0] * m.m[2][1] * m.m[3][3]) + (m.m[1][1] * m.m[2][3] * m.m[3][0]) +
	               (m.m[1][3] * m.m[2][0] * m.m[3][1]) - (m.m[1][3] * m.m[2][1] * m.m[3][0]) -
	               (m.m[1][1] * m.m[2][0] * m.m[3][3]) - (m.m[1][0] * m.m[2][3] * m.m[3][1]));

	Return.m[2][1] =
	    (1 / A) * (-(m.m[0][0] * m.m[2][1] * m.m[3][3]) - (m.m[0][1] * m.m[2][3] * m.m[3][0]) -
	               (m.m[0][3] * m.m[2][0] * m.m[3][1]) + (m.m[0][3] * m.m[2][1] * m.m[3][0]) +
	               (m.m[0][1] * m.m[2][0] * m.m[3][3]) + (m.m[0][0] * m.m[2][3] * m.m[3][1]));

	Return.m[2][2] =
	    (1 / A) * ((m.m[0][0] * m.m[1][1] * m.m[3][3]) + (m.m[0][1] * m.m[1][3] * m.m[3][0]) +
	               (m.m[0][3] * m.m[1][0] * m.m[3][1]) - (m.m[0][3] * m.m[1][1] * m.m[3][0]) -
	               (m.m[0][1] * m.m[1][0] * m.m[3][3]) - (m.m[0][0] * m.m[1][3] * m.m[3][1]));

	Return.m[2][3] =
	    (1 / A) * (-(m.m[0][0] * m.m[1][1] * m.m[2][3]) - (m.m[0][1] * m.m[1][3] * m.m[2][0]) -
	               (m.m[0][3] * m.m[1][0] * m.m[2][1]) + (m.m[0][3] * m.m[1][1] * m.m[2][0]) +
	               (m.m[0][1] * m.m[1][0] * m.m[2][3]) + (m.m[0][0] * m.m[1][3] * m.m[2][1]));

	Return.m[3][0] =
	    (1 / A) * (-(m.m[1][0] * m.m[2][1] * m.m[3][2]) - (m.m[1][1] * m.m[2][2] * m.m[3][0]) -
	               (m.m[1][2] * m.m[2][0] * m.m[3][1]) + (m.m[1][2] * m.m[2][1] * m.m[3][0]) +
	               (m.m[1][1] * m.m[2][0] * m.m[3][2]) + (m.m[1][0] * m.m[2][2] * m.m[3][1]));

	Return.m[3][1] =
	    (1 / A) * ((m.m[0][0] * m.m[2][1] * m.m[3][2]) + (m.m[0][1] * m.m[2][2] * m.m[3][0]) +
	               (m.m[0][2] * m.m[2][0] * m.m[3][1]) - (m.m[0][2] * m.m[2][1] * m.m[3][0]) -
	               (m.m[0][1] * m.m[2][0] * m.m[3][2]) - (m.m[0][0] * m.m[2][2] * m.m[3][1]));

	Return.m[3][2] =
	    (1 / A) * (-(m.m[0][0] * m.m[1][1] * m.m[3][2]) - (m.m[0][1] * m.m[1][2] * m.m[3][0]) -
	               (m.m[0][2] * m.m[1][0] * m.m[3][1]) + (m.m[0][2] * m.m[1][1] * m.m[3][0]) +
	               (m.m[0][1] * m.m[1][0] * m.m[3][2]) + (m.m[0][0] * m.m[1][2] * m.m[3][1]));

	Return.m[3][3] =
	    (1 / A) * ((m.m[0][0] * m.m[1][1] * m.m[2][2]) + (m.m[0][1] * m.m[1][2] * m.m[2][0]) +
	               (m.m[0][2] * m.m[1][0] * m.m[2][1]) - (m.m[0][2] * m.m[1][1] * m.m[2][0]) -
	               (m.m[0][1] * m.m[1][0] * m.m[2][2]) - (m.m[0][0] * m.m[1][2] * m.m[2][1]));

	return Return;
}

// 転置行列
Matrix4x4 Transpose(const Matrix4x4& m) {
	Matrix4x4 Return{};
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			Return.m[i][j] = m.m[j][i];
		}
	}
	return Return;
}

// 単位行列
Matrix4x4 MakeIdentity4x4() {
	Matrix4x4 Return{};
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			if (i == j) {
				Return.m[i][j] = 1;
			}
		}
	}
	return Return;
}

// ベクトルと行列の積
Vector3 Multiply(Vector3 vector, Matrix4x4 matrix) {
	Vector3 Return = {};
	Return.x =
	    (vector.x * matrix.m[0][0]) + (vector.y * matrix.m[1][0]) + (vector.z * matrix.m[2][0]);
	Return.y =
	    (vector.x * matrix.m[0][1]) + (vector.y * matrix.m[1][1]) + (vector.z * matrix.m[2][1]);
	Return.z =
	    (vector.x * matrix.m[0][2]) + (vector.y * matrix.m[1][2]) + (vector.z * matrix.m[2][2]);
	return Return;
}

// 1.平行移動行列
Matrix4x4 MakeTranslateMatrix(const Vector3& translate) {
	Matrix4x4 Return{};
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			if (i == j) {
				Return.m[i][j] = 1;
			}
		}
	}
	Return.m[3][0] = translate.x;
	Return.m[3][1] = translate.y;
	Return.m[3][2] = translate.z;
	return Return;
}

// 2.拡大縮小行列
Matrix4x4 MakeScaleMatrix(const Vector3& scale) {
	Matrix4x4 Return{};
	Return.m[0][0] = scale.x;
	Return.m[1][1] = scale.y;
	Return.m[2][2] = scale.z;
	Return.m[3][3] = 1;
	return Return;
}

// 3.座標返還
Vector3 Transform(const Vector3& vector, const Matrix4x4& matrix) {
	Vector3 Return{};
	Return.x = vector.x * matrix.m[0][0] + vector.y * matrix.m[1][0] + vector.z * matrix.m[2][0] +
	           1.0f * matrix.m[3][0];
	Return.y = vector.x * matrix.m[0][1] + vector.y * matrix.m[1][1] + vector.z * matrix.m[2][1] +
	           1.0f * matrix.m[3][1];
	Return.z = vector.x * matrix.m[0][2] + vector.y * matrix.m[1][2] + vector.z * matrix.m[2][2] +
	           1.0f * matrix.m[3][2];
	float w = vector.x * matrix.m[0][3] + vector.y * matrix.m[1][3] + vector.z * matrix.m[2][3] +
	          1.0f * matrix.m[3][3];
	assert(w != 0.0f);
	Return.x /= w;
	Return.y /= w;
	Return.z /= w;
	return Return;
}

// 1.X軸回転行列
Matrix4x4 MakeRotateXMatrix(float radian) {
	Matrix4x4 Return{};
	Return.m[0][0] = 1;
	Return.m[1][1] = std::cos(radian);
	Return.m[1][2] = std::sin(radian);
	Return.m[2][1] = -std::sin(radian);
	Return.m[2][2] = std::cos(radian);
	Return.m[3][3] = 1;
	return Return;
}

// 2.Y軸回転行列
Matrix4x4 MakeRotateYMatrix(float radian) {
	Matrix4x4 Return{};
	Return.m[0][0] = std::cos(radian);
	Return.m[0][2] = -std::sin(radian);
	Return.m[1][1] = 1;
	Return.m[2][0] = std::sin(radian);
	Return.m[2][2] = std::cos(radian);
	Return.m[3][3] = 1;
	return Return;
}

// 3.Z軸回転行列
Matrix4x4 MakeRotateZMatrix(float radian) {
	Matrix4x4 Return{};
	Return.m[0][0] = std::cos(radian);
	Return.m[0][1] = std::sin(radian);
	Return.m[1][0] = -std::sin(radian);
	Return.m[1][1] = std::cos(radian);
	Return.m[2][2] = 1;
	Return.m[3][3] = 1;
	return Return;
}

Matrix4x4 MakeAffineMatrix(const Vector3& scale, const Vector3& rotate, const Vector3& translate) {
	Matrix4x4 result{};
	result = MakeScaleMatrix(scale);
	Matrix4x4 rotateXMatrix = MakeRotateXMatrix(rotate.x);
	Matrix4x4 rotateYMatrix = MakeRotateYMatrix(rotate.y);
	Matrix4x4 rotateZMatrix = MakeRotateZMatrix(rotate.z);
	Matrix4x4 rotateXYZMatrix = Multiply(rotateXMatrix, Multiply(rotateYMatrix, rotateZMatrix));
	result = Multiply(result, rotateXYZMatrix);
	result = Multiply(result, MakeTranslateMatrix(translate));
	return result;
}

Vector3 Vec3FloatMultiplication(const Vector3& a, const float& b) { 
	Vector3 result{};
	result.x = a.x * b;
	result.y = a.y * b;
	result.z = a.z * b;
	return result;
}

Vector3 TransformNormal(const Vector3& v, const Matrix4x4& m) {
	Vector3 result{
	    v.x * m.m[0][0] + v.y * m.m[1][0] + v.z * m.m[2][0],
	    v.x * m.m[0][1] + v.y * m.m[1][1] + v.z * m.m[2][1],
	    v.x * m.m[0][2] + v.y * m.m[1][2] + v.z * m.m[2][2]};
	return result;
}

Vector3 Normalize(const Vector3& a) {

	Vector3 result{};

	float length = sqrtf(a.x * a.x + a.y * a.y + a.z * a.z);

	if (length != 0.0f) {
		result.x = a.x / length;
		result.y = a.y / length;
		result.z = a.z / length;
	}

	return result;
}

//内積
float Dot(const Vector3 v1, const Vector3 v2) {
	float Result{};
	Result = (v1.x * v2.x) + (v1.y * v2.y) + (v1.z * v2.z);
	return Result;
}

//1. 透視投影行列
Matrix4x4 MakePerspectiveFovMatrix(float fovY, float aspectRatio, float nearClip, float farClip) {
	Matrix4x4 result{};
	result.m[0][0] = (1 / aspectRatio) * (1 / std::tan(fovY / 2));
	result.m[1][1] = (1 / std::tan(fovY / 2));
	result.m[2][2] = (farClip / (farClip - nearClip));
	result.m[2][3] = 1;
	result.m[3][2] = (-nearClip * farClip) / (farClip - nearClip);
	return result;
}

//2. 正射影行列
Matrix4x4 MakeOrthographicMatrix(float left, float top, float right, float bottom, float nearClip, float farClip) {
	Matrix4x4 result{};
	result.m[0][0] = 2 / (right - left);
	result.m[1][1] = 2 / (top - bottom);
	result.m[2][2] = 1 / (farClip - nearClip);
	result.m[3][0] = (left + right) / (left - right);
	result.m[3][1] = (top + bottom) / (bottom - top);
	result.m[3][2] = nearClip / (nearClip - farClip);
	result.m[3][3] = 1;
	return result;
}

// 3. ビューポート変換行列
Matrix4x4 MakeViewportMatrix(
    float left, float top, float width, float height, float minDepth, float maxDepth) {
	Matrix4x4 result{};
	result.m[0][0] = width / 2;
	result.m[1][1] = -(height / 2);
	result.m[2][2] = maxDepth - minDepth;
	result.m[3][0] = left + (width / 2);
	result.m[3][1] = top + (height / 2);
	result.m[3][2] = minDepth;
	result.m[3][3] = 1;
	return result;
}


void DrawGrid(const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix) {
	const float kGridHalfWidth = 2.0f;//Gridの半分の幅
	const uint32_t kSubdivision = 10;//分割数
	const float kGridEvery = (kGridHalfWidth * 2.0f) / float(kSubdivision);//1つ分の長さ
	Vector3 StartPos{ 0, 0, 0 };

	//奥から手前への線を順々に引いていく
	for (uint32_t xIndex = 0; xIndex <= kSubdivision; ++xIndex) {
		Vector3 worldStart{};
		Vector3 worldEnd{};
		worldStart.z = StartPos.z + kGridHalfWidth;
		worldEnd.z = worldStart.z - kGridHalfWidth * 2.0f;
		worldStart.x = kGridHalfWidth - (xIndex * kGridEvery);
		worldEnd.x = worldStart.x;

		Vector3 StartVertex{};
		Vector3 EndVertex{};

		Vector3 StartndcVertex = Transform(worldStart, viewProjectionMatrix);
		StartVertex = Transform(StartndcVertex, viewportMatrix);
		Vector3 EndndcVertex = Transform(worldEnd, viewProjectionMatrix);
		EndVertex = Transform(EndndcVertex, viewportMatrix);

		if (xIndex == 5) {
			Novice::DrawLine((int)StartVertex.x, (int)StartVertex.y, (int)EndVertex.x, (int)EndVertex.y, BLACK);
		} else {
			Novice::DrawLine((int)StartVertex.x, (int)StartVertex.y, (int)EndVertex.x, (int)EndVertex.y, 0xAAAAAAFF);
		}
	}

	for (uint32_t zIndex = 0; zIndex <= kSubdivision; ++zIndex) {
		Vector3 worldStart{};
		Vector3 worldEnd{};
		worldStart.x = kGridHalfWidth;
		worldEnd.x = worldStart.x - kGridHalfWidth * 2.0f;
		worldStart.z = kGridHalfWidth - (zIndex * kGridEvery);
		worldEnd.z = worldStart.z;

		Vector3 StartVertex{};
		Vector3 EndVertex{};

		Vector3 StartndcVertex = Transform(worldStart, viewProjectionMatrix);
		StartVertex = Transform(StartndcVertex, viewportMatrix);
		Vector3 EndndcVertex = Transform(worldEnd, viewProjectionMatrix);
		EndVertex = Transform(EndndcVertex, viewportMatrix);

		if (zIndex == 5) {
			Novice::DrawLine((int)StartVertex.x, (int)StartVertex.y, (int)EndVertex.x, (int)EndVertex.y, BLACK);
		} else {
			Novice::DrawLine((int)StartVertex.x, (int)StartVertex.y, (int)EndVertex.x, (int)EndVertex.y, 0xAAAAAAFF);
		}
	}
}

void DrawSphere(const Sphere& sphere, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix, uint32_t color) {
	const uint32_t kSubdivision = 26;//分割数
	const float kLonEvery = DEGtoRAD(360 / kSubdivision);//経度分割1つ分の角度
	const float kLatEvery = DEGtoRAD(360 / kSubdivision);//緯度分割1つ分の角度
	//緯度の方向に分割 0 ~ 2π
	for (uint32_t latIndex = 0; latIndex < kSubdivision; ++latIndex) {
		float lat = -(float)M_PI / 2.0f + kLatEvery * latIndex;//現在の緯度
		//経度の方向に分割 0 ~ 2π
		for (uint32_t lonIndex = 0; lonIndex < kSubdivision; ++lonIndex) {
			float lon = lonIndex * kLonEvery;//現在の経度

			//world座標系でのa, b, cを求める
			Vector3 a{}, b{}, c{};
			a.x = sphere.center.x + (cosf(lat) * cosf(lon) * sphere.radius);
			a.y = sphere.center.y + (sinf(lat) * sphere.radius);
			a.z = sphere.center.z + (cosf(lat) * sinf(lon) * sphere.radius);

			b.x = sphere.center.x + (cosf(lat + kLatEvery) * cosf(lon) * sphere.radius);
			b.y = sphere.center.y + (sinf(lat + kLatEvery) * sphere.radius);
			b.z = sphere.center.z + (cosf(lat + kLatEvery) * sinf(lon) * sphere.radius);

			c.x = sphere.center.x + (cosf(lat) * cosf(lon + kLonEvery) * sphere.radius);
			c.y = sphere.center.y + (sinf(lat) * sphere.radius);
			c.z = sphere.center.z + (cosf(lat) * sinf(lon + kLonEvery) * sphere.radius);

			// a,b,cをScreen座標系まで変換
			Vector3 VertexA{};
			Vector3 ndcVertexA = Transform(a, viewProjectionMatrix);
			VertexA = Transform(ndcVertexA, viewportMatrix);

			Vector3 VertexB{};
			Vector3 ndcVertexB = Transform(b, viewProjectionMatrix);
			VertexB = Transform(ndcVertexB, viewportMatrix);

			Vector3 VertexC{};
			Vector3 ndcVertexC = Transform(c, viewProjectionMatrix);
			VertexC = Transform(ndcVertexC, viewportMatrix);

			//ab,bcで線を引く
			Novice::DrawLine((int)VertexA.x, (int)VertexA.y, (int)VertexB.x, (int)VertexB.y, color);
			Novice::DrawLine((int)VertexA.x, (int)VertexA.y, (int)VertexC.x, (int)VertexC.y, color);
		}
	}
}

void DrawLineTriangle(const Triangle& triangle, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix, uint32_t color) {
	Vector3 screenVertices[3];
	for (uint32_t i = 0; i < 3; ++i) {
		Vector3 ndcVertex = Transform(triangle.LocalVertices[i], viewProjectionMatrix);
		screenVertices[i] = Transform(ndcVertex, viewportMatrix);
	}
	Novice::DrawTriangle((int)screenVertices[0].x, (int)screenVertices[0].y, (int)screenVertices[1].x, (int)screenVertices[1].y, (int)screenVertices[2].x, (int)screenVertices[2].y, color, kFillModeWireFrame);
}

//
Vector3 Perpendicular(const Vector3& vector) {
	if (vector.x != 0.0f || vector.y != 0.0f) {
		return { -vector.y, vector.x, 0.0f };
	}
	return{ 0.0f, -vector.z, vector.y };
}

//平面の描画
void DrawPlane(const Plane& plane, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix, uint32_t color) {
	Vector3 center = Vec3FloatMultiplication(plane.normal, plane.distsnce);
	Vector3 perpendiculars[4];
	perpendiculars[0] = Normalize(Perpendicular(plane.normal));
	perpendiculars[1] = { -perpendiculars[0].x, -perpendiculars[0].y, -perpendiculars[0].z };
	perpendiculars[2] = Cross(plane.normal, perpendiculars[0]);
	perpendiculars[3] = { -perpendiculars[2].x, -perpendiculars[2].y, -perpendiculars[2].z };

	Vector3 points[4];
	for (int32_t index = 0; index < 4; ++index) {
		Vector3 extend = Vec3FloatMultiplication(perpendiculars[index], 2.0f);
		Vector3 point = Add(center, extend);
		points[index] = Transform(Transform(point, viewProjectionMatrix), viewportMatrix);
	}

	Novice::DrawLine((int)points[0].x, (int)points[0].y, (int)points[2].x, (int)points[2].y, color);
	Novice::DrawLine((int)points[2].x, (int)points[2].y, (int)points[1].x, (int)points[1].y, color);
	Novice::DrawLine((int)points[1].x, (int)points[1].y, (int)points[3].x, (int)points[3].y, color);
	Novice::DrawLine((int)points[3].x, (int)points[3].y, (int)points[0].x, (int)points[0].y, color);
}
