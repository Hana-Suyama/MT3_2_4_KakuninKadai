#pragma once
#include <Matrix4x4.h>
#define _USE_MATH_DEFINES
#include "cmath"
#include <Vector3.h>
#include <cassert>

static const int kRowHeight = 20;
static const int kColumnWidth = 60;

struct Sphere {
	Vector3 center{};	//!< 中心点
	float radius{};	//!< 半径
};

struct Line {
	Vector3 origin;	//!<始点
	Vector3 diff;	//!<終点への差分ベクトル
};

struct Ray {
	Vector3 origin;	//!<始点
	Vector3 diff;	//!<終点への差分ベクトル
};

struct Segment {
	Vector3 origin;	//!<始点
	Vector3 diff;	//!<終点への差分ベクトル
};

struct Plane {
	Vector3 normal;//法線
	float distsnce;//距離
};

struct Triangle {
	Vector3 LocalVertices[3];
};

Vector3 Project(const Vector3& v1, const Vector3& v2);

Vector3 ClosestPoint(const Vector3& point, const Segment& segment);

//衝突判定
bool IsCollision(const Sphere& sphere, const Plane& plane);
bool IsCollision(const Segment& segment, const Plane& plane);
bool IsCollision(const Line& line, const Plane& plane);
bool IsCollision(const Ray& ray, const Plane& plane);
bool IsCollision(const Triangle& triangle, const Segment& segment);

//行列の加法
Matrix4x4 Add(Matrix4x4 matrix1, Matrix4x4 matrix2);

//行列の減法
Matrix4x4 Subtract(Matrix4x4 matrix1, Matrix4x4 matrix2);

//行列の積
Matrix4x4 Multiply(Matrix4x4 matrix1, Matrix4x4 matrix2);

//Vector3の加法
Vector3 Add(Vector3 a, Vector3 b);

//Vector3の減法
Vector3 Subtract(Vector3 a, Vector3 b);

//Vector3の積
Vector3 Multiply(Vector3 a, Vector3 b);

//クロス積
Vector3 Cross(const Vector3& v1, const Vector3& v2);

//逆行列
Matrix4x4 Inverse(const Matrix4x4& m);

//転置行列
Matrix4x4 Transpose(const Matrix4x4& m);

//単位行列
Matrix4x4 MakeIdentity4x4();

//ベクトルと行列の積
Vector3 Multiply(const Vector3 vector, const Matrix4x4 matrix);

//1.平行移動行列
Matrix4x4 MakeTranslateMatrix(const Vector3& translate);

//2.拡大縮小行列
Matrix4x4 MakeScaleMatrix(const Vector3& scale);

//3.座標返還
Vector3 Transform(const Vector3& vector, const Matrix4x4& matrix);

//1.X軸回転行列
Matrix4x4 MakeRotateXMatrix(float radian);

//2.Y軸回転行列
Matrix4x4 MakeRotateYMatrix(float radian);

//3.Z軸回転行列
Matrix4x4 MakeRotateZMatrix(float radian);

Matrix4x4 MakeAffineMatrix(const Vector3& scale, const Vector3& rotate, const Vector3& translate);

//1. 透視投影行列
Matrix4x4 MakePerspectiveFovMatrix(float fovY, float aspectRatio, float nearClip, float farClip);

//2. 正射影行列
Matrix4x4 MakeOrthographicMatrix(float left, float top, float right, float bottom, float nearClip, float farClip);

// 3. ビューポート変換行列
Matrix4x4 MakeViewportMatrix(float left, float top, float width, float height, float minDepth, float maxDepth);

//Vector3とfloatの乗算
Vector3 Vec3FloatMultiplication(const Vector3& a, const float& b);

//ベクトル変換
Vector3 TransformNormal(const Vector3& v, const Matrix4x4& m);

//正規化
Vector3 Normalize(const Vector3& a);

//内積
float Dot(const Vector3 v1, const Vector3 v2);

//グリッドの描画
void DrawGrid(const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix);

//球の描画
void DrawSphere(const Sphere& sphere, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix, uint32_t color);

//三角形の描画
void DrawLineTriangle(const Triangle& triangle, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix, uint32_t color);

//
Vector3 Perpendicular(const Vector3& vector);

//平面の描画
void DrawPlane(const Plane& plane, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix, uint32_t color);