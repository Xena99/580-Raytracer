#pragma once
#include <cmath>
#include <vector>
struct Matrix;
struct Vector3;

class Raytracer {

	//State machine
public:
	Matrix modelMatrix; //Current matrix for object space to world space
	Matrix inverseModelMatrix; //Current matrix for World space to object space
	struct Pixel {
		short r, g, b;
	};

	struct Display {
		Pixel* frameBuffer;
		int xRex, yRes;
	};

	struct RaycastHitInfo {
		Vector3 hitPoint;
		Vector3 normal;
	};

	struct Ray {
		Vector3 origin; //World space
		Vector3 direction; //Normalized
	};

	struct Vertex {
		Vector3 vertexPos;
		Vector3 vertexNormal;
		Vector2 texture;
	};

	struct Triangle {
		Vertex v0;
		Vertex v1;
		Vertex v2;
	};

	struct Mesh {
		std::vector<Triangle> triangles;
	};

	//Todo: need this later for generating ray
	//Perspective & Camera
	struct Camera
	{
		NtMatrix        viewMatrix;		/* world to image space */
		NtMatrix        projectMatrix;  /* perspective projection */
		Vector3 viewDirection;
		Vector3 from;
		Vector3 to;
		float near, far, right, left, top, bottom;
		int xRes;
		int yRes;
	};


	//This is the main function against the whole scene, but we will need some helper raycasts
	//Returns true if intersects with an object
	bool Raycast(Ray& ray, RaycastHitInfo& hitInfo);


	//Helper ray cast functions
	//Möller–Trumbore intersection algorithm
	bool RaycastTriangle(Ray& ray, Triangle& triangle, RaycastHitInfo hitInfo);
};

struct Vector3 {
	union {
		struct {
			float x, y, z;

		};
		float v[3];
	};

	Vector3() : x(0.0f), y(0.0f), z(0.0f) {}

	// Constructor for initializing Vector3 from individual x, y, and z values
	Vector3(float xVal, float yVal, float zVal) : x(xVal), y(yVal), z(zVal) {}

	// Constructor for initializing Vector3 from an array of 3 values
	Vector3(const float values[3]) : x(values[0]), y(values[1]), z(values[2]) {}

	Vector3 operator/(const float scalar) const {
		return { x / scalar, y / scalar, z / scalar };
	}

	Vector3 operator*(const float scalar) const {
		return { x * scalar, y * scalar, z * scalar };
	}

	//Component wise multiplication
	Vector3 operator*(const Vector3& other) const {
		return { x * other.x, y * other.y, z * other.z };
	}

	// Vector subtraction
	Vector3 operator-(const Vector3& other) const {
		return { x - other.x, y - other.y, z - other.z };
	}

	Vector3 operator+(const Vector3& other) const {
		return { x + other.x, y + other.y, z + other.z };
	}

	// Normalize the vector
	void normalize() {
		float length = std::sqrt(x * x + y * y + z * z);
		x /= length; y /= length; z /= length;
	}

	// Cross product
	static Vector3 cross(const Vector3& a, const Vector3& b) {
		return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
	}

	Vector3 cross(const Vector3& other) {
		return { y * other.z - z * other.y, z * other.x - x * other.z, x * other.y - y * other.x };
	}

	// Dot product
	static float dot(const Vector3& a, const Vector3& b) {
		return a.x * b.x + a.y * b.y + a.z * b.z;
	}

	float dot(const Vector3& other) {
		return x * other.x + y * other.y + z * other.z;
	}

	float dot(const Vector3& other) const {
		return x * other.x + y * other.y + z * other.z;
	}

	static Vector3 reflect(const Vector3& I, const Vector3& N) {
		float IDotN = I.dot(N);
		IDotN *= 2;
		Vector3 _N = N * IDotN;
		return I - _N;
	}
};

struct Vector2 {
	union {
		struct {
			float x, y;
		};
		float v[2];
	};
	//Default constructor
	Vector2() : x(0.0f), y(0.0f) {}

	// Constructor for initializing Vector2 from individual x and y values
	Vector2(float xVal, float yVal) : x(xVal), y(yVal) {}

	// Constructor for initializing Vector2 from an array of 2 values
	Vector2(const float values[2]) : x(values[0]), y(values[1]) {}
};

typedef struct Matrix {
	float m[4][4];

	// Overload the subscript operator to allow direct access to internal array
	float* operator[](int index) {
		return m[index];
	}
	const float* operator[](int index) const {
		return m[index];
	}

	Matrix operator*(const Matrix& other) const {
		Matrix result;
		for (int i = 0; i < 4; ++i) {
			for (int j = 0; j < 4; ++j) {
				result.m[i][j] = 0;
				for (int k = 0; k < 4; ++k) {
					result.m[i][j] += this->m[i][k] * other.m[k][j];
				}
			}
		}
		return result;
	}

	Matrix operator+(const Matrix& other) const {
		Matrix result;
		for (int i = 0; i < 4; ++i) {
			for (int j = 0; j < 4; ++j) {
				result.m[i][j] = this->m[i][j] + other.m[i][j];
			}
		}
		return result;
	}

	Matrix operator-(const Matrix& other) const {
		Matrix result;
		for (int i = 0; i < 4; ++i) {
			for (int j = 0; j < 4; ++j) {
				result.m[i][j] = this->m[i][j] - other.m[i][j];
			}
		}
		return result;
	}

	Matrix transpose() const {
		Matrix result;
		for (int i = 0; i < 4; ++i) {
			for (int j = 0; j < 4; ++j) {
				result.m[i][j] = this->m[j][i];
			}
		}
		return result;
	}
} NtMatrix;
