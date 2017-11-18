typedef float vt;
// Vector3 클래스
// 3차원 벡터값을 관리하는 클래스입니다.
class Vector3 {
private:
	vt x = 0;
	vt y = 0;
	vt z = 0;

public:
	// constructor
	Vector3() : x(0), y(0), z(0) {};
	Vector3(const Vector3 &v) : x(v.x), y(v.y), z(v.z) {};
	Vector3(const vt &_x, const vt &_y, const vt &_z) : x(_x), y(_y), z(_z) {};


	// setter
	void set(const Vector3 &v) { x = v.x; y = v.y; z = v.z; }
	void set(const vt &_x, const vt &_y, const vt &_z) { x = _x; y = _y; z = _z; }
	void setX(const vt &_x) { x = _x; }
	void setY(const vt &_y) { y = _y; }
	void setZ(const vt &_z) { z = _z; }
	void setdX(const vt &_x) { x += _x; }
	void setdY(const vt &_y) { y += _y; }
	void setdZ(const vt &_z) { z += _z; }


	// getter

	vt getX() { return x; }
	vt getY() { return y; }
	vt getZ() { return z; }
	// 벡터의 크기를 구합니다.
	float getMagnitude() { return sqrt(x*x + y*y + z*z); }

	// 벡터의 단위 벡터를 구합니다.
	Vector3 getDirection() { return *this / getMagnitude(); }

	// x축과 벡터 사이의 각도의 cos 값을 구합니다.
	float getDirCosAlpha() { return (float)x / getMagnitude(); }

	// y축과 벡터 사이의 각도의 cos 값을 구합니다.
	float getDirCosBeta() { return (float)y / getMagnitude(); }

	// z축과 벡터 사이의 각도의 cos 값을 구합니다.
	float getDirCosGamma() { return (float)z / getMagnitude(); }


	// 벡터 연산

	Vector3 operator+(const Vector3 &v) { return Vector3(x + v.x, y + v.y, z + v.z); }
	Vector3 operator-(const Vector3 &v) { return Vector3(x - v.x, y - v.y, z - v.z); }

	Vector3 operator+(const vt &d) { return Vector3(x + d, y + d, z + d); }
	Vector3 operator-(const vt &d) { return Vector3(x - d, y - d, z - d); }

	void operator+=(const Vector3 &v) { x += v.x; y += v.y; z += v.z; }
	void operator-=(const Vector3 &v) { x -= v.x; y -= v.y; z -= v.z; }

	void operator+=(const vt &d) { x += d; y += d; z += d; }
	void operator-=(const vt &d) { x -= d; y -= d; z -= d; }

	Vector3 operator*(const vt &d) { return Vector3(x*d, y*d, z*d); }
	Vector3 operator/(const vt &d) { return Vector3(x / d, y / d, z / d); }

	void operator*=(const vt &d) { x *= d; y *= d; z *= d; }
	void operator/=(const vt &d) { x /= d; y /= d; z /= d; }

	vt dotTo(const Vector3 &v) { return x*v.x + y*v.y + z*v.z; }
	Vector3 crossTo(const Vector3 &v) { return Vector3(y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x); }


	// Serial print
	void printxyz() {
		Serial.print(x);
		Serial.print(' ');
		Serial.print(y);
		Serial.print(' ');
		Serial.print(z);
	}

	void printxyzln() { printxyz(); Serial.println(); }

	void printcos() {
		Serial.print(getDirCosAlpha());
		Serial.print(' ');
		Serial.print(getDirCosBeta());
		Serial.print(' ');
		Serial.print(getDirCosGamma());
	}


	// destructor(auto)
	// ~Vector3();
};