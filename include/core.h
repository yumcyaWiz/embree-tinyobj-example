#ifndef _CORE_H
#define _CORE_H
#include <cmath>
#include <limits>

constexpr float RAY_EPS = 1e-5f;

struct Vec3 {
  float v[3];

  Vec3() { v[0] = v[1] = v[2] = 0; }
  Vec3(float x) { v[0] = v[1] = v[2] = x; }
  Vec3(float x, float y, float z) {
    v[0] = x;
    v[1] = y;
    v[2] = z;
  }

  float operator[](int i) const { return v[i]; }
  float& operator[](int i) { return v[i]; }

  Vec3 operator-() const { return Vec3(-v[0], -v[1], -v[2]); }

  Vec3& operator+=(const Vec3& v) {
    for (int i = 0; i < 3; ++i) {
      this->v[i] += v[i];
    }
    return *this;
  }
  Vec3& operator*=(const Vec3& v) {
    for (int i = 0; i < 3; ++i) {
      this->v[i] *= v[i];
    }
    return *this;
  }
  Vec3& operator/=(const Vec3& v) {
    for (int i = 0; i < 3; ++i) {
      this->v[i] /= v[i];
    }
    return *this;
  }
};

inline Vec3 operator+(const Vec3& v1, const Vec3& v2) {
  return Vec3(v1[0] + v2[0], v1[1] + v2[1], v1[2] + v2[2]);
}
inline Vec3 operator+(const Vec3& v1, float k) {
  return Vec3(v1[0] + k, v1[1] + k, v1[2] + k);
}
inline Vec3 operator+(float k, const Vec3& v2) { return v2 + k; }

inline Vec3 operator-(const Vec3& v1, const Vec3& v2) {
  return Vec3(v1[0] - v2[0], v1[1] - v2[1], v1[2] - v2[2]);
}
inline Vec3 operator-(const Vec3& v1, float k) {
  return Vec3(v1[0] - k, v1[1] - k, v1[2] - k);
}
inline Vec3 operator-(float k, const Vec3& v2) {
  return Vec3(k - v2[0], k - v2[1], k - v2[2]);
}

inline Vec3 operator*(const Vec3& v1, const Vec3& v2) {
  return Vec3(v1[0] * v2[0], v1[1] * v2[1], v1[2] * v2[2]);
}
inline Vec3 operator*(const Vec3& v1, float k) {
  return Vec3(v1[0] * k, v1[1] * k, v1[2] * k);
}
inline Vec3 operator*(float k, const Vec3& v2) { return v2 * k; }

inline Vec3 operator/(const Vec3& v1, const Vec3& v2) {
  return Vec3(v1[0] / v2[0], v1[1] / v2[1], v1[2] / v2[2]);
}
inline Vec3 operator/(const Vec3& v1, float k) {
  return Vec3(v1[0] / k, v1[1] / k, v1[2] / k);
}
inline Vec3 operator/(float k, const Vec3& v2) {
  return Vec3(k / v2[0], k / v2[1], k / v2[2]);
}

inline float dot(const Vec3& v1, const Vec3& v2) {
  return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

inline Vec3 cross(const Vec3& v1, const Vec3& v2) {
  return Vec3(v1[1] * v2[2] - v1[2] * v2[1], v1[2] * v2[0] - v1[0] * v2[2],
              v1[0] * v2[1] - v1[1] * v2[0]);
}

inline float length(const Vec3& v) { return std::sqrt(dot(v, v)); }
inline float length2(const Vec3& v) { return dot(v, v); }
inline Vec3 normalize(const Vec3& v) { return v / length(v); }

class Ray {
  Vec3 origin;
  Vec3 direction;
  static constexpr float tmin = RAY_EPS;
  mutable float tmax = std::numeric_limits<float>::max();

  Ray() {}
  Ray(const Vec3& origin, const Vec3& direction)
      : origin(origin), direction(direction) {}

  Vec3 operator()(float t) const { return origin + t * direction; }
};

struct SurfaceInfo {
  Vec3 position;
  Vec3 normal;
  Vec3 dpdu;
  Vec3 dpdv;
};

// forward declaration
class Primitive;

struct IntersectInfo {
  float t;
  SurfaceInfo surfaceInfo;
  const Primitive* hitPrimitive;
};

#endif