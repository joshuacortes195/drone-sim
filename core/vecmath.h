#pragma once

#include <cmath>

// 3d vector for positions, velocities, forces and torques
struct Vec3 {
  double x = 0, y = 0, z = 0;
};

inline Vec3 operator+(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 operator-(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3 operator*(Vec3 a, double s) { return {a.x * s, a.y * s, a.z * s}; }
inline Vec3 operator*(double s, Vec3 a) { return a * s; }
inline double dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline double norm(Vec3 a) { return std::sqrt(dot(a, a)); }

// cross product, used for torques (r x F) and the spin wobble term
inline Vec3 cross(Vec3 a, Vec3 b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

// multiplies matching parts, used for the diagonal inertia
inline Vec3 mul(Vec3 a, Vec3 b) { return {a.x * b.x, a.y * b.y, a.z * b.z}; }

// orientation as a hamilton quaternion w, x, y, z that turns body vectors into world vectors
struct Quat {
  double w = 1, x = 0, y = 0, z = 0;
};

inline Quat operator+(Quat a, Quat b) { return {a.w + b.w, a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Quat operator*(Quat a, double s) { return {a.w * s, a.x * s, a.y * s, a.z * s}; }

// chains two rotations together
inline Quat operator*(Quat a, Quat b) {
  return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
          a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
          a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
          a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}

inline Quat conj(Quat q) { return {q.w, -q.x, -q.y, -q.z}; }

// scales back to length 1 so it stays a real rotation
inline Quat normalized(Quat q) {
  double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  return q * (1 / n);
}

// rotation by angle radians about a unit axis
inline Quat axis_angle(Vec3 axis, double angle) {
  Vec3 v = axis * std::sin(angle / 2);
  return {std::cos(angle / 2), v.x, v.y, v.z};
}

// turns a body frame vector into the world frame (q v q*)
inline Vec3 rotate(Quat q, Vec3 v) {
  Quat r = q * Quat{0, v.x, v.y, v.z} * conj(q);
  return {r.x, r.y, r.z};
}

// which way the nose points on the map, radians from east toward north (the z in a z-y-x turn)
inline double heading(Quat q) {
  return std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
}
