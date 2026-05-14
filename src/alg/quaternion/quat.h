#pragma once
#include <math.h>

// ─────────────────────────────────────────────────────────────────────────────
// quat.h  —  Quaternion type and basic math helpers
//
// Convention:  q = w + xi + yj + zk  (Hamilton convention, w is the scalar)
// All functions are pure / inline; no dynamic allocation.
// ─────────────────────────────────────────────────────────────────────────────

struct Quat {
    float w, x, y, z;

    // Identity quaternion (no rotation)
    static inline Quat identity() { return {1.0f, 0.0f, 0.0f, 0.0f}; }

    // L2 norm
    inline float norm() const {
        return sqrtf(w*w + x*x + y*y + z*z);
    }

    // Normalize in-place; returns false if degenerate (should never happen in AHRS)
    inline bool normalize() {
        float n = norm();
        if (n < 1e-6f) return false;
        float inv = 1.0f / n;
        w *= inv; x *= inv; y *= inv; z *= inv;
        return true;
    }

    // Hamilton product:  this ⊗ rhs
    inline Quat operator*(const Quat &r) const {
        return {
            w*r.w - x*r.x - y*r.y - z*r.z,
            w*r.x + x*r.w + y*r.z - z*r.y,
            w*r.y - x*r.z + y*r.w + z*r.x,
            w*r.z + x*r.y - y*r.x + z*r.w
        };
    }

    // Scalar multiply
    inline Quat operator*(float s) const { return {w*s, x*s, y*s, z*s}; }

    // Component-wise add
    inline Quat operator+(const Quat &r) const {
        return {w+r.w, x+r.x, y+r.y, z+r.z};
    }
    inline Quat &operator+=(const Quat &r) {
        w+=r.w; x+=r.x; y+=r.y; z+=r.z; return *this;
    }

    // Conjugate / inverse (unit quaternion: conjugate == inverse)
    inline Quat conjugate() const { return {w, -x, -y, -z}; }
};

// ─── Euler angles (ZYX / yaw-pitch-roll, radians) ────────────────────────────
struct Euler {
    float roll;   // φ  rotation around X  (rad)
    float pitch;  // θ  rotation around Y  (rad)
    float yaw;    // ψ  rotation around Z  (rad)
};

// Convert unit quaternion → Euler angles (ZYX intrinsic, radians)
inline Euler quat_to_euler(const Quat &q)
{
    Euler e;
    // roll  (x-axis)
    float sinr_cosp = 2.0f * (q.w*q.x + q.y*q.z);
    float cosr_cosp = 1.0f - 2.0f*(q.x*q.x + q.y*q.y);
    e.roll  = atan2f(sinr_cosp, cosr_cosp);

    // pitch (y-axis) — clamp for numerical safety
    float sinp = 2.0f*(q.w*q.y - q.z*q.x);
    if      (sinp >=  1.0f) e.pitch =  1.5707963f;   //  π/2
    else if (sinp <= -1.0f) e.pitch = -1.5707963f;   // -π/2
    else                    e.pitch =  asinf(sinp);

    // yaw   (z-axis)
    float siny_cosp = 2.0f*(q.w*q.z + q.x*q.y);
    float cosy_cosp = 1.0f - 2.0f*(q.y*q.y + q.z*q.z);
    e.yaw   = atan2f(siny_cosp, cosy_cosp);
    return e;
}

// Gravity vector estimated from the current quaternion.
// This is the third column of the rotation matrix R = q ⊗ [0,0,1] ⊗ q*
// Used by the Mahony/Madgwick filter to correct gyro drift.
inline void quat_gravity(const Quat &q, float &vx, float &vy, float &vz)
{
    vx = 2.0f*(q.x*q.z - q.w*q.y);
    vy = 2.0f*(q.w*q.x + q.y*q.z);
    vz = q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z;
}
