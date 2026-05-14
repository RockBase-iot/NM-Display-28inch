#include "mahony.h"
#include <math.h>

// ─────────────────────────────────────────────────────────────────────────────
// MahonyAHRS implementation
// ─────────────────────────────────────────────────────────────────────────────

MahonyAHRS::MahonyAHRS(float kp, float ki, float dt_s)
    : _kp(kp), _ki(ki), _half_dt(dt_s * 0.5f)
{
    reset();
}

void MahonyAHRS::reset()
{
    _q  = Quat::identity();
    _ix = _iy = _iz = 0.0f;
}

void MahonyAHRS::update(float gx, float gy, float gz,
                        float ax, float ay, float az)
{
    // ── Accelerometer correction ──────────────────────────────────────────
    // Only apply when acceleration magnitude is plausible (not in free-fall
    // or under strong linear acceleration).  Threshold: 0.5g – 2.0g.
    const float accSqNorm = ax*ax + ay*ay + az*az;
    if (accSqNorm > (0.5f*0.5f*9.81f*9.81f) &&
        accSqNorm < (2.0f*2.0f*9.81f*9.81f))
    {
        // Normalize accelerometer vector
        float invNorm = 1.0f / sqrtf(accSqNorm);
        float nx = ax * invNorm;
        float ny = ay * invNorm;
        float nz = az * invNorm;

        // Estimated gravity direction from current quaternion
        float vx, vy, vz;
        quat_gravity(_q, vx, vy, vz);

        // Cross product: error = measured × estimated
        // (rotation error vector in body frame)
        float ex = ny*vz - nz*vy;
        float ey = nz*vx - nx*vz;
        float ez = nx*vy - ny*vx;

        // PI controller
        if (_ki > 0.0f) {
            _ix += ex * _ki * (2.0f * _half_dt);   // dt = 2*half_dt
            _iy += ey * _ki * (2.0f * _half_dt);
            _iz += ez * _ki * (2.0f * _half_dt);
            gx += _ix;
            gy += _iy;
            gz += _iz;
        }
        gx += ex * _kp;
        gy += ey * _kp;
        gz += ez * _kp;
    }

    // ── Quaternion integration (1st-order Runge-Kutta) ────────────────────
    // dq/dt = 0.5 * q ⊗ ω_quat,   ω_quat = [0, gx, gy, gz]
    // Δq = half_dt * q ⊗ ω_quat
    float qw = _q.w, qx = _q.x, qy = _q.y, qz = _q.z;
    _q.w += _half_dt * (-qx*gx - qy*gy - qz*gz);
    _q.x += _half_dt * ( qw*gx + qy*gz - qz*gy);
    _q.y += _half_dt * ( qw*gy - qx*gz + qz*gx);
    _q.z += _half_dt * ( qw*gz + qx*gy - qy*gx);

    // Re-normalize
    _q.normalize();
}

void MahonyAHRS::get_integral(float &ix, float &iy, float &iz) const
{
    ix = _ix; iy = _iy; iz = _iz;
}
