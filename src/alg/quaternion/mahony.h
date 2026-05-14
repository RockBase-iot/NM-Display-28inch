#pragma once
#include "quat.h"

// ─────────────────────────────────────────────────────────────────────────────
// mahony.h  —  Mahony AHRS filter (6-DOF, accelerometer + gyroscope only)
//
// Reference:
//   Mahony, R. et al., "Nonlinear Complementary Filters on the Special
//   Orthogonal Group", IEEE TAC 2008.
//
// How it works:
//   1. Integrate gyroscope data to propagate the quaternion (predict step).
//   2. Estimate the gravity direction from the current quaternion.
//   3. Compute the cross-product error between estimated and measured gravity.
//   4. Run a PI controller on the error to correct the gyro reading before
//      integration (Kp term) and to accumulate a bias estimate (Ki term).
//
// Without magnetometer there is NO yaw observability — yaw will drift.
// Yaw is still numerically valid but its absolute value is meaningless.
//
// Tuning:
//   Kp  — proportional gain.  Higher = faster convergence, but more accel
//          noise enters the attitude.  Typical: 0.5 – 2.0.
//   Ki  — integral gain.  Removes steady gyro bias.  Typical: 0.005 – 0.01.
//          Set to 0 if using ImuBias calibration (bias subtracted externally).
// ─────────────────────────────────────────────────────────────────────────────

class MahonyAHRS {
public:
    // kp, ki: filter gains; dt_s: fixed sample interval in seconds
    MahonyAHRS(float kp = 1.0f, float ki = 0.005f, float dt_s = 0.02f);

    // Reset to identity quaternion and zero integral.
    void reset();

    // Feed one sample.
    //   gx, gy, gz : gyroscope   [rad/s]  (bias already removed)
    //   ax, ay, az : accelerometer [any unit, does not need to be g]
    //                Pass (0,0,0) to skip accel correction this cycle.
    void update(float gx, float gy, float gz,
                float ax, float ay, float az);

    // Current attitude quaternion (unit quaternion, updated by update()).
    const Quat &quaternion() const { return _q; }

    // Convenience: Euler angles in radians (ZYX / roll-pitch-yaw).
    Euler euler() const { return quat_to_euler(_q); }

    // Read back the PI integral accumulator (useful for diagnostics).
    void get_integral(float &ix, float &iy, float &iz) const;

private:
    float _kp, _ki, _half_dt;   // half_dt = dt/2 for quaternion derivative
    Quat  _q;                   // current orientation
    float _ix, _iy, _iz;       // PI integral term (rad/s)
};
