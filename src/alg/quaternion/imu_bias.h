#pragma once
#include <stdint.h>
#include <string.h>

// ─────────────────────────────────────────────────────────────────────────────
// imu_bias.h  —  Static gyroscope zero-rate offset (ZRO) calibration
//
// Usage:
//   1. At startup, while the device is stationary, call feed() for N samples.
//   2. Call ready() to check if calibration is complete.
//   3. Subtract the bias from every subsequent gyro reading before passing
//      to MahonyAHRS::update().
//
// Notes:
//   - Only gyroscope bias is handled here.  Accelerometer bias (offset from
//     1g on the vertical axis) is handled dynamically by the Mahony filter
//     (via the PI integral term) and is much less critical.
//   - The calibration is NOT stored to flash; re-run on every power-on.
//     If you want persistent calibration, save the result to NVS yourself.
// ─────────────────────────────────────────────────────────────────────────────

class ImuBias {
public:
    // samples : number of samples to average (e.g. 200 @ 50Hz ≈ 4 s)
    explicit ImuBias(uint16_t samples = 200);

    // Reset and start a new calibration session.
    void reset();

    // Feed one raw gyro sample [rad/s or any consistent unit].
    // Returns true once calibration completes (i.e., on the last sample).
    bool feed(float gx, float gy, float gz);

    // True after feed() has been called 'samples' times.
    bool ready() const { return _ready; }

    // Remove bias from a gyro reading in-place.
    // If not yet ready, does nothing.
    void apply(float &gx, float &gy, float &gz) const;

    // Access the computed bias values directly.
    void get(float &bx, float &by, float &bz) const;

private:
    uint16_t _target;         // number of samples required
    uint16_t _count;          // samples received so far
    float    _sum[3];         // running sum for each axis
    float    _bias[3];        // computed bias (mean)
    bool     _ready;
};
