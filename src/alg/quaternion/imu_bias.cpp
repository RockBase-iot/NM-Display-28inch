#include "imu_bias.h"

ImuBias::ImuBias(uint16_t samples)
    : _target(samples), _count(0), _ready(false)
{
    _sum[0] = _sum[1] = _sum[2] = 0.0f;
    _bias[0] = _bias[1] = _bias[2] = 0.0f;
}

void ImuBias::reset()
{
    _count  = 0;
    _ready  = false;
    _sum[0] = _sum[1] = _sum[2] = 0.0f;
    _bias[0] = _bias[1] = _bias[2] = 0.0f;
}

bool ImuBias::feed(float gx, float gy, float gz)
{
    if (_ready) return true;

    _sum[0] += gx;
    _sum[1] += gy;
    _sum[2] += gz;
    _count++;

    if (_count >= _target) {
        float inv = 1.0f / (float)_target;
        _bias[0] = _sum[0] * inv;
        _bias[1] = _sum[1] * inv;
        _bias[2] = _sum[2] * inv;
        _ready = true;
        return true;
    }
    return false;
}

void ImuBias::apply(float &gx, float &gy, float &gz) const
{
    if (!_ready) return;
    gx -= _bias[0];
    gy -= _bias[1];
    gz -= _bias[2];
}

void ImuBias::get(float &bx, float &by, float &bz) const
{
    bx = _bias[0];
    by = _bias[1];
    bz = _bias[2];
}
