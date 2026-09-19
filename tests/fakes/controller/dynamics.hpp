#pragma once
#include <cstddef>

// Deterministic pose-dependent model; no hardware or URDF needed.
class Dynamics {
public:
    double bias = 1.0;
    bool valid = true;
    size_t count = 7;
    bool IsValid() const { return valid; }
    size_t GetJointCount() const { return count; }
    void GetGravity(const double* q, double* gravity) {
        for (size_t i = 0; i < count; ++i) gravity[i] = bias + q[i];
    }
    void GetCoriolis(const double*, const double*, double* coriolis) {
        for (size_t i = 0; i < count; ++i) coriolis[i] = 0.0;
    }
};
