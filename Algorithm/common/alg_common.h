#pragma once

// Algorithm common definitions
// Keep this header lightweight and platform-agnostic.

#include <cstdint>
#include <cstddef>

// Unified math constants
#ifndef PI
#define PI 3.14159265358979323846f
#endif

#ifndef TWO_PI
#define TWO_PI (2.0f * PI)
#endif

#ifndef DEG_TO_RAD
#define DEG_TO_RAD (PI / 180.0f)
#endif

#ifndef RAD_TO_DEG
#define RAD_TO_DEG (180.0f / PI)
#endif

// RPM conversions
#ifndef RPM_2_ANGLE_PER_SEC
#define RPM_2_ANGLE_PER_SEC 6.0f // ×360°/60sec
#endif

#ifndef RPM_2_RAD_PER_SEC
#define RPM_2_RAD_PER_SEC (TWO_PI / 60.0f) // ×2pi/60sec
#endif

#ifndef RPM_TO_RADPS
#define RPM_TO_RADPS RPM_2_RAD_PER_SEC
#endif
