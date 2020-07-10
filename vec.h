#pragma once

#include <k4a/k4a.h>

typedef struct _vec {
    float x;
    float y;
    float z;

    // Get a vector between two points
    _vec(k4a_float3_t from, k4a_float3_t to) {
        x = to.xyz.x - from.xyz.x;
        y = to.xyz.y - from.xyz.y;
        z = to.xyz.z - from.xyz.z;
    }
} vec;

// Get the angle between two vectors
float twoVecsToAngle(vec vec1, vec vec2);