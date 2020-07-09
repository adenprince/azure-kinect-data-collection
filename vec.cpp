#define _USE_MATH_DEFINES
#include <cmath>

#include "vec.h"

float twoVecsToAngle(vec vec1, vec vec2) {
    float dotProduct = vec1.x * vec2.x + vec1.y * vec2.y + vec1.z * vec2.z;
    float vec1Mag = sqrtf(vec1.x * vec1.x + vec1.y * vec1.y + vec1.z * vec1.z);
    float vec2Mag = sqrtf(vec2.x * vec2.x + vec2.y * vec2.y + vec2.z * vec2.z);
    float res = acosf(dotProduct / (vec1Mag * vec2Mag)) * 180 / (float)M_PI;
    return res;
}