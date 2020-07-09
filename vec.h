#include <k4a/k4a.h>

typedef struct _vec {
    float x;
    float y;
    float z;
} vec;

// Get a vector between two points
vec twoPointsToVec(struct _xyz from, struct _xyz to);

// Get the angle between two vectors
float twoVecsToAngle(vec vec1, vec vec2);