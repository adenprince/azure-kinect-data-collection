/* Aden Prince
 * HiMER Lab at U. of Illinois, Chicago
 * Azure Kinect Data Collection
 * 
 * vec.h
 * Contains code that defines a vector type.
 */

#include <k4a/k4a.h>

typedef struct _vec {
    float x;
    float y;
    float z;

    // Get a vector between two points
    _vec(k4a_float3_t& from, k4a_float3_t& to) {
        x = to.xyz.x - from.xyz.x;
        y = to.xyz.y - from.xyz.y;
        z = to.xyz.z - from.xyz.z;
    }
} vec;