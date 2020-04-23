#ifndef COMMON_H
#define COMMON_H

const float PI = 3.14159265;
const float INFINITY = 1.0 / 0.0;

layout(set = 0, binding = 0) uniform Common {
    // Maps local node space to clip space
    mat4 view_projection;
    // Maps clip space to view space
    mat4 inverse_projection;
    float fog_density;
    float time;
};

#endif
