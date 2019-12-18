#ifndef COMMON_H
#define COMMON_H

const float PI = 3.14159265;

layout(set = 0, binding = 0) uniform Common {
    mat4 projection;
    float time;
};

#endif
