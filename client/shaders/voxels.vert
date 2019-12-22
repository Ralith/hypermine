#version 450

#include "common.h"

layout (location = 0) in uvec3 position;

void main()  {
    gl_Position = projection * vec4(position, 1);
}
