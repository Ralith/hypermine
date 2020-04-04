#version 450

#include "common.h"

layout(location = 0) in vec3 position;
layout(location = 1) in vec2 texcoords;
layout(location = 2) in vec3 normal;

layout(location = 0) out vec2 texcoords_out;
layout(location = 1) out vec4 normal_out;

layout(push_constant) uniform PushConstants {
    mat4 transform;
};

void main() {
    gl_Position = projection * transform * vec4(position, 1);
    texcoords_out = texcoords;
    normal_out = transform * vec4(normal, 0);
}
