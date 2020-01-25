#version 450

#include "common.h"

layout(location = 0) in vec2 texcoords;

layout(location = 0) out vec4 color;

void main() {
  color = vec4(mix(vec3(0.25, 0.25, 1.0), vec3(0.5, 0.5, 1.0), texcoords.y), 1.0);
}
