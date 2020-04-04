#version 450

layout(location = 0) in vec2 texcoords;
layout(location = 1) in vec4 normal;

layout(location = 0) out vec4 color_out;

layout(set = 1, binding = 0) uniform sampler2D color;

void main() {
    color_out = texture(color, texcoords);
}
