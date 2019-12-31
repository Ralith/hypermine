#version 450

layout(location = 0) in vec2 texcoords;
layout(location = 0) out vec4 color;

void main() {
    color = vec4(texcoords.xy, 0, 0);
}
