#version 450

layout(location = 0) in vec3 texcoords;
layout(location = 1) in float occlusion;
layout(location = 0) out vec4 color;

layout(set = 1, binding = 1) uniform sampler2DArray textures;

void main() {
    color = texture(textures, texcoords) * occlusion;
}
