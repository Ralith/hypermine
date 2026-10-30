#version 450

layout(location = 0) in vec2 texcoords;
layout(location = 1) in vec4 normal;

layout(location = 0) out vec4 color_out;

layout(set = 1, binding = 0) uniform sampler2D color;

void main() {
    vec4 raw_color = texture(color, texcoords);
    // Alpha testing. We can switch this to alpha-to-coverage when we add MSAA.
    if (raw_color.a < 0.5) {
        discard;
    }
    color_out = vec4(raw_color.rgb / raw_color.a, 1.0);
}
