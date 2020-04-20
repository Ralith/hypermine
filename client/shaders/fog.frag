#version 450

#include "common.h"

layout(location = 0) in vec2 texcoords;

layout(location = 0) out vec4 fog;

layout(input_attachment_index=0, set=0, binding=1) uniform subpassInput depth;

void main() {
    vec4 clip_pos = vec4(texcoords * 2.0 - 1.0, subpassLoad(depth).x, 1.0);
    vec4 scaled_view_pos = inverse_projection * clip_pos;
    // Cancel out perspective
    vec3 view_pos = scaled_view_pos.xyz / scaled_view_pos.w;
    // Hyperbolic distance
    float dist = atanh(min(length(view_pos), 1));
    // Exponential-squared fog
    fog = vec4(0.2, 0.15, 0.8, exp(-pow(dist * fog_density, 2)));
}
