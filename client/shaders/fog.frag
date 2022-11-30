#version 450

#include "common.h"

layout(location = 0) in vec2 texcoords;

layout(location = 0) out vec4 fog;

layout(input_attachment_index=0, set=0, binding=1) uniform subpassInput depth;

void main() {
    vec4 clip_pos = vec4(texcoords * 2.0 - 1.0, subpassLoad(depth).x, 1.0);
    vec4 scaled_view_pos = inverse_projection * clip_pos;
    // Cancel out perspective, obtaining klein ball position
    vec3 view_pos = scaled_view_pos.xyz / scaled_view_pos.w;
    float view_length = length(view_pos);
    // Convert to true hyperbolic distance, taking care to respect atanh's domain
    float dist = view_length >= 1.0 ? INFINITY : atanh(view_length);
    // Exponential^k fog
    if (length(scaled_view_pos.xy) < 0.01) {
        fog = vec4(0.0, 0.0, 0.0, 0.3);
    } else {
        fog = vec4(0.5, 0.65, 0.9, dist < 0.02 ? 0.5 : exp(-pow(dist * fog_density, 5)));
    }
}
