#version 450

#include "common.h"
#include "surface-extraction/surface.h"

layout(location = 0) out vec3 texcoords_out;
layout(location = 1) out float occlusion;

layout(set = 1, binding = 0) readonly restrict buffer Surfaces {
    Surface surfaces[];
};

layout(set = 1, binding = 1) readonly restrict buffer Transforms {
    mat4 transform[];
};

layout(push_constant) uniform PushConstants {
    uint draw_index;
    uint dimension;
    bool reflected;
};

// Each set of 6 vertices makes a ring around the quad, with the middle and start/end vertices
// duplicated. Note that the sign only indicates the winding of the face; all faces contain the
// origin regardless.
const uvec3 vertices[12][6] = {
    {{0, 0, 0}, {0, 0, 1}, {0, 1, 1}, {0, 1, 1}, {0, 1, 0}, {0, 0, 0}}, // -X
    {{0, 0, 0}, {1, 0, 0}, {1, 0, 1}, {1, 0, 1}, {0, 0, 1}, {0, 0, 0}}, // -Y
    {{0, 0, 0}, {0, 1, 0}, {1, 1, 0}, {1, 1, 0}, {1, 0, 0}, {0, 0, 0}}, // -Z

    {{0, 0, 0}, {0, 1, 0}, {0, 1, 1}, {0, 1, 1}, {0, 0, 1}, {0, 0, 0}}, // +X
    {{0, 0, 0}, {0, 0, 1}, {1, 0, 1}, {1, 0, 1}, {1, 0, 0}, {0, 0, 0}}, // +Y
    {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {1, 1, 0}, {0, 1, 0}, {0, 0, 0}}, // +Z

    // Versions of the above rotated 90 degrees so the diagonal goes the other way, used to improve
    // the consistency of barycentric interpolation of ambient occlusion
    {{0, 0, 1}, {0, 1, 1}, {0, 1, 0}, {0, 1, 0}, {0, 0, 0}, {0, 0, 1}}, // -X
    {{1, 0, 0}, {1, 0, 1}, {0, 0, 1}, {0, 0, 1}, {0, 0, 0}, {1, 0, 0}}, // -Y
    {{0, 1, 0}, {1, 1, 0}, {1, 0, 0}, {1, 0, 0}, {0, 0, 0}, {0, 1, 0}}, // -Z

    {{0, 0, 1}, {0, 0, 0}, {0, 1, 0}, {0, 1, 0}, {0, 1, 1}, {0, 0, 1}}, // +X
    {{1, 0, 0}, {0, 0, 0}, {0, 0, 1}, {0, 0, 1}, {1, 0, 1}, {1, 0, 0}}, // +Y
    {{0, 1, 0}, {0, 0, 0}, {1, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}}  // +Z
};

const uvec2 texcoords[4][6] = {
    {{0, 0}, {0, 1}, {1, 1}, {1, 1}, {1, 0}, {0, 0}},
    {{0, 0}, {1, 0}, {1, 1}, {1, 1}, {0, 1}, {0, 0}},
    // Rotated versions
    {{0, 1}, {1, 1}, {1, 0}, {1, 0}, {0, 0}, {0, 1}},
    {{0, 1}, {0, 0}, {1, 0}, {1, 0}, {1, 1}, {0, 1}},
};

// Map [0,1]^3 to a point in a cubic tile
vec4 cube_vertex(vec3 point) {
    const float a = sqrt(1+sqrt(5))/2;
    const float w = sqrt(3*pow(a,2)+1);
    return vec4((point * 2 - 1) * vec3(a, a, a), w);
}

void main()  {
    uint index = gl_VertexIndex / 6;
    uint vertex = gl_VertexIndex % 6;
    if (reflected) {
        vertex = 5 - vertex;
    }
    Surface s = surfaces[index];
    uvec3 pos = get_pos(s);
    uint axis = get_axis(s);
    uvec2 uv = texcoords[axis / 3][vertex];
    texcoords_out = vec3(uv, get_mat(s) - 1);
    occlusion = get_occlusion(s, uv);
    vec3 relative_coords = vertices[axis][vertex] + pos;
    gl_Position = projection * transform[draw_index] * cube_vertex(relative_coords / dimension);
}
