#version 450

#include "common.h"
#include "surface-extraction/surface.h"

layout(location = 0) out vec3 texcoords_out;
layout(location = 1) out float occlusion;

layout(set = 1, binding = 0) readonly restrict buffer Surfaces {
    Surface surfaces[];
};

const uvec3 vertices[6][6] = {
    {{0, 0, 0}, {0, 0, 1}, {0, 1, 1}, {0, 0, 0}, {0, 1, 1}, {0, 1, 0}}, // -X
    {{0, 0, 0}, {1, 0, 0}, {1, 0, 1}, {0, 0, 0}, {1, 0, 1}, {0, 0, 1}}, // -Y
    {{0, 0, 0}, {0, 1, 0}, {1, 1, 0}, {0, 0, 0}, {1, 1, 0}, {1, 0, 0}}, // -Z

    {{0, 0, 0}, {0, 1, 1}, {0, 0, 1}, {0, 0, 0}, {0, 1, 0}, {0, 1, 1}}, // +X
    {{0, 0, 0}, {1, 0, 1}, {1, 0, 0}, {0, 0, 0}, {0, 0, 1}, {1, 0, 1}}, // +Y
    {{0, 0, 0}, {1, 1, 0}, {0, 1, 0}, {0, 0, 0}, {1, 0, 0}, {1, 1, 0}}, // +Z
};

const uvec2 texcoords[2][6] = {
    {{0, 0}, {0, 1}, {1, 1}, {0, 0}, {1, 1}, {1, 0}},
    {{0, 0}, {1, 1}, {0, 1}, {0, 0}, {1, 0}, {1, 1}},
};

void main()  {
    uint index = gl_VertexIndex / 6;
    uint vertex = gl_VertexIndex % 6;
    Surface s = surfaces[index];
    uvec3 pos = get_pos(s);
    uint axis = get_axis(s);
    uvec2 uv = texcoords[uint(axis >= 3)][vertex];
    texcoords_out = vec3(uv, get_mat(s) - 1);
    occlusion = get_occlusion(s, uv);
    gl_Position = projection * vec4(vertices[axis][vertex] + pos, 1);
}
