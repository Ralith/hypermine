#version 450

#include "common.h"
#include "surface-extraction/surface.h"

layout(location = 0) out vec2 texcoords_out;

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

const vec2 texcoords[2][6] = {
    {{0, 0}, {0, 1}, {1, 1}, {0, 0}, {1, 1}, {1, 0}},
    {{0, 0}, {1, 1}, {0, 1}, {0, 0}, {1, 0}, {1, 1}},
};

void main()  {
    uint index = gl_VertexIndex / 6;
    uint vertex = gl_VertexIndex % 6;
    Surface s = surfaces[index];
    uvec3 pos = get_pos(s);
    uint axis = get_axis(s);
    texcoords_out = texcoords[axis / 3][vertex];
    gl_Position = projection * vec4(vertices[axis][vertex] + pos, 1);
}
