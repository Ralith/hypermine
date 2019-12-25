#ifndef SURFACE_EXTRACTION_SURFACE_H_
#define SURFACE_EXTRACTION_SURFACE_H_

struct Surface {
    uint pos_axis;
    uint mat;
};

uvec3 get_pos(Surface s) {
    return uvec3(s.pos_axis & 0xFF, (s.pos_axis >> 8) & 0xFF, (s.pos_axis >> 16) & 0xFF);
}

uint get_axis(Surface s) {
    return s.pos_axis >> 24;
}

Surface surface(uvec3 pos, uint axis, uint mat) {
    Surface result;
    result.pos_axis = pos.x | pos.y << 8 | pos.z << 16 | axis << 24;
    result.mat = mat;
    return result;
}

#endif
