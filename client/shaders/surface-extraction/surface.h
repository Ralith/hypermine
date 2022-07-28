#ifndef SURFACE_EXTRACTION_SURFACE_H_
#define SURFACE_EXTRACTION_SURFACE_H_

// A face between a voxel and its neighbor in the -X, -Y, or -Z direction
struct Surface {
    // From most to least significant byte, (axis, z, y, x)
    uint pos_axis;
    // From most to least significant byte, (occlusion, <padding>, mat, mat)
    uint occlusion_mat;
};

// [0,2^8)^3
uvec3 get_pos(Surface s) {
    return uvec3(s.pos_axis & 0xFF, (s.pos_axis >> 8) & 0xFF, (s.pos_axis >> 16) & 0xFF);
}

// Identifies the order in which the vertices should be rendered. The vertex positions are the same,
// but winding and diagonal position vary. A flipped diagonal is used to ensure barycentric
// interpolation of ambient occlusion is isotropic, and does not affect texture coordinates
//
// [0,3) are -X/-Y/-Z
// [3,6) are +X/+Y/+Z
// [6,9) are -X/-Y/-Z flipped
// [9,12) are +X/+Y/+Z flipped
uint get_axis(Surface s) {
    return s.pos_axis >> 24;
}

uint get_mat(Surface s) {
    return s.occlusion_mat & 0xFFFF;
}

float get_occlusion(Surface s, uvec2 texcoords) {
    return float((s.occlusion_mat >> (24 + 2 * (texcoords.x | texcoords.y << 1))) & 0x03) / 3.0;
}

Surface surface(uvec3 pos, uint axis, bool reverse, uint mat, uvec4 occlusion) {
    Surface result;
    // Flip the quad if necessary to prevent the triangle dividing line from being parallel to the
    // gradient of ambient occlusion, ensuring isotropy.
    axis += 3 * uint(reverse) + 6 * uint(occlusion.y + occlusion.z > occlusion.x + occlusion.w);
    result.pos_axis = pos.x | pos.y << 8 | pos.z << 16 | axis << 24;
    result.occlusion_mat = mat | occlusion.x << 24 | occlusion.y << 26 | occlusion.z << 28 | occlusion.w << 30;
    return result;
}

#endif
