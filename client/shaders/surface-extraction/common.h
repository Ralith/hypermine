#ifndef SURFACE_EXTRACTION_COMMON_H_
#define SURFACE_EXTRACTION_COMMON_H_

uint invocation_index() {
    return gl_GlobalInvocationID.z * gl_NumWorkGroups.x * gl_NumWorkGroups.y
        + gl_GlobalInvocationID.y * gl_NumWorkGroups.x
        + gl_GlobalInvocationID.x;
}

layout(set = 0, binding = 0) readonly restrict buffer Voxels {
    uint voxel_pair[];
};

uint get_voxel(uvec3 coords) {
    // We assume that all dimensions are equal, except that x occurs three times (once for each
    // face). There's also an extra dispatch per dimension to get the positive-sided border faces.
    uint dim = gl_NumWorkGroups.y - 1;
    uint linear = coords.x + coords.y * dim + coords.z * dim * dim;
    uint pair = voxel_pair[linear / 2];
    return (linear % 2) == 0 ? pair & 0xFFFF : pair >> 16;
}

// A face between a voxel and its neighbor in the -X, -Y, or -Z direction
struct Face {
    // coordinates of the voxel
    uvec3 voxel;
    // [0,6), indicating which axis this face is perpendicular to and whether it's facing inward
    uint axis;
    // whether the normal is facing towards the center of this voxel
    bool inward;
    // contents of the solid voxel incident to the face, which may be a neighbor
    uint material;
};

bool find_face(out Face info) {
    // We only look at negative-facing faces of the current voxel, and iterate one past the end on
    // each dimension to enclose it fully.
    const ivec3 offsets[3] = {
        {-1,  0,  0},
        { 0, -1,  0},
        { 0,  0, -1},
    };
    info.voxel = ivec3(gl_GlobalInvocationID.x % gl_NumWorkGroups.y, gl_GlobalInvocationID.yz);
    info.axis = gl_GlobalInvocationID.x / gl_NumWorkGroups.y;
    ivec3 padding_coords = ivec3(gl_NumWorkGroups.x / 3, gl_NumWorkGroups.yz) - ivec3(1);
    ivec3 neighbor = ivec3(info.voxel) + offsets[info.axis];
    bool neighbor_oob = any(lessThan(neighbor, ivec3(0))) || any(equal(neighbor, padding_coords));
    bool self_oob = any(equal(info.voxel, padding_coords));
    uint neighbor_mat = neighbor_oob ? 0 : get_voxel(neighbor);
    uint self_mat = self_oob ? 0 : get_voxel(info.voxel);
    info.axis += 3 * uint(self_mat == 0);
    info.material = self_mat | neighbor_mat;
    return (neighbor_mat == 0) != (self_mat == 0);
}

bool face_exists() {
    Face dummy;
    return find_face(dummy);
}

#endif
