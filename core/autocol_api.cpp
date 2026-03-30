/*
 * autocol_api.cpp  –  DLL entry point / exported C functions
 */
#include "autocol_api.h"

/* Declarations from other TUs */
int fit_obb(const float* verts, int nv, const int* tris, int nt, OBBResult* out);
int fit_aabb(const float* verts, int nv, const int* tris, int nt, OBBResult* out);
int fit_multi_obb(const float* verts, int nv, const int* tris, int nt,
                  int K, int use_obb, OBBResult* out_array, int* out_count, int* face_labels);

extern "C" {

AUTOCOL_API int compute_obb(
    const float* verts, int nv,
    const int*   tris,  int nt,
    OBBResult*   out)
{
    return fit_obb(verts, nv, tris, nt, out);
}

AUTOCOL_API int compute_aabb(
    const float* verts, int nv,
    const int*   tris,  int nt,
    OBBResult*   out)
{
    return fit_aabb(verts, nv, tris, nt, out);
}

AUTOCOL_API int compute_multi_obb(
    const float* verts,    int nv,
    const int*   tris,     int nt,
    int          max_boxes,
    int          use_obb,
    OBBResult*   out_array,
    int*         out_count,
    int*         face_labels)
{
    return fit_multi_obb(verts, nv, tris, nt, max_boxes, use_obb, out_array, out_count, face_labels);
}

} // extern "C"
