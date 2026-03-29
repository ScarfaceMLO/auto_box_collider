/*
 * autocol_api.cpp  –  DLL entry point / exported C functions
 */
#include "autocol_api.h"

/* Declarations from other TUs */
int fit_obb(const float* verts, int nv, const int* tris, int nt, OBBResult* out);
int fit_aabb(const float* verts, int nv, const int* tris, int nt, OBBResult* out);
int fit_multi_obb(const float* verts, int nv, const int* tris, int nt,
                  int K, OBBResult* out_array, int* out_count);

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
    OBBResult*   out_array,
    int*         out_count)
{
    return fit_multi_obb(verts, nv, tris, nt, max_boxes, out_array, out_count);
}

} // extern "C"
