#pragma once
#ifdef _WIN32
  #define AUTOCOL_API __declspec(dllexport)
#else
  #define AUTOCOL_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Result for a single Oriented Bounding Box */
typedef struct {
    float center[3];      /* world-space center */
    float rotation[9];    /* 3x3 rotation matrix, row-major (col0, col1, col2) */
    float extents[3];     /* half-sizes along each local axis */
} OBBResult;

/*
 * Compute a single OBB that tightly fits the supplied mesh.
 *
 * verts   – flat array of float3 positions, length = nv*3
 * tris    – flat array of int3 indices,     length = nt*3
 * nv, nt  – vertex / triangle counts
 * out     – caller-allocated OBBResult that receives the result
 *
 * Returns 0 on success, non-zero on error.
 */
AUTOCOL_API int compute_obb(
    const float* verts, int nv,
    const int*   tris,  int nt,
    OBBResult*   out);

/*
 * Compute a single AABB (Axis-Aligned Bounding Box) for the mesh.
 * Rotation will always be identity.
 *
 * Returns 0 on success.
 */
AUTOCOL_API int compute_aabb(
    const float* verts, int nv,
    const int*   tris,  int nt,
    OBBResult*   out);

/*
 * Compute up to max_boxes OBBs by k-means clustering the mesh faces,
 * then fitting one OBB per cluster.
 *
 * out_array – caller-allocated array of at least max_boxes OBBResult
 * out_count – receives the actual number of boxes written
 *
 * Returns 0 on success.
 */
AUTOCOL_API int compute_multi_obb(
    const float* verts,     int nv,
    const int*   tris,      int nt,
    int          max_boxes,
    int          use_obb,
    OBBResult*   out_array,
    int*         out_count,
    int*         face_labels);

#ifdef __cplusplus
}
#endif
