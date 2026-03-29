/*
 * autocol_obb.cpp
 * Provides two fitting functions:
 *   fit_obb  – Oriented Bounding Box via area-weighted PCA (Jacobi)
 *   fit_aabb – Axis-Aligned Bounding Box (identity rotation, clean boxes)
 */
#include "autocol_api.h"
#include "math3.h"
#include <cfloat>
#include <vector>

/* ============================================================================
 * fit_aabb — Axis-Aligned Bounding Box
 * Simply finds the world-space min/max extents of the referenced vertices.
 * Rotation is identity, so all boxes are upright and axis-aligned.
 * ========================================================================= */
int fit_aabb(const float* verts, int nv,
             const int*   tris,  int nt,
             OBBResult*   out)
{
    if(!verts || !tris || nv<1 || nt<1 || !out) return -1;

    // Only consider vertices actually referenced by these triangles
    std::vector<bool> used(nv, false);
    for(int t=0;t<nt;t++){
        used[tris[t*3+0]] = true;
        used[tris[t*3+1]] = true;
        used[tris[t*3+2]] = true;
    }

    float mn[3] = { FLT_MAX,  FLT_MAX,  FLT_MAX};
    float mx[3] = {-FLT_MAX, -FLT_MAX, -FLT_MAX};

    bool any = false;
    for(int i=0;i<nv;i++){
        if(!used[i]) continue;
        any = true;
        for(int k=0;k<3;k++){
            float v = verts[i*3+k];
            if(v < mn[k]) mn[k]=v;
            if(v > mx[k]) mx[k]=v;
        }
    }
    if(!any) return -2;

    for(int k=0;k<3;k++){
        out->center[k]  = 0.5f*(mn[k]+mx[k]);
        out->extents[k] = 0.5f*(mx[k]-mn[k]);
    }

    // Identity rotation
    float id[9] = {1,0,0, 0,1,0, 0,0,1};
    for(int i=0;i<9;i++) out->rotation[i] = id[i];

    return 0;
}

/* ============================================================================
 * fit_obb — Oriented Bounding Box via area-weighted covariance PCA
 * ========================================================================= */
int fit_obb(const float* verts, int nv,
            const int*   tris,  int nt,
            OBBResult*   out)
{
    if(!verts || !tris || nv<3 || nt<1 || !out) return -1;

    // Mark only vertices used by these triangles
    std::vector<bool> used(nv, false);
    for(int t=0;t<nt;t++){
        used[tris[t*3+0]] = true;
        used[tris[t*3+1]] = true;
        used[tris[t*3+2]] = true;
    }

    // ---- 1. Area-weighted centroid ------------------------------------------
    double totalArea = 0.0;
    double cx=0, cy=0, cz=0;

    for(int t=0; t<nt; t++) {
        int ia = tris[t*3+0], ib = tris[t*3+1], ic = tris[t*3+2];
        vec3 a(verts[ia*3], verts[ia*3+1], verts[ia*3+2]);
        vec3 b(verts[ib*3], verts[ib*3+1], verts[ib*3+2]);
        vec3 c(verts[ic*3], verts[ic*3+1], verts[ic*3+2]);

        vec3 tc = (a + b + c) * (1.f/3.f);
        float area = (b-a).cross(c-a).norm() * 0.5f;
        if(area < 1e-12f) continue;
        totalArea += area;
        cx += area * tc.x; cy += area * tc.y; cz += area * tc.z;
    }
    if(totalArea < 1e-12) return -2;

    vec3 mean((float)(cx/totalArea), (float)(cy/totalArea), (float)(cz/totalArea));

    // ---- 2. Area-weighted covariance relative to global centroid -------------
    double cov[6] = {0};
    for(int t=0; t<nt; t++) {
        int ia = tris[t*3+0], ib = tris[t*3+1], ic = tris[t*3+2];
        vec3 a(verts[ia*3], verts[ia*3+1], verts[ia*3+2]);
        vec3 b(verts[ib*3], verts[ib*3+1], verts[ib*3+2]);
        vec3 c(verts[ic*3], verts[ic*3+1], verts[ic*3+2]);
        float area = (b-a).cross(c-a).norm() * 0.5f;
        if(area < 1e-12f) continue;
        vec3 pts[3] = {a, b, c};
        for(int k=0;k<3;k++) {
            double dx=pts[k].x-mean.x, dy=pts[k].y-mean.y, dz=pts[k].z-mean.z;
            double w=area/3.0;
            cov[0]+=w*dx*dx; cov[1]+=w*dx*dy; cov[2]+=w*dx*dz;
            cov[3]+=w*dy*dy; cov[4]+=w*dy*dz; cov[5]+=w*dz*dz;
        }
    }

    mat3 C;
    C.m[0][0]=(float)(cov[0]/totalArea); C.m[0][1]=(float)(cov[1]/totalArea); C.m[0][2]=(float)(cov[2]/totalArea);
    C.m[1][0]=(float)(cov[1]/totalArea); C.m[1][1]=(float)(cov[3]/totalArea); C.m[1][2]=(float)(cov[4]/totalArea);
    C.m[2][0]=(float)(cov[2]/totalArea); C.m[2][1]=(float)(cov[4]/totalArea); C.m[2][2]=(float)(cov[5]/totalArea);

    // ---- 3. PCA -------------------------------------------------------------
    float eigenvalues[3];
    mat3 V;
    jacobi3(C, eigenvalues, V);

    // Guarantee right-hand system
    vec3 ax = V.col(0).normalized();
    vec3 ay = V.col(1).normalized();
    vec3 az = ax.cross(ay).normalized();
    V.m[0][2]=az.x; V.m[1][2]=az.y; V.m[2][2]=az.z;

    // ---- 4. Project used vertices onto local axes ---------------------------
    float minE[3]={ FLT_MAX, FLT_MAX, FLT_MAX};
    float maxE[3]={-FLT_MAX,-FLT_MAX,-FLT_MAX};
    for(int i=0;i<nv;i++){
        if(!used[i]) continue;
        vec3 p(verts[i*3], verts[i*3+1], verts[i*3+2]);
        vec3 d = p - mean;
        for(int k=0;k<3;k++){
            float proj = V.col(k).dot(d);
            if(proj<minE[k]) minE[k]=proj;
            if(proj>maxE[k]) maxE[k]=proj;
        }
    }

    // ---- 5. OBB center and extents -----------------------------------------
    vec3 lc(0.5f*(minE[0]+maxE[0]), 0.5f*(minE[1]+maxE[1]), 0.5f*(minE[2]+maxE[2]));
    vec3 wc = mean + V.col(0)*lc.x + V.col(1)*lc.y + V.col(2)*lc.z;
    float obb_e[3];
    for(int k=0; k<3; k++) obb_e[k] = 0.5f*(maxE[k]-minE[k]);
    double obb_vol = (double)obb_e[0] * obb_e[1] * obb_e[2];

    // ---- 6. Comparison with AABB (Stability Check) -------------------------
    // If the OBB isn't significantly tighter than the AABB, use the AABB.
    // This prevents "crooked" boxes on nearly-cubic or diagonal parts.
    float mn[3]={FLT_MAX,FLT_MAX,FLT_MAX}, mx[3]={-FLT_MAX,-FLT_MAX,-FLT_MAX};
    for(int i=0; i<nv; i++) {
        if(!used[i]) continue;
        for(int k=0; k<3; k++) {
            float v = verts[i*3+k];
            if(v < mn[k]) mn[k]=v;
            if(v > mx[k]) mx[k]=v;
        }
    }
    float aabb_e[3];
    for(int k=0; k<3; k++) aabb_e[k] = 0.5f * (mx[k] - mn[k]);
    double aabb_vol = (double)aabb_e[0] * aabb_e[1] * aabb_e[2];

    // Threshold: OBB must be at least 10% better (vol < 0.9 * aabb_vol)
    if(obb_vol >= aabb_vol * 0.9) {
        // USE AABB (Identity rotation, centered AABB)
        for(int k=0; k<3; k++) {
            out->center[k]  = 0.5f * (mn[k] + mx[k]);
            out->extents[k] = aabb_e[k];
        }
        float id[9] = {1,0,0, 0,1,0, 0,0,1};
        for(int i=0; i<9; i++) out->rotation[i] = id[i];
    } else {
        // USE OBB
        out->center[0]=wc.x; out->center[1]=wc.y; out->center[2]=wc.z;
        for(int k=0; k<3; k++) out->extents[k] = obb_e[k];
        for(int r=0; r<3; r++) for(int c2=0; c2<3; c2++) out->rotation[r*3+c2] = V.m[r][c2];
    }

    return 0;
}
