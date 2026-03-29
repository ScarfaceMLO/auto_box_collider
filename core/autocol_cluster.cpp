/*
 * autocol_cluster.cpp
 *
 * Two-phase approach for robust collision decomposition:
 *
 * Phase 1 – Normal-based region growing (40° threshold):
 *   Groups faces with similar normals into coherent "patches".
 *   A cylinder becomes a few patches; a flat seat becomes one patch.
 *
 * Phase 2 – Spatial K-means++ on patch centroids:
 *   Groups the patches by 3D position into K final clusters.
 *   This naturally separates: legs (low, spread out) from seat (mid)
 *   from backrest (high), regardless of their surface normals.
 *
 * Phase 3 – AABB per final cluster:
 *   One clean axis-aligned box per cluster.
 */
#include "autocol_api.h"
#include "math3.h"
#include <vector>
#include <cstdlib>
#include <cstring>
#include <cfloat>
#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <queue>

int fit_aabb(const float* verts, int nv, const int* tris, int nt, OBBResult* out);

/* ---- dist squared -------------------------------------------------------- */
static float dist2(const vec3& a, const vec3& b){
    float dx=a.x-b.x,dy=a.y-b.y,dz=a.z-b.z;
    return dx*dx+dy*dy+dz*dz;
}

/* ---- Face adjacency (edge-shared neighbours) ----------------------------- */
static void build_adjacency(int nt, const int* tris,
                             std::vector<std::vector<int>>& adj)
{
    adj.assign(nt,{});
    std::unordered_map<long long,std::vector<int>> em;
    em.reserve(nt*3);
    for(int t=0;t<nt;t++)
        for(int e=0;e<3;e++){
            int a=tris[t*3+e],b=tris[t*3+(e+1)%3];
            if(a>b){int tmp=a;a=b;b=tmp;}
            em[(long long)a*1000000LL+b].push_back(t);
        }
    for(auto& kv:em){
        auto& ts=kv.second;
        for(int i=0;i<(int)ts.size();i++)
            for(int j=i+1;j<(int)ts.size();j++){
                adj[ts[i]].push_back(ts[j]);
                adj[ts[j]].push_back(ts[i]);
            }
    }
}

/* ========================================================================== */
int fit_multi_obb(
    const float* verts, int nv,
    const int*   tris,  int nt,
    int          K,
    OBBResult*   out_array,
    int*         out_count)
{
    if(!verts||!tris||nv<3||nt<1||!out_array||!out_count) return -1;
    if(K<1) K=1;
    if(K>nt) K=nt;

    /* ---- Per-face data --------------------------------------------------- */
    std::vector<vec3>  normals(nt), centroids(nt);
    std::vector<float> areas(nt);
    for(int t=0;t<nt;t++){
        int ia=tris[t*3+0],ib=tris[t*3+1],ic=tris[t*3+2];
        vec3 a(verts[ia*3],verts[ia*3+1],verts[ia*3+2]);
        vec3 b(verts[ib*3],verts[ib*3+1],verts[ib*3+2]);
        vec3 c(verts[ic*3],verts[ic*3+1],verts[ic*3+2]);
        vec3 cr=(b-a).cross(c-a);
        areas[t]    =cr.norm()*0.5f;
        normals[t]  =(areas[t]>1e-10f)?cr.normalized():vec3(0,1,0);
        centroids[t]=(a+b+c)*(1.f/3.f);
    }

    /* ==================================================================
     * PHASE 1 – Normal-based region growing
     * Creates many small "patches" of faces with coherent normals.
     * ================================================================= */
    const float COS_THRESH = 0.766f; // ~40°
    std::vector<std::vector<int>> adj;
    build_adjacency(nt, tris, adj);

    std::vector<int> patch(nt, -1);
    int num_patches = 0;

    // Seed order: largest faces first
    std::vector<int> order(nt);
    for(int i=0;i<nt;i++) order[i]=i;
    std::sort(order.begin(),order.end(),[&](int a,int b){return areas[a]>areas[b];});

    for(int seed : order){
        if(patch[seed]>=0) continue;
        int pid = num_patches++;
        patch[seed] = pid;
        vec3 rn = normals[seed];
        float rw = areas[seed];
        std::queue<int> q;
        q.push(seed);
        while(!q.empty()){
            int t=q.front(); q.pop();
            for(int nb:adj[t]){
                if(patch[nb]>=0) continue;
                if(normals[nb].dot(rn)>COS_THRESH){
                    patch[nb]=pid;
                    float w=areas[nb];
                    rn=vec3(rn.x*rw+normals[nb].x*w,
                            rn.y*rw+normals[nb].y*w,
                            rn.z*rw+normals[nb].z*w).normalized();
                    rw+=w;
                    q.push(nb);
                }
            }
        }
    }

    /* ---- Compute area-weighted centroid for each patch -------------------- */
    std::vector<vec3>  pcen(num_patches, {0,0,0});
    std::vector<float> parea(num_patches, 0.f);
    for(int t=0;t<nt;t++){
        int pid=patch[t]; float w=areas[t];
        pcen[pid]  =vec3(pcen[pid].x+centroids[t].x*w,
                         pcen[pid].y+centroids[t].y*w,
                         pcen[pid].z+centroids[t].z*w);
        parea[pid]+=w;
    }
    for(int p=0;p<num_patches;p++)
        if(parea[p]>1e-10f){
            float inv=1.f/parea[p];
            pcen[p]=pcen[p]*inv;
        }

    /* ==================================================================
     * PHASE 2 – Spatial K-means++ on patch centroids
     * Groups patches by their 3D position into K final clusters.
     * ================================================================= */
    int Kp = std::min(K, num_patches);

    // K-means++ initialisation: pick seeds that are maximally spread
    std::vector<vec3> means(Kp);
    means[0] = pcen[0];
    // Find patch with largest area for first seed
    {int best=0; for(int p=1;p<num_patches;p++) if(parea[p]>parea[best]) best=p; means[0]=pcen[best];}

    for(int k=1;k<Kp;k++){
        // Weighted by squared distance to nearest existing mean
        float total=0;
        std::vector<float> d2(num_patches);
        for(int p=0;p<num_patches;p++){
            float best=FLT_MAX;
            for(int j=0;j<k;j++){float d=dist2(pcen[p],means[j]);if(d<best)best=d;}
            d2[p]=best*parea[p]; // area-weighted so large patches dominate
            total+=d2[p];
        }
        // Pick the patch that is most "alone" (farthest, area-weighted)
        float bestv=0; int bestp=0;
        for(int p=0;p<num_patches;p++) if(d2[p]>bestv){bestv=d2[p];bestp=p;}
        means[k]=pcen[bestp];
    }

    // K-means iterations
    std::vector<int> clabel(num_patches, 0);
    const int MAX_ITER=100;
    for(int iter=0;iter<MAX_ITER;iter++){
        bool changed=false;
        // Assignment
        for(int p=0;p<num_patches;p++){
            float best=FLT_MAX; int bk=0;
            for(int k=0;k<Kp;k++){float d=dist2(pcen[p],means[k]);if(d<best){best=d;bk=k;}}
            if(clabel[p]!=bk){clabel[p]=bk;changed=true;}
        }
        if(!changed) break;
        // Update means (area-weighted)
        std::vector<vec3>  sums(Kp,{0,0,0});
        std::vector<float> wcnt(Kp,0.f);
        for(int p=0;p<num_patches;p++){
            int k=clabel[p]; float w=parea[p];
            sums[k]=vec3(sums[k].x+pcen[p].x*w, sums[k].y+pcen[p].y*w, sums[k].z+pcen[p].z*w);
            wcnt[k]+=w;
        }
        for(int k=0;k<Kp;k++)
            if(wcnt[k]>1e-10f) means[k]=sums[k]*(1.f/wcnt[k]);
    }

    /* ---- Map face → final cluster ---------------------------------------- */
    //   face → patch → kmeans cluster
    std::vector<int> flabel(nt);
    for(int t=0;t<nt;t++) flabel[t]=clabel[patch[t]];

    /* ==================================================================
     * PHASE 3 – One AABB per cluster (local remapped mesh)
     * ================================================================= */
    int written=0;
    for(int k=0;k<Kp && written<K;k++){
        std::unordered_map<int,int> old_to_new;
        std::vector<float> lv;
        std::vector<int>   lt;

        for(int t=0;t<nt;t++){
            if(flabel[t]!=k) continue;
            for(int vi=0;vi<3;vi++){
                int oi=tris[t*3+vi];
                auto it=old_to_new.find(oi);
                if(it==old_to_new.end()){
                    int ni=(int)(lv.size()/3);
                    old_to_new[oi]=ni;
                    lv.push_back(verts[oi*3+0]);
                    lv.push_back(verts[oi*3+1]);
                    lv.push_back(verts[oi*3+2]);
                }
                lt.push_back(old_to_new[oi]);
            }
        }
        if(lt.empty()) continue;

        if(fit_aabb(lv.data(),(int)(lv.size()/3),
                    lt.data(),(int)(lt.size()/3),
                    &out_array[written])==0)
            written++;
    }

    *out_count=written;
    return (written>0)?0:-3;
}
