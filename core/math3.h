#pragma once
/*
 * Minimal 3-D math helpers (vec3, mat3) used internally.
 * No external dependencies.
 */
#include <cmath>
#include <cstring>

struct vec3 {
    float x, y, z;
    vec3() : x(0), y(0), z(0) {}
    vec3(float x, float y, float z) : x(x), y(y), z(z) {}
    vec3 operator+(const vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    vec3 operator-(const vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    vec3 operator*(float s)        const { return {x*s,   y*s,   z*s  }; }
    vec3& operator+=(const vec3& o){ x+=o.x; y+=o.y; z+=o.z; return *this; }
    float dot(const vec3& o) const { return x*o.x + y*o.y + z*o.z; }
    vec3  cross(const vec3& o) const {
        return {y*o.z - z*o.y, z*o.x - x*o.z, x*o.y - y*o.x};
    }
    float norm() const { return std::sqrt(x*x + y*y + z*z); }
    vec3 normalized() const {
        float n = norm();
        if (n < 1e-10f) return {1,0,0};
        return {x/n, y/n, z/n};
    }
};

/* 3x3 matrix stored column-major: col[c][r] */
struct mat3 {
    float m[3][3]; // m[row][col]
    mat3() { memset(m, 0, sizeof(m)); }

    static mat3 identity() {
        mat3 r;
        r.m[0][0] = r.m[1][1] = r.m[2][2] = 1.f;
        return r;
    }

    vec3 col(int c) const { return {m[0][c], m[1][c], m[2][c]}; }
    vec3 row(int r) const { return {m[r][0], m[r][1], m[r][2]}; }

    /* matrix * vector */
    vec3 operator*(const vec3& v) const {
        return {
            m[0][0]*v.x + m[0][1]*v.y + m[0][2]*v.z,
            m[1][0]*v.x + m[1][1]*v.y + m[1][2]*v.z,
            m[2][0]*v.x + m[2][1]*v.y + m[2][2]*v.z
        };
    }

    /* matrix * matrix */
    mat3 operator*(const mat3& o) const {
        mat3 r;
        for(int i=0;i<3;i++)
          for(int j=0;j<3;j++)
            for(int k=0;k<3;k++)
              r.m[i][j] += m[i][k]*o.m[k][j];
        return r;
    }

    mat3 transposed() const {
        mat3 r;
        for(int i=0;i<3;i++) for(int j=0;j<3;j++) r.m[i][j]=m[j][i];
        return r;
    }
};

/*
 * Jacobi eigen-decomposition for a 3x3 SYMMETRIC matrix A.
 * Fills eigenvalues[] and columns of V with eigenvectors.
 * Sorted descending by eigenvalue magnitude.
 */
inline void jacobi3(const mat3& A, float eigenvalues[3], mat3& V) {
    // Copy A into working matrix
    float a[3][3];
    for(int i=0;i<3;i++) for(int j=0;j<3;j++) a[i][j]=A.m[i][j];

    // V = identity
    float v[3][3] = {{1,0,0},{0,1,0},{0,0,1}};

    const int MAX_ITER = 100;
    for(int iter=0; iter<MAX_ITER; iter++) {
        // Find largest off-diagonal element
        float maxVal = 0.f;
        int p=0, q=1;
        for(int i=0;i<3;i++) for(int j=i+1;j<3;j++) {
            float v2 = std::abs(a[i][j]);
            if(v2 > maxVal){ maxVal=v2; p=i; q=j; }
        }
        if(maxVal < 1e-10f) break;

        // Compute Jacobi rotation
        float theta = 0.5f*(a[q][q]-a[p][p]) / a[p][q];
        float t = (theta >= 0) ?
            1.f/(theta + std::sqrt(1.f+theta*theta)) :
            1.f/(theta - std::sqrt(1.f+theta*theta));
        float c = 1.f / std::sqrt(1.f+t*t);
        float s = t*c;

        // Apply rotation to a
        float app = a[p][p], aqq = a[q][q], apq = a[p][q];
        a[p][p] = c*c*app - 2*s*c*apq + s*s*aqq;
        a[q][q] = s*s*app + 2*s*c*apq + c*c*aqq;
        a[p][q] = a[q][p] = 0.f;
        for(int r=0;r<3;r++) {
            if(r==p||r==q) continue;
            float arp=a[r][p], arq=a[r][q];
            a[r][p] = a[p][r] = c*arp - s*arq;
            a[r][q] = a[q][r] = s*arp + c*arq;
        }
        // Apply rotation to v
        for(int r=0;r<3;r++) {
            float vrp=v[r][p], vrq=v[r][q];
            v[r][p] = c*vrp - s*vrq;
            v[r][q] = s*vrp + c*vrq;
        }
    }

    // Sort eigenvectors by descending eigenvalue
    float ev[3] = {a[0][0], a[1][1], a[2][2]};
    int idx[3] = {0,1,2};
    // Bubble sort descending
    for(int i=0;i<2;i++) for(int j=i+1;j<3;j++) {
        if(std::abs(ev[idx[j]]) > std::abs(ev[idx[i]])) {
            int tmp=idx[i]; idx[i]=idx[j]; idx[j]=tmp;
        }
    }
    for(int i=0;i<3;i++) eigenvalues[i] = ev[idx[i]];
    for(int i=0;i<3;i++) for(int r=0;r<3;r++) V.m[r][i] = v[r][idx[i]];
}
