// omxfast — 사람 한 명의 3D 점·거리 계산을 한 번에 (파이썬 numpy 로 프레임당 사람 1명 약 7 ms → C++ 1 ms 미만).
//
// 파이썬 기준 구현: omx/perception/geom.py 의 person_geometry_py (결과가 같아야 한다 — tests/test_core.py).
// 순서: 마스크 안 깊이 → 범위 필터 → 개수 확인 → 중앙값 띠 필터(배경 제거) → 고른 간격 추리기(max_points)
//       → 카메라 3D → (보정 시) 베이스 좌표 → 링크 캡슐 거리·로봇 몸체 점 제외 → 대표 위치(축별 중앙값).
#include <stdint.h>
#include <math.h>
#include <algorithm>
#include <vector>

namespace {

// 점 p 와 선분 a-b 사이 거리 (numpy seg_dist 와 같은 식)
inline double seg_dist(const double* p, const double* a, const double* b) {
    double ab[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]};
    double L2 = std::max(ab[0] * ab[0] + ab[1] * ab[1] + ab[2] * ab[2], 1e-9);
    double ap[3] = {p[0] - a[0], p[1] - a[1], p[2] - a[2]};
    double t = (ap[0] * ab[0] + ap[1] * ab[1] + ap[2] * ab[2]) / L2;
    t = std::min(1.0, std::max(0.0, t));
    double d[3] = {p[0] - (a[0] + t * ab[0]), p[1] - (a[1] + t * ab[1]), p[2] - (a[2] + t * ab[2])};
    return sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
}

// numpy.median 과 같은 값 (짝수 개면 가운데 둘의 평균)
template <typename T>
T median_inplace(std::vector<T>& v) {
    size_t n = v.size(), m = n / 2;
    std::nth_element(v.begin(), v.begin() + m, v.end());
    T hi = v[m];
    if (n % 2) return hi;
    T lo = *std::max_element(v.begin(), v.begin() + m);
    return (lo + hi) / (T)2;
}

}  // namespace

extern "C" {

// out: [npts, cam_dist, dist, on_frac, near_frac, pos_x, pos_y, pos_z, n_off]
// 반환: 0 = 계산함, 1 = 깊이 픽셀 부족(파이썬에서 거리 0 = 가장 가까움 처리)
// 마스크를 감싸는 사각형(x0, y0, w, h)만 훑는다. mask·depth 는 사각형 왼쪽 위를 가리키고, 한 줄 간격은 stride(픽셀).
int of_person(const uint8_t* mask, const uint16_t* depth, int w, int h, int stride, int x0, int y0,
              float depth_scale, float lo, float hi,
              int min_px, int max_points, double fx, double fy, double cx, double cy,
              const double* T, const double* links, int nlinks, const double* radii, double margin,
              double near, double* out) {
    // 1차: 범위 안 깊이의 히스토그램 (깊이 원시값은 uint16 → 정렬 없이 정확한 중앙값)
    thread_local std::vector<uint32_t> hist(65536);
    thread_local std::vector<uint8_t> acc(65536);
    // 깊이 범위에 해당하는 원시값 칸만 쓴다 (범위 밖 값은 아래 float 비교에서 걸러지므로 여유 2칸)
    int rmin = std::max(0, (int)floorf(lo / depth_scale) - 2), rmax = std::min(65535, (int)ceilf(hi / depth_scale) + 2);
    std::fill(hist.begin() + rmin, hist.begin() + rmax + 1, 0u);
    size_t N = 0;
    for (int v = 0; v < h; v++) {
        const uint8_t* mr = mask + (size_t)v * stride;
        const uint16_t* dr = depth + (size_t)v * stride;
        for (int u = 0; u < w; u++) {
            if (!mr[u]) continue;
            float z = (float)dr[u] * depth_scale;
            if (z > lo && z < hi) { hist[dr[u]]++; N++; }
        }
    }
    if ((int)N < min_px) return 1;
    // numpy.median: 홀수면 가운데, 짝수면 가운데 둘의 평균 (float32)
    size_t m = N / 2, cum = 0;
    int r_lo = -1, r_hi = -1;
    for (int r = rmin; r <= rmax && r_hi < 0; r++) {
        if (!hist[r]) continue;
        if (r_lo < 0 && N % 2 == 0 && cum + hist[r] > m - 1) r_lo = r;
        cum += hist[r];
        if (cum > m) r_hi = r;
    }
    float zhi = (float)r_hi * depth_scale;
    float med = (N % 2) ? zhi : ((float)r_lo * depth_scale + zhi) / 2.0f;
    float band = std::max(0.08f, 0.25f * med);
    size_t k = 0;
    for (int r = rmin; r <= rmax; r++) {
        if (!hist[r]) continue;
        acc[r] = fabsf((float)r * depth_scale - med) < band;       // 배경 섞임 제거 (같은 float 식)
        if (acc[r]) k += hist[r];
    }
    size_t n = k;
    if ((int)n > max_points) n = max_points;          // 고른 간격: 통과한 점 중 i = j * k / n 번째
    // 2차: 통과한 점을 행 순서로 세며 필요한 것만 3D 로
    thread_local std::vector<double> P;
    P.resize(n * 3);
    double cam_min = INFINITY;
    size_t ord = 0, j = 0, target = 0;
    for (int v = 0; v < h && j < n; v++) {
        const uint8_t* mr = mask + (size_t)v * stride;
        const uint16_t* dr = depth + (size_t)v * stride;
        for (int u = 0; u < w && j < n; u++) {
            if (!mr[u]) continue;
            float zf = (float)dr[u] * depth_scale;
            if (!(zf > lo && zf < hi) || !acc[dr[u]]) continue;
            if (ord++ != target) continue;
            double z = zf;
            double* p = &P[j * 3];
            p[0] = (u + x0 - cx) * z / fx;
            p[1] = (v + y0 - cy) * z / fy;
            p[2] = z;
            cam_min = std::min(cam_min, sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]));
            j++;
            target = (n == k) ? j : (size_t)((long long)j * (long long)k / (long long)n);
        }
    }
    out[0] = (double)n;
    out[1] = cam_min;
    out[3] = 0.0; out[4] = 0.0;
    size_t n_off = n, n_pos = n;
    if (T) {
        for (size_t j = 0; j < n; j++) {
            double* p = &P[j * 3];
            double x = p[0], y = p[1], z = p[2];
            for (int r = 0; r < 3; r++) p[r] = T[r * 4 + 0] * x + T[r * 4 + 1] * y + T[r * 4 + 2] * z + T[r * 4 + 3];
        }
        if (links) {
            // 링크 캡슐: links[1..nlinks-2] → links[2..nlinks-1], 반경 radii[0..nlinks-3]
            // 근접 비율: links[0..nlinks-2] → links[1..nlinks-1], 반경 [0.030, radii...]
            size_t on = 0, nr = 0, w2 = 0;
            double dist = INFINITY;
            for (size_t j = 0; j < n; j++) {
                const double* p = &P[j * 3];
                double dmin = INFINITY;
                for (int s = 1; s < nlinks - 1; s++)
                    dmin = std::min(dmin, seg_dist(p, links + 3 * s, links + 3 * (s + 1)) - radii[s - 1]);
                double dn = INFINITY;
                for (int s = 0; s < nlinks - 1; s++)
                    dn = std::min(dn, seg_dist(p, links + 3 * s, links + 3 * (s + 1)) - (s == 0 ? 0.030 : radii[s - 1]));
                if (dn < near) nr++;
                if (dmin < margin) { on++; continue; }
                dist = std::min(dist, dmin);
                if (w2 != j) { P[w2 * 3] = p[0]; P[w2 * 3 + 1] = p[1]; P[w2 * 3 + 2] = p[2]; }
                w2++;
            }
            n_off = w2;
            out[2] = n_off ? std::max(0.0, dist) : 0.0;
            out[3] = n ? (double)on / n : 0.0;
            out[4] = n ? (double)nr / n : 0.0;
            n_pos = n_off ? n_off : n;   // 점이 전부 몸체 위: 대표 위치는 전체 점으로 (거리 0 = 가장 가까움)
        } else {
            out[2] = INFINITY;          // 파이썬이 쓰지 않는 값
        }
    } else {
        out[2] = cam_min;
    }
    // 대표 위치: 축별 중앙값 (P 앞쪽 n_pos 개)
    thread_local std::vector<double> axis;
    for (int a = 0; a < 3; a++) {
        axis.resize(n_pos);
        for (size_t j = 0; j < n_pos; j++) axis[j] = P[j * 3 + a];
        out[5 + a] = n_pos ? median_inplace(axis) : NAN;
    }
    out[8] = (double)n_off;
    return 0;
}

}  // extern "C"
