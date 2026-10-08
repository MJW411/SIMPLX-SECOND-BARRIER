// omxgpu — YOLO(seg) 앞뒤 처리를 GPU에서 한다 (CPU 부담 줄이기).
//
//  - 전처리: 원본 BGR(uint8)만 GPU로 올리고, 레터박스 크기 조정·BGR→RGB·/255·CHW 변환을
//    커널 하나로 TensorRT 입력 버퍼에 바로 쓴다. (파이썬: 640x640x3 float 배열을 CPU에서 만들어 4.9 MB 복사)
//  - 후처리: 8400개 후보 중 한 클래스(사람) 점수가 기준을 넘는 것만 GPU에서 골라 박스·점수·마스크 계수만
//    내려받는다. (파이썬: 출력 전체 약 7 MB를 매 프레임 CPU로 복사)
//  - 마스크: 고른 검출만 sigmoid(계수·프로토타입) > 0.5 를 박스 안에서 계산해 160x160 uint8 로 내려받는다.
//
// 파이썬(ctypes)에서 부른다: omx/perception/gpu_post.py. 결과는 기존 numpy 경로와 같다(tests/test_gpu.py).
#include <cuda_runtime.h>
#include <cuda_fp16.h>
#include <stdint.h>
#include <string.h>

#define MAX_CAND 1024

struct OgCtx {
    int imgsz, n_anchor, nc, nm, ph, pw, fp16;
    uint8_t* d_img;        // 원본 BGR
    size_t img_cap;
    int* d_count;          // 후보 수
    float* d_cand;         // 후보: [anchor, cx,cy,w,h, score, cls, coef x nm] x MAX_CAND
    float* d_coef;         // 마스크용 계수 (k x nm)
    float* d_box;          // 마스크용 박스 (프로토 좌표 x1,y1,x2,y2) (k x 4)
    uint8_t* d_mask;       // k x ph x pw
    int* h_count;          // 고정(pinned) 호스트 메모리
    float* h_cand;
    uint8_t* h_mask;
    int max_masks;
};

static int ck(cudaError_t e) { return e == cudaSuccess ? 0 : (int)e; }

extern "C" {

// 프로세스에서 CUDA를 처음 쓰기 전에 부른다: GPU를 기다릴 때 CPU가 바쁘게 돌지 않고 잠든다.
int og_blocking_sync(void) { return ck(cudaSetDeviceFlags(cudaDeviceScheduleBlockingSync)); }

void* og_create(int imgsz, int n_anchor, int nc, int nm, int ph, int pw, int fp16, int max_masks) {
    OgCtx* c = new OgCtx();
    memset(c, 0, sizeof(*c));
    c->imgsz = imgsz; c->n_anchor = n_anchor; c->nc = nc; c->nm = nm; c->ph = ph; c->pw = pw;
    c->fp16 = fp16; c->max_masks = max_masks;
    int stride = 7 + nm;
    if (cudaMalloc(&c->d_count, sizeof(int)) ||
        cudaMalloc(&c->d_cand, sizeof(float) * stride * MAX_CAND) ||
        cudaMalloc(&c->d_coef, sizeof(float) * nm * max_masks) ||
        cudaMalloc(&c->d_box, sizeof(float) * 4 * max_masks) ||
        cudaMalloc(&c->d_mask, (size_t)ph * pw * max_masks) ||
        cudaMallocHost(&c->h_count, sizeof(int)) ||
        cudaMallocHost(&c->h_cand, sizeof(float) * stride * MAX_CAND) ||
        cudaMallocHost(&c->h_mask, (size_t)ph * pw * max_masks)) {
        return nullptr;
    }
    return c;
}

void og_destroy(void* p) {
    OgCtx* c = (OgCtx*)p;
    if (!c) return;
    cudaFree(c->d_img); cudaFree(c->d_count); cudaFree(c->d_cand); cudaFree(c->d_coef);
    cudaFree(c->d_box); cudaFree(c->d_mask);
    cudaFreeHost(c->h_count); cudaFreeHost(c->h_cand); cudaFreeHost(c->h_mask);
    delete c;
}

}  // extern "C"

// ---------------------------------------------------------------- 전처리
// cv2.resize(INTER_LINEAR) 와 같은 좌표 규칙: src = (dst + 0.5) / s - 0.5, 가장자리는 복제.
template <typename T>
__global__ void k_letterbox(const uint8_t* __restrict__ img, int w, int h, T* __restrict__ out, int S,
                            int nw, int nh, int px, int py, float sx, float sy) {
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x >= S || y >= S) return;
    float r = 114.f, g = 114.f, b = 114.f;
    int u = x - px, v = y - py;
    if (u >= 0 && v >= 0 && u < nw && v < nh) {
        float fx = (u + 0.5f) * sx - 0.5f, fy = (v + 0.5f) * sy - 0.5f;
        fx = fmaxf(fx, 0.f); fy = fmaxf(fy, 0.f);
        int x0 = min((int)fx, w - 1), y0 = min((int)fy, h - 1);
        int x1 = min(x0 + 1, w - 1), y1 = min(y0 + 1, h - 1);
        float ax = fx - x0, ay = fy - y0;
        const uint8_t* p00 = img + (y0 * w + x0) * 3;
        const uint8_t* p01 = img + (y0 * w + x1) * 3;
        const uint8_t* p10 = img + (y1 * w + x0) * 3;
        const uint8_t* p11 = img + (y1 * w + x1) * 3;
        float c[3];
        for (int k = 0; k < 3; k++)
            c[k] = (p00[k] * (1 - ax) + p01[k] * ax) * (1 - ay) + (p10[k] * (1 - ax) + p11[k] * ax) * ay;
        b = rintf(c[0]); g = rintf(c[1]); r = rintf(c[2]);      // cv2 는 uint8 로 반올림한 뒤 /255
    }
    size_t plane = (size_t)S * S, i = (size_t)y * S + x;
    out[i] = (T)(r / 255.f);
    out[plane + i] = (T)(g / 255.f);
    out[2 * plane + i] = (T)(b / 255.f);
}

extern "C" int og_preprocess(void* p, const uint8_t* img, int w, int h, void* d_input, cudaStream_t st,
                             float* s_out, int* px_out, int* py_out) {
    OgCtx* c = (OgCtx*)p;
    size_t need = (size_t)w * h * 3;
    if (need > c->img_cap) {
        cudaFree(c->d_img);
        if (cudaMalloc(&c->d_img, need)) { c->img_cap = 0; return -1; }
        c->img_cap = need;
    }
    int S = c->imgsz;
    float s = (float)S / (float)(w > h ? w : h);
    int nw = (int)lrintf(w * s), nh = (int)lrintf(h * s);
    int px = (S - nw) / 2, py = (S - nh) / 2;
    int e = ck(cudaMemcpyAsync(c->d_img, img, need, cudaMemcpyHostToDevice, st));
    if (e) return e;
    dim3 bs(32, 8), gs((S + 31) / 32, (S + 7) / 8);
    float sx = (float)w / nw, sy = (float)h / nh;
    if (c->fp16) k_letterbox<__half><<<gs, bs, 0, st>>>(c->d_img, w, h, (__half*)d_input, S, nw, nh, px, py, sx, sy);
    else k_letterbox<float><<<gs, bs, 0, st>>>(c->d_img, w, h, (float*)d_input, S, nw, nh, px, py, sx, sy);
    *s_out = s; *px_out = px; *py_out = py;
    return ck(cudaGetLastError());
}

// ---------------------------------------------------------------- 후보 고르기
// pred: (4 + nc + nm, N) 행 우선. cls_mask 에 켜진 클래스들 중 최고 점수를 본다 (예: human + fall-down).
// numpy 경로와 같다: 고른 클래스 밖의 점수는 0으로 보고 argmax (같은 점수면 낮은 번호).
// 순서는 GPU 원자 연산 순서라 섞이므로 파이썬에서 앵커 번호로 다시 정렬한다(기존 numpy 결과와 같은 순서 → 같은 NMS 결과).
template <typename T>
__global__ void k_select(const T* __restrict__ pred, int N, int nc, int nm, unsigned int cls_mask, float conf,
                         int* count, float* cand) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= N) return;
    float sc = 0.f;
    int best = 0;
    for (int c = 0; c < nc && c < 32; c++) {
        if (!((cls_mask >> c) & 1u)) continue;
        float v = (float)pred[(size_t)(4 + c) * N + i];
        if (v > sc) { sc = v; best = c; }
    }
    if (!(sc > conf)) return;
    int k = atomicAdd(count, 1);
    if (k >= MAX_CAND) return;
    int stride = 7 + nm;       // [anchor, cx, cy, w, h, score, cls, coef...]
    float* o = cand + (size_t)k * stride;
    o[0] = (float)i;
    for (int r = 0; r < 4; r++) o[1 + r] = (float)pred[(size_t)r * N + i];
    o[5] = sc;
    o[6] = (float)best;
    for (int m = 0; m < nm; m++) o[7 + m] = (float)pred[(size_t)(4 + nc + m) * N + i];
}

extern "C" int og_select(void* p, const void* d_pred, unsigned int cls_mask, float conf, cudaStream_t st, float** out) {
    // 반환: 후보 수 (음수 = 오류). out 에 [anchor, cx, cy, w, h, score, cls, coef x nm] x n 호스트 배열.
    OgCtx* c = (OgCtx*)p;
    int N = c->n_anchor, stride = 7 + c->nm;
    int e = ck(cudaMemsetAsync(c->d_count, 0, sizeof(int), st));
    if (e) return -e;
    if (c->fp16) k_select<__half><<<(N + 255) / 256, 256, 0, st>>>((const __half*)d_pred, N, c->nc, c->nm, cls_mask, conf, c->d_count, c->d_cand);
    else k_select<float><<<(N + 255) / 256, 256, 0, st>>>((const float*)d_pred, N, c->nc, c->nm, cls_mask, conf, c->d_count, c->d_cand);
    if ((e = ck(cudaGetLastError()))) return -e;
    if ((e = ck(cudaMemcpyAsync(c->h_count, c->d_count, sizeof(int), cudaMemcpyDeviceToHost, st)))) return -e;
    if ((e = ck(cudaStreamSynchronize(st)))) return -e;
    int n = *c->h_count < MAX_CAND ? *c->h_count : MAX_CAND;
    if (n > 0) {
        if ((e = ck(cudaMemcpyAsync(c->h_cand, c->d_cand, sizeof(float) * stride * n, cudaMemcpyDeviceToHost, st)))) return -e;
        if ((e = ck(cudaStreamSynchronize(st)))) return -e;
    }
    *out = c->h_cand;
    return n;
}

// ---------------------------------------------------------------- 마스크
template <typename T>
__global__ void k_mask(const T* __restrict__ proto, int nm, int ph, int pw, const float* __restrict__ coef,
                       const float* __restrict__ box, uint8_t* __restrict__ out) {
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int k = blockIdx.z;
    if (x >= pw || y >= ph) return;
    const float* b = box + 4 * k;
    // 기존 numpy: crop[max(0,int(y1)):ceil(y2), max(0,int(x1)):ceil(x2)] = 1
    int x1 = max(0, (int)b[0]), y1 = max(0, (int)b[1]);
    int x2 = (int)ceilf(b[2]), y2 = (int)ceilf(b[3]);
    uint8_t v = 0;
    if (x >= x1 && x < x2 && y >= y1 && y < y2) {
        const float* cf = coef + (size_t)nm * k;
        float acc = 0.f;
        size_t plane = (size_t)ph * pw, i = (size_t)y * pw + x;
        for (int m = 0; m < nm; m++) acc += cf[m] * (float)proto[m * plane + i];
        v = acc > 0.f;          // sigmoid(acc) > 0.5  ⇔  acc > 0
    }
    out[(size_t)k * ph * pw + (size_t)y * pw + x] = v;
}

extern "C" int og_masks(void* p, const void* d_proto, const float* coef, const float* box, int k,
                        cudaStream_t st, uint8_t** out) {
    OgCtx* c = (OgCtx*)p;
    if (k <= 0) return 0;
    if (k > c->max_masks) k = c->max_masks;
    int e;
    if ((e = ck(cudaMemcpyAsync(c->d_coef, coef, sizeof(float) * c->nm * k, cudaMemcpyHostToDevice, st)))) return -e;
    if ((e = ck(cudaMemcpyAsync(c->d_box, box, sizeof(float) * 4 * k, cudaMemcpyHostToDevice, st)))) return -e;
    dim3 bs(32, 8), gs((c->pw + 31) / 32, (c->ph + 7) / 8, k);
    if (c->fp16) k_mask<__half><<<gs, bs, 0, st>>>((const __half*)d_proto, c->nm, c->ph, c->pw, c->d_coef, c->d_box, c->d_mask);
    else k_mask<float><<<gs, bs, 0, st>>>((const float*)d_proto, c->nm, c->ph, c->pw, c->d_coef, c->d_box, c->d_mask);
    if ((e = ck(cudaGetLastError()))) return -e;
    size_t bytes = (size_t)c->ph * c->pw * k;
    if ((e = ck(cudaMemcpyAsync(c->h_mask, c->d_mask, bytes, cudaMemcpyDeviceToHost, st)))) return -e;
    if ((e = ck(cudaStreamSynchronize(st)))) return -e;
    *out = c->h_mask;
    return k;
}
