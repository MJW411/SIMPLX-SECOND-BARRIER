#!/usr/bin/env bash
# 네이티브 가속 라이브러리 빌드 (소스가 바뀌었을 때만). 실패해도 프로그램은 같은 계산을 하는 파이썬 경로로 동작한다.
#  - omx/native/libomxgpu.so  : YOLO 앞뒤 처리 GPU(CUDA) — Orin = sm_87
#  - omx/native/libomxfast.so : 사람 3D 점·거리 계산 C++
cd "$(dirname "$0")/.."
build() {  # build 소스 결과 명령...
  local src=$1 out=$2; shift 2
  if [ ! -f "$out" ] || [ "$src" -nt "$out" ]; then
    if "$@" -o "$out.tmp" "$src" && mv "$out.tmp" "$out"; then echo "빌드: $out"; else rm -f "$out.tmp"; echo "빌드 실패: $out — 파이썬 경로로 실행"; fi
  fi
}
build omx/native/omxfast.cpp omx/native/libomxfast.so g++ -O3 -mcpu=native -std=c++17 -shared -fPIC
NVCC=${NVCC:-/usr/local/cuda/bin/nvcc}
if [ -x "$NVCC" ]; then
  build omx/native/omxgpu.cu omx/native/libomxgpu.so "$NVCC" -O3 -std=c++17 -arch=sm_87 -shared -Xcompiler -fPIC -lcudart
else
  echo "nvcc 없음 — GPU 앞뒤 처리 없이 실행"
fi
