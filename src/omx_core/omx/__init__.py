"""omx-safety-demo 패키지.

수치 라이브러리 스레드 제한 — numpy·OpenCV 를 불러오기 전에 정해야 해서 패키지 맨 앞에 둔다.
OpenBLAS 는 작은 행렬 곱(마스크 계수 @ 프로토타입 등)에도 코어 수만큼 작업 스레드를 깨우고, 끝난 뒤에도
바쁜 대기(spin)로 코어를 붙잡는다. 실측(2026-09-27, 사람 클립 가상 서버): 기본 CPU 254%·추론 88.8 ms →
1 스레드 CPU 40%·추론 37.1 ms. 우리 계산은 작아서 다중 스레드 이득이 없다.
"""
import os

for _k in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_k, "1")
