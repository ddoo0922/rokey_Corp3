# 📄 궤적 스무딩 & Jerk 제한 관련 논문

폴리싱 시 방향 전환 자국(trajectory mark)을 방지하기 위한 궤적 스무딩 기술의 근거입니다.

---

## [T1] Jerk-limited Real-time Trajectory Generation with Arbitrary Target States (Ruckig)

- **저자**: Lars Berscheid, Torsten Kröger
- **학회**: Robotics: Science and Systems XVII (RSS), 2021
- **DOI**: https://doi.org/10.15607/RSS.2021.XVII.015
- **ArXiv**: https://arxiv.org/abs/2105.04830
- **GitHub**: https://github.com/pantor/ruckig (오픈소스, MIT 라이센스)
- **핵심 내용**:
  - **Ruckig**: 가속도, 속도, 위치 제약을 모두 만족하는 시간 최적 실시간 궤적 생성
  - 3차 Jerk 제한 → S-curve 가감속 프로파일 자동 생성
  - 실시간(< 1μs 계산 시간) 적용 가능
  - ROS 2와 호환되는 C++/Python 라이브러리 제공
- **우리 프로젝트에 적용**:
  - 현재 구현된 사다리꼴(trapezoidal) 속도 프로파일의 상위 호환
  - 향후 S-curve 가감속 필요 시 Ruckig 라이브러리 통합 가능

---

## [T2] Parameterizable and Jerk-Limited Trajectories with Blending for Robot Motion Planning

- **출처**: Technical University of Munich (TUM), 2021
- **핵심 내용**:
  - 웨이포인트 간 블렌딩(blending)으로 정지 없이 연속 이동
  - Jerk 제한 + 속도/가속도 제한 동시 적용
  - 다축 로봇의 동기화된 관절 궤적 생성
  - 블렌드 반경을 파라미터로 조절 가능
- **우리 프로젝트에 적용**: 래스터 패턴의 방향 전환 시 블렌딩 적용으로 자국 방지

---

## [T3] Path-Accurate Online Trajectory Generation for Jerk-Limited Industrial Robots

- **출처**: German Aerospace Center (DLR), 2016
- **링크**: https://elib.dlr.de/
- **핵심 내용**:
  - 경로 정확도를 유지하면서 Jerk 제한을 적용하는 온라인 궤적 생성
  - 산업용 로봇의 공진 주파수를 피하는 Jerk 한계 설정
  - 위치 오차 < 0.1mm 수준 유지 가능
- **우리 프로젝트에 적용**: 정밀 폴리싱에서 궤적 정확도 보장 방법

---

## [T4] Analysis and Simulation of Polishing Robot Operation Trajectory Planning

- **저널**: Machines (MDPI), 2025
- **핵심 내용**:
  - Quintic B-spline 보간으로 에너지 효율 + 부드러운 궤적 생성
  - 동적 환경에서 로컬 제어 가능
  - 관절 속도/가속도 연속성 보장

---

## 보간 방식 비교 (현재 구현 vs 논문 기반 권장)

| 방식 | 부드러움 | 계산 비용 | 표면 자국 방지 | 현재 구현 |
|------|---------|----------|--------------|----------|
| 선형(Linear) | ⭐⭐ | 낮음 | 보통 | ✅ 구현됨 |
| 사다리꼴(Trapezoid) | ⭐⭐⭐ | 낮음 | 좋음 | ✅ 구현됨 |
| 3차 스플라인(Cubic) | ⭐⭐⭐⭐ | 중간 | 좋음 | ✅ 구현됨 |
| **S-curve (Jerk 제한)** | ⭐⭐⭐⭐⭐ | 중간 | **최고** | ❌ (Ruckig 통합 시) |
| B-spline | ⭐⭐⭐⭐⭐ | 높음 | 최고 | ❌ (향후) |

> **현재 상태**: 사다리꼴 속도 프로파일이 구현되어 있어 기본적인 Jerk 제한 효과가 있음.
> 더 높은 품질이 필요하면 Ruckig 라이브러리(pip install ruckig) 통합을 추천.
