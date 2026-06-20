# 📄 경로 패턴(Path Planning) 관련 논문

래스터(지그재그) 패턴이 평판 폴리싱에 최적인 근거 자료입니다.

---

## [P1] Path Planning under Force Control in Robotic Polishing of the Complex Curved Surfaces

- **저자**: Imran Mohsin, Kai He, Zheng Li, Ruxu Du
- **저널**: Applied Sciences (MDPI), 2019, Vol.9, No.24, Article 5489
- **DOI**: https://doi.org/10.3390/app9245489
- **링크**: https://www.mdpi.com/2076-3417/9/24/5489
- **핵심 내용**:
  - 로봇 폴리싱에서 래스터/지그재그 경로 패턴의 구현 및 힘 제어 통합
  - **Jerk 회피 전략**을 경로 계획에 통합하여 표면 긁힘 제거, 표면 거칠기 개선
  - 복잡 곡면에서도 일정한 접촉 힘 유지가 경로 패턴보다 마감 품질에 더 중요함을 입증
- **우리 프로젝트에 적용**: 래스터 패턴 + 사다리꼴 속도 프로파일의 타당성 근거

---

## [P2] Process Optimization of Robotic Polishing for Mold Steel Based on Response Surface Method

- **저자**: Yongchao Xie, Guoqiang Chang, Jie Yang, Man Zhao, Jun Li
- **저널**: Machines (MDPI), 2022, Vol.10, No.4, Article 283
- **DOI**: https://doi.org/10.3390/machines10040283
- **링크**: https://www.mdpi.com/2075-1702/10/4/283
- **핵심 내용**:
  - 반응표면법(RSM)으로 폴리싱 파라미터(힘, 속도, 회전수, 줄 간격) 최적화
  - **래스터 스캔 경로** 기반으로 실험 수행
  - 줄 간격(step-over)이 표면 거칠기(Ra)에 미치는 영향 정량적 분석
  - 최적 파라미터 조합으로 Ra 90% 이상 개선 달성
- **우리 프로젝트에 적용**: SANDING_STEP(줄 간격) 설정의 과학적 근거

---

## [P3] Study on Trajectory Planning for Polishing Free-Form Surfaces (XY-3-RPS Hybrid Robot)

- **저널**: Machines (MDPI), 2025
- **링크**: https://www.mdpi.com/journal/machines (최신호 검색)
- **핵심 내용**:
  - 곡률이 급변하는 영역에서 경계 스무딩 알고리즘 적용
  - 동적 파티셔닝으로 표면을 영역별 분할 후 각 영역에 최적 경로 적용
  - **평판 영역에는 래스터가 가장 효율적**, 곡면에는 적응형 경로 사용
- **우리 프로젝트에 적용**: 1m×1m 평판에 래스터 패턴 선택의 근거

---

## [P4] Deep Fusion of Kinematic Features and Task-Aware Partition Planning for Mold Surface Robotic Polishing

- **저널**: Machines (MDPI), 2026
- **핵심 내용**:
  - CAD 없이 포인트클라우드 데이터로 적응형 경로 생성
  - 표면을 기하학적 유사성 기반으로 분할(segmentation)하여 각 영역에 최적 전략 적용
  - **단순 평면 → 래스터, 복잡 곡면 → 적응형** 이라는 일반 원칙 확인

---

## [P5] An Investigation of Real-Time Robotic Polishing Motion Planning Using a Dynamical System

- **저널**: Machines (MDPI), 2024
- **링크**: https://www.mdpi.com/journal/machines
- **핵심 내용**:
  - 동적 시스템 기반 실시간 경로 생성
  - 임피던스 제어와 결합한 적응형 polishing motion 구현
  - 실시간 힘 피드백으로 경로 수정

---

## 패턴별 적합성 종합 (논문 기반)

| 패턴 | 평판 적합도 | 곡면 적합도 | 구현 난이도 | 논문 근거 |
|------|-----------|-----------|-----------|----------|
| **래스터** | ⭐⭐⭐⭐⭐ | ⭐⭐⭐ | 낮음 | P1, P2, P3 |
| 스파이럴 | ⭐⭐⭐ | ⭐⭐⭐ | 중간 | P3 |
| 컨투어 | ⭐⭐ | ⭐⭐ | 중간 | P3 |
| 적응형 | ⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ | 높음 | P3, P4 |
