# 📄 힘 제어(Force Control) 관련 논문

폴리싱에서 일정한 접촉 힘 유지의 중요성과 구현 방법에 대한 근거 자료입니다.

---

## [F1] Assessment of Force Control for Surface Finishing — UR10e vs FerRobotics ACF

- **저자**: Stefan Gadringer, Hubert Gattringer, Andreas Mueller
- **저널**: Mechanical Sciences (Copernicus), 2022, Vol.13, pp.361-370
- **DOI**: https://doi.org/10.5194/ms-13-361-2022
- **링크**: https://ms.copernicus.org/articles/13/361/2022/
- **핵심 내용**:
  - UR10e 내장 힘 제어 vs FerRobotics ACF-K 109/04 외장 장치 직접 비교 실험
  - **UR10e**: 단순 직선, 저속 이동에서는 충분한 힘 제어 성능
  - **FerRobotics ACF**: 고속, 복잡한 경로에서 압도적 성능 (고주파 힘 조절)
  - **결론**: 평판 래스터 폴리싱처럼 단순한 경로에서는 UR10e 내장 제어로 충분
- **우리 프로젝트에 적용**: UR10 + 래스터 경로 조합의 현실성 확인

---

## [F2] Robotic Polishing of Unknown-Model Workpieces With Constant Normal Contact Force Control

- **저널**: IEEE/ASME Transactions on Mechatronics, 2022
- **DOI**: https://doi.org/10.1109/TMECH.2022.3216314
- **링크**: https://ieeexplore.ieee.org/document/9932574
- **핵심 내용**:
  - 미지 형상의 워크피스에서도 일정한 법선 접촉 힘을 유지하는 제어 알고리즘
  - 실시간 경로 조정 + 힘 피드백 통합
  - **일정 힘 유지가 경로 패턴보다 표면 품질에 2배 이상 영향**
  - 힘 변동 ±10% 이내 시 Ra 개선 효과 극대화
- **우리 프로젝트에 적용**: 향후 힘 제어 추가 시 참고

---

## [F3] Design and Analysis of a Compliant End-Effector for Robotic Polishing Using Flexible Beams

- **저널**: Actuators (MDPI), 2022, Vol.11, No.10, Article 284
- **DOI**: https://doi.org/10.3390/act11100284
- **링크**: https://www.mdpi.com/2076-0825/11/10/284
- **핵심 내용**:
  - 유연 빔(flexible beam) 기반 패시브 컴플라이언트 엔드이펙터 설계
  - 관성 변위를 억제하여 워크피스 표면 보호
  - 능동(active) 제어 없이도 표면 높이 변화 ±2mm까지 보상 가능
  - 제작 비용이 낮고 유지보수 간단
- **우리 프로젝트에 적용**: 저비용 컴플라이언트 엔드이펙터 설계 시 참고

---

## [F4] Active Compliance Smart Control Strategy of Hybrid Mechanism for Bonnet Polishing

- **출처**: Hong Kong Polytechnic University 연구
- **핵심 내용**:
  - 하이브리드 메커니즘의 고주파 임피던스 제어
  - 매크로-미니 아키텍처 (로봇=위치, 엔드이펙터=힘 조절)
  - 이 구조가 UR10 같은 협동 로봇의 응답 시간 한계를 극복

---

## 힘 제어 방식 비교

| 방식 | 장점 | 단점 | 비용 | 적합 상황 |
|------|------|------|------|----------|
| UR10e 내장 FT | 추가 장비 불필요 | 대역폭 낮음 | 무료 | 저속, 단순 경로 |
| FerRobotics ACF | 고주파, 고성능 | 고가 (~$5k+) | 높음 | 고속, 복잡 경로 |
| 패시브 컴플라이언트 | 저비용, 유지보수 쉬움 | 조절 불가 | 낮음 | 균일 평판 |
| 임피던스 제어 | 유연한 설정 | 구현 복잡 | 중간 | 다양한 곡면 |
