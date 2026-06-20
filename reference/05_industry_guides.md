# 📄 산업용 가이드 & 실무 자료

논문 외에 실무에서 참고할 수 있는 산업용 가이드/매뉴얼/기술 문서입니다.

---

## [G1] Robotiq — Sanding/Polishing Path Overlap Guide

- **출처**: Robotiq (로봇 그리퍼/피니싱 키트 제조사)
- **링크**: https://blog.robotiq.com/how-to-calculate-robot-sanding-path
- **핵심 내용**:
  - **오버랩 권장 비율: 25% ~ 50%**
  - 계산 공식: `줄 간격(A) = 패드 폭 × (100% - 오버랩%)`
  - 예시: 5인치 패드 × 40% 오버랩 = 3인치(7.62cm) 간격
  - **높은 오버랩(50%)**: 더 균일한 마감, 폴리싱에 적합
  - **낮은 오버랩(25%)**: 더 빠른 작업, 거친 샌딩에 적합
  - **그릿(Grit) 순서**: 한 단계 이상 건너뛰지 말 것 (80→120 OK, 80→220 NG)
- **우리 프로젝트에 적용**: SANDING_STEP = 0.09m (150mm × 60%) 설정의 직접 근거

---

## [G2] Universal Robots — UR10 Technical Specifications

- **출처**: Universal Robots 공식
- **링크**: https://www.universal-robots.com/products/ur10-robot/
- **데이터시트**: https://www.universal-robots.com/media/1807465/ur10e-rgb-fact-sheet-landscape-a4.pdf
- **핵심 사양**:

| 항목 | UR10 | UR10e |
|------|------|-------|
| 페이로드 | 10 kg | 12.5 kg |
| 도달거리 | 1300 mm | 1300 mm |
| 자유도 | 6 | 6 |
| 반복 정밀도 | ±0.1 mm | ±0.05 mm |
| 무게 | 28.9 kg | 33.5 kg |
| 내장 FT 센서 | ❌ | ✅ |
| 힘 제어 | 외부 | 내장 (Force Mode) |

---

## [G3] FerRobotics — Active Contact Flange (ACF)

- **출처**: FerRobotics Compliant Robot Technology
- **링크**: https://www.ferrobotics.com/en/technologies/active-contact-flange/
- **핵심 내용**:
  - 로봇 플랜지에 장착하는 능동 컴플라이언트 장치
  - 공압 기반으로 0.5~50N 범위 일정 힘 유지
  - UR10/UR10e와 호환
  - 표면 높이 변화 ±10mm까지 실시간 보상
- **우리 프로젝트에 적용**: 고품질 폴리싱 필요 시 하드웨어 추가 옵션

---

## [G4] PushCorp — Robotic Finishing End-of-Arm Tooling

- **출처**: PushCorp (산업용 로봇 피니싱 도구 전문)
- **링크**: https://www.pushcorp.com/
- **핵심 내용**:
  - 로봇 피니싱용 스핀들, 컴플라이언트 디바이스, 힘 제어 장치 라인업
  - 일정 힘 제어(Constant Force Control)의 산업 표준 구현체
  - 패드 크기와 접촉 힘의 관계 가이드

---

## [G5] Mirka — Automated Sanding Systems

- **출처**: Mirka (산업용 연마재 제조사)
- **링크**: https://www.mirka.com/en/solutions/automated-solutions/
- **핵심 내용**:
  - 산업용 자동 샌딩/폴리싱 시스템 라인업
  - 자동 도구 교환기(ATC)를 활용한 다중 패드 크기 전략
  - 150mm 패드가 산업 표준 크기로 가장 많이 사용됨
  - 소모품(연마재) 종류별 적용 가이드

---

## [G6] 오버랩 비율 빠른 참조표 (Quick Reference)

150mm (6인치) 패드 기준:

| 오버랩 % | 줄 간격 | 1m 판 패스 수 | 총 경로 | 적합 작업 |
|----------|---------|-------------|---------|----------|
| 25% | 112.5mm | ~9줄 | ~9m | 거친 샌딩 |
| 30% | 105mm | ~10줄 | ~10m | 일반 샌딩 |
| **40%** | **90mm** | **~12줄** | **~12m** | **폴리싱 (권장)** |
| 50% | 75mm | ~14줄 | ~14m | 정밀 폴리싱 |
| 60% | 60mm | ~17줄 | ~17m | 초정밀 마감 |
