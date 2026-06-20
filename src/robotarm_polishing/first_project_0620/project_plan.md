# First Project 0620: M0609 곡면 폴리싱 시뮬레이션 프로젝트

## 프로젝트 개요
두산 M0609 로봇 팔과 Robotiq 샌딩 키트(Sanding Kit)를 결합하여, 실제 Depth Camera로 스캔한 매직마우스 형태의 3D 곡면을 매끄럽게 폴리싱(Polishing)하는 시뮬레이션 환경을 구축합니다.

## 작업 단계 (Roadmap)

### 1단계: 샌딩 키트 3D 에셋 변환 (STEP -> USD)
- **대상 파일**: `sanding-kit-robotiq-1.snapshot.2/Step/Robotiq_20Sanding_20Kit_20190617.STEP`
- **작업 내용**: Isaac Sim의 CAD Importer(Asset Converter)를 활용해 STEP 파일을 USD 포맷으로 변환. 충돌 메쉬(Collision Mesh)와 질량 속성 최적화.

### 2단계: 로봇-툴 어셈블리 (Robot-Tool Assembly)
- **작업 내용**: M0609 로봇 팔(`m0609.usd`)의 끝단(`link_6`)에 변환 완료된 샌딩 키트(`sanding_kit.usd`)를 부착(Attach).
- **결과물**: 폴리싱 전용 로봇 모델인 `m0609_polishing.usd` 생성.

### 3단계: 곡면 추종 경로 알고리즘 개발
- **입력 데이터**: `scan_project`에서 획득한 `real_camera_surface_points.ply`
- **작업 내용**: 
  1. 스캔된 3D 포인트 클라우드 표면의 법선 벡터(Normal Vector)를 계산하여 툴의 기울기(Orientation)를 동적으로 결정.
  2. M0609 RMPflow 컨트롤러에 타겟 좌표 및 방향을 순차적으로 전달하여 연속적인 래스터(Raster) 폴리싱 궤적을 그리게 함.

### 4단계: 시뮬레이션 통합 및 검증
- **작업 내용**: `run_polishing_sim.py` 스크립트를 통해 전체 환경을 렌더링하고 로봇이 매직마우스 표면을 훑는 동작을 최종적으로 확인.
