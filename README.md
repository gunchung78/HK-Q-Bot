# HK-Q-Bot

강화학습 기반 4족 보행 로봇 프로젝트입니다. 베지어 곡선 보행 생성기와 역기구학으로 기본 걸음을 만들고, 그 위에 강화학습(ARS)으로 학습한 보정값을 얹어 안정적인 보행을 구현합니다. PyBullet 시뮬레이션에서 학습한 정책을 ROS를 통해 실제 로봇까지 옮기는 sim-to-real 파이프라인을 담고 있습니다.

[moribots/spot_mini_mini](https://github.com/moribots/spot_mini_mini) (MIT License)를 기반으로 하며, 하드웨어는 SpotMicro / [OpenQuadruped](https://github.com/adham-elarabawy/OpenQuadruped) 설계를 사용합니다.

## 동작 원리

이 프로젝트는 걸음 전체를 강화학습으로 배우지 않습니다. 핵심은 **모델 기반 보행 + 학습 기반 보정**의 하이브리드 구조입니다.

```
조이스틱/상태머신 명령 (StepLength, YawRate, ...)
        │
        ▼
베지어 보행 생성기 ──── 발끝 궤적 생성 (트롯 보행, 12제어점 베지어 곡선)
        │
        ▼
  + ARS 정책 보정 ──── 14차원 잔차: 발 클리어런스, 몸통 높이, 4발 XYZ 오프셋
        │
        ▼
SE(3) 역기구학 ──────── 몸통 자세 + 발 위치 → 12개 관절각
        │
        ▼
PyBullet 시뮬레이션 / 실제 로봇 (12x 서보)
```

- **보행 생성**: MIT Cheetah 논문의 베지어 곡선 궤적을 사용합니다. 다리별 위상차로 트롯 보행(대각 다리 쌍 동기화)을 만들고, 전진·횡보·제자리 회전을 파라미터로 표현합니다.
- **강화학습**: ARS(Augmented Random Search) 선형 정책이 보행 생성기의 파라미터를 실시간 보정합니다. 탐색 공간이 작아 수렴이 빠르고, 실기 이식에 안전합니다.
- **Sim-to-Real**: 질량·관성 도메인 랜덤화, 센서 노이즈 주입, 랜덤 지형(heightfield)으로 시뮬레이션과 현실의 격차를 줄입니다.

## 프로젝트 구조

```
HK-Q-Bot/
├── hkq/                  # 핵심 라이브러리
│   ├── Kinematics/       #   SE(3) Lie 대수, 다리 역기구학
│   ├── GaitGenerator/    #   베지어 곡선 보행 생성기
│   ├── GymEnvs/          #   PyBullet 기반 OpenAI Gym 환경
│   ├── OpenLoopSM/       #   학습용 명령 상태머신 (전진/횡보/회전 커리큘럼)
│   └── util/             #   URDF, STL 메시, GUI 등
├── hkq_bullet/           # 강화학습
│   ├── src/spot_ars.py   #   ARS 학습 (병렬 rollout)
│   ├── src/spot_ars_eval.py  # 학습된 정책 평가
│   ├── src/env_tester.py #   GUI 슬라이더로 보행 파라미터 수동 테스트
│   └── models/           #   학습된 정책 체크포인트 (접촉센서 유/무)
└── mini_ros/             # ROS 패키지 (Melodic)
    ├── src/              #   상태머신, 텔레옵, PyBullet/실기 인터페이스
    ├── policies/         #   배포용 학습 정책 (방향별)
    ├── launch/           #   시뮬레이션·RViz·실기 launch 파일
    └── urdf/             #   로봇 모델
```

## 시작하기

### 요구 사항

```bash
pip install numpy gym pybullet matplotlib
```

ROS 실행은 Ubuntu 18.04 + ROS Melodic 기준입니다. (시뮬레이션·학습만 할 경우 ROS 불필요)

### 시뮬레이션 테스트 (학습 없이)

GUI 슬라이더로 보행 파라미터(보폭, 요레이트, 몸통 자세 등)를 조절하며 로봇을 걷게 해볼 수 있습니다.

```bash
cd hkq_bullet/src
python env_tester.py
```

### 학습

```bash
cd hkq_bullet/src
python spot_ars.py
```

| 옵션 | 설명 |
|---|---|
| `-hf` | 랜덤 지형(HeightField)에서 학습 |
| `-nc` | 발 접촉 센서 비활성화 |
| `-dr` | 도메인 랜덤화 끄기 |
| `-s <seed>` | 랜덤 시드 지정 (기본 0) |

### 학습된 정책 평가

```bash
cd hkq_bullet/src
python spot_ars_eval.py
```

### ROS 시뮬레이션 구동

```bash
roslaunch mini_ros spot_move.launch
```

조이스틱 명령을 받아 학습된 정책으로 PyBullet 속 로봇을 구동합니다. RViz 시각화는 `view_spot.launch`를 사용하세요.

## 크레딧

- 원본 프로젝트: [Maurice Rahme — spot_mini_mini](https://github.com/moribots/spot_mini_mini)
- 하드웨어 설계: [OpenQuadruped](https://github.com/adham-elarabawy/OpenQuadruped) / [SpotMicroAI](https://spotmicroai.readthedocs.io/en/latest/)
- 베지어 보행 궤적: [MIT Cheetah 논문](https://dspace.mit.edu/handle/1721.1/98270)

## 라이선스

[MIT License](LICENSE)
