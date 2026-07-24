# Following-Cart

> NFC 인증 및 UWB 기반 사용자 추종 자율주행 스마트 카트

---

## 프로젝트 개요

**Following-Cart**는 실내 환경에서 사용자를 자동으로 따라다니는 자율주행 스마트 카트이다.

사용자는 별도의 UWB 태그 없이 본인의 스마트폰만 소지하며, 카트는 UWB를 이용하여 사용자의 스마트폰 위치를 실시간으로 추적한다. LiDAR를 이용하여 주변 장애물을 인식하고 안전한 이동 경로를 생성하여 사용자를 안정적으로 추종한다.

NFC 인증 기능을 적용하여 등록된 사용자만 카트를 잠금 및 해제할 수 있도록 설계하여 보안성과 편의성을 향상시킨다. 사용자는 스마트폰을 NFC 리더에 태그하는 것만으로 카트를 활성화할 수 있다. Raspberry Pi 5와 Arduino를 ROS2 기반으로 분산 연결하여 AI 연산과 모터 제어를 분리함으로써 안정적이고 효율적인 자율주행 시스템을 구현한다.

---

## 개발 배경

대형마트, 공항, 병원 및 물류센터에서는 많은 짐을 직접 운반해야 하는 경우가 많아 사용자의 피로도가 높다. 기존 카트는 사용자가 직접 밀거나 끌어야 하며 장애물 인식이나 자동 추종 기능이 부족하다.

실내 환경에서는 GPS 사용이 어렵고, Bluetooth 기반 추종 방식은 위치 정확도가 낮아 안정적인 사용자 추종이 어렵다. 이를 해결하기 위해 **UWB 기반 위치 추적**과 **ROS2 자율주행 기술**을 결합한 스마트 카트를 개발한다.

---

## 차별성

| 방식 | 한계 |
|------|------|
| GPS | 실내 사용 불가 |
| Bluetooth | 위치 오차 큼 |
| Camera 단독 | 사람이 가려질 경우 추종 어려움 |

**Following-Cart의 핵심 차별점:**

- UWB 기반 고정밀 사용자 위치 추적 (사용자 스마트폰을 직접 추적, 별도 태그 불필요)
- LiDAR 기반 장애물 회피
- IMU + Wheel Encoder를 이용한 EKF Sensor Fusion (UWB range-only 사용자 위치 추정에도 활용)
- ROS2 기반 분산 제어 시스템
- NFC 기반 사용자 인증 및 잠금/해제 (스마트폰 태그 방식)

---

## 개발 목표

**최종 목표:** 실내 환경에서 사용자를 자동으로 따라오며 장애물을 회피하는 스마트 카트 개발

**세부 목표:**
- UWB 기반 사용자 스마트폰 위치 추적 (range-only + Odometry EKF 융합)
- LiDAR 기반 장애물 탐지
- ROS2 기반 분산 시스템 구축
- PID 기반 안정적인 모터 제어
- NFC 기반 사용자 인증 및 잠금/해제 (스마트폰 태그)
- Android 컴패니언 앱 개발 (NFC 활성화 트리거 + UWB 레인징 세션 유지)

---

## 시스템 구성

### AI 처리부 — Raspberry Pi 5 (8GB)

| 기능 |
|------|
| LiDAR 데이터 처리 |
| UWB 거리 기반 사용자 위치 추정 (range-only EKF) |
| EKF 기반 Sensor Fusion (IMU + Wheel Encoder + UWB) |
| Navigation2 |
| 장애물 회피 및 경로 생성 |
| NFC 인증 처리 |

> Jetson Nano 대신 Raspberry Pi 5를 채택. ROS2 Jazzy(Ubuntu 24.04)를 공식 이미지로 바로 사용할 수 있어 개발환경 구축 리스크가 적고, 카메라 기반 딥러닝 인식을 계획에서 제외했기 때문에 GPU가 없어도 무방함.

### 제어부 — Arduino

| 기능 |
|------|
| DC 모터 제어 |
| Encoder 처리 |
| PWM 출력 |
| PID 속도 제어 |

### 센서 및 구동부

**센서:** LiDAR (YDLIDAR X4 Pro) / UWB 모듈 (Arduino Stella, 로봇 탑재) / IMU / Wheel Encoder / NFC Reader (RC522) / OLED 디스플레이 (SSD1306)

**구동부:** Encoder DC Motor ×2 / Motor Driver (MD20A)

> 사용자 측에는 별도 UWB 태그/팔찌가 없으며, 사용자의 스마트폰(UWB 지원 기종)이 태그 역할을 겸함. 카메라는 설계에서 제외.

---

## ⚙️ 시스템 동작 과정

```
1. 사용자 스마트폰 NFC 태그 인식 → 카트 활성화
2. UWB(Stella) → 사용자 스마트폰과의 거리 측정
3. Raspberry Pi 5 → 로봇 오도메트리(IMU+Encoder) 기반 EKF로 사용자 위치 추정
4. LiDAR → 장애물 탐지 및 costmap 생성
5. Navigation2 → 추정된 사용자 위치를 목표로 로컬 경로 생성 (장애물 회피 포함)
6. cmd_vel → Arduino 전송
7. Arduino → PID 제어 (엔코더 모터)
8. 카트 → 장애물 회피 + 사용자 추종
9. 사용 종료 → 스마트폰 NFC 태그 인식 → 카트 잠금
```

---

## 사용 기술

| 분류 | 내용 |
|------|------|
| OS | Ubuntu 24.04 |
| 미들웨어 | ROS2 Jazzy |
| 언어 | C++, Python, Kotlin/Java (Android 앱) |
| 개발 도구 | VSCode, GitHub, Fusion 360, KiCad, Arduino IDE, Android Studio |
| 라이브러리 | ROS2, Navigation2, TF2, Eigen, PCL |
| 모바일 | Android UWB Jetpack API (`androidx.core.uwb`), NFC HCE |

---

## 주요 시스템 개선 사항

**① IMU 적용**
- EKF 기반 Sensor Fusion으로 위치 및 자세 추정 정확도 향상
- UWB range-only 거리값과 결합해 사용자 위치 추정에도 활용

**② Wheel Encoder 적용**
- PID 속도 제어 및 Odometry 생성

**③ PCA9685 PWM Driver**
- Servo PWM 신호 안정화 및 떨림 방지

**④ 전원 분리**
- RPi5 전용 레귤레이터 / LiDAR 전용 레귤레이터로 계통 분리 → 전압 강하 및 노이즈 감소
- 보드용 배터리(14.8V)와 모터용 배터리(22.2V) 이원화
- LiDAR는 USB_DATA(RPi5, 데이터)와 USB_PWR(별도 5V 레일, 전원)을 분리 연결

**⑤ ROS2 분산 시스템**
- Raspberry Pi 5: AI 연산 및 경로 생성
- Arduino: 엔코더 모터 제어

**⑥ NFC 사용자 인증**
- 등록된 사용자 스마트폰만 카트 사용 가능
- NFC 기반 잠금/잠금 해제

**⑦ UWB range-only + Odometry 융합 추적**
- 별도 AoA 하드웨어 없이 단일 UWB 모듈(Stella)만으로 사용자 위치 추정
- 초기 위치 fix(비선형 최소자승법) → 지속 추적(EKF) → 로컬 목표 지점 갱신

---

## 하드웨어 설계

Fusion 360으로 직접 설계 후 3D 프린터로 제작. 모듈형 구조로 유지보수 용이. 전장 회로는 KiCad로 별도 설계 진행 중.

설계 항목: 카트 프레임 / LiDAR 거치대 / UWB(Stella) 거치대 / NFC 모듈 거치대 / OLED 디스플레이 거치대 / 보드용·모터용 배터리 홀더(분리) / 전자부품 보호 케이스 / 케이블 정리 구조

---

## 성능 목표

| 항목 | 목표값 |
|------|--------|
| 사용자 추종 성공률 | 95% 이상 |
| 장애물 회피 성공률 | 95% 이상 |
| 추종 가능 거리 | 1 ~ 10m |
| 위치 오차 | ±20 ~ 30cm |
| 최고 주행 속도 | 약 1m/s |
| 제어 주기 | 10 ~ 20Hz |
| 연속 운행 시간 | 2시간 이상 (※ 보드용 배터리 용량 기준 재검토 필요 — 하단 BOM 비고 참고) |

---

## 기대 효과

**기술적 효과**
- ROS2 기반 자율주행 시스템 구현
- UWB 기반 실내 위치 추적 기술 확보
- Sensor Fusion 및 분산 제어 시스템 구현
- NFC 기반 사용자 인증 기술 적용

**경제적 효과**
- 물류 자동화, 인건비 절감, 운반 효율 향상

**사회적 효과**
- 스마트 쇼핑 카트 / 병원 물품 운반 / 공항 수하물 운반
- 공장 자동 운반 / 노약자 이동 보조 / 실내 서비스 로봇

---

## 역할 분담

| 역할 | 담당 |
|------|------|
| 팀장 | 일정 관리, ROS2 통합, 발표 |
| AI 및 위치추적 | Navigation2, UWB 사용자 추종 알고리즘(EKF) |
| 센서 | LiDAR, UWB, IMU, NFC, Sensor Fusion |
| 제어 | Arduino, PID 제어, Encoder, Servo, Motor Driver |
| 하드웨어 | Fusion 360/KiCad 설계, 3D 프린팅, 조립, 전원 설계, 테스트 |
| 모바일 앱 *(미배정)* | Android UWB+NFC 컴패니언 앱 개발 |

---

## 개발 일정

| 주차 | 내용 |
|------|------|
| 1주차 | 요구사항 분석, 시스템 설계 |
| 2주차 | CAD 설계, 부품 구매 |
| 3주차 | ROS2 환경 구축, Raspberry Pi 5 / Arduino 설정 |
| 4주차 | LiDAR, UWB, NFC 개별 테스트, Android 앱 프로토타입 |
| 5주차 | 모터 제어, Encoder, PID 제어, Servo 제어 |
| 6주차 | 사용자 추종 알고리즘(UWB range-only EKF), Navigation2 적용, Sensor Fusion |
| 7주차 | 시스템 통합, 성능 튜닝 |
| 8주차 | 최종 테스트, 발표 자료 제작, 시연 준비 |

---

## 향후 발전 방향

- 음성 명령 기능
- 자율 복귀 기능
- 자동 충전 시스템
- AI 기반 사람 인식
- 클라우드 모니터링
- 다중 사용자 추종
- 원격 모니터링 시스템
- iOS 앱 지원 확대