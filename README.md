# PostureIQ

발 압력 인솔 센서 기반 자세·보행 모니터링 시스템 (캡스톤 프로젝트).

스마트 인솔(ESP32)이 BLE로 보낸 압력 데이터를 안드로이드 앱과 웹에서 실시간으로 시각화합니다.

```
┌─────────────────┐   BLE    ┌──────────────────┐
│  스마트 인솔      │ ───────▶ │  PostureIQ 앱     │
│  ESP32-C3        │  10 Hz   │  (Android / Web)  │
│  HX711 + MLX90393│  8 bytes │                   │
└─────────────────┘          └──────────────────┘
```

---

## 저장소 구조

| 경로 | 내용 |
|---|---|
| `insole_ble_app.ino` | **펌웨어** — ESP32 Arduino 스케치 (오른발 인솔) |
| `application/` | **앱** — Spring Boot 웹앱 + Capacitor 안드로이드 앱 (HTML 단일 소스) |
| `application/docs/ble-protocol.md` | **BLE 프로토콜 사양서** — 펌웨어↔앱 규약, 현재 구현 상태 |
| `render.yaml` | Render 배포 정의 (웹앱) |

---

## 하드웨어

| 부품 | 역할 |
|---|---|
| ESP32-C3 SuperMini | MCU + BLE |
| 로드셀 + HX711 | 총 하중 측정 |
| MLX90393 | 자기장 변화로 밟음 감지 (하중 측정 게이트) |
| 3.7V 리포 배터리 + TP4056 | 전원 |

**배선**

| 부품 핀 | 보드 핀 |
|---|---|
| HX711 DT / SCK | 10 / 3 |
| MLX90393 SDA / SCL | 0 / 1 |
| 센서 VIN / GND | 3V3 / G |

> ⚠️ 이 보드는 `5V` 핀이 USB VBUS와 직결이고 충전 회로가 없습니다.
> 배터리가 연결된 채 USB를 꽂으면 배터리에 5V가 걸립니다. **업로드 전 배터리를 분리**하세요.

---

## 1. 펌웨어 빌드

**필요 라이브러리** (Arduino IDE 라이브러리 관리자)
- `NimBLE-Arduino` (h2zero)
- `HX711 Arduino Library` (Bogdan Necula)
- `Adafruit MLX90393`

**보드 설정**
- 보드: `ESP32C3 Dev Module`
- **USB CDC On Boot: Enabled** (안 켜면 시리얼 출력이 안 보임)

**업로드 전 확인**
- 스케치 상단의 `NIMBLE_MAJOR` 를 설치된 NimBLE 메이저 버전(1 또는 2)에 맞출 것.
  틀리면 컴파일 에러가 납니다 (조용한 미동작보다 낫도록 의도한 것).
- 왼발 보드를 만들 때는 `FOOT_SIDE` 와 `DEVICE_NAME` 두 곳만 `L` / `PostureIQ-L` 로 변경.

**업로드 후** 시리얼 모니터(115200)에 다음이 뜨면 정상입니다.

```
baseline(자기장)=xx.x
BLE 시작: PostureIQ-R 광고 중.
```

---

## 2. 웹앱 로컬 실행

```bash
cd application
SPRING_PROFILES_ACTIVE=dev ./gradlew bootRun
```

→ `http://localhost:8080`. dev 프로파일이면 HTML 수정이 빌드 없이 즉시 반영됩니다.

---

## 3. 안드로이드 APK 빌드

```bash
cd application
npm install
npm run apk
```

→ `application/android/app/build/outputs/apk/debug/app-debug.apk`

**사전 준비**: Node.js 18+, JDK 21, Android SDK (API 35 + Build-Tools 35).

> ⚠️ **Windows 주의** — 프로젝트 경로에 한글이나 공백이 있으면
> Android Gradle Plugin 이 `project path contains non-ASCII characters` 로 빌드를 거부합니다.
> 그럴 때는 프로젝트를 영문 경로(예: `C:\dev\postureiq`)로 복사해서 빌드하세요.

---

## 4. BLE 프로토콜 요약

전체 사양은 [`application/docs/ble-protocol.md`](application/docs/ble-protocol.md) 참고.

| 항목 | 값 |
|---|---|
| 광고 이름 | `PostureIQ-R` / `PostureIQ-L` |
| Service | `0xA0F9` (`0000a0f9-0000-1000-8000-00805f9b34fb`) |
| `pressure_data` | `0xA0F1` Notify, **8 bytes** = LE uint16 × 4 (heel, arch, ball, toe) |
| `device_info` | `0xA0F2` Read, 16 bytes (발 구분 + 펌웨어 버전) |
| `calibrate` | `0xA0F3` Write, 1 byte (`0x01` = 영점 보정) |
| 주기 | 100ms (10 Hz 목표) |

현재 하드웨어는 로드셀 1개(총 하중)라 **`heel` 칸만 사용**하고 나머지 3칸은 0으로 보냅니다.

앱은 이름이 아니라 **서비스 UUID로 스캔**합니다. 안드로이드의 이름 필터는 OS가 캐시한
`BluetoothDevice.getName()` 을 보기 때문에 처음 연결하는 기기는 `null` 로 걸러지기 때문입니다.

---

## 5. 현재 상태 / TODO

**동작 확인됨**
- [x] 펌웨어 → 앱 BLE 연결 및 실시간 데이터 수신
- [x] 앱 내 페어링, 자동 재연결, 영점 보정 명령

**남은 작업**
- [ ] `CAL_FACTOR` / `LOAD_MAX_G` 실측 보정 → 현재 Live 화면의 kg 값은 보정 전 raw 기준
- [ ] HX711 샘플링 속도 (10 SPS → 80 SPS, RATE 핀 개조) — 보행 분석에 필요
- [ ] 왼발 인솔 제작 및 좌우 밸런스 검증
- [ ] `battery_level` (0x2A19) 펌웨어 구현 — 앱은 이미 읽도록 되어 있음
- [ ] PostgreSQL 연동 및 세션 기록 저장
- [ ] 정적 HTML → Thymeleaf 전환, 사용자 인증
