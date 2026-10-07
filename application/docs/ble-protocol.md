# PostureIQ BLE Protocol Specification

**대상**: 펌웨어 개발자 (아두이노/ESP32 측), 앱 개발자 (Capacitor 측)
**버전**: 0.2 (앱 BLE 수신 구현 + 펌웨어 v5 반영)
**최종 수정**: 2026-09-03

> **현재 구현 상태 요약 (2026-09-03)** — 자세한 내용은 [§8](#8-현재-구현-상태-펌웨어-v5--앱)
>
> | 항목 | 펌웨어 `insole_ble_app.ino` (v5) | 앱 `static/js/sensor-data.js` |
> |---|---|---|
> | 광고 이름 | `PostureIQ-R` (오른발만, 스캔응답에 이름) | `PostureIQ-` 접두어로 스캔, 접미어로 L/R 구분 |
> | pressure_data 0xA0F1 | ✅ Notify 8B, **heel 칸에 로드셀 총합**, 나머지 0 | ✅ LE uint16×4 파싱, 한쪽만 연결 시 밸런스 50/50 |
> | device_info 0xA0F2 | ✅ Read 16B | ✅ `readDeviceInfo()` |
> | calibrate 0xA0F3 | ✅ Write 1B (`0x01` tare+baseline) | ✅ `calibrate()` ← device-setup "지금 보정하기" |
> | battery_level 0x2A19 | ❌ 미구현 (배터리 회로 없음) | 읽기 시도 후 없으면 `—` 표시 |
> | sampling_rate 0xA0F4 | ❌ 미구현 (고정 100ms) | 미사용 |

---

## 0. 합의 사항 (먼저 결정)

펌웨어 시작 전에 팀에서 다음 결정사항을 확정하고 이 문서에 반영하세요:

| 항목 | 기본값(권장) | 확정값 |
|---|---|---|
| MCU/BLE 모듈 | ESP32-WROOM-32 (BLE 내장) | ESP32-C3/S3 계열 (classic ESP32 는 핀 충돌로 펌웨어가 `#error`) — 보드 최종 확정 TBD |
| 인솔 구성 | 좌·우 각각 독립 BLE 디바이스 (2개 페어링) | 1차: **오른발 1개** (`PostureIQ-R`). 앱은 L/R 둘 다 지원 |
| 압력 센서 종류 | FSR-402 또는 Velostat | **로드셀 1개 + HX711** (총하중) + **MLX90393** 자기장 (밟음 감지 게이트) |
| 발당 센서 개수 | **4개** (뒤꿈치/아치/볼/발가락) | **1개(총합)** → `sensor_heel` 칸에 넣고 arch/ball/toe = 0 |
| ADC 해상도 | 10-bit (0~1023) — 12-bit 도 OK | 0~1023 (하중 g → `LOAD_MAX_G` 기준 스케일, 값은 실험 후 확정) |
| 샘플링 주기 | 100ms (10 Hz) | 100ms 목표 (HX711 10SPS 면 밟을 때 늘어남 — 실측 필요) |
| 배터리 | 3.7V Li-Po 500mAh ~ 1000mAh | TBD (battery_level 미구현) |

---

## 1. 디바이스 발견 (Discovery)

### 1.1 Advertising

좌·우 인솔은 각각 독립 BLE 페리페럴로 동작.

| 인솔 | Local Name (광고 이름) |
|---|---|
| 왼발 | `PostureIQ-L` |
| 오른발 | `PostureIQ-R` |

**Advertising 데이터에 포함**:
- Local Name (위 형식)
- Service UUID (아래 1.2)
- TX Power (선택)
- Manufacturer Data: `[0x50, 0x49, foot]` — `'P'`, `'I'`, foot(`L`=0x4C, `R`=0x52)

**광고 주기**: 100~200ms (페어링 중일 때만)
**페어링 후**: 광고 중단

### 1.2 Service UUID

본 프로젝트 전용 128-bit UUID:

```
PostureIQ Pressure Service:  0xA0F9 (Custom 16-bit, vendor-allocated)
                             또는 풀 UUID: 0000a0f9-0000-1000-8000-00805f9b34fb
```

> 캡스톤이라 vendor UUID 등록은 불필요. 16-bit `0xA0F9` 사용해도 동작.

---

## 2. GATT 구조

```
PostureIQ Pressure Service (0xA0F9)
├── pressure_data        (Notify)     — 실시간 압력 데이터
├── battery_level        (Read+Notify) — 표준 0x2A19
├── device_info          (Read)        — 펌웨어 정보 + 발 식별
├── calibrate            (Write)       — 캘리브레이션 명령
└── sampling_rate        (Read+Write)  — 샘플링 주기 변경
```

### 2.1 pressure_data (Notify)

**UUID**: `0xA0F1` (Custom)
**Properties**: NOTIFY only
**Packet 크기**: 8 bytes
**Frequency**: 10 Hz (100ms 주기)

**Packet 포맷** (Little-Endian uint16):

| Offset | Bytes | Name | Range | 설명 |
|---|---|---|---|---|
| 0~1 | 2 | sensor_heel | 0~1023 | 뒤꿈치 (Heel) |
| 2~3 | 2 | sensor_arch | 0~1023 | 아치 (Midfoot) |
| 4~5 | 2 | sensor_ball | 0~1023 | 발볼 (Forefoot, 1st metatarsal head) |
| 6~7 | 2 | sensor_toe | 0~1023 | 발가락 (Hallux/Big toe) |

**예시 패킷** (왼발 뒤꿈치 870, 아치 312, 볼 580, 발가락 220):
```
[0x66 0x03  0x38 0x01  0x44 0x02  0xDC 0x00]
   870        312        580        220
```

### 2.2 battery_level (Read + Notify)

**UUID**: `0x2A19` (표준 Battery Service Characteristic)
**Properties**: READ, NOTIFY
**Packet 크기**: 1 byte
**Value**: 0~100 (%)

배터리 15% 이하로 떨어지면 NOTIFY로 알림 (선택).

### 2.3 device_info (Read)

**UUID**: `0xA0F2` (Custom)
**Properties**: READ
**Packet 크기**: 16 bytes (고정)

| Offset | Bytes | Name | 예시 |
|---|---|---|---|
| 0 | 1 | foot_side | `0x4C` ('L') or `0x52` ('R') |
| 1 | 1 | firmware_major | `0x01` |
| 2 | 1 | firmware_minor | `0x00` |
| 3 | 1 | firmware_patch | `0x00` |
| 4~7 | 4 | mac_lower | MAC 주소 하위 4 byte |
| 8~15 | 8 | reserved | 0으로 채움 |

### 2.4 calibrate (Write)

**UUID**: `0xA0F3` (Custom)
**Properties**: WRITE
**Packet 크기**: 1 byte

| Value | 의미 |
|---|---|
| `0x00` | 캘리브레이션 취소 |
| `0x01` | 영점 (Tare) 캘리브레이션 시작 — 5초간 평균값을 baseline으로 저장 |
| `0x02` | 최대값 캘리브레이션 — 사용자가 최대 압력 인가 시점 기준 |

캘리브레이션 완료 시 `pressure_data` notification에 임시 플래그를 띄우는 대신, 단순히 다음 notification부터 정상 값 송출.

### 2.5 sampling_rate (Read + Write)

**UUID**: `0xA0F4` (Custom)
**Properties**: READ, WRITE
**Packet 크기**: 1 byte
**Value**: Hz (1~100)

기본값 10. 앱에서 5/10/20/50 등으로 변경 가능. 배터리 절약 시 1 Hz, 고정밀 시 50 Hz.

---

## 3. 데이터 해석 (앱 측)

### 3.1 원본 → 정규화

펌웨어가 보내는 `0~1023` ADC 값을 앱은 **0~100% 정규화 압력**으로 변환:

```js
function normalize(raw) {
  // 캘리브레이션 baseline 제거 (옵션)
  const adjusted = Math.max(0, raw - baseline);
  // 0~100% 클램핑
  return Math.min(100, (adjusted / 1023) * 100);
}
```

### 3.2 좌우 밸런스 계산

```js
const leftTotal  = L.heel + L.arch + L.ball + L.toe;
const rightTotal = R.heel + R.arch + R.ball + R.toe;
const total = leftTotal + rightTotal;

const leftPct  = (leftTotal  / total) * 100;
const rightPct = (rightTotal / total) * 100;
```

### 3.3 이벤트 트리거

- 좌우 차이가 **15% 이상 + 10초 이상 지속** → "불균형 알림"
- 한쪽 발의 한 센서가 다른 센서보다 **3배 이상** → "압력 집중"
- 캘리브레이션 baseline 대비 모든 센서 0에 가까움 → "발 떨어짐 감지" (insole 벗었을 가능성)

---

## 4. 펌웨어 의사 코드 (ESP32 BLE 기준)

팀원에게 참고용. 실제 코드는 BLE 라이브러리(`NimBLE-Arduino` 등) 사용 권장.

```cpp
// setup()
BLEDevice::init("PostureIQ-L");  // 또는 -R
BLEServer* server = BLEDevice::createServer();
BLEService* svc = server->createService("0000a0f9-0000-1000-8000-00805f9b34fb");

// pressure_data characteristic
pressureChar = svc->createCharacteristic(
  "0000a0f1-0000-1000-8000-00805f9b34fb",
  BLECharacteristic::PROPERTY_NOTIFY);
pressureChar->addDescriptor(new BLE2902());

// loop()
uint16_t heel = analogRead(PIN_HEEL);
uint16_t arch = analogRead(PIN_ARCH);
uint16_t ball = analogRead(PIN_BALL);
uint16_t toe  = analogRead(PIN_TOE);

uint8_t packet[8];
memcpy(packet + 0, &heel, 2);
memcpy(packet + 2, &arch, 2);
memcpy(packet + 4, &ball, 2);
memcpy(packet + 6, &toe,  2);

pressureChar->setValue(packet, 8);
pressureChar->notify();
delay(100);  // 10 Hz
```

---

## 5. 테스트 도구

펌웨어가 위 사양대로 동작하는지 **앱 없이도** 검증 가능:

- **nRF Connect** (안드로이드/iOS 무료 앱)
  - `PostureIQ-L` 검색 → 연결 → pressure_data Notify 구독 → 16진수 패킷 확인
- **Bluefruit Connect** (Adafruit, 비슷한 기능)
- **bleak** (Python 라이브러리, PC에서 테스트)

펌웨어 1차 완성 시 위 도구로 검증 → 검증 통과 후 앱 연결.

---

## 6. 통합 체크리스트

펌웨어 ↔ 앱 통합 시 확인:

- [ ] Local Name이 정확히 `PostureIQ-L` / `PostureIQ-R`
- [ ] Service UUID `0xA0F9` advertising에 포함
- [ ] pressure_data 패킷 8 bytes, Little-Endian uint16
- [ ] Notify 주기 100ms (±10ms 허용)
- [ ] battery_level 1 byte, 0~100
- [ ] device_info의 foot_side가 'L'/'R' 정확
- [ ] calibrate 명령 수신 후 baseline 갱신 확인
- [ ] 연결 끊김 시 자동 재연결 가능 (advertising 재개)

---

## 7. 변경 이력

| 버전 | 날짜 | 변경 사항 |
|---|---|---|
| 0.1 | 2026-05-13 | 초안 작성 |
| 0.2 | 2026-09-03 | 앱 BLE 수신(`BluetoothDataSource`) 구현, 펌웨어 v5(calibrate/device_info 추가) 반영, §8 추가 |

---

## 8. 현재 구현 상태 (펌웨어 v5 ↔ 앱)

### 8.1 펌웨어 (`insole_ble_app.ino`, 저장소 루트)

- 광고 패킷 = 플래그 + 128-bit 서비스 UUID, **스캔 응답 = 이름 `PostureIQ-R`** (31바이트 제한 때문에 분리).
  앱과 nRF Connect 모두 액티브 스캔이라 이름이 정상 표시됨.
- `pressure_data`: 밟음(MLX90393 자기장 변화 3회 연속) 일 때만 HX711 로 하중을 읽고,
  `loadRaw / LOAD_MAX_G * 1023` 을 **heel** 칸에 넣어 100ms 마다 notify. 안 밟으면 모두 0.
- `calibrate` `0x01`: 로드셀 `tare()` + 자기장 baseline 20회 재측정 (**발을 뗀 상태에서** 앱의 "지금 보정하기").
  `0x02`(최대값) 는 `LOAD_MAX_G` 확정 전까지 미구현.
- `device_info`: `['R', 1, 0, 0, MAC 하위 4B, 0…]`.
- 왼발 보드를 만들 때: `FOOT_SIDE = 'L'`, `DEVICE_NAME "PostureIQ-L"` 두 곳만 바꾸면 됨.
- `NIMBLE_MAJOR` 를 설치된 NimBLE-Arduino 메이저 버전(1 또는 2)에 맞출 것. 틀리면 컴파일 에러.

### 8.2 앱 (`application/src/main/resources/static/js/sensor-data.js`)

```
PostureIQ.createDataSource()        // localStorage 'postureiq.source' 가 'ble' 면 BluetoothDataSource
source.start()                      // 저장된 L/R 장치(localStorage 'postureiq.ble.device.{L|R}')에 재연결
source.pair()                       // 앱: 8초 스캔 후 PostureIQ-L/R 전부 연결 / 브라우저: 선택 창
source.on('reading', r => ...)      // { feet:{L,R}, left, right, balance, source:'ble' }
source.calibrate(0x01)              // 연결된 발 전부에 write
```

- **안드로이드 앱**: `@capacitor-community/bluetooth-le` 네이티브 플러그인 사용. 번들러가 없으므로
  `static/js/vendor/capacitor.js` + `bluetooth-le.js` (플러그인의 IIFE 빌드) 를 `<script>` 로 로드.
  플러그인 업데이트 시 `npm run vendor:ble` 로 갱신.
- **브라우저(Render 웹앱)**: 같은 플러그인의 Web Bluetooth 구현으로 동작 (Chrome/Edge, HTTPS 필수).
  브라우저는 보안상 **사용자 클릭 안에서만** 장치 선택이 가능 → 페이지를 열 때마다 [인솔 검색·연결] 버튼 필요.
- 끊기면 3초 후 자동 재연결. 페이지를 이동해도 네이티브 쪽 연결은 유지되고, 다음 페이지의 `start()` 가
  "Already connected" 로 즉시 성공함.

### 8.3 화면 연결

| 화면 | 동작 |
|---|---|
| `live.html` | 저장된 소스(Mock/BLE)로 시작. 한쪽 발만 연결되면 "오른발 측정 중" 으로 표시하고 좌우 불균형 판정은 생략. 하단에 Mock↔BLE 전환 / 인솔 검색 버튼 |
| `device.html` | 좌/우 카드에 실제 연결 상태·배터리 표시. [새 장치 페어링] → 스캔+연결+저장. 데이터 소스(Mock/BLE) 선택, 저장된 인솔 지우기 |
| `device-setup.html` | 연결 상태 표시 + [지금 보정하기] → `calibrate 0x01` |

### 8.4 안드로이드 권한 (`android/app/src/main/AndroidManifest.xml`)

`BLUETOOTH_SCAN`(neverForLocation) / `BLUETOOTH_CONNECT` 를 선언했고 `BleClient.initialize({ androidNeverForLocation: true })`
로 초기화하므로 Android 12+ 에서는 위치 권한 없이 **"근처 기기" 권한**만 요청함. Android 11 이하는 위치 권한 요청.

### 8.5 통합 테스트 순서

1. 펌웨어 업로드 → 시리얼에 `BLE 시작: PostureIQ-R 광고 중.` 확인
2. nRF Connect 로 `PostureIQ-R` 보이는지, 0xA0F1 Notify 켜면 8바이트가 100ms 마다 오는지 확인
3. 앱 → Device 탭 → [새 장치 페어링] → 카드가 "연결됨" 으로 바뀌는지
4. Live 탭 → Source 가 `BLE`, 밟으면 오른발 히트맵 heel 이 밝아지는지, Latency(패킷 간격) ≈ 100ms 인지
5. Device 설정 → [지금 보정하기] → 시리얼에 `[calibrate] cmd=1` 과 새 baseline 출력되는지
