// ================================================================
//  스마트 인솔 - 오른발, 앱(PostureIQ) BLE 전송 [v18 - 채널별 부호 설정]
//
//  v18 - LOAD_SIGN 추가. 배선 방향에 따라 누를 때 하중이 음수로 가는데,
//        앱은 0~1023 양수만 받으므로 음수 채널은 항상 0 으로 전송되어
//        시리얼은 변하는데 앱 화면만 멈춘 것처럼 보인다.
//
//  v17 - 피크 초기화 플래그를 로드셀별로 분리.
//        v14 는 전역 플래그를 루프 밖에서 무조건 세워, 첫 루프에 HX711 이
//        변환 중이면 피크가 첫 측정값이 아니라 0 에서 누적되기 시작했다.
//
//  v16 - 전단/수직(자기장 유래)을 전용 특성 0xA0F5 로 전송.
//        압력 패킷(0xA0F1)에 섞지 않는다 - 그 4칸은 압력 규약이라
//        자기장 값을 넣으면 앱의 수직 하중 합산이 오염된다.
//
//  v15 - 자계 센서를 3축으로 분리해 전단/수직을 구분한다.
//        v14 까지는 sqrt(x²+y²+z²) 크기 하나만 써서 방향 정보가 버려졌다.
//        전단 = √(ΔBx²+ΔBy²) 수평 변위, 수직 = ΔBz.
//        (자석이 에코플렉스 같은 연질 층 위에 있어야 변위가 생긴다)
//
//  v14 - v13 의 피크가 누적되지 않던 버그 수정.
//        첫 측정 분기에서 peakInit 을 세우지 않아 매 샘플 리셋됐다.
//        시리얼 t(영점)에도 피크 초기화를 연결.
//
//  v13 - 하중의 최소/최대를 기록해 표시. 흘러가는 숫자만으로는 손으로 누른
//        변화가 노이즈에 묻혀 보이지 않는다. p 로 초기화.
//
//  v12 - r 명령을 일회성 출력에서 연속 출력 토글로 변경.
//        배선을 흔들며 접점을 찾으려면 값이 계속 찍혀야 한다 (0.2초 간격).
//
//  v11 - 로드셀 배선 끊김(24비트 ADC 포화)을 명시적으로 잡아낸다.
//        raw 가 ±8388607 에 붙으면 측정값이 아니라 회로가 끊긴 것이다.
//        그 상태로 tare 하면 영점이 쓰레기값이 되어 하중이 0 에 고정된다.
//
//  v10 - 벤치 테스트용 시리얼 명령 추가 (t=영점, b=기준선, r=원시ADC)
//        + mag 출력에 기준선 대비 차이(Δ) 표시 - STEP_DELTA 튜닝 근거
//
//  v9 - v8 대비 변경점 (실측 데이터로 드러난 문제):
//   [1] 음수 클램프 제거 (버그). 무부하 노이즈는 0 을 중심으로 +,- 로 흔들리는데
//       v8 이 음수만 잘라내서 평균이 위로 밀렸다. 노이즈가 큰 2번 셀의 기준선이
//       0 이 아니라 ~700 으로 떠 있던 원인. 클램프는 앱 전송 직전에만 한다.
//   [2] LOAD_SMOOTH 를 로드셀별 배열로 (노이즈가 큰 셀만 더 눌러줄 수 있도록)
//   [3] MLX90393 변환 시간 단축. 기본값(OSR_3+FILTER_7)은 3축 변환에 약 200ms 라
//       루프 주기가 311ms 로 측정됐다. OSR_1+FILTER_4 로 낮춘다.
//   [4] 고정 delay(100) -> 이번 루프에 쓴 시간을 뺀 나머지만 대기 (실제 10Hz 유지)
//
//  v8 - v7 대비 변경점:
//   [1] 자계 센서와 압력 센서를 분리. v7 은 '밟음' 으로 판정됐을 때만 하중을 읽었지만
//       (노카운트 게이트), 이제 둘을 독립적으로 매 루프 측정한다.
//       밟음 여부는 여전히 계산해서 표시하되 하중 측정을 막지 않는다.
//   [2] HX711 2개 지원. 각각 DT/SCK 와 보정계수를 따로 가진다.
//       (로드셀마다 감도가 달라서 CAL_FACTOR 를 공유하면 안 된다)
//   [3] 두 하중을 각각 출력하고 '차이(1번 - 2번)' 도 함께 출력. 차이는 부호 있는 값.
//   [4] HX711 읽기를 논블로킹으로 변경 - v8 의 핵심.
//       게이트를 없애면 2개를 매 루프 읽게 되는데, HX711 이 10SPS 면
//       get_units(3) x 2 = 약 600ms 라 루프 주기가 무너진다.
//       is_ready() 로 확인하고 '준비된 것만' 1회 읽어 마지막 값을 유지한다.
//       -> 루프는 100ms 를 지키고, 하중은 센서 속도(10SPS)로 갱신된다.
//
//  v7 - v6 대비 (앱 Latency 가 790~850ms 로 나온 문제):
//   - Serial.setTxTimeoutMs(0). USB CDC 는 모니터를 안 열면 write 가 100ms 씩 블로킹된다
//   - 진단 출력을 매 루프 -> 1초 1회, Serial 9600 -> 115200
//
//  v6 - v5 대비 (앱이 인솔을 못 찾던 문제):
//   - 이름을 스캔 응답이 아니라 광고 본 패킷에 넣고 서비스 UUID 는 16비트 축약형 사용
//
//  v5 - calibrate(0xA0F3) / device_info(0xA0F2) 특성 추가
//
//  ※ 실측 근거 없이 바꾸지 않은 것:
//   - 밟음 판정의 비대칭 디바운스(진입 3회 / 이탈 1회)
//   - MLX90393 gain/resolution/oversampling 설정
//
//  케이블/실험 있어야 채우는 값 (지금은 임시):
//   - CAL_FACTOR[] : 로드셀별로 1kg 올려 raw 확인 후 raw/1000
//   - LOAD_MAX_G   : 앱 1023 에 대응할 최대 하중
//
//  앱 쪽 (application/src/main/resources/static/js/sensor-data.js):
//   - 이름 "PostureIQ-R" 또는 서비스 UUID 0xA0F9 로 스캔 -> 0xA0F1 Notify 구독
//   - 8바이트 LE uint16 x4 파싱 (heel, arch, ball, toe)
//   - 로드셀 1번 -> heel, 2번 -> ball 칸에 실어 보낸다 (arch/toe 는 0)
//
//  라이브러리: NimBLE-Arduino / HX711(bogde) / Adafruit MLX90393
//  배선: HX711#1 DT->10, SCK->3 / HX711#2 DT->4, SCK->5 / MLX SDA->0, SCL->1
// ================================================================

#include <NimBLEDevice.h>
#include "HX711.h"
#include <Wire.h>
#include "Adafruit_MLX90393.h"
#include <esp_system.h>
#if __has_include("esp_mac.h")
  #include "esp_mac.h"        // Arduino core 3.x (IDF 5): esp_read_mac 선언 위치
#endif

// ────────────────────────────────────────────────────────────────
// NimBLE 메이저 버전을 직접 지정할 것 (1 또는 2)
//   확인법: Arduino IDE > 스케치 > 라이브러리 포함 > 라이브러리 관리 > "NimBLE" 검색
//   틀리게 적으면 override 때문에 '컴파일 에러'가 납니다.
//   -> 연결은 되는데 데이터만 안 오는 조용한 실패보다 낫습니다.
// ────────────────────────────────────────────────────────────────
#define NIMBLE_MAJOR 2

// 진단용 시리얼 출력 (1초에 한 줄). 배터리로 실사용할 때는 0 으로 두면 완전히 꺼진다.
#define DEBUG_SERIAL 1

// ────────────────────────────────────────────────────────────────
// 핀 충돌 가드
//   ESP32 classic  : GPIO1=U0TXD, GPIO3=U0RXD -> 아래 배정은 Serial과 충돌
//   ESP32-C3       : 스트래핑 핀 GPIO2/8/9 -> 아래 배정은 이 셋을 피했다
//   ESP32-S3       : 스트래핑 핀 GPIO0/3/45/46 -> SDA=0, HX1_SCK=3 주의
// ────────────────────────────────────────────────────────────────
#if defined(CONFIG_IDF_TARGET_ESP32)
  #error "ESP32(classic)에서는 GPIO1/GPIO3이 UART0(Serial)입니다. 핀 배정을 바꾼 뒤 이 #error 블록을 삭제하십시오."
#endif

// ── 로드셀 2개 ──
//   ★ 2번 로드셀 핀(4, 5)은 제안값이다. 실제 배선에 맞게 바꿀 것.
//     C3 에서 GPIO4~7 은 JTAG 겸용이지만 USB-JTAG 을 쓰므로 GPIO 로 써도 된다.
#define LOAD_CELLS 2
const int HX_DT [LOAD_CELLS] = {10, 4};    // 로드셀 1, 2 의 DT(데이터)
const int HX_SCK[LOAD_CELLS] = { 3, 5};    // 로드셀 1, 2 의 SCK(클럭)

// ★ 실험 전 임시값 (케이블 연결 후 확정)
//   로드셀마다 감도가 다르므로 보정계수는 '따로' 가져야 한다.
//   계산식이 (raw - offset) / CAL_FACTOR 이므로, 값을 키우려면 작은 수를 넣는다.
//   [임시] 같은 힘에 1H=40000, 2H=20000 으로 2배 차이가 나서 2번만 0.5 로 맞췄다.
//   이건 두 채널의 '상대 크기'만 맞춘 것이고 절대 단위(g) 보정은 아직 아니다.
//   아는 무게를 올려 측정하면 두 값 모두 제대로 된 계수로 바꿀 수 있다.
const float CAL_FACTOR[LOAD_CELLS] = {1.0, 0.5};  // 1.0 = 보정 안 됨, 0.5 = 2배로 증폭

// [v18] 채널별 부호. 배선(A+/A- 방향)에 따라 누를 때 값이 음수로 갈 수 있다.
//   앱은 압력을 0~1023 양수로만 받으므로, 음수로 가는 채널은 전송 단계에서
//   전부 0 으로 잘려 앱 화면이 멈춘 것처럼 보인다.
//   시리얼에서 '누를 때 하중이 음수로 간다' 면 그 채널을 -1.0 으로 바꿀 것.
//   [실측] 누를 때 1번 -51,506 / 2번 -93,468 로 양쪽 다 음수로 가는 배선이라
//   둘 다 -1.0 으로 뒤집었다. 배선을 바꾸면 이 값도 다시 봐야 한다.
const float LOAD_SIGN[LOAD_CELLS] = {-1.0, -1.0};  // 1.0 = 그대로, -1.0 = 뒤집기
// 앱의 0~1023 에 대응할 '채널 하나당' 최대 하중(g).
//   CAL_FACTOR 보정 후 1 count = 1 g 이므로 이 값이 곧 g 단위다.
//   채널 하나(로드셀 2개)가 받을 수 있는 최대를 50kg 으로 잡았다.
//   ★ 앱의 live.html 에도 같은 값이 KG_PER_COUNT 로 들어간다. 바꾸면 양쪽 다 바꿀 것.
const float LOAD_MAX_G = 50000.0;  // 50 kg

// 하중 평활화 계수 (지수이동평균). 1 = 평활 없음(원값), 클수록 부드럽지만 느려진다.
//   v7 까지 쓰던 get_units(3) 의 평균 효과를 논블로킹에서 대신한다.
//   ★ 로드셀별로 따로 준다. 노이즈가 큰 셀은 값을 올려서 눌러줄 수 있다.
//     다만 평활은 노이즈를 '가려줄 뿐' 분해능을 되살리지는 못한다.
//     근본 해결은 배선/장착 쪽이다.
const float LOAD_SMOOTH[LOAD_CELLS] = {3.0, 3.0};

// 펌웨어 버전 (device_info 로 앱에 전달)
const uint8_t FW_MAJOR = 1, FW_MINOR = 1, FW_PATCH = 0;
const char    FOOT_SIDE = 'R';     // 왼발 보드는 'L' 로 바꾸고 이름도 PostureIQ-L
#define DEVICE_NAME "PostureIQ-R"

HX711 scale[LOAD_CELLS];
Adafruit_MLX90393 mlx = Adafruit_MLX90393();

bool  hxReady[LOAD_CELLS] = {false, false};  // 로드셀별 초기화 성공 여부
float loadRaw[LOAD_CELLS] = {0, 0};          // 보정 전 하중값 (CAL=1.0 이면 실제 g 아님)

// [v11] 24비트 ADC 포화 감지.
//   입력이 ±8388607(0x7FFFFF) 에 붙으면 그건 '측정값' 이 아니라 회로가 끊긴 것이다.
//   브리지 신호선(A+/A-)이 떨어졌거나 로드셀이 망가진 경우 이 값이 나온다.
//   이 상태에서 tare() 를 하면 쓰레기값을 영점으로 저장해서, 하중이 0 으로
//   고정된 채 영원히 안 움직인다. 조용히 실패하지 않도록 명시적으로 잡아낸다.
const long HX_RAIL = 8388600L;
long rawAdc  [LOAD_CELLS] = {0, 0};          // 마지막 원시 ADC 값
bool hxRailed[LOAD_CELLS] = {false, false};  // 포화 여부

// [v13] 피크 기록(최소/최대).
//   흘러가는 숫자만 봐서는 손으로 누른 변화가 노이즈에 묻혀 안 보인다.
//   눌렀다 떼도 그 사이 최대값이 화면에 남아 있어야 '반응했는지' 판단할 수 있다.
float loadMin[LOAD_CELLS] = {0, 0};
float loadMax[LOAD_CELLS] = {0, 0};
// [v17] 초기화 여부를 '로드셀별로' 관리한다.
//   v14 는 전역 플래그 하나를 루프 밖에서 무조건 true 로 바꿨다. 그래서 첫 루프에
//   HX711 이 아직 변환 중이라 아무 셀도 안 읽혔어도 '초기화 완료' 로 처리됐고,
//   피크가 첫 측정값이 아니라 전역 초기값 0 에서 누적되기 시작했다.
//   하중이 계속 음수면 최대값이 0 에 박혀 [-84245 ~ 0] 처럼 보인다.
bool  peakInit[LOAD_CELLS] = {false, false};

void resetPeaks() {
  for (int i = 0; i < LOAD_CELLS; i++) {
    loadMin[i] = loadMax[i] = loadRaw[i];
    peakInit[i] = true;
  }
  Serial.println("[피크] 초기화");
}
bool  mlxReady = false;                      // MLX90393 초기화 성공 여부

// ── GATT UUID (docs/ble-protocol.md 와 동일) ──
#define SERVICE_UUID      "0000a0f9-0000-1000-8000-00805f9b34fb"
#define PRESSURE_UUID     "0000a0f1-0000-1000-8000-00805f9b34fb"   // Notify, 8 bytes
#define DEVICE_INFO_UUID  "0000a0f2-0000-1000-8000-00805f9b34fb"   // Read, 16 bytes
#define CALIBRATE_UUID    "0000a0f3-0000-1000-8000-00805f9b34fb"   // Write, 1 byte
// [v16] 전단/수직(자기장 유래) 전용 특성.
//   압력 패킷(0xA0F1)에 섞지 않는다. 그 패킷의 4칸은 '압력' 규약이라
//   자기장 값을 넣으면 앱의 수직 하중 합산이 오염된다.
#define SHEAR_UUID        "0000a0f5-0000-1000-8000-00805f9b34fb"   // Notify, 4 bytes

NimBLECharacteristic* loadChar;
NimBLECharacteristic* infoChar;
NimBLECharacteristic* calibChar;
NimBLECharacteristic* shearChar;   // [v16]
volatile bool    deviceConnected = false;   // BLE 호스트 태스크에서 write, loop에서 read
volatile uint8_t calibRequest    = 0;       // 앱이 쓴 명령 (0=없음, 1=tare, 2=max)

class ServerCallbacks : public NimBLEServerCallbacks {
#if NIMBLE_MAJOR >= 2
  void onConnect(NimBLEServer* s, NimBLEConnInfo& info) override {
    deviceConnected = true;
  }
  void onDisconnect(NimBLEServer* s, NimBLEConnInfo& info, int reason) override {
    deviceConnected = false;
    NimBLEDevice::startAdvertising();   // 2.x는 자동 재광고 안 함 -> 수동 재시작 필수
  }
#else
  void onConnect(NimBLEServer* s) override {
    deviceConnected = true;
  }
  void onDisconnect(NimBLEServer* s) override {
    deviceConnected = false;
    NimBLEDevice::startAdvertising();   // 1.x는 자동 재시작이지만 명시해도 무해
  }
#endif
};

// calibrate 쓰기 콜백: BLE 태스크에서 실행되므로 플래그만 세우고 loop 에서 처리
class CalibrateCallbacks : public NimBLECharacteristicCallbacks {
  void handle(NimBLECharacteristic* c) {
    auto v = c->getValue();             // 1.3: std::string / 1.4+, 2.x: NimBLEAttValue (둘 다 size(), [] 지원)
    if (v.size() >= 1) calibRequest = (uint8_t)v[0];
  }
#if NIMBLE_MAJOR >= 2
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& info) override { handle(c); }
#else
  void onWrite(NimBLECharacteristic* c) override { handle(c); }
#endif
};

float baselineB = 0;
const float STEP_DELTA = 30.0;     // 실험으로 검증 필요
int  stepCounter = 0;              // 연속 초과 횟수 (노이즈 방지)
const int STEP_NEEDED = 3;         // 3회 연속이면 밟음 확정

// [v15] 3축을 각각 보관한다.
//   v14 까지는 sqrt(x²+y²+z²) 로 '크기' 하나만 반환했는데, 그러면 방향 정보가
//   사라져서 전단(수평 변위)을 알 수 없다. 자석이 옆으로 밀렸는지 아래로
//   눌렸는지 구분하려면 축을 분리해야 한다.
//     수직 하중 -> 자석이 가까워짐 -> Bz 변화
//     전단 하중 -> 자석이 옆으로   -> Bx, By 변화
float magX = 0, magY = 0, magZ = 0;      // 최근 측정값
float baseX = 0, baseY = 0, baseZ = 0;   // 기준선(발 뗀 상태)
float magShear = 0;                      // √(ΔBx² + ΔBy²)  수평 = 전단
float magNormal = 0;                     // ΔBz             수직 (부호 있음)

bool readMag() {
  float x, y, z;
  if (!mlx.readData(&x, &y, &z)) return false;
  magX = x; magY = y; magZ = z;
  return true;
}

// 크기(기존 밟음 판정에 그대로 사용)
float magMagnitude() {
  return sqrt(magX * magX + magY * magY + magZ * magZ);
}

// 발 뗀 상태에서 20회 평균 -> baselineB. 부팅 시와 calibrate(0x01) 시 호출.
void measureBaseline() {
  // [v15] 축별로 평균을 낸다. 전단/수직을 분리하려면 축마다 기준선이 필요하다.
  float sx = 0, sy = 0, sz = 0; int cnt = 0;
  for (int i = 0; i < 20; i++) {
    if (readMag()) { sx += magX; sy += magY; sz += magZ; cnt++; }
    delay(20);
  }
  if (cnt > 0) {
    baseX = sx / cnt; baseY = sy / cnt; baseZ = sz / cnt;
    baselineB = sqrt(baseX * baseX + baseY * baseY + baseZ * baseZ);
  } else {
    baseX = baseY = baseZ = 0; baselineB = 0;
    Serial.println("[경고] baseline 측정 실패 - 자기장 값을 하나도 못 읽음");
  }
  magShear = magNormal = 0;
  stepCounter = 0;
  Serial.print("baseline  X="); Serial.print(baseX, 1);
  Serial.print(" Y=");          Serial.print(baseY, 1);
  Serial.print(" Z=");          Serial.print(baseZ, 1);
  Serial.print("  크기=");       Serial.println(baselineB, 1);
}

// 모든 로드셀 영점 잡기 (발을 뗀 상태에서 호출할 것)
void tareAll() {
  for (int i = 0; i < LOAD_CELLS; i++) {
    if (!hxReady[i]) continue;
    if (scale[i].wait_ready_timeout(500)) {
      // [v11] 포화 상태에서 영점을 잡으면 쓰레기값이 기준이 되어 하중이 0 에 고정된다.
      //   덮어쓰지 말고 거부하고 알린다.
      long probe = scale[i].read();
      if (probe >= HX_RAIL || probe <= -HX_RAIL) {
        Serial.print("[tare] 로드셀"); Serial.print(i + 1);
        Serial.print(" ★영점 건너뜀 - ADC 포화(raw="); Serial.print(probe);
        Serial.println("). 배선부터 고칠 것");
        hxRailed[i] = true;
        continue;
      }
      scale[i].tare();
      loadRaw[i] = 0;
      Serial.print("[tare] 로드셀"); Serial.print(i + 1); Serial.println(" 영점 완료");
    } else {
      Serial.print("[tare] 로드셀"); Serial.print(i + 1); Serial.println(" 응답 없음");
    }
  }
}

// [v12] 원시 ADC 연속 출력 on/off. 배선을 흔들면서 값 변화를 보려면
//   일회성 출력으로는 안 되고 계속 찍혀야 한다.
bool rawMonitor = false;

// [v10] 벤치 테스트용 시리얼 명령
//   t = 로드셀 2개 영점 재설정
//   b = 자기장 기준선 재측정
//   r = 원시 ADC 연속 출력 토글 (배선 진단)
void handleSerialCommand() {
  if (!Serial.available()) return;
  int c = Serial.read();
  while (Serial.available()) Serial.read();   // 개행 등 나머지 버퍼 비우기
  switch (c) {
    case 't': case 'T':
      Serial.println("[명령] 영점 재설정");
      tareAll();
      resetPeaks();   // [v14] 영점을 바꿨으면 피크도 같이 초기화해야 의미가 맞는다
      break;
    case 'b': case 'B':
      Serial.println("[명령] 자기장 기준선 재측정");
      measureBaseline();
      break;
    case 'p': case 'P':
      resetPeaks();
      break;
    case 'r': case 'R':
      rawMonitor = !rawMonitor;
      Serial.println(rawMonitor
        ? "[명령] 원시 ADC 연속 출력 ON - 배선을 흔들며 값 변화를 볼 것 (끄려면 r 다시)"
        : "[명령] 원시 ADC 연속 출력 OFF");
      break;
    default: break;
  }
}

// device_info 16바이트 채우기 (ble-protocol.md 2.3)
void fillDeviceInfo(uint8_t* buf) {
  memset(buf, 0, 16);
  buf[0] = (uint8_t)FOOT_SIDE;
  buf[1] = FW_MAJOR; buf[2] = FW_MINOR; buf[3] = FW_PATCH;
  uint8_t mac[6] = {0};
  if (esp_read_mac(mac, ESP_MAC_BT) == ESP_OK) {
    memcpy(buf + 4, mac + 2, 4);     // MAC 하위 4바이트
  }
}

// 하중(보정 전) -> 앱 범위(0~1023).
//   float 에서 먼저 자르고 캐스팅한다. 범위를 벗어난 float->uint16 변환은 UB 라서
//   캐스팅 후 클램프하면 랩어라운드된 값이 그대로 통과할 수 있다.
uint16_t toAppScale(float raw) {
  float scaled = raw / LOAD_MAX_G * 1023.0f;
  if (scaled < 0.0f)    scaled = 0.0f;
  if (scaled > 1023.0f) scaled = 1023.0f;
  return (uint16_t)scaled;
}

void setup() {
  Serial.begin(115200);
#if ARDUINO_USB_CDC_ON_BOOT
  // ESP32-C3/S3 는 Serial 이 USB CDC 다. 시리얼 모니터를 열어두지 않으면
  // 송신 버퍼가 차면서 write 가 기본 100ms 씩 블로킹된다.
  // 0 = 읽는 쪽이 없으면 버리고 즉시 반환 (블로킹 없음).
  Serial.setTxTimeoutMs(0);
#endif
  delay(500);

  // HX711 x2: 미연결이면 tare()가 무한 대기하므로 타임아웃으로 먼저 확인.
  //   하나가 없어도 나머지와 BLE 는 계속 동작하게 둔다 (디버깅 가능하도록).
  for (int i = 0; i < LOAD_CELLS; i++) {
    scale[i].begin(HX_DT[i], HX_SCK[i]);
    if (scale[i].wait_ready_timeout(1000)) {
      scale[i].set_scale(CAL_FACTOR[i]);
      scale[i].tare();
      hxReady[i] = true;
      Serial.print("HX711 #"); Serial.print(i + 1); Serial.println(" 준비 완료");
    } else {
      Serial.print("[경고] HX711 #"); Serial.print(i + 1);
      Serial.print(" 응답 없음 - DT="); Serial.print(HX_DT[i]);
      Serial.print(" SCK=");            Serial.print(HX_SCK[i]);
      Serial.println(" 배선/전원 확인");
    }
  }

  // MLX90393: 실패하면 readMag()가 계속 -1 -> stepping 이 영원히 false
  //   (v8 부터 하중 측정은 이것과 무관하게 계속 돈다)
  Wire.begin(0, 1);
  mlxReady = mlx.begin_I2C(0x18);
  if (!mlxReady) {
    Serial.println("[경고] MLX90393 응답 없음 - SDA/SCL 배선, I2C 주소 확인");
  } else {
    // [v9] 변환 시간을 줄인다. Adafruit 기본값(OSR_3 + FILTER_7)은 3축 변환에
    //   약 200ms 가 걸려서 readData() 한 번이 루프의 2/3 를 먹는다 (실측 주기 311ms).
    //   밟음 감지는 '기준선 대비 변화량' 만 보므로 이 정도 정밀도가 필요 없다.
    //   ★ 노이즈가 늘면 STEP_DELTA 를 같이 올려야 한다. 아래 두 값은 튜닝 대상.
    mlx.setOversampling(MLX90393_OSR_1);
    mlx.setFilter(MLX90393_FILTER_4);
  }

  measureBaseline();

  // ── BLE ──
  NimBLEDevice::init(DEVICE_NAME);
  NimBLEServer* server = NimBLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());
  NimBLEService* svc = server->createService(SERVICE_UUID);

  loadChar  = svc->createCharacteristic(PRESSURE_UUID, NIMBLE_PROPERTY::NOTIFY);

  infoChar  = svc->createCharacteristic(DEVICE_INFO_UUID, NIMBLE_PROPERTY::READ);
  uint8_t info[16];
  fillDeviceInfo(info);
  infoChar->setValue(info, 16);

  calibChar = svc->createCharacteristic(CALIBRATE_UUID,
                                        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  calibChar->setCallbacks(new CalibrateCallbacks());

  shearChar = svc->createCharacteristic(SHEAR_UUID, NIMBLE_PROPERTY::NOTIFY);   // [v16]

  svc->start();

  // 광고 한 패킷에 '이름 + 서비스 UUID' 를 모두 넣는다.
  //   128비트 UUID(18B) + 이름(13B) + 플래그(3B) = 34B > 31B 라 한 패킷에 안 들어가므로,
  //   같은 UUID 의 16비트 축약형(4B)을 쓴다. 3 + 4 + 13 = 20B.
  //   (0xA0F9 는 블루투스 base UUID 계열이라 안드로이드가 128비트로 복원해서 매칭한다)
  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  NimBLEAdvertisementData advData;
  advData.setFlags(BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP);
  advData.addServiceUUID(NimBLEUUID((uint16_t)0xA0F9));
  advData.setName(DEVICE_NAME);
  adv->setAdvertisementData(advData);
  NimBLEDevice::startAdvertising();

  Serial.println("BLE 시작: " DEVICE_NAME " 광고 중.");
}

void loop() {
  // 진단용: 실제 루프 주기 측정
  static unsigned long tPrev = 0;
  unsigned long tNow = millis();
  unsigned long period = tNow - tPrev;   // 첫 루프값은 무의미
  tPrev = tNow;

  // 0-a) 시리얼 명령 (t=영점, b=기준선, r=원시ADC) - 벤치 테스트용
  handleSerialCommand();

  // 0-b) 앱에서 온 캘리브레이션 명령 처리 (발을 뗀 상태에서 눌러야 함)
  if (calibRequest) {
    uint8_t cmd = calibRequest;
    calibRequest = 0;
    Serial.print("[calibrate] cmd="); Serial.println(cmd);
    if (cmd == 0x01) {
      tareAll();          // 로드셀 2개 모두 영점
      measureBaseline();  // 자기장 기준값 재측정
      resetPeaks();       // 영점을 다시 잡았으면 피크도 초기화
    }
    // 0x02(최대값 보정) 는 LOAD_MAX_G 확정 전까지 미구현, 0x00 은 취소(할 일 없음)
  }

  // ── 1) 자계 센서 (압력과 독립) ─────────────────────────────
  //   밟음 여부는 '표시용' 으로만 쓴다. v7 과 달리 하중 측정을 막지 않는다.
  bool stepping = false;
  bool magOk = readMag();
  float mag = magOk ? magMagnitude() : -1.0f;
  if (magOk) {
    // [v15] 기준선 대비 변위를 축별로 분해
    //   수평 성분 = 자석이 옆으로 밀린 양 -> 전단
    //   수직 성분 = 자석이 가까워/멀어진 양 -> 법선(수직 하중)
    float dx = magX - baseX, dy = magY - baseY, dz = magZ - baseZ;
    magShear  = sqrt(dx * dx + dy * dy);
    magNormal = dz;
  }
  if (mag >= 0 && fabs(mag - baselineB) > STEP_DELTA) {
    stepCounter++;
    if (stepCounter >= STEP_NEEDED) stepping = true;
  } else {
    stepCounter = 0;               // 한 번이라도 벗어나면 초기화
  }

  // ── 2) 압력 센서 2개 (자계와 독립, 논블로킹) ────────────────
  //   is_ready() 로 '변환이 끝난 것만' 읽는다. 준비 안 된 로드셀은 직전 값을 유지.
  //   블로킹 읽기를 쓰면 10SPS x 2개 = 루프가 수백 ms 로 늘어난다.
  for (int i = 0; i < LOAD_CELLS; i++) {
    if (!hxReady[i]) continue;
    if (scale[i].is_ready()) {
      // [v11] get_units() 대신 raw 를 직접 읽는다. 포화 판정을 하려면 원시값이 필요하다.
      //   (get_units 는 내부에서 offset/scale 을 적용해버려 레일 상태가 안 보인다)
      long raw = scale[i].read();        // is_ready() 확인 후라 즉시 반환
      rawAdc[i]   = raw;
      hxRailed[i] = (raw >= HX_RAIL || raw <= -HX_RAIL);
      // [v18] 부호를 여기서 한 번에 적용한다. 이후 모든 계산(피크, 차이, 앱 전송)이
      //   같은 방향을 쓰게 되어 시리얼과 앱이 어긋나지 않는다.
      float v = LOAD_SIGN[i] * (float)(raw - scale[i].get_offset()) / scale[i].get_scale();
      // [v9] 여기서 음수를 0 으로 자르면 안 된다.
      //   무부하 상태의 노이즈는 0 을 중심으로 +,- 로 흔들리는데
      //   음수만 잘라내면 평균이 위로 밀려서 '있지도 않은 하중' 이 생긴다.
      //   노이즈가 큰 로드셀일수록 이 편향이 커진다 (2번 셀의 기준선 ~700 의 원인).
      //   음수 클램프는 앱 전송 직전(toAppScale)에서만 한다.
      const float s = LOAD_SMOOTH[i];
      loadRaw[i] = (s <= 1.0f) ? v : (loadRaw[i] * (s - 1.0f) + v) / s;
      // [v14] 피크 갱신.
      //   v13 은 첫 측정 분기에서 peakInit 을 세우지 않아, p 를 누르기 전까지
      //   매 샘플마다 최소/최대가 현재값으로 리셋됐다 (항상 [X ~ X] 로 표시).
      //   첫 바퀴는 초기화만 하고, 그 다음부터 누적한다.
      // [v17] 그 셀을 실제로 읽은 순간에만 초기화 완료로 표시한다.
      if (!peakInit[i]) {
        loadMin[i] = loadMax[i] = loadRaw[i];
        peakInit[i] = true;
      } else {
        if (loadRaw[i] < loadMin[i]) loadMin[i] = loadRaw[i];
        if (loadRaw[i] > loadMax[i]) loadMax[i] = loadRaw[i];
      }
    }
  }

  // ── 3) 두 하중의 차이 (부호 있음: 1번이 크면 양수) ──────────
  float loadDiff = loadRaw[0] - loadRaw[1];

  // ── 4) 앱 전송용 8바이트 패킷 ──────────────────────────────
  //   로드셀 1번 -> heel, 2번 -> ball. arch/toe 는 아직 센서가 없어 0.
  //   차이는 앱에서 heel - ball 로 계산할 수 있으므로 따로 싣지 않는다
  //   (uint16 이라 음수를 담을 수 없기도 하다).
  //   ESP32 는 little-endian, 앱은 DataView.getUint16(offset, true) 로 LE 파싱한다.
  uint16_t heel = toAppScale(loadRaw[0]);
  uint16_t ball = toAppScale(loadRaw[1]);
  uint16_t arch = 0, toe = 0;
  uint8_t packet[8];
  memcpy(packet + 0, &heel, 2);
  memcpy(packet + 2, &arch, 2);
  memcpy(packet + 4, &ball, 2);
  memcpy(packet + 6, &toe,  2);

  // ── 5) 앱 전송 (앱이 startNotifications 로 CCCD 를 켜야 실제로 흐름) ──
  if (deviceConnected) {
    loadChar->setValue(packet, 8);
    loadChar->notify();

    // [v16] 전단/수직 (자기장 유래) 4바이트.
    //   int16 두 개, 각각 0.1 단위로 실어 보낸다 (×10 후 반올림).
    //   전단은 항상 0 이상, 수직은 부호가 있어 int16 을 쓴다.
    //   범위를 넘으면 자르고 보낸다 (랩어라운드 방지).
    float sh = magShear  * 10.0f;
    float nz = magNormal * 10.0f;
    if (sh >  32767.0f) sh =  32767.0f;  if (sh < -32768.0f) sh = -32768.0f;
    if (nz >  32767.0f) nz =  32767.0f;  if (nz < -32768.0f) nz = -32768.0f;
    int16_t shearOut  = (int16_t)sh;
    int16_t normalOut = (int16_t)nz;
    uint8_t spacket[4];
    memcpy(spacket + 0, &shearOut,  2);
    memcpy(spacket + 2, &normalOut, 2);
    shearChar->setValue(spacket, 4);
    shearChar->notify();
  }

  // ── 6) 진단 출력 (1초에 한 번) ─────────────────────────────
  //   매 루프 출력하면 출력 자체가 루프 주기를 늘려서 측정값을 망친다.
  //   완전히 끄려면 파일 위쪽 DEBUG_SERIAL 을 0 으로.
  //   CAL 미보정이라 하중은 '보정전' 값이다.
#if DEBUG_SERIAL
  // [v12] 원시 ADC 연속 출력 (r 로 토글). 1초 간격이면 선을 흔드는 반응을 놓치므로
  //   0.2초마다 찍는다. 추가 변환 없이 위에서 이미 읽은 값을 그대로 쓴다.
  if (rawMonitor) {
    static unsigned long tRaw = 0;
    if (tNow - tRaw >= 200) {
      tRaw = tNow;
      Serial.print("raw1=");  Serial.print(rawAdc[0]);
      if (hxRailed[0]) Serial.print(" ★포화");
      Serial.print("\traw2="); Serial.print(rawAdc[1]);
      if (hxRailed[1]) Serial.print(" ★포화");
      Serial.println();
    }
  }

  static unsigned long tPrint = 0;
  if (tNow - tPrint >= 1000) {
    tPrint = tNow;
    // mag 는 절대값보다 '기준선 대비 차이' 가 중요하다. STEP_DELTA 튜닝의 근거가 된다.
    Serial.print("mag=");       Serial.print(mag, 1);
    Serial.print("(Δ");         Serial.print(mag >= 0 ? fabs(mag - baselineB) : -1.0, 1);
    Serial.print("/");          Serial.print(STEP_DELTA, 0);
    Serial.print(") 밟음=");     Serial.print(stepping ? "O" : "X");
    Serial.print(" | 하중1=");   Serial.print(loadRaw[0], 0);
    Serial.print(" 하중2=");     Serial.print(loadRaw[1], 0);
    Serial.print(" 차이=");      Serial.print(loadDiff, 0);
    Serial.print(" | 앱값=");    Serial.print(heel);
    Serial.print(",");          Serial.print(ball);
    Serial.print(" 연결=");      Serial.print(deviceConnected ? "O" : "X");
    // [v18] 앱이 실제로 받는 값이 0 에 고정돼 있으면 부호가 뒤집힌 것이다.
    //   (하중은 변하는데 앱값이 계속 0 이면 LOAD_SIGN 을 -1.0 으로)
    if (deviceConnected && heel == 0 && ball == 0 && (loadRaw[0] < -50 || loadRaw[1] < -50)) {
      Serial.print(" ★[부호확인] 하중이 음수라 앱에 0 으로 전송됨");
    }
    Serial.print(" 주기(ms)=");  Serial.print(period);
    // [v11] 포화는 배선 끊김이다. 값이 0 으로 고정돼 보이는 것과 구분되도록 크게 알린다.
    if (hxRailed[0] || hxRailed[1]) {
      Serial.print("  ★[배선확인] ADC 포화:");
      for (int i = 0; i < LOAD_CELLS; i++) {
        if (hxRailed[i]) { Serial.print(" "); Serial.print(i + 1); Serial.print("번"); }
      }
      Serial.print(" (신호선 A+/A- 끊김 의심)");
    }
    Serial.println();

    // [v13] 피크. 눌렀다 떼도 그 사이 최대/최소가 남아 있어 반응 여부를 판단할 수 있다.
    Serial.print("   피크  1번[");  Serial.print(loadMin[0], 0);
    Serial.print(" ~ ");            Serial.print(loadMax[0], 0);
    Serial.print("]  2번[");        Serial.print(loadMin[1], 0);
    Serial.print(" ~ ");            Serial.print(loadMax[1], 0);
    Serial.println("]   (p = 초기화)");

    // [v15] 자기장 3축. 자석을 옆으로 밀면 전단, 위아래로 움직이면 수직이 변한다.
    Serial.print("   자기장 X=");  Serial.print(magX, 1);
    Serial.print(" Y=");           Serial.print(magY, 1);
    Serial.print(" Z=");           Serial.print(magZ, 1);
    Serial.print("  |  전단=");     Serial.print(magShear, 1);
    Serial.print("  수직=");        Serial.println(magNormal, 1);
  }
#endif

  // [v9] 고정 delay(100) 대신 '이번 루프에 쓴 시간' 을 빼고 남은 만큼만 쉰다.
  //   센서 읽기 시간이 그대로 주기에 더해지던 문제를 없애 10Hz 를 실제로 지킨다.
  const unsigned long TARGET_MS = 100;
  unsigned long spent = millis() - tNow;
  if (spent < TARGET_MS) delay(TARGET_MS - spent);
}
