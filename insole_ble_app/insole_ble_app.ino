// ================================================================
//  스마트 인솔 - 오른발, 앱(PostureIQ) BLE 전송 [v8 - 로드셀 2개 / 센서 분리]
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
const float CAL_FACTOR[LOAD_CELLS] = {1.0, 1.0};  // 1.0 = 보정 안 됨(raw에 가까움)
const float LOAD_MAX_G = 1000.0;   // 앱 0~1023 에 대응할 최대 하중(g)

// 하중 평활화 계수 (지수이동평균). 1 = 평활 없음(원값), 클수록 부드럽지만 느려진다.
//   v7 까지 쓰던 get_units(3) 의 평균 효과를 논블로킹에서 대신한다.
const float LOAD_SMOOTH = 3.0;

// 펌웨어 버전 (device_info 로 앱에 전달)
const uint8_t FW_MAJOR = 1, FW_MINOR = 1, FW_PATCH = 0;
const char    FOOT_SIDE = 'R';     // 왼발 보드는 'L' 로 바꾸고 이름도 PostureIQ-L
#define DEVICE_NAME "PostureIQ-R"

HX711 scale[LOAD_CELLS];
Adafruit_MLX90393 mlx = Adafruit_MLX90393();

bool  hxReady[LOAD_CELLS] = {false, false};  // 로드셀별 초기화 성공 여부
float loadRaw[LOAD_CELLS] = {0, 0};          // 보정 전 하중값 (CAL=1.0 이면 실제 g 아님)
bool  mlxReady = false;                      // MLX90393 초기화 성공 여부

// ── GATT UUID (docs/ble-protocol.md 와 동일) ──
#define SERVICE_UUID      "0000a0f9-0000-1000-8000-00805f9b34fb"
#define PRESSURE_UUID     "0000a0f1-0000-1000-8000-00805f9b34fb"   // Notify, 8 bytes
#define DEVICE_INFO_UUID  "0000a0f2-0000-1000-8000-00805f9b34fb"   // Read, 16 bytes
#define CALIBRATE_UUID    "0000a0f3-0000-1000-8000-00805f9b34fb"   // Write, 1 byte

NimBLECharacteristic* loadChar;
NimBLECharacteristic* infoChar;
NimBLECharacteristic* calibChar;
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

float readMag() {
  float x, y, z;
  if (mlx.readData(&x, &y, &z)) return sqrt(x*x + y*y + z*z);
  return -1;
}

// 발 뗀 상태에서 20회 평균 -> baselineB. 부팅 시와 calibrate(0x01) 시 호출.
void measureBaseline() {
  float sum = 0; int cnt = 0;
  for (int i = 0; i < 20; i++) {
    float b = readMag();
    if (b >= 0) { sum += b; cnt++; }
    delay(20);
  }
  baselineB = (cnt > 0) ? sum / cnt : 0;
  stepCounter = 0;
  if (cnt == 0) Serial.println("[경고] baseline 측정 실패 - 자기장 값을 하나도 못 읽음");
  Serial.print("baseline(자기장)="); Serial.println(baselineB, 1);
}

// 모든 로드셀 영점 잡기 (발을 뗀 상태에서 호출할 것)
void tareAll() {
  for (int i = 0; i < LOAD_CELLS; i++) {
    if (!hxReady[i]) continue;
    if (scale[i].wait_ready_timeout(500)) {
      scale[i].tare();
      loadRaw[i] = 0;
      Serial.print("[tare] 로드셀"); Serial.print(i + 1); Serial.println(" 영점 완료");
    } else {
      Serial.print("[tare] 로드셀"); Serial.print(i + 1); Serial.println(" 응답 없음");
    }
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

  // 0) 앱에서 온 캘리브레이션 명령 처리 (발을 뗀 상태에서 눌러야 함)
  if (calibRequest) {
    uint8_t cmd = calibRequest;
    calibRequest = 0;
    Serial.print("[calibrate] cmd="); Serial.println(cmd);
    if (cmd == 0x01) {
      tareAll();          // 로드셀 2개 모두 영점
      measureBaseline();  // 자기장 기준값 재측정
    }
    // 0x02(최대값 보정) 는 LOAD_MAX_G 확정 전까지 미구현, 0x00 은 취소(할 일 없음)
  }

  // ── 1) 자계 센서 (압력과 독립) ─────────────────────────────
  //   밟음 여부는 '표시용' 으로만 쓴다. v7 과 달리 하중 측정을 막지 않는다.
  bool stepping = false;
  float mag = readMag();
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
      float v = scale[i].get_units(1);   // is_ready() 확인 후라 즉시 반환
      if (v < 0) v = 0;                  // 음수 하중은 0 으로 (영점 드리프트)
      loadRaw[i] = (LOAD_SMOOTH <= 1.0f)
                 ? v
                 : (loadRaw[i] * (LOAD_SMOOTH - 1.0f) + v) / LOAD_SMOOTH;
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
  }

  // ── 6) 진단 출력 (1초에 한 번) ─────────────────────────────
  //   매 루프 출력하면 출력 자체가 루프 주기를 늘려서 측정값을 망친다.
  //   완전히 끄려면 파일 위쪽 DEBUG_SERIAL 을 0 으로.
  //   CAL 미보정이라 하중은 '보정전' 값이다.
#if DEBUG_SERIAL
  static unsigned long tPrint = 0;
  if (tNow - tPrint >= 1000) {
    tPrint = tNow;
    Serial.print("mag=");       Serial.print(mag, 1);
    Serial.print(" 밟음=");      Serial.print(stepping ? "O" : "X");
    Serial.print(" | 하중1=");   Serial.print(loadRaw[0], 0);
    Serial.print(" 하중2=");     Serial.print(loadRaw[1], 0);
    Serial.print(" 차이=");      Serial.print(loadDiff, 0);
    Serial.print(" | 앱값=");    Serial.print(heel);
    Serial.print(",");          Serial.print(ball);
    Serial.print(" 연결=");      Serial.print(deviceConnected ? "O" : "X");
    Serial.print(" 주기(ms)=");  Serial.println(period);
  }
#endif

  delay(100);   // 목표 10Hz - 실제 주기는 위 출력으로 확인할 것
}
