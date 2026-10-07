// ================================================================
//  스마트 인솔 - 오른발, 앱(PostureIQ) BLE 전송 [v7 - 루프 주기 수정]
//
//  v7 - v6 대비 변경점 (앱 Latency 가 100ms 목표인데 790~850ms 로 나온 문제):
//   [1] Serial.setTxTimeoutMs(0) 추가. ESP32-C3/S3 는 Serial 이 USB CDC 라서
//       시리얼 모니터를 안 열어두면 write 가 100ms 씩 블로킹된다.
//       loop 안에 print 가 10개 있었으니 그것만으로 ~800ms. 이게 주범.
//   [2] 진단 출력을 매 루프 -> 1초에 1회로 (DEBUG_SERIAL 로 완전히 끌 수도 있음)
//   [3] 진단용 scale.read() 삭제 - HX711 10SPS 기준 100ms 를 더 먹던 줄
//   [4] Serial.begin(9600) -> 115200
//
//  v6 - v5 대비 변경점 (실기기에서 앱이 인솔을 못 찾던 문제):
//   [D-v6] 이름을 스캔 응답이 아니라 '광고 본 패킷'에 넣고, 서비스 UUID는 16비트 축약형 사용.
//          v5는 안드로이드에서 이름이 null 로 보여 앱 스캔 필터에 걸리지 않았다.
//          (앱도 이름 대신 서비스 UUID로 스캔하도록 같이 고침)
//
//  v5 (앱 연동) - v4 대비 변경점:
//   [A] get_units(3) 중복 호출 제거 (v4에 같은 줄이 두 번 -> 밟을 때 블로킹 2배)
//   [B] calibrate 특성(0xA0F3, Write) 추가 - 앱 "지금 보정하기" 버튼과 연결
//        0x01 = 로드셀 tare + 자기장 baseline 재측정 (loop 에서 처리, 콜백은 플래그만)
//   [C] device_info 특성(0xA0F2, Read, 16바이트) 추가 - 앱이 발 구분/펌웨어 버전 확인
//   [D] 광고 데이터를 명시적으로 구성: 광고 패킷 = 플래그 + 128bit 서비스 UUID,
//        스캔 응답 = 이름. (UUID 18바이트 + 이름 13바이트 > 31바이트라 한 패킷에 안 들어감.
//        NimBLE 1.x/2.x 공통 API 라 버전 분기 불필요)
//   [E] 앱 규약 문서화: 앱(sensor-data.js)은 8바이트를 uint16 LE 4개로 파싱하고
//        heel 칸만 사용, arch/ball/toe 는 0 으로 온다는 것을 알고 있음.
//
//  v4 에서 그대로 둔 것 (실측 근거 없이 바꾸지 않음):
//   - 밟음 판정의 비대칭 디바운스(진입 3회 / 이탈 1회)
//   - get_units(3) 샘플 수, delay(100), 핀 배정, MLX 설정
//   - 진단용 시리얼 출력 (주기(ms), mag, raw)
//
//  [주기 측정] 시리얼의 "주기(ms)" 값을 밟았을 때 / 안 밟았을 때 각각 확인할 것.
//   HX711 모듈이 10SPS면 밟는 순간 300ms 이상 늘어남 -> 보행 분석에 부족.
//
//  케이블/실험 있어야 채우는 값 (지금은 임시):
//   - CAL_FACTOR : 보정 안 하면 raw 그대로. 1kg 올려 raw 확인 후 raw/1000
//   - LOAD_MAX_G : 앱 1023에 대응할 최대 하중. 실제 발 하중 범위 보고 결정
//
//  앱 쪽 (application/src/main/resources/static/js/sensor-data.js):
//   - 이름 "PostureIQ-R" 로 스캔 -> 연결 -> 0xA0F1 Notify 구독
//   - 8바이트 LE uint16 x4 파싱, 한쪽만 연결되면 좌우 밸런스는 50/50 처리
//   - device-setup 의 "지금 보정하기" -> 0xA0F3 에 0x01 write
//
//  라이브러리: NimBLE-Arduino / HX711(bogde) / Adafruit MLX90393
//  배선: HX711 DT->10, SCK->3 / MLX SDA->0, SCL->1
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
// 핀 충돌 가드 (보드 미확정 상태이므로 핀 값 자체는 v4 그대로 둠)
//   ESP32 classic  : GPIO1=U0TXD, GPIO3=U0RXD -> 아래 배정은 Serial과 충돌
//   ESP32-C3       : 스트래핑 핀 GPIO2/8/9
//   ESP32-S3       : 스트래핑 핀 GPIO0/3/45/46 -> SDA=0, HX_SCK=3 주의
// ────────────────────────────────────────────────────────────────
#if defined(CONFIG_IDF_TARGET_ESP32)
  #error "ESP32(classic)에서는 GPIO1/GPIO3이 UART0(Serial)입니다. 핀 배정을 바꾼 뒤 이 #error 블록을 삭제하십시오."
#endif

const int HX_DT  = 10;
const int HX_SCK = 3;

// ★ 실험 전 임시값 (케이블 연결 후 확정)
const float CAL_FACTOR = 1.0;      // 보정계수 (1.0 = 보정 안 됨, raw에 가까움)
const float LOAD_MAX_G = 1000.0;   // 앱 0~1023에 대응할 최대 하중(g)

// 펌웨어 버전 (device_info 로 앱에 전달)
const uint8_t FW_MAJOR = 1, FW_MINOR = 0, FW_PATCH = 0;
const char    FOOT_SIDE = 'R';     // 왼발 보드는 'L' 로 바꾸고 이름도 PostureIQ-L
#define DEVICE_NAME "PostureIQ-R"

HX711 scale;
Adafruit_MLX90393 mlx = Adafruit_MLX90393();

bool hxReady  = false;             // HX711 초기화 성공 여부
bool mlxReady = false;             // MLX90393 초기화 성공 여부

// ── GATT UUID (docs/ble-protocol.md 와 동일) ──
#define SERVICE_UUID      "0000a0f9-0000-1000-8000-00805f9b34fb"
#define PRESSURE_UUID     "0000a0f1-0000-1000-8000-00805f9b34fb"   // Notify, 8 bytes
#define DEVICE_INFO_UUID  "0000a0f2-0000-1000-8000-00805f9b34fb"   // Read, 16 bytes
#define CALIBRATE_UUID    "0000a0f3-0000-1000-8000-00805f9b34fb"   // Write, 1 byte

NimBLECharacteristic* loadChar;
NimBLECharacteristic* infoChar;
NimBLECharacteristic* calibChar;
volatile bool    deviceConnected = false;   // BLE 호스트 태스크에서 write, loop에서 read
volatile uint8_t calibRequest    = 0;       // [B] 앱이 쓴 명령 (0=없음, 1=tare, 2=max)

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

// [B] calibrate 쓰기 콜백: BLE 태스크에서 실행되므로 플래그만 세우고 loop 에서 처리
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

// [C] device_info 16바이트 채우기 (ble-protocol.md 2.3)
void fillDeviceInfo(uint8_t* buf) {
  memset(buf, 0, 16);
  buf[0] = (uint8_t)FOOT_SIDE;
  buf[1] = FW_MAJOR; buf[2] = FW_MINOR; buf[3] = FW_PATCH;
  uint8_t mac[6] = {0};
  if (esp_read_mac(mac, ESP_MAC_BT) == ESP_OK) {
    memcpy(buf + 4, mac + 2, 4);     // MAC 하위 4바이트
  }
}

void setup() {
  Serial.begin(115200);
#if ARDUINO_USB_CDC_ON_BOOT
  // [v7] ESP32-C3/S3 는 Serial 이 USB CDC 다. 시리얼 모니터를 열어두지 않으면
  //      송신 버퍼가 차면서 write 가 기본 100ms 씩 블로킹된다.
  //      loop 안에 print 가 여러 개라 이것만으로 루프 주기가 800ms 까지 늘어난다.
  //      0 = 읽는 쪽이 없으면 버리고 즉시 반환 (블로킹 없음).
  Serial.setTxTimeoutMs(0);
#endif
  delay(500);

  // HX711: 미연결이면 tare()가 무한 대기하므로 타임아웃으로 먼저 확인.
  scale.begin(HX_DT, HX_SCK);
  if (scale.wait_ready_timeout(1000)) {
    scale.set_scale(CAL_FACTOR);
    scale.tare();
    hxReady = true;
  } else {
    Serial.println("[경고] HX711 응답 없음 - DT/SCK 배선, 전원 확인");
  }

  // MLX90393: 실패하면 readMag()가 계속 -1 -> stepping이 영원히 false
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

  // [D-v6] 광고 한 패킷에 '이름 + 서비스 UUID'를 모두 넣는다.
  //   v5는 이름을 스캔 응답에만 실었는데, 안드로이드는 한 번도 연결한 적 없는 기기의
  //   BluetoothDevice.getName() 이 null 이라 앱의 이름 필터에서 전부 걸러졌다.
  //   128비트 UUID(18B) + 이름(13B) + 플래그(3B) = 34B > 31B 라서 한 패킷에 안 들어가므로,
  //   같은 UUID의 16비트 축약형(4B)을 쓴다. 3 + 4 + 13 = 20B 로 여유 있게 들어간다.
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
  // 진단용: 실제 루프 주기 측정 (밟음 O/X 각각 확인 후 튜닝 결정, 그 뒤 삭제 가능)
  static unsigned long tPrev = 0;
  unsigned long tNow = millis();
  unsigned long period = tNow - tPrev;   // 첫 루프값은 무의미
  tPrev = tNow;

  // 0) [B] 앱에서 온 캘리브레이션 명령 처리 (발을 뗀 상태에서 눌러야 함)
  if (calibRequest) {
    uint8_t cmd = calibRequest;
    calibRequest = 0;
    Serial.print("[calibrate] cmd="); Serial.println(cmd);
    if (cmd == 0x01) {
      if (hxReady && scale.wait_ready_timeout(500)) scale.tare();
      measureBaseline();
    }
    // 0x02(최대값 보정) 는 LOAD_MAX_G 확정 전까지 미구현, 0x00 은 취소(할 일 없음)
  }

  // 1) 밟음 감지 + 노이즈 방지(3회 연속)
  bool stepping = false;
  float mag = readMag();
  if (mag >= 0 && fabs(mag - baselineB) > STEP_DELTA) {
    stepCounter++;
    if (stepCounter >= STEP_NEEDED) stepping = true;
  } else {
    stepCounter = 0;               // 한 번이라도 벗어나면 초기화
  }

  // 2) 밟았을 때만 총 하중 측정 (노카운트 게이트)
  float loadRaw = 0;               // 보정 전 하중값 (CAL=1.0이면 실제 g 아님)
  if (stepping && hxReady) {
    // 동작 중 케이블이 빠져도 loop가 멈추지 않도록 진입 가드.
    // 주의: get_units(3)의 2~3번째 read()는 여전히 블로킹임.
    if (scale.wait_ready_timeout(200)) {
      loadRaw = scale.get_units(3);
      // [v7] 진단용 scale.read() 삭제 - HX711 이 10SPS 면 이 한 줄이 100ms 를 더 먹었다.
      if (loadRaw < 0) loadRaw = 0;
    }
  }

  // 3) 하중 -> 앱 범위(0~1023): float 에서 먼저 자르고 캐스팅
  float scaled = loadRaw / LOAD_MAX_G * 1023.0f;
  if (scaled < 0.0f)    scaled = 0.0f;
  if (scaled > 1023.0f) scaled = 1023.0f;
  uint16_t totalLoad = (uint16_t)scaled;

  // 4) 8바이트 패킷: 로드셀 1개 총합이라 heel 칸에만 넣고 나머지 0
  //    [E] ESP32 는 little-endian. 앱(sensor-data.js parsePressurePacket)은
  //        DataView.getUint16(offset, true) 로 LE 파싱함. 규약 일치 확인됨.
  uint16_t heel = totalLoad, arch = 0, ball = 0, toe = 0;
  uint8_t packet[8];
  memcpy(packet + 0, &heel, 2);
  memcpy(packet + 2, &arch, 2);
  memcpy(packet + 4, &ball, 2);
  memcpy(packet + 6, &toe,  2);

  // 5) 앱 전송 (앱이 startNotifications 로 CCCD 를 켜야 실제로 흐름)
  if (deviceConnected) {
    loadChar->setValue(packet, 8);
    loadChar->notify();
  }

  // [v7] 진단 출력은 1초에 한 번만.
  //      매 루프 출력하면 출력 자체가 루프 주기를 늘려서 측정값을 망친다.
  //      완전히 끄려면 파일 위쪽 DEBUG_SERIAL 을 0 으로.
  //      CAL 미보정이라 하중은 '보정전'으로 정직하게 표기.
#if DEBUG_SERIAL
  static unsigned long tPrint = 0;
  if (tNow - tPrint >= 1000) {
    tPrint = tNow;
    Serial.print("mag=");           Serial.print(mag, 1);
    Serial.print("  밟음=");         Serial.print(stepping ? "O" : "X");
    Serial.print("  하중(보정전)=");  Serial.print(loadRaw, 0);
    Serial.print("  앱값=");         Serial.print(totalLoad);
    Serial.print("  연결=");         Serial.print(deviceConnected ? "O" : "X");
    Serial.print("  주기(ms)=");     Serial.println(period);
  }
#endif

  delay(100);   // 목표 10Hz - 실제 주기는 위 출력으로 확인할 것
}
