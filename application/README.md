# PostureIQ

발 압력 인솔 센서 기반 자세·보행 모니터링 시스템 (캡스톤 프로젝트).

이 저장소는 **두 가지 배포 형태**를 함께 담고 있습니다:

| 형태 | 빌드 | 배포 |
|---|---|---|
| **웹 앱** | Spring Boot 4 + 정적 HTML | Render Docker → https://postureiq-ac33.onrender.com |
| **안드로이드 앱 (.apk)** | Capacitor 7로 같은 HTML을 WebView 패키징 | 로컬 빌드 → 휴대폰에 직접 설치 |

같은 `src/main/resources/static/` 안의 HTML이 **양쪽 모두의 단일 소스**입니다.

---

## 페이지 구조

| 경로 | 페이지 | 메인↔서브 관계 |
|---|---|---|
| `/index.html` | 대시보드 | → `/analysis.html` |
| `/live.html` | 실시간 모니터링 | → `/precision.html` |
| `/reports.html` | 리포트 | → `/clinical-report.html` |
| `/device.html` | 디바이스 | → `/device-setup.html` |

---

## 1. 웹 앱 로컬 실행

```powershell
$env:SPRING_PROFILES_ACTIVE = "dev"
.\gradlew.bat bootRun
```

→ `http://localhost:8080`. dev 프로파일이면 HTML 수정이 빌드 없이 즉시 반영됩니다.

---

## 2. 안드로이드 .apk 빌드

### 사전 준비 (한 번만)

1. **Node.js 18+** — 이미 v24.x 설치됨 ✅
2. **JDK 21** — 이미 Temurin 21 설치됨 ✅
3. **Android Studio** — 아래에서 설치

#### Android Studio 설치

1. https://developer.android.com/studio 에서 Windows 인스톨러 다운로드 (~1GB)
2. 설치 마법사에서 **"Standard"** 선택 → Android SDK · Platform Tools · Build Tools 자동 설치 (~5~10GB 추가 다운로드)
3. 설치 끝나면 한 번 실행 → **More Actions → SDK Manager** 에서:
   - SDK Platforms: **Android 14 (API 34)** 이상 체크
   - SDK Tools: **Android SDK Build-Tools**, **Android SDK Platform-Tools** 체크
4. `ANDROID_HOME` 환경변수 설정 (보통 `C:\Users\<user>\AppData\Local\Android\Sdk`)

### 🔄 매번 코드 수정 후 → 새 APK 만드는 워크플로우 (A 방식)

```powershell
# 프로젝트 루트에서 실행 — sync + 빌드 한 번에
cd C:\Users\user\OneDrive\Desktop\capstone
npm.cmd run apk
```

이 한 줄이 자동으로 다음을 수행합니다:
1. `src/main/resources/static/`의 HTML 변경사항을 `android/app/src/main/assets/public/`로 동기화
2. Gradle로 debug APK 빌드

처음엔 ~5분, 이후엔 ~30초~1분 소요.

#### 빌드 후 APK 위치

```
C:\Users\user\OneDrive\Desktop\capstone\android\app\build\outputs\apk\debug\app-debug.apk
```

이 `.apk` 파일을 카톡/USB/메일로 폰에 보내서 설치 (기존 PostureIQ 앱 위에 덮어쓰기로 업데이트됨 — 데이터 손실 없음).

#### 기타 명령

```powershell
npm.cmd run sync          # HTML만 sync (빌드 X)
npm.cmd run android:open  # Android Studio에서 열기 (GUI 빌드)
npm.cmd run apk:clean     # 캐시 다 지우고 클린 빌드 (문제 발생 시)
```

### Android Studio GUI로 빌드 (대안)

1. Android Studio 실행 → **Open** → `C:\Users\user\OneDrive\Desktop\capstone\android` 폴더 선택
2. Gradle sync 자동 시작 (첫 실행 시 의존성 다운로드 ~5분)
3. 상단 메뉴 **Build → Generate App Bundles or APKs → Generate APKs**
4. 우하단 알림에서 **"locate"** 클릭 → APK 파일 위치로 이동
5. 또는 폰을 USB로 연결한 채 ▶️ Run 버튼 → 폰에 바로 설치/실행

---

## 3. 프로젝트 구조

작업 루트: `C:\Users\user\OneDrive\Desktop\capstone\`

```
capstone/
├── build.gradle                       # Spring Boot 4 + Java 21  [필수]
├── settings.gradle                    # 31 bytes, Dockerfile이 COPY함  [필수]
├── gradlew, gradlew.bat, gradle/      # Gradle wrapper 9.4.1  [필수]
│                                      #   gradlew(확장자 없음) = Render(리눅스)용
│                                      #   gradlew.bat          = 내 PC(윈도우)용
├── Dockerfile                         # Render 멀티스테이지 빌드  [필수]
├── render.yaml                        # Render IaC  [필수]
│
├── package.json                       # npm run apk 등 스크립트 정의  [필수]
├── package-lock.json                  # Capacitor 7.6.5 버전 고정  [필수]
├── capacitor.config.json              # appId, webDir 경로  [필수]
│
├── docs/
│   ├── ble-protocol.md                # ⭐ 아두이노 팀 전달용 BLE 사양서
│   └── PostureIQ_Capstone_5min.pptx   # 5분 발표 자료
│
├── src/main/
│   ├── java/com/postureiq/
│   │   └── PostureIqApplication.java  # Spring Boot 엔트리
│   └── resources/
│       ├── application.properties
│       ├── application-dev.properties
│       └── static/                    # ⭐ 웹/앱 공용 단일 소스
│           ├── index.html ~ device-setup.html  (8 페이지)
│           ├── js/sensor-data.js      # Mock 데이터 + BLE 소스 골격
│           ├── tailwind-config.js
│           ├── favicon.svg
│           └── robots.txt
│
├── src/test/java/com/postureiq/
│   └── PostureIqApplicationTests.java # 스모크 테스트 (배포 전 안전장치)
│
├── android/                           # Capacitor 생성 안드로이드 프로젝트
│   ├── app/                           # 앱 모듈
│   ├── local.properties               # SDK 경로, git 제외 + 복구 불가  [필수]
│   ├── capacitor.settings.gradle      # node_modules/@capacitor/android 참조
│   └── build.gradle, settings.gradle, variables.gradle
│
└── node_modules/                      # npm install 로 복구 가능하지만
                                       # gradlew assembleDebug 가 직접 참조함  [유지]
```

> **삭제하면 안 되는 것들**: 위에 `[필수]` 표시된 파일은 작아 보여도 지우면 빌드·배포가 깨집니다.
> 특히 `gradlew`(확장자 없는 쪽)는 내 PC에선 안 쓰여서 중복처럼 보이지만 **Render 배포가 이걸 씁니다**.
>
> **지워도 되는 것들** (전부 자동 재생성): `build/`, `.gradle/`, `bin/`,
> `android/build/`, `android/app/build/`, `android/.gradle/`, `android/.idea/`

---

## 4. BLE 인솔 연동 (펌웨어 ↔ 앱)

펌웨어는 저장소 루트의 `insole_ble_app.ino`, 앱 쪽 수신은 `static/js/sensor-data.js` 의 `BluetoothDataSource` 입니다.
프로토콜과 현재 구현 상태는 **`docs/ble-protocol.md` §8** 을 보세요.

```
static/js/vendor/capacitor.js      @capacitor/core 런타임 (IIFE)          ┐ npm run vendor:ble 로 갱신
static/js/vendor/bluetooth-le.js   @capacitor-community/bluetooth-le (IIFE) ┘
static/js/sensor-data.js           MockDataSource / BluetoothDataSource / createDataSource()
```

- 안드로이드 앱에서는 네이티브 BLE 플러그인, 브라우저(Chrome, HTTPS)에서는 Web Bluetooth 로 같은 코드가 동작합니다.
- Device 탭 → **[새 장치 페어링]** 으로 `PostureIQ-R` 을 찾아 연결하면 데이터 소스가 자동으로 BLE 로 바뀌고,
  Live 탭이 실제 값으로 그려집니다. Mock 으로 되돌리려면 Device 탭의 데이터 소스에서 Mock 선택.
- 플러그인을 추가했으므로 **APK 는 반드시 `npm run apk` (cap sync 포함) 로 다시 빌드**해야 합니다.
  `android/app/src/main/AndroidManifest.xml` 에 BLE 권한이 추가되어 있습니다.

## 5. 다음 단계 (TODO)

- [x] 블루투스 BLE 데이터 인제스트 (오른발 1개, `docs/ble-protocol.md` §8)
  - [ ] 왼발 인솔 펌웨어 (`FOOT_SIDE='L'`, 이름 `PostureIQ-L`)
  - [ ] `CAL_FACTOR` / `LOAD_MAX_G` 실측 후 live.html 의 kg 환산 계수 맞추기
  - [ ] battery_level(0x2A19) 펌웨어 구현 → 앱은 이미 읽도록 되어 있음
- [ ] PostgreSQL 연동 (`spring-boot-starter-data-jpa`)
- [ ] 세션·샘플 도메인 (`com.postureiq.session.*`)
- [ ] 정적 HTML → Thymeleaf 템플릿 전환 (실 데이터 바인딩)
- [ ] Spring Security + 사용자 인증
- [ ] APK 서명 → Play Store / 외부 배포용 release APK
