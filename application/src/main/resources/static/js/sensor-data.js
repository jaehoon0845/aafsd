/* ──────────────────────────────────────────────────────────────
 * PostureIQ — Sensor data source abstraction
 *
 * 두 가지 구현체를 같은 인터페이스로 제공:
 *   - MockDataSource:      가짜 데이터로 UI 동작 검증 / 시연
 *   - BluetoothDataSource: 실제 BLE 인솔 연결 (insole_ble_app.ino 펌웨어)
 *
 * 페이지에서 사용:
 *   const source = PostureIQ.createDataSource();   // 저장된 설정(mock/ble)에 따라 생성
 *   source.on('reading', (r) => updateUI(r));
 *   source.start();
 *
 * Reading 이벤트 페이로드 (docs/ble-protocol.md 참고):
 *   {
 *     timestamp: ms,
 *     source: 'mock' | 'ble',
 *     feet: { L: bool, R: bool },                  // 실제로 연결된 발
 *     left:  { heel, arch, ball, toe, battery },   // raw 0~1023, battery: 0~100 | null
 *     right: { heel, arch, ball, toe, battery },
 *     balance: { leftPct, rightPct },              // 0~100 (한쪽만 연결되면 50/50)
 *   }
 *
 * BluetoothDataSource 이벤트:
 *   'connected'      { foot:'L'|'R', deviceId, name }
 *   'disconnected'   { foot, deviceId } (foot 없으면 전체 중지)
 *   'battery'        { foot, level }
 *   'needs-pairing'  { reason }        — 저장된 장치가 없거나 브라우저에서 사용자 제스처 필요
 *   'scanning'       { active: bool }
 *   'error'          { message, error }
 *
 * 의존성(HTML에서 sensor-data.js 보다 먼저 로드):
 *   /js/vendor/capacitor.js      (@capacitor/core 런타임, 전역 capacitorExports)
 *   /js/vendor/bluetooth-le.js   (@capacitor-community/bluetooth-le, 전역 capacitorCommunityBluetoothLe)
 *   -> 안드로이드 앱에서는 네이티브 플러그인, 브라우저(Chrome)에서는 Web Bluetooth 로 동작.
 * ────────────────────────────────────────────────────────────── */

(function () {
  'use strict';

  // ── 프로토콜 상수 (docs/ble-protocol.md, insole_ble_app.ino 와 일치) ──
  const BLE = {
    NAME_PREFIX:     'PostureIQ-',
    SERVICE:         '0000a0f9-0000-1000-8000-00805f9b34fb',
    PRESSURE:        '0000a0f1-0000-1000-8000-00805f9b34fb',  // Notify, 8 bytes LE uint16 x4
    DEVICE_INFO:     '0000a0f2-0000-1000-8000-00805f9b34fb',  // Read, 16 bytes
    CALIBRATE:       '0000a0f3-0000-1000-8000-00805f9b34fb',  // Write, 1 byte
    SAMPLING_RATE:   '0000a0f4-0000-1000-8000-00805f9b34fb',  // Read/Write, 1 byte (펌웨어 미구현)
    BATTERY_SERVICE: '0000180f-0000-1000-8000-00805f9b34fb',  // 표준 Battery Service
    BATTERY_LEVEL:   '00002a19-0000-1000-8000-00805f9b34fb',  // Read/Notify, 1 byte (펌웨어 미구현)
    PACKET_BYTES: 8,
  };

  const STORAGE = {
    SOURCE: 'postureiq.source',                  // 'mock' | 'ble'
    DEVICE: (foot) => `postureiq.ble.device.${foot}`,  // JSON { deviceId, name }
  };

  const RECONNECT_DELAY_MS = 3000;
  const CONNECT_TIMEOUT_MS = 10000;
  const SCAN_TIMEOUT_MS = 8000;

  // ── 공통 이벤트 디스패처 ────────────────────────────────
  class EventEmitter {
    constructor() { this._listeners = {}; }
    on(event, cb)  { (this._listeners[event] ||= []).push(cb); return this; }
    off(event, cb) { this._listeners[event] = (this._listeners[event] || []).filter(f => f !== cb); }
    emit(event, payload) { (this._listeners[event] || []).forEach(cb => { try { cb(payload); } catch (e) { console.error(e); } }); }
  }

  // ── 헬퍼: 정규화 / 합산 ────────────────────────────────
  function sumFoot(f) { return f.heel + f.arch + f.ball + f.toe; }
  function computeBalance(left, right) {
    const lSum = sumFoot(left);
    const rSum = sumFoot(right);
    const total = lSum + rSum;
    if (total === 0) return { leftPct: 50, rightPct: 50 };
    return {
      leftPct:  (lSum / total) * 100,
      rightPct: (rSum / total) * 100,
    };
  }
  function emptyFoot(battery) {
    return { heel: 0, arch: 0, ball: 0, toe: 0, battery: battery ?? null };
  }

  /** 8바이트 LE uint16 x4 패킷 -> { heel, arch, ball, toe } (펌웨어 memcpy 순서와 동일) */
  function parsePressurePacket(dataView) {
    if (!dataView || dataView.byteLength < BLE.PACKET_BYTES) return null;
    return {
      heel: dataView.getUint16(0, true),
      arch: dataView.getUint16(2, true),
      ball: dataView.getUint16(4, true),
      toe:  dataView.getUint16(6, true),
    };
  }

  /** 광고 이름 -> 'L' | 'R' | null */
  function footFromName(name) {
    if (!name || !name.startsWith(BLE.NAME_PREFIX)) return null;
    const suffix = name.slice(BLE.NAME_PREFIX.length).toUpperCase();
    return suffix === 'L' || suffix === 'R' ? suffix : null;
  }

  // ── 저장된 설정 ───────────────────────────────────────
  function safeStorage(fn, fallback) {
    try { return fn(window.localStorage); } catch (_) { return fallback; }
  }
  function getSourcePreference() {
    return safeStorage(s => s.getItem(STORAGE.SOURCE), null) === 'ble' ? 'ble' : 'mock';
  }
  function setSourcePreference(value) {
    safeStorage(s => s.setItem(STORAGE.SOURCE, value === 'ble' ? 'ble' : 'mock'));
  }
  function getPairedDevice(foot) {
    return safeStorage(s => { const v = s.getItem(STORAGE.DEVICE(foot)); return v ? JSON.parse(v) : null; }, null);
  }
  function setPairedDevice(foot, device) {
    safeStorage(s => device
      ? s.setItem(STORAGE.DEVICE(foot), JSON.stringify({ deviceId: device.deviceId, name: device.name || '' }))
      : s.removeItem(STORAGE.DEVICE(foot)));
  }
  function getPairedDevices() {
    return { L: getPairedDevice('L'), R: getPairedDevice('R') };
  }
  function forgetPairedDevices() {
    setPairedDevice('L', null);
    setPairedDevice('R', null);
  }

  // ── BLE 런타임 접근 ───────────────────────────────────
  function bleRuntime() {
    const plugin = window.capacitorCommunityBluetoothLe;
    const cap = window.Capacitor;
    if (!plugin || !plugin.BleClient || !cap) return null;
    return { BleClient: plugin.BleClient, Capacitor: cap };
  }
  function isNativePlatform() {
    const cap = window.Capacitor;
    return !!(cap && typeof cap.isNativePlatform === 'function' && cap.isNativePlatform());
  }
  /** 이 환경에서 BLE 사용 가능 여부 (앱: 항상, 브라우저: Web Bluetooth 지원 시) */
  function isBluetoothAvailable() {
    if (!bleRuntime()) return false;
    if (isNativePlatform()) return true;
    return typeof navigator !== 'undefined' && !!navigator.bluetooth;
  }

  // ── MockDataSource ────────────────────────────────────
  // 시뮬레이션:
  //   - 자연스러운 압력 베이스라인 (뒤꿈치 > 볼 > 아치 > 발가락)
  //   - 노이즈 ±5%
  //   - 주기적 불균형 에피소드 (15초간 우측 쏠림, 15초 회복)
  class MockDataSource extends EventEmitter {
    constructor(opts = {}) {
      super();
      this.intervalMs = opts.intervalMs || 100;      // 10 Hz default
      this.imbalanceCycleMs = opts.imbalanceCycleMs || 30000;  // 30s 주기
      this._timer = null;
      this._startTime = 0;
      this._battery = { L: 78, R: 52 };
    }

    get kind() { return 'mock'; }

    start() {
      if (this._timer) return;
      this._startTime = Date.now();
      this.emit('connected', { foot: 'L', deviceId: 'mock-L', name: 'PostureIQ-L (mock)' });
      this.emit('connected', { foot: 'R', deviceId: 'mock-R', name: 'PostureIQ-R (mock)' });
      this._timer = setInterval(() => this._tick(), this.intervalMs);
    }

    stop() {
      if (this._timer) clearInterval(this._timer);
      this._timer = null;
      this.emit('disconnected', {});
    }

    calibrate() { return Promise.resolve(); }

    _tick() {
      const t = (Date.now() - this._startTime) / 1000;  // seconds since start
      // 불균형 에피소드: 0~15초 균형, 15~30초 우측 쏠림
      const cyclePhase = (t * 1000) % this.imbalanceCycleMs;
      const imbalanceFactor = cyclePhase > this.imbalanceCycleMs / 2 ? 0.18 : 0;  // 우측 18% 증가

      const baseline = { heel: 380, arch: 180, ball: 280, toe: 120 };
      const noise = () => (Math.random() - 0.5) * 60;

      const left = {
        heel: Math.round(baseline.heel + noise()),
        arch: Math.round(baseline.arch + noise()),
        ball: Math.round(baseline.ball + noise()),
        toe:  Math.round(baseline.toe + noise()),
        battery: this._battery.L,
      };
      const right = {
        heel: Math.round(baseline.heel * (1 + imbalanceFactor) + noise()),
        arch: Math.round(baseline.arch * (1 + imbalanceFactor) + noise()),
        ball: Math.round(baseline.ball * (1 + imbalanceFactor) + noise()),
        toe:  Math.round(baseline.toe  * (1 + imbalanceFactor) + noise()),
        battery: this._battery.R,
      };

      // 배터리 천천히 감소 (시연용)
      if (Math.random() < 0.0005) this._battery.L = Math.max(0, this._battery.L - 1);
      if (Math.random() < 0.0008) this._battery.R = Math.max(0, this._battery.R - 1);

      const balance = computeBalance(left, right);
      this.emit('reading', {
        timestamp: Date.now(),
        source: 'mock',
        feet: { L: true, R: true },
        left, right, balance,
      });
    }
  }

  // ── BluetoothDataSource ───────────────────────────────
  //
  // 동작 흐름:
  //   start()  -> BleClient.initialize -> localStorage 에 저장된 L/R 장치에 재연결
  //               (저장된 장치가 없으면 'needs-pairing' 이벤트 -> 페이지가 pair() 호출)
  //   pair()   -> 앱: 8초 스캔해서 PostureIQ-L / PostureIQ-R 를 찾아 전부 연결
  //               브라우저: 장치 선택 창(사용자 클릭 필요)에서 하나 선택
  //   연결 후  -> pressure_data Notify 구독, 패킷마다 'reading' 발생
  //   끊기면   -> 3초 후 자동 재연결 (stop() 전까지)
  //
  class BluetoothDataSource extends EventEmitter {
    constructor(opts = {}) {
      super();
      this.connectTimeoutMs = opts.connectTimeoutMs || CONNECT_TIMEOUT_MS;
      this.reconnectDelayMs = opts.reconnectDelayMs || RECONNECT_DELAY_MS;
      this.scanTimeoutMs = opts.scanTimeoutMs || SCAN_TIMEOUT_MS;
      this._running = false;
      this._initialized = false;
      this._scanning = false;
      this.lastScan = [];            // 마지막 스캔에서 본 모든 BLE 기기 (진단용)
      this._feet = { L: this._newFootState(), R: this._newFootState() };
    }

    get kind() { return 'ble'; }

    static isAvailable() { return isBluetoothAvailable(); }

    _newFootState() {
      return {
        deviceId: null, name: null,
        connected: false, connecting: false,
        battery: null,
        last: null,          // 마지막 { heel, arch, ball, toe }
        lastAt: 0,
        reconnectTimer: null,
      };
    }

    /** 현재 연결 상태 스냅샷 (UI 표시용) */
    getStatus() {
      const snap = {};
      for (const foot of ['L', 'R']) {
        const f = this._feet[foot];
        snap[foot] = { deviceId: f.deviceId, name: f.name, connected: f.connected, connecting: f.connecting, battery: f.battery, lastAt: f.lastAt };
      }
      return snap;
    }

    isConnected(foot) {
      if (foot) return this._feet[foot].connected;
      return this._feet.L.connected || this._feet.R.connected;
    }

    async _ensureInitialized() {
      const rt = bleRuntime();
      if (!rt) throw new Error('BLE 런타임이 로드되지 않았습니다 (/js/vendor/capacitor.js, bluetooth-le.js 확인)');
      if (!isBluetoothAvailable()) throw new Error('이 브라우저는 Web Bluetooth 를 지원하지 않습니다. 안드로이드 앱 또는 Chrome 을 사용하세요.');
      if (this._initialized) return rt.BleClient;
      await rt.BleClient.initialize({ androidNeverForLocation: true });
      this._initialized = true;
      if (isNativePlatform()) {
        try {
          if (!(await rt.BleClient.isEnabled())) await rt.BleClient.requestEnable();
        } catch (e) {
          // 사용자가 블루투스 켜기를 거부하면 connect 에서 다시 실패함
          console.warn('[BLE] requestEnable', e);
        }
      }
      return rt.BleClient;
    }

    /** 저장된 장치에 연결 시작. 저장된 장치가 없으면 'needs-pairing' 발생. */
    async start() {
      if (this._running) return;
      this._running = true;
      let BleClient;
      try {
        BleClient = await this._ensureInitialized();
      } catch (error) {
        this._running = false;
        this.emit('error', { message: error.message, error });
        return;
      }

      const paired = getPairedDevices();
      const targets = ['L', 'R'].filter(foot => paired[foot] && paired[foot].deviceId);
      if (targets.length === 0) {
        this.emit('needs-pairing', { reason: 'no-paired-device' });
        return;
      }

      if (!isNativePlatform()) {
        // Web Bluetooth: 이전에 허용한 장치는 getDevices() 로 다시 얻을 수 있음
        // (chrome://flags/#enable-web-bluetooth-new-permissions-backend 필요할 수 있음).
        // 안 되면 사용자 클릭으로 pair() 를 다시 호출해야 한다.
        let known = [];
        try { known = await BleClient.getDevices(targets.map(f => paired[f].deviceId)); } catch (_) { known = []; }
        const knownIds = new Set(known.map(d => d.deviceId));
        const missing = targets.filter(f => !knownIds.has(paired[f].deviceId));
        if (missing.length === targets.length) {
          this.emit('needs-pairing', { reason: 'web-user-gesture-required' });
          return;
        }
      } else {
        // 네이티브: 플러그인 내부 장치 목록에 등록 (스캔 없이 MAC 으로 바로 연결 가능)
        try { await BleClient.getDevices(targets.map(f => paired[f].deviceId)); } catch (e) { console.warn('[BLE] getDevices', e); }
      }

      await Promise.all(targets.map(foot => this._connectFoot(foot, paired[foot])));
    }

    /**
     * 새 인솔 검색 + 연결 (사용자 버튼 클릭에서 호출).
     * 앱: 스캔으로 발견되는 PostureIQ-L/R 모두 연결. 브라우저: 선택 창에서 하나 선택.
     * @returns {Promise<Array<{foot, deviceId, name}>>} 연결된 장치 목록
     */
    async pair() {
      const BleClient = await this._ensureInitialized();
      this._running = true;
      const found = new Map();  // foot -> device

      // 필터 없이 전부 받아서 앱이 직접 판정한다.
      //   안드로이드 ScanFilter 에 맡기면(이름이든 UUID든) 걸러진 이유를 알 수 없고
      //   화면에는 '못 찾음'으로만 보인다. 특히 이름 필터는 OS가 캐시한
      //   BluetoothDevice.getName() 을 보기 때문에 처음 보는 기기는 null 이라 항상 탈락한다.
      //   그래서 raw 결과를 모두 모은 뒤 이름 또는 광고된 서비스 UUID 로 고르고,
      //   실패하면 무엇이 잡혔는지 사용자에게 보여준다(진단용).
      const seen = new Map();         // deviceId -> { deviceId, name, uuids, rssi }
      const candidates = new Map();   // deviceId -> { deviceId, name }
      const target = BLE.SERVICE.toLowerCase();
      const isOurs = (e) =>
        footFromName(e.name) !== null ||
        e.uuids.some(u => {
          const s = String(u).toLowerCase();
          // 128비트 전체형, 16비트 축약형(a0f9) 양쪽 모두 허용
          return s === target || s === 'a0f9' || s === '0xa0f9';
        });

      if (isNativePlatform()) {
        this._setScanning(true);
        try {
          await BleClient.requestLEScan({ allowDuplicates: false }, (result) => {
            const entry = {
              deviceId: result.device.deviceId,
              name: result.localName || result.device.name || '',
              uuids: result.uuids || [],
              rssi: result.rssi,
            };
            seen.set(entry.deviceId, entry);
            if (isOurs(entry)) candidates.set(entry.deviceId, { deviceId: entry.deviceId, name: entry.name });
          });
          // 두 발 다 찾으면 조기 종료, 아니면 타임아웃까지
          const deadline = Date.now() + this.scanTimeoutMs;
          while (Date.now() < deadline && candidates.size < 2) {
            await new Promise(r => setTimeout(r, 200));
          }
        } finally {
          try { await BleClient.stopLEScan(); } catch (_) { /* ignore */ }
          this._setScanning(false);
        }
      } else {
        // 브라우저는 OS 선택 창에서 사용자가 직접 고른다 (필터 없이 전부 표시)
        const device = await BleClient.requestDevice({
          optionalServices: [BLE.SERVICE, BLE.BATTERY_SERVICE],
        });
        candidates.set(device.deviceId, { deviceId: device.deviceId, name: device.name || '' });
      }

      // 진단 정보: 이번 스캔에서 본 모든 기기 (신호 센 순)
      this.lastScan = Array.from(seen.values()).sort((a, b) => (b.rssi ?? -999) - (a.rssi ?? -999));
      this.emit('scan-report', { seen: this.lastScan, matched: candidates.size });

      // 발(L/R) 판정: 광고 이름 -> device_info(0xA0F2) -> 기본값 'R'
      for (const device of candidates.values()) {
        const foot = await this._resolveFoot(device);
        if (!found.has(foot)) found.set(foot, device);
      }

      if (found.size === 0) {
        const scanned = this.lastScan || [];
        let msg;
        if (scanned.length === 0) {
          msg = 'BLE 기기가 하나도 잡히지 않았습니다. 폰 블루투스가 켜져 있는지, 앱에 "근처 기기" 권한을 허용했는지 확인하세요.';
        } else {
          const list = scanned.slice(0, 8)
            .map(d => `${d.name || '(이름없음)'}${d.rssi != null ? ' ' + d.rssi + 'dBm' : ''}`)
            .join(' / ');
          msg = `주변 기기 ${scanned.length}개는 잡혔지만 PostureIQ 인솔은 없습니다. 인솔 전원과 펌웨어를 확인하세요.\n잡힌 기기: ${list}`;
        }
        const err = new Error(msg);
        this.emit('error', { message: err.message, error: err });
        throw err;
      }

      const connected = [];
      for (const [foot, device] of found) {
        setPairedDevice(foot, device);
        const ok = await this._connectFoot(foot, device);
        if (ok) connected.push({ foot, ...device });
      }
      if (connected.length > 0) setSourcePreference('ble');
      return connected;
    }

    /**
     * 어느 발인지 판정: 광고 이름 -> device_info(0xA0F2) -> 기본값 'R'.
     * 이름을 못 읽는 경우가 있어(캐시 미존재, 31바이트 광고 제한 등) 연결해서 직접 물어본다.
     * 성공하면 연결을 유지해 두고, 뒤따르는 _connectFoot 가 "Already connected" 로 재사용한다.
     */
    async _resolveFoot(device) {
      const byName = footFromName(device.name);
      if (byName) return byName;
      const BleClient = bleRuntime().BleClient;
      try {
        await BleClient.connect(device.deviceId, undefined, { timeout: this.connectTimeoutMs });
        const dv = await BleClient.read(device.deviceId, BLE.SERVICE, BLE.DEVICE_INFO);
        const side = (dv && dv.byteLength >= 1) ? String.fromCharCode(dv.getUint8(0)).toUpperCase() : '';
        if (side === 'L' || side === 'R') return side;
      } catch (error) {
        console.warn('[BLE] device_info 로 발 판정 실패, 기본값 R 사용', error);
      }
      return 'R';   // 현재 펌웨어는 오른발 1개
    }

    async _connectFoot(foot, device) {
      const state = this._feet[foot];
      if (state.connecting) return false;
      state.connecting = true;
      state.deviceId = device.deviceId;
      state.name = device.name || `${BLE.NAME_PREFIX}${foot}`;
      const BleClient = bleRuntime().BleClient;
      try {
        await BleClient.connect(device.deviceId, (id) => this._onDisconnected(foot, id), { timeout: this.connectTimeoutMs });
        await BleClient.startNotifications(
          device.deviceId, BLE.SERVICE, BLE.PRESSURE,
          (value) => this._onPacket(foot, value)
        );
        state.connected = true;
        state.connecting = false;
        this.emit('connected', { foot, deviceId: device.deviceId, name: state.name });
        this._readBattery(foot);   // 펌웨어에 없으면 조용히 실패
        return true;
      } catch (error) {
        state.connecting = false;
        state.connected = false;
        console.warn(`[BLE] ${foot} connect 실패`, error);
        this.emit('error', { foot, message: `${state.name} 연결 실패: ${error.message || error}`, error });
        this._scheduleReconnect(foot);
        return false;
      }
    }

    async _readBattery(foot) {
      const state = this._feet[foot];
      try {
        const dv = await bleRuntime().BleClient.read(state.deviceId, BLE.BATTERY_SERVICE, BLE.BATTERY_LEVEL);
        if (dv && dv.byteLength >= 1) {
          state.battery = dv.getUint8(0);
          this.emit('battery', { foot, level: state.battery });
        }
      } catch (_) {
        state.battery = null;   // 현재 펌웨어는 battery_level 미구현
      }
    }

    _onDisconnected(foot) {
      const state = this._feet[foot];
      const wasConnected = state.connected;
      state.connected = false;
      state.connecting = false;
      if (wasConnected) this.emit('disconnected', { foot, deviceId: state.deviceId });
      this._scheduleReconnect(foot);
    }

    _scheduleReconnect(foot) {
      const state = this._feet[foot];
      if (!this._running || !state.deviceId || state.reconnectTimer) return;
      state.reconnectTimer = setTimeout(async () => {
        state.reconnectTimer = null;
        if (!this._running || state.connected) return;
        await this._connectFoot(foot, { deviceId: state.deviceId, name: state.name });
      }, this.reconnectDelayMs);
    }

    _onPacket(foot, dataView) {
      const sensors = parsePressurePacket(dataView);
      if (!sensors) {
        console.warn(`[BLE] ${foot} 패킷 길이 이상:`, dataView && dataView.byteLength);
        return;
      }
      const state = this._feet[foot];
      state.last = sensors;
      state.lastAt = Date.now();
      this._emitReading();
    }

    _emitReading() {
      const L = this._feet.L, R = this._feet.R;
      const feet = { L: L.connected && !!L.last, R: R.connected && !!R.last };
      const left  = feet.L ? { ...L.last, battery: L.battery } : emptyFoot(L.battery);
      const right = feet.R ? { ...R.last, battery: R.battery } : emptyFoot(R.battery);
      // 한쪽만 연결된 상태에서 좌우 밸런스는 의미가 없으므로 50/50 으로 둔다.
      const balance = (feet.L && feet.R) ? computeBalance(left, right) : { leftPct: 50, rightPct: 50 };
      this.emit('reading', {
        timestamp: Date.now(),
        source: 'ble',
        feet, left, right, balance,
      });
    }

    /**
     * 캘리브레이션 명령 전송 (calibrate 0xA0F3).
     *   0x00 취소 / 0x01 영점(tare) / 0x02 최대값
     * 연결된 모든 발에 전송. 펌웨어는 0x01 에서 로드셀 tare + 자기장 baseline 재측정.
     */
    async calibrate(mode = 0x01) {
      const BleClient = await this._ensureInitialized();
      const targets = ['L', 'R'].filter(f => this._feet[f].connected);
      if (targets.length === 0) throw new Error('연결된 인솔이 없습니다.');
      const value = new DataView(Uint8Array.from([mode & 0xff]).buffer);
      await Promise.all(targets.map(f =>
        BleClient.write(this._feet[f].deviceId, BLE.SERVICE, BLE.CALIBRATE, value)
      ));
      return targets;
    }

    /** device_info (0xA0F2) 읽기: { footSide, firmware:'1.0.0', macLower:[..] } (펌웨어 지원 시) */
    async readDeviceInfo(foot) {
      const state = this._feet[foot];
      if (!state.connected) throw new Error(`${foot} 인솔이 연결되어 있지 않습니다.`);
      const dv = await bleRuntime().BleClient.read(state.deviceId, BLE.SERVICE, BLE.DEVICE_INFO);
      if (!dv || dv.byteLength < 8) throw new Error('device_info 길이 이상');
      return {
        footSide: String.fromCharCode(dv.getUint8(0)),
        firmware: `${dv.getUint8(1)}.${dv.getUint8(2)}.${dv.getUint8(3)}`,
        macLower: [4, 5, 6, 7].map(i => dv.getUint8(i)),
      };
    }

    async stop() {
      this._running = false;
      const rt = bleRuntime();
      for (const foot of ['L', 'R']) {
        const state = this._feet[foot];
        if (state.reconnectTimer) { clearTimeout(state.reconnectTimer); state.reconnectTimer = null; }
        if (state.connected && rt) {
          try { await rt.BleClient.stopNotifications(state.deviceId, BLE.SERVICE, BLE.PRESSURE); } catch (_) { /* ignore */ }
          try { await rt.BleClient.disconnect(state.deviceId); } catch (_) { /* ignore */ }
        }
        state.connected = false;
        state.connecting = false;
        state.last = null;
      }
      if (this._scanning && rt) { try { await rt.BleClient.stopLEScan(); } catch (_) { /* ignore */ } }
      this._setScanning(false);
      this.emit('disconnected', {});
    }

    _setScanning(active) {
      if (this._scanning === active) return;
      this._scanning = active;
      this.emit('scanning', { active });
    }
  }

  /**
   * 저장된 설정에 따라 데이터 소스 생성.
   *   - 'ble' 이고 BLE 사용 가능 -> BluetoothDataSource
   *   - 그 외 -> MockDataSource
   * @param {{ force?: 'mock'|'ble', mock?: object, ble?: object }} opts
   */
  function createDataSource(opts = {}) {
    const wanted = opts.force || getSourcePreference();
    if (wanted === 'ble' && isBluetoothAvailable()) return new BluetoothDataSource(opts.ble || {});
    return new MockDataSource(opts.mock || {});
  }

  // 전역으로 노출 (HTML 페이지에서 사용)
  window.PostureIQ = window.PostureIQ || {};
  Object.assign(window.PostureIQ, {
    BLE,
    MockDataSource,
    BluetoothDataSource,
    createDataSource,
    computeBalance,
    sumFoot,
    parsePressurePacket,
    footFromName,
    getSourcePreference,
    setSourcePreference,
    getPairedDevices,
    forgetPairedDevices,
    isBluetoothAvailable,
    isNativePlatform,
  });
})();
