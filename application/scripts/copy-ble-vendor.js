// node_modules 의 Capacitor 런타임 + BLE 플러그인 브라우저 번들(IIFE)을
// static/js/vendor/ 로 복사한다. (npm run vendor:ble)
//
// 이 앱은 번들러 없이 <script> 태그로 JS를 로드하므로, 플러그인의 ESM 대신
// dist/plugin.js (IIFE, 전역 capacitorCommunityBluetoothLe) 를 그대로 서빙한다.
// 플러그인 버전을 올리면 이 스크립트를 다시 실행해서 vendor 파일을 갱신할 것.
const fs = require('fs');
const path = require('path');

const root = path.resolve(__dirname, '..');
const outDir = path.join(root, 'src', 'main', 'resources', 'static', 'js', 'vendor');

const files = [
  ['@capacitor/core/dist/capacitor.js', 'capacitor.js'],
  ['@capacitor-community/bluetooth-le/dist/plugin.js', 'bluetooth-le.js'],
];

fs.mkdirSync(outDir, { recursive: true });
for (const [src, dst] of files) {
  const from = path.join(root, 'node_modules', src);
  const to = path.join(outDir, dst);
  fs.copyFileSync(from, to);
  console.log(`copied ${src} -> static/js/vendor/${dst}`);
}
