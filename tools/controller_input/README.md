# コントローラー入力表示（Mac）

```sh
python3 tools/controller_input.py
open -a 'Google Chrome' http://127.0.0.1:8766/
```

画面を前面にし「入力確認を開始する」をクリック後、コントローラーのボタンやスティックを操作する。
Gamepad APIの実入力を表示。左右スティックは標準のAxis0/1・2/3。未定義マッピングでは全軸/ボタン番号を優先して確認する。
USB機器名が見えるだけではGamepad APIの入力確認にはならない。ブラウザーの保護コンテキスト・機器モード・対応状況により未検出の場合がある。
キーボードモード確認欄は、この画面が前面のときのキーコードをメモリー内に表示するだけ。実キーボードとの区別はできない。

127.0.0.1:8766でのみ待ち受け、POSTを拒否。HTML以外の任意ファイルを配信しない。
ブラウザーのconnect-srcはnone。ESP32へのHTTP/シリアル/CAN接続や、コントローラーの振動出力は一切実装していない。
入力はファイル保存・サーバー送信しない。

2026-10-26：JS構文確認、Python構文確認、Chromeでの初期画面描画を確認。
ユーザーが操作した実際のボタン/スティック変化は未確認。
スクリーンショット：diagnostics/controller_input_screen.png。

参考： https://developer.mozilla.org/en-US/docs/Web/API/Gamepad_API/Using_the_Gamepad_API

## Mac直読みフォールバック（2026-10-26追加）

Bluetooth Switch互換（VID 057e / PID 2009、Game Pad、Transport Bluetooth）として接続後、IOHIDManagerの読み取りは成功し、0x30/49byteの入力レポートと実際の軸値変化を確認した。
一方、CLIのGameController APIは0台、ユーザーのChrome画面ではGamepad API表示が変化しなかった。特定のAPI経路が動かない理由そのものは未確定。

`controller_input.py`は`hid_reader.m`を必要時に/private/tmpへコンパイルし、対象Bluetoothゲームパッドだけを非排他的に読み取る。設定変更・振動・初期化用出力レポートは送らない。
`/api/hid-state`を通じて同じMacの画面へ表示する。ブラウザーGamepad APIよりこの受信を優先。500msレポートが途絶すれば古い入力を無効化する。
`hid_decode.py`で、観測した0x30/49byteだけを復号する。ボタン名はNintendo配列。軸は12bit・仮中心2048で正規化し、機器への校正要求は送っていない。今後ロボット操作に使う前には、中立点と軸方向の実測確認が必要。

検証：`python3 tests/test_controller_hid.py`で中立・全ボタン・軸端・不正サイズ/別report拒否を確認。
実レポートから左スティック値の変化を確認。更新後のAPIでopenResult=0、reports=385、ageMs=22、全軸中立を確認。
これも入力表示だけで、ESP32へは接続していない。

入力形式の参考：ChromiumのControllerData/UnpackShorts実装
https://chromium.googlesource.com/chromium/src/+/HEAD/device/gamepad/nintendo_controller.cc


## Wi-Fi操縦モード（追加）

`python3 tools/controller_input.py --drive`でhttp://127.0.0.1:8766/driveを開く。通常起動の入力表示専用モードと区別する。
操縦モードだけがESP32の固定Wi-Fiアドレス192.168.4.1へアクセスする。USBシリアルは開かない。
初期状態は無効、明示的な有効化と中央スティックからのL保持が必要。
左スティック全方向移動＋右X旋回、各目標±300 RPM・電流±1000、詳細と未検証範囲はルートのOMNI_CONTROL.mdを参照。
