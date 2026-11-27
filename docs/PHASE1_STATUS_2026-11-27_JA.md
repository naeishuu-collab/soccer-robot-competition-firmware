# Phase 1 診断基盤 状態記録（2026-11-27）

## 実装済み

- `soccer_camera/libraries/RobotDiagnostics/src/Diagnostics.h`: 共通バージョン/ビルド識別子、停止理由enum、固定長イベントリングを追加。履歴はRAMのみで、容量超過時は古い記録を上書きする。
- MAIN: `i`で拡張経路の停止理由・未成立条件mask・現在/直前状態・IR/カメラage・ゴール/白線・安全mask・送信方向/出力/旋回・IR polling/応答/timeout・RTT・カメラparser errors・制御処理時間と直近イベントを表示。状態/停止理由、センサー有効性、大きな出力変化を16件記録。ZIP PD経路の診断ビルドではP/D、計算PWM、安全後PWM、送信PWMを表示する。標準拡張経路ではPD非使用と明記。
- IR: `i`で最後のサンプル12個とactive mask、ボール方位/強度/valid、サンプル時間・周期・角度処理時間、要求・応答・不正要求数を表示。既存raw検出とfiltered処理は変更していない。
- MOTOR: `i`+Enterで既存通信統計と目標/現在/適用PWM、timeout/reverse wait/ramp状態を表示。停止遷移を16件記録。適用PWMはHブリッジ向け指令値であり、車輪回転検出値ではない。
- OpenMV: 既存`DEBUG_STATS=True`経路にfirmware識別子、カメラ/ミラー準備状態、校正モード、ターゲット色、露出/gain、撮像失敗数、機能状態を追加表示。OSErrorによる撮像失敗は回数を保持して安全側で次フレームを待つ。既存FPSと段階別avg/max/p95、白線mask/escape、黄/青ゴール診断を再利用。
- `docs/DEBUG_GUIDE.md`: 実在する操作だけを記載した初動手順を追加。

## 検証

| 内容 | 結果 |
|---|---|
| Arduino CLI compile（MAIN RP2354B / IR RP2354A / LINE / MOTOR RP2354A） | VERIFIED: 4対象すべてコンパイル成功。アップロードなし。LINEは既存スクリプトがコンパイルするだけで、競技構成への接続・書き込みはしていない |
| ホストC++ suite | VERIFIED:全18実行ファイル成功。ZIP PDは10,000比較一致、core 65ケース/5,165 checks、integration 14シナリオ/107 checks、loop/compatibility群失敗0 |
| Python unit tests | VERIFIED: 106 tests成功 |
| Python syntax (`compileall`) | VERIFIED |
| 実機/USBログ/PWM/RS-485/OpenMV FPS | PENDING_HARDWARE: 今回は未実施 |

## 残作業と制限

Phase 1は**PARTIAL**。診断行の全項目を各基板へそろえた状態ではない。OpenMVのUART送信失敗数、IRの詳細通信エラー分類、MAINでのMOTOR適用状態受信は未実装。PDのP/D表示はZIP追尾経路に限り、標準拡張経路はPDを使用しない。MOTOR→MAINの返信はバス競合と互換性を検証していないため追加していない。CSV/JSON Linesは未実装。

Arduinoコンパイルスクリプトは実行時のHEAD SHAを一時ヘッダー経由で各スケッチへ埋め込む。スクリプトを使わない手動ビルドでは`unembedded`と表示する。OpenMVは`camera_calibration.py`内の識別子を手動更新する必要がある。診断自体が実機の安全性や旋回性能を保証するわけではない。

## 判定

- Phase 0: VERIFIED（ソース監査と復元SHA記録。実機状態ではない）
- Phase 1: PARTIAL（ソフトウェアビルド・ホスト検証済み。診断項目の全網羅と実機確認は未完）
- Phase 2以降: NOT_EXECUTED。Phase 1完了とは扱わず、戦術機能の追加へ進まない。
- 大会投入: **NO-GO**。現在標準の拡張経路はリポジトリ上でも実機未検証。診断変更も未実機。
