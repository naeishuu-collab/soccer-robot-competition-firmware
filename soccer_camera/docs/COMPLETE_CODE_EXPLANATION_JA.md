# サッカーロボット competition コード全体解説（最新版）

更新日: 2026-11-19
対象: 現在の `main` ブランチ（ローカルの最新変更を含む）。仕様・設定値はソースから読み取れる範囲を説明します。実機の配線、センサー較正、車輪方向、キッカー回路の動作を保証するものではありません。

> 古い説明（2026-11-15）は BNO オフセット未適用／300ms ウォッチドッグ有効／カメラ連携なし、など現在と差分がありました。本ファイルが最新です。短い早見版は [CODE_AT_A_GLANCE_JA.md](CODE_AT_A_GLANCE_JA.md) を参照してください。

---

## 1. 全体像

4 枚の基板で役割を分担します。MAIN だけが RS485 共有バスの親（マスター）です。

```text
   [OpenMV H7 Plus カメラ] ─UART2 9600 / 0x20─→ MAIN
   [BNO055 ジャイロ] ──I2C 0x28 / 50kHz──────────────→ MAIN
   [USB コマンド] ─────AUTO / ARM IR / S / M─→ MAIN
                                                 │
                              RS485 共有バス(115200)
                 照会 0x01 │ CRC照会 A5 5A │ 命令 AA…55
                           ▼              ▼           ▼
                          IR            LINE       MOTOR → 4輪
                      ボール方位      白線(故障中)   TB67H450×4
```

基本順番は **IR を聞く → LINE を聞く → MOTOR に命令**（約 40ms 周期）。`AUTO IR` では故障した LINE を飛ばして **IR → MOTOR** です。カメラは GPIO ではなく Serial2 で MAIN に接続します。

## 2. 各スケッチ

### `main/main.ino` — 統合判断

- USB コマンド（`AUTO` / `AUTO FLOAT` / `AUTO FLOOR` / `AUTO IR` / `ARM IR` / `S` / `M <v0..v3>`）を解釈します。**USB から 1 バイトでも来ると起動時の自動 ARM 待ちは解除**されます。
- バス調停（`Rs485MasterArbiter`）で IR／LINE の要求を 1 つだけ保持し、応答かタイムアウトで MOTOR 命令へ進みます。
- BNO055 を読み、相対方位の急変フィルタ（単発は保留、連続で最大 30° ずつ追従）を適用します。オフセットは実機動作テスト由来の値を適用します。
- `MainController::step()` は競技用 AUTO の状態機械です。現在は構成ゲートが閉じており `AUTO FLOAT` / `AUTO FLOOR` は起動できません。
- `AUTO IR` では新鮮なカメラとIRが必要です。両方が有効なら、壁・白線回避、IR追跡、ゴール整列を順に判断します。
- キックにはIRの近接・正面と、校正済みカメラのゴール整列がそろう必要があります。発射時は GPIO11 に30 msのパルスを出します。

### `ir/ir.ino` — ボール方位

- GPIO18〜29 の 12 センサーを 5ms ごとに読み、cos/sin テーブル（30° 刻み）から方向ベクトルを合成します。
- 16 サンプルの履歴、コヒーレンス、相対信号の重み付き平均、30ms 先の予測（最大 720°/s）を計算します。相対信号は cm ではありません。
- `0x01` の 1 バイト要求に `AA dirH dirL relH relL 55` の 6 バイトで応答します。共有バス上のフレーム内 `0x01` は `SharedBusIrRequestParser` が読み飛ばします。

### `line/line.ino` — 白線（故障中）

- GPIO2〜25 の 24 入力を 20ms ごとに読みます。故障 6ch（1, 5, 8, 9, 11, 21）をマスクし 18ch で運用します。
- 極性・実角度・デバウンスはいずれも未確定／無効です。方向が不明なときは推測せず停止側へ倒します。
- MAIN から CRC 付き要求が来た時だけ応答します。

### `motor/motor.ino` — 4 輪駆動

- 12 バイト命令 `AA 04 [v0低 v0高] … [v3低 v3高] XOR 55` の値域（-255..255、256=ブレーキ）・XOR・終端を検査します。不正なら即全輪安全停止します。
- 出力は IN1/IN2 の真理値表（正転／逆転／空転／ブレーキ）、20kHz PWM、範囲 255、200ms 反転待ち、200 超の Duty は 10ms ごとに 5 ずつランプです。
- **300ms ソフトウォッチドッグを再導入**しました（`kSoftwareWatchdogEnabled=true`）。MAIN の通信が途切れると自動停止します。`false` にすれば旧挙動（物理スイッチのみ）に戻せます。`S` と物理電源スイッチは常に有効です。

## 3. 共通モジュール（`common/src` → 4 スケッチに複製）

| モジュール | 役割 |
|---|---|
| `Rs485.ino` | 半二重送信、DE 制御、`flush()` 完了待ち、100µs 静穏、親局調停、診断 |
| `RobotProtocol.ino` | LINE の CRC フレーム、MOTOR の 12 バイト、各種パーサ、USB コマンド解釈 |
| `RobotControl.ino` | 線情報、4 輪運動学、TB67H450 変換、加減速／反転待ち、戦術状態機械、方位制御、`IrTestController` |
| `RobotDevices.ino` | BNO055 監視、キッカー（5 状態＋イベント履歴）、ドリブラーゲート |
| `BoardConfig.ino` | ピン割当、故障マスク、安全ゲート定数 |

4 スケッチの `BEGIN/END INTEGRATED COMMON CODE` 内は `common/src` とバイト単位で一致している必要があり、`tests/check_gpio.py` が検査します。変更時は 5 か所すべてに同じ変更を入れてください。

`IrTestController` は IR ロスト時に `disarm()` せず `searching()=true` として旋回指令（0.12）を出し、IR 復帰で自動追従に戻ります。

## 4. 通信プロトコル

| 種類 | 形式 | 検査 |
|---|---|---|
| IR 要求 / 応答 | `01` / `AA dirH dirL relH relL 55` | 形式のみ |
| LINE 要求 / 応答 | `A5 5A 01 03 00 seq CRC` / `A5 5A 01 01 len seq payload CRC` | CRC16 (0x1021)・連番 |
| MOTOR 命令 | `AA 04 v0L v0H … v3L v3H XOR 55` | 値域・XOR・終端（ACK なし） |
| OpenMV ナビ | `20 vx vy turn aimed mode flags xor` | XOR、200 msで失効 |


## 5. 全方位カメラ

ミラーのリング内でゴール色を検出し、機体の正面を基準に方位を求めます。放射状に緑床から白線・黒壁への変化を探し、実測した半径と床面距離の表で近接を判定します。攻撃色は各ハーフ前に設定します。

`0x20` の mode は 0: WAIT、1: AVOID、2: TRACK IR、3: ALIGN、4: AIMED です。flags の bit 5 は新しい形式、bit 0 は回避中を表します。aimed が立ってもMAINで新鮮なIRの近接・正面を確認するまでキックしません。送信周期の目標は50 msです。

[全方位カメラの校正](OPENMV_CALIBRATION_JA.md) に撮像・取付・通信の確認手順があります。

## 6. 安全設定

- `kCompetitionArmed=false`、`AUTO FLOAT` / `AUTO FLOOR` は起動不可。
- `AUTO IR` はIRまたはカメラ通信が失効すると停止します。
- 停止は `S`・物理電源スイッチ・300ms ウォッチドッグ。
- MAIN に物理 E-stop はありません。MOTOR の物理スイッチが最終手段です。
- 実機未検証: RS485 実配線、IR/LINE 実測、BNO 実動作、車輪の合成方向、キッカー高圧回路、カメラの色較正。

## 7. テスト・検査

| ファイル | 内容 |
|---|---|
| `tests/test_core.cpp` | 共通ロジックとOpenMV/IRの合流判定 |
| `tests/test_integration.cpp` | システム統合 **14 シナリオ / 107 チェック** |
| `tests/check_gpio.py` | GPIO プロファイル＋共通コード一致 **12 件** |
| `tests/test_openmv_logic.py` | ミラー校正・境界検出・ゴール整列・UART形式 |
| `tests/run_tests.ps1` | g++（なければ MSVC）で C++ テストをビルド・実行 |

ホスト上のモック試験であり、実波形・実 I2C・実カメラ画像を証明するものではありません。

## 8. 読み順

1. `README.md`
2. `CODE_AT_A_GLANCE_JA.md`（早見版）
3. `common/src/RobotProtocol.ino`（フレーム形式）
4. `main/main.ino` の `loop()`
5. `common/src/RobotControl.ino` の `MainController::step()` / `IrTestController`
6. `openmv/goal_wall_navigation.py` の `decide_navigation()` と `main/main.ino のOpenMV受信・判定処理`
7. `motor/motor.ino` の `motorOutput()` / `receiveCommands()`
