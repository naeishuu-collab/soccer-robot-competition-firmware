# Arduino IDEセットアップと検証手順

## スケッチと統合コード

| 基板 | スケッチ |
|---|---|
| MAIN | `main/main.ino` |
| IR | `ir/ir.ino` |
| LINE | `line/line.ino` |
| MOTOR | `motor/motor.ino` |

4基板の各`.ino`に共通ロジックを直接統合しました。`common/src/*.ino`はホストテスト用の原本として残しています。各基板の処理は対応する `.ino` に統合されています。追加インクルードパスは不要です。`Arduino.h`、`Wire.h`、センサーライブラリのヘッダーは引き続き使います。

## Arduino-Pico環境

- Raspberry Pi Pico/RP2040/RP2350 core 6.3.2
- MAIN/IR: Generic RP2350、variant RP2350A、2MB no filesystem
- LINE: Generic RP2350、variant RP2350B、2MB no filesystem
- MOTOR: ZIP親基板のBOMに合わせRP2350Aを選択。実機MCU刻印は書き込み前に確認する
- Adafruit BNO055 1.6.4、Adafruit Unified Sensor 1.1.15、Adafruit BusIO 1.17.4
- Upload methodは各自作基板のBOOT/Flash設計を確認してから決定

4スケッチのコンパイル（Arduino CLIがPATHにあるPowerShellから直接実行。`.ps1`スクリプトの実行ポリシー変更は不要）:

```powershell
arduino-cli compile --fqbn rp2040:rp2040:generic_rp2350:variantchip=RP2350A,flash=2097152_0 --warnings all .\main
arduino-cli compile --fqbn rp2040:rp2040:generic_rp2350:variantchip=RP2350A,flash=2097152_0 --warnings all .\ir
arduino-cli compile --fqbn rp2040:rp2040:generic_rp2350:variantchip=RP2350B,flash=2097152_0 --warnings all .\line
arduino-cli compile --fqbn rp2040:rp2040:generic_rp2350:variantchip=RP2350A,flash=2097152_0 --warnings all .\motor
```

この設定でコードのコンパイル/リンクは確認できますが、実基板のブート、Flash、書込み経路は証明しません。

## 通信設定

MAIN/IR/LINEは115200bpsのUART0（Arduino-Pico Serial1）候補を設定しています。MOTORは基準コードのTX=GP0、RX=GP1、EN/DE=GP2、115200bpsを設定し、ピン選択後にSerial1を初期化します。MAIN→MOTORの指令と4輪回転はユーザー報告で成功しています。現物導通・DE極性・終端の測定、およびIR/LINE各基板との通信は未確認です。詳細は[GPIO_WIRING_JA.md](GPIO_WIRING_JA.md)を参照してください。

MAINソフトはIRへ1 byte `0x01`を要求し、IRは`AA DIR_H DIR_L REL_H REL_L 55`を返します。方向は0〜359度、RELは未校正の相対検出値で、距離ではありません。LINEはCRC16付き`A5 5A version type length sequence`照会に対し、生レベル24bit・角度cdeg・検出フラグ・個数を返します。LINEの極性・機体角度とBNO055の実測値は未確認です。

MOTOR指令は12 byte `AA 04 [int16 little-endian ×4] XOR 55`（-255..255、256=ブレーキ）で、既存のCRC16/連番形式とは別です。ユーザー報告ではMAIN→RS485→MOTOR→4輪の実機回転が成功しています。MOTOR側は100ms期限切れで指令を停止します。MAIN基板に物理E-stopはありません。ユーザー報告ではMOTOR基板の物理スイッチがモーター電源を切りますが、この作業では通信断停止やスイッチ遮断を実機確認していません。

起動時はLINE照会なしの `AUTO IR` 待機です。USBコマンドは `AUTO`／`AUTO FLOAT`、`AUTO FLOOR`、`AUTO IR` → `ARM IR`、`S`、`M <v0> <v1> <v2> <v3>` です。自動ARMは無効です。通常AUTO系の構成ゲートは現設定で閉じています。

`AUTO IR` はIRでボールを追跡し、校正済みOpenMVで壁・白線回避とゴール方位を判断します。走行には両方の新鮮な応答とARMが必要です。各輪PWM上限は180です。MAINの非ゼロ指令の最大車輪値は最低150です。MOTORの実出力では各輪の非ゼロ値を120〜200に制限するため、車輪間の比率は変化する場合があります。MOTOR側には100 msの通信ウォッチドッグがあります。

このモードは候補の車輪方向を使い、LINE境界検出はありません。USBの`S`はソフト停止で、物理E-stopではありません。MAINにE-stopスイッチはないため、車輪を浮かせ、MOTOR基板の電源スイッチをすぐ操作できる状態で短く確認してください。車輪の合成方向、MAIN↔IRとMAIN→MOTORのRS485実通信が確認できるまで床上で走らせないでください。

MOTOR親基板はZIP設計上RP2354Aで、UART/PWMネットはユーザー確認済みGPIO表と一致します。一方、ZIPの子基板はIR4427系/MOSFET回路で、実機として報告されたTB67H450×4とは異なります。MAINに物理E-stopはなく、MOTORの安全GPIO入力も未設定です。MOTOR基板の電源スイッチはユーザー報告で確認していますが、この作業では遮断を実測していません。現物ハーネス・終端・センサー基板との通信も未確認です。床上AUTOの設定ゲートは閉じたままにしてください。

## BNO055

MAINが`Wire.setSDA(4)`、`Wire.setSCL(5)`、`Wire.begin()`を実行し、BNO055を0x28で初期化します。アドレスstrapが0x29の個体では`BoardConfig.ino`と一致するよう設定を変えてください。BNO055はMAINのI2C専用です。Adafruit_BNO055 1.6.4のArduino-Pico実装に合わせI2C速度は50kHzです。I2C実波形、アドレス、プルアップ、校正、基板上の取り付け方向は実機で確認します。

MAINはIMUPLUSモードを使い、未確認のオフセット値はkBnoOffsetsVerified=falseの間は適用せず、外部水晶設定後500msは待機状態で他の処理を継続します。安定待ち後の最初のheadingを相対角の基準にします。30度を超える方位変化は単発ノイズを保留し、同じ値が続く場合や同方向の回転が続く場合は最大30度ずつ通します。heading自体は有限値かつSYS校正3の場合だけ有効です。オフセット値や方位基準がセンサー個体・設置方向に合うことは未検証です。

## 起動と診断

MAINは自動ARMしません。新鮮なIRボール検出中に `AUTO IR`、`ARM IR` の順に送ってください。車輪を浮かせ、独立したモーター電源遮断をすぐ操作できる状態で起動してください。各基板のUSB Serialは115200です。

- MAIN: IR応答、LINE CRC/鮮度/連番、UART診断、BNO状態、MOTOR形式指令の送信数、キッカー状態を表示
- IR: 相対信号、方向コヒーレンス、サンプル鮮度、RS485診断を表示
- LINE: 生H/L 24bit、極性/角度状態、CRC/照会期限を表示
- MOTOR: RS485の受信バイト数、形式/値域/チェックサム/部分フレームエラー、通信期限、出力許可状態を表示。成功ログはRS485実機動作の検証を意味しません

LINE極性と角度が未設定なら生レベルだけを記録します。MOTORソフトは20kHz PWMを使い、4番目のIN1/IN2をchase.zipに合わせてGPIO8/9に設定します。通常AUTOは設定ゲートで止まり、IR試験モードだけはPWM 180を上限として候補車輪設定を使用します。IR試験はLINE境界検出とMAIN基板の物理E-stopを使わないため、床上では動かさないでください。MOTOR起動時は4チャンネルを停止レベルLOW/LOWへ設定します。

## キッカーとOpenMV

OpenMV UART3 TXをMAIN GP3（Serial2 RX）へ接続し、GNDを共通にします。速度は9600 bpsです。MAIN GP2はTXです。本番カメラは `0x20` で境界回避・ゴール整列を送り、MAINはIRの近接・正面判定と合わせてキックを判断します。MAIN GPIO11の物理出力は車輪浮上・キッカー出力無効の状態から確認します。詳しくは [全方位カメラの校正](OPENMV_CALIBRATION_JA.md) を参照してください。

## 実機確認の順序

1. 各基板のRP2354型番、Flash、Boot/USB書込み経路を確認します。
2. +BATT、車輪、キッカー、ドリブラーを切り離し、USBのみで個別に起動ログを見ます。電池電圧は未確定のため値を仮定しません。
3. 電源断状態でMAIN/IR/LINEのA/B/GND、各トランシーバー電源、DEと/RE、終端抵抗を実測します。
4. MOTOR親基板のRP2354A/PWM/RS485ネットを現物導通と照合し、TB67H450入力および安全遮断の実装を個別に確認します。ZIP子基板のIR4427系出力論理はTB67H450へ流用しません。
5. BNO055 I2C、IRセンサー、LINE入力段を個別に試験します。NJL7502Lの入力回路と閾値は回路図で確認してください。
6. MOTOR基板の物理電源スイッチでモーター電源が切れることと、ドライバー論理表を個別に確認します。

ホスト試験とコンパイルは書込み、実RS485、実I2C、センサー電圧、PWM、走行や物理スイッチの遮断確認を意味しません。AUTO IRは車輪を浮かせ、MOTOR基板の電源スイッチをすぐ操作できる場合だけ短く試します。LINE境界検出とMAINの物理E-stopはありません。床上AUTOは車輪方向、通信、センサー、停止経路を確認するまでゲートを閉じます。

