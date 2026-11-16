# Arduino IDEセットアップと検証手順

## スケッチと共通ライブラリ

| 基板 | スケッチ |
|---|---|
| MAIN | `competition/main/main.ino` |
| IR | `competition/ir/ir.ino` |
| LINE | `competition/line/line.ino` |
| MOTOR | `competition/motor/motor.ino` |

共通コードは`competition/common`です。Arduino CLIではこのフォルダーを`--library`に指定します。IDEが認識しない場合はArduinoのライブラリフォルダーにコピーして使用します。

## Arduino-Pico環境

- Raspberry Pi Pico/RP2040/RP2350 core 6.3.2
- MAIN/IR: Generic RP2350、variant RP2350A、2MB no filesystem
- LINE: Generic RP2350、variant RP2350B、2MB no filesystem
- MOTOR: ビルドスクリプトはRP2350Bを仮選択。製造新版のMCU/packageはUNKNOWNで、実機向け確定設定ではない
- Adafruit BNO055 1.6.4、Adafruit Unified Sensor 1.1.15、Adafruit BusIO 1.17.4
- Upload methodは各自作基板のBOOT/Flash設計を確認してから決定

4スケッチのコンパイル（Arduino CLIがPATHにあるPowerShellから直接実行。`.ps1`スクリプトの実行ポリシー変更は不要）:

```powershell
arduino-cli compile --fqbn rp2040:rp2040:generic_rp2350:variantchip=RP2350A,flash=2097152_0 --library .\competition\common --warnings all .\competition\main
arduino-cli compile --fqbn rp2040:rp2040:generic_rp2350:variantchip=RP2350A,flash=2097152_0 --library .\competition\common --warnings all .\competition\ir
arduino-cli compile --fqbn rp2040:rp2040:generic_rp2350:variantchip=RP2350B,flash=2097152_0 --library .\competition\common --warnings all .\competition\line
arduino-cli compile --fqbn rp2040:rp2040:generic_rp2350:variantchip=RP2350B,flash=2097152_0 --library .\competition\common --warnings all .\competition\motor
```

この設定でコードのコンパイル/リンクは確認できますが、実基板のブート、Flash、書込み経路は証明しません。

## 通信設定

MAIN/IR/LINEは設計資料上の候補として115200bpsのUART0（Arduino-Pico Serial1）を設定しています。MOTOR新版の通信ピンとRS485経路はUNKNOWNで、MOTORスケッチはUARTを初期化しません。DEと/REの実配線、極性、ボーレートは各現物で確認が必要です。詳細は[GPIO_WIRING_JA.md](GPIO_WIRING_JA.md)を参照してください。

MAINソフトはIRへの0x01要求とLINEへのCRC16照会を順次行い、要求待ち中に別の要求を開始しません。合間にMOTOR形式のCRC16/連番付き指令も送信しますが、これはプロトコル生成の確認であり、製造新版への物理経路や受信を意味しません。診断モードではenable=falseです。MOTOR現行設定はUART未初期化のため、100ms停止処理はホストモデル/ファームウェア内部の論理であり実基板の動作確認ではありません。

MAIN/IR/LINEの候補配線は現存する設計資料を元にしていますが、現物の改版と導通は未確認です。MOTOR関連のZIP KiCadは旧版のLEGACY_REFERENCEであり、製造新版の端子配列・RS485接続はUNKNOWNです。ハーネス、終端、コネクター極性を電源OFFで照合するまで4基板をまとめて接続しないでください。

## BNO055

MAINが`Wire.setSDA(4)`、`Wire.setSCL(5)`、`Wire.begin()`を実行し、BNO055を0x28で初期化します。アドレスstrapが0x29の個体では`BoardConfig.h`と一致するよう設定を変えてください。BNO055はMAINのI2C専用です。Adafruit_BNO055 1.6.4のArduino-Pico実装に合わせI2C速度は50kHzです。I2C実波形、アドレス、プルアップ、校正、基板上の取り付け方向は実機で確認します。

MAINはIMUPLUSモードを使い、未確認のオフセット値はkBnoOffsetsVerified=falseの間は適用せず、外部水晶設定後500msは待機状態で他の処理を継続します。安定待ち後の最初のheadingを相対角の基準にします。30度を超える方位変化は単発ノイズを保留し、同じ値が続く場合や同方向の回転が続く場合は最大30度ずつ通します。heading自体は有限値かつSYS校正3の場合だけ有効です。オフセット値や方位基準がセンサー個体・設置方向に合うことは未検証です。

## 起動と診断

MAINの`kDiagnosticMode=true`、競技ARM=false、MOTOR設定falseが既定です。各基板のUSB Serialを115200で確認します。

- MAIN: IR応答、LINE CRC/鮮度/連番、UART診断、BNO状態、MOTOR形式指令の送信数、キッカーモック状態を表示
- IR: 相対信号、方向コヒーレンス、サンプル鮮度、RS485診断を表示
- LINE: 生H/L 24bit、極性/角度状態、CRC/照会期限を表示
- MOTOR: ピン未確認のためUARTが開始せず、現物ログは得られません

LINE極性と角度が未設定なら生レベルだけを記録します。MOTORソフトは提供された試験値20kHz/範囲255/Duty上限128を保持しますが、GPIO割当てとPWM許可マスクは空なので全指令の出力は無効です。

## 実機確認の順序

1. 各基板のRP2354型番、Flash、Boot/USB書込み経路を確認します。
2. +BATT、車輪、キッカー、ドリブラーを切り離し、USBのみで個別に起動ログを見ます。電池電圧は未確定のため値を仮定しません。
3. 電源断状態でMAIN/IR/LINEのA/B/GND、各トランシーバー電源、DEと/RE、終端抵抗を実測します。
4. MOTOR新版の承認済み回路図と端子表を入手し、旧KiCadの端子名を使わず、現物リビジョンと電源OFF導通を照合します。
5. BNO055 I2C、IRセンサー、LINE入力段を個別に試験します。NJL7502Lの入力回路と閾値は回路図で確認してください。
6. 停止回路と外部電源遮断、ドライバー論理表を確認するまで車輪を駆動しません。

ホスト試験とコンパイルは書込み、実RS485、実I2C、センサー電圧、PWM、走行の合格を意味しません。現行MOTOR新版の回路資料がそろうまでMOTORへの接続・通電・駆動は禁止です。

