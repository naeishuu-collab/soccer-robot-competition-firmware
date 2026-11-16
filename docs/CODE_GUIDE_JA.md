# コードガイド

## 4つのスケッチ

| スケッチ | 役割 |
|---|---|
| `main/main.ino` | IR/LINEを順次照会し、故障LINE番号を除外してMOTOR形式指令を生成。MOTORの物理経路と出力は未確認・無効。BNO055方位を戦術制御へ渡す |
| `ir/ir.ino` | 12入力から方向/相対信号を計算し、コヒーレンスと鮮度を診断。要求`0x01`に6byte形式で応答 |
| `line/line.ino` | GPIO2〜25の24センサーを読み、故障6番号を検出処理から除外。MAINからCRC付き照会を受けた場合だけ状態応答を返す |
| `motor/motor.ino` | CRC/連番付き指令処理、安全ゲート、TB67H450の入出力変換と反転待ちを実装。報告済みGPIOは記録するが出力ゲートは閉じたまま |

## 共有バスの動き

MAINの通信ソフトは`Rs485MasterArbiter`でIRまたはLINEのどちらか1要求だけを保持します。実ハーネス、とくにMOTOR新版のバス接続は未確認です。IR応答か期限切れの後にLINEを照会し、その完了後にMOTOR指令を送ります。LINEは周期送信せず、MOTOR形式指令にACKはありません。製造新版への物理到達や複数基板のバス成立は未確認です。MOTOR指令は40ms間隔を目安に送信し、センサー要求応答を優先します。

IRの1byte要求`0x01`は既存仕様のままです。共有バス上ではLINE/MOTORのCRCフレーム内部にも数値`0x01`が現れるため、IR側に境界デマルチプレクサを追加しました。CRCフレーム全体を読み飛ばし、独立した1byte要求だけに応答します。IR処理は方向ベクトルのコヒーレンス、相対信号履歴/予測、サンプル鮮度の診断を追加しています。6byte応答形式は維持します。

## 共通通信コード

- `common/src/Rs485.h`: UARTコールバック、非ブロッキング1byte受信、送信中DE制御、`flush()`完了待ち、送受信切替待ち、送信診断、直近受信/送信後の静穏時間、共有バス要求権を管理します。
- `common/src/RobotProtocol.h`: CRC16、フレーム符号化/部分受信/タイムアウト、連番、IR応答検査、共有バス上でIR要求を見分ける処理を持ちます。
- `common/src/RobotControl.h`: LINE状態符号化、LINE要求形式、4輪運動学、TB67H450真理値表/ duty制限/方向反転待ち、HeadingController、戦術状態機械を持ちます。
- `common/src/RobotDevices.h`: BNO055接続監視とキッカー/ドリブラーの安全ゲートを持ちます。
- `common/src/BoardConfig.h`: MAIN/IR/LINEの候補ピン、LINE故障マスク、報告済みMOTOR PWM GPIO、未確認の通信/安全ピンと出力ゲートをまとめます。

MAIN/IR/LINEは設計候補としてArduino-Pico Serial1（UART0）を設定しています。MOTOR新版のUART/DEはUNKNOWNで未割当てのため、MOTORスケッチはUARTを初期化しません。PIO UART、Serial2は使用しません。

## BNO055

MAINのGPIO4をSDA、GPIO5をSCLとして`Wire`を初期化し、BNO055を0x28で使います。IMUPLUSと外部水晶を設定し、旧オフセット値はkBnoOffsetsVerified=falseのため適用せず、500msの安定待ち後に最初の方位を基準にします。30度を超える変化は単発なら保留し、連続した高速回転または同じ角度の再確認があれば最大30度ずつ追従します。SYS校正値が3で、有限な方位が得られた時だけHeadingControllerへ有効値を渡します。目標方位は未設定です。I2C実機、取り付け軸、オフセットは未検証です。

## 安全設定

- `kDiagnosticMode=true`、`kCompetitionArmed=false`です。
- ユーザー報告のPWM 20kHz、範囲255、Duty上限128、4組GPIO（13/12、11/10、9/8、7/6）を記録します。MOTOR側RS485端子・車輪極性・安全入力は不明なのでGPIO数0、TX/RX/DE=-1、ハードウェア設定falseで出力しません。
- LINE故障1,5,8,9,11,21番をマスクし、18個を使用します。極性と24個の角度は未確定です。センサー方向は推測しません。
- IRの`DIST_H/DIST_L`は相対信号強度です。物理距離cmとして扱いません。
- MAINのMOTOR形式指令にACKはありません。MAIN-MOTOR RS485試験成功はユーザー報告ですが、MOTOR通信ピンはUNKNOWNでUARTを開始せず、実機停止を確認していません。

## テスト

`tests/run_tests.ps1`は共通ロジックのホスト単体/統合テストを実行します。`tests/compile_arduino.ps1`は4スケッチをArduino-Pico Generic RP2350 A/Bでコンパイルします。ホストのモック試験はRS485実波形や実I2C通信を証明しません。




