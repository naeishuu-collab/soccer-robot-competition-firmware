# コードガイド

## 4つのスケッチ

| スケッチ | 役割 |
|---|---|
| `main/main.ino` | IR/LINEを順次照会し、4輪指令を12 byte形式でMOTORへ送信。物理RS485と出力は未確認・無効。BNO055方位を戦術制御へ渡す |
| `ir/ir.ino` | 12入力から方向/相対信号を計算し、コヒーレンスと鮮度を診断。要求`0x01`に6byte形式で応答 |
| `line/line.ino` | GPIO2〜25の24センサーを読み、故障6番号を検出処理から除外。MAINからCRC付き照会を受けた場合だけ状態応答を返す |
| `motor/motor.ino` | 12 byte指令の値域/チェックサム/終端/期限を検査し、安全ゲート、TB67H450の入出力変換と反転待ちを実装。出力ゲートは閉じたまま |

## 共有バスの動き

MAINの通信ソフトは`Rs485MasterArbiter`でIRまたはLINEのどちらか1要求だけを保持します。IR応答か期限切れの後にLINEを照会し、その完了後にMOTOR指令を送ります。MOTOR指令は40ms間隔を目安に送る一方向の12 byteフレームで、ACKはありません。物理ハーネス、終端、MOTORの受信、共有バス成立は未確認です。

IRの1byte要求`0x01`は既存仕様のままです。共有バスではLINEのCRCフレームとMOTORの `AA 04` フレーム内部に数値`0x01`が現れるため、IR側の境界デマルチプレクサは両フレームを読み飛ばし、独立した1byte要求だけに応答します。IR処理は方向ベクトルのコヒーレンス、相対信号履歴/予測、サンプル鮮度の診断を追加しています。6byte応答形式は維持します。

## 共通通信コード

各基板スケッチ内に下記5モジュールを直接統合しています。`common/src`はホストテスト用の原本で、`tests/check_gpio.py`が各スケッチ内のコピーとの一致を検査します。共通処理を変更した場合は4スケッチへ同じ変更を反映してください。

- `common/src/Rs485.ino`: UARTコールバック、非ブロッキング1byte受信、送信中DE制御、`flush()`完了待ち、送受信切替待ち、送信診断、直近受信/送信後の静穏時間、共有バス要求権を管理します。
- `common/src/RobotProtocol.ino`: LINE用CRC16フレーム、MOTOR用12 byteフレーム、各部分受信/値域/タイムアウト、LINE用連番、IR応答検査、共有バス上でIR要求を見分ける処理を持ちます。
- `common/src/RobotControl.ino`: LINE状態符号化、LINE要求形式、4輪運動学、TB67H450真理値表/ duty制限/方向反転待ち、HeadingController、戦術状態機械を持ちます。
- `common/src/RobotDevices.ino`: BNO055接続監視とキッカー/ドリブラーの安全ゲートを持ちます。
- `common/src/BoardConfig.ino`: MAIN/IR/LINEの候補ピン、LINE故障マスク、報告済みMOTOR PWM GPIO、未確認の通信/安全ピンと出力ゲートをまとめます。

MAIN/IR/LINEは設計候補としてArduino-Pico Serial1（UART0）を設定しています。MOTORは基準コードからGP0/GP1/GP2を設定し、115200bpsでSerial1を初期化します。実配線とDE動作は未確認です。PIO UART、Serial2は使用しません。

## BNO055

MAINのGPIO4をSDA、GPIO5をSCLとして`Wire`を初期化し、BNO055を0x28で使います。IMUPLUSと外部水晶を設定し、旧オフセット値はkBnoOffsetsVerified=falseのため適用せず、500msの安定待ち後に最初の方位を基準にします。30度を超える変化は単発なら保留し、連続した高速回転または同じ角度の再確認があれば最大30度ずつ追従します。SYS校正値が3で、有限な方位が得られた時だけHeadingControllerへ有効値を渡します。目標方位は未設定です。I2C実機、取り付け軸、オフセットは未検証です。

## 安全設定

- `kDiagnosticMode=true`、`kCompetitionArmed=false`です。
- ユーザー報告のPWM 20kHz、範囲255、Duty上限128、IN1={6,12,11,8}/IN2={7,13,10,9}を使用します。基準コードのRS485はGP0/1/2に設定しましたが実配線は未確認です。車輪極性・安全入力は未設定、ハードウェア出力ゲートはfalseです。
- LINE故障1,5,8,9,11,21番をマスクし、18個を使用します。極性と24個の角度は未確定です。センサー方向は推測しません。
- IRの`DIST_H/DIST_L`は相対信号強度です。物理距離cmとして扱いません。
- MAINのMOTOR指令にACKはありません。RS485実機通信の検証は未実施です。ユーザーが実機で確認したのはモーターの正転・逆転・停止・ブレーキです。

## テスト

`tests/run_tests.ps1`は共通ロジックのホスト単体/統合テストを実行します。`tests/compile_arduino.ps1`は4スケッチをArduino-Pico Generic RP2350 A/Bでコンパイルします。ホストのモック試験はRS485実波形や実I2C通信を証明しません。




