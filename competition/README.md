# RCJ Soccer Infrared

IRの検出角でZIP原本のPD追尾を行い、白線に近い場合は前後の安全な方向へ退避します。走行状態は `CHASE`、`ESCAPE`、`STOP` の3つです。カメラは白線・緑床・黄ゴール・青ゴールを認識します。ゴール方位は表示に使い、追尾用PWMはIRと白線安全情報で決まります。

| 基板 | 入口 | 役割 |
| --- | --- | --- |
| MAIN | [main/main.ino](main/main.ino) | IRポーリング、カメラ受信、追尾と安全判定、4輪指令、USB診断 |
| IR | [ir/ir.ino](ir/ir.ino) | 12入力を5msサンプリングし、合成ベクトルから方位・強度を返信 |
| MOTOR | [motor/motor.ino](motor/motor.ino) | CRC検証、逆転待機、ウォッチドッグ、TB67H450 PWM |
| OpenMV H7 Plus | [openmv/main.py](openmv/main.py) | 撮像、検出の実行、UART、計測、USB診断 |

- `main/chase_pd.h` はZIP PDと出力制限、`main/control.h` は状態と停止理由、`main/line_safety.h` は既定の経路マスクとPC比較候補、`main/diagnostics.h` はUSB表示とイベント。
- `motor/motor_output.h` は指令期限と200msの反転保護。
- `openmv/vision.py` はミラー座標、走査、白線退避マスク、ゴール選別、パケット生成。`openmv/preview.py` はIDE描画と処理時間集計、`openmv/camera_calibration.py` は基準校正値と比較設定。
- `libraries/RobotCommon` はUART／RS485定義、フレーム検証、送受信、USB出力。Arduino IDEでは同ライブラリを登録。
- `tests` はスケッチ・ZIPを動かすArduinoモック、経路比較、カメラ試験。`scripts` はArduino CLIビルドとPC試験。

```powershell
./competition/scripts/compile.ps1
# Visual Studio Developer PowerShell、または PATH 上に g++ / clang++
python -m pip install mpy-cross==1.28.0.post2
python competition/scripts/test.py --mpy-cross --zip 'C:/path/to/chase (4).zip'
```

[構成・通信・監査](docs/ARCHITECTURE.md) · [書き込み](docs/FLASH_GUIDE.md) · [動かないとき](docs/DEBUG_GUIDE.md) · [停止条件](docs/STOP_CONDITIONS.md) · [校正](docs/CALIBRATION_GUIDE.md) · [実機試験](docs/HARDWARE_TEST_GUIDE.md) · [復元](docs/ROLLBACK_GUIDE.md) · [ソフトウェア試験結果](docs/VERIFICATION.md)

**PENDING_HARDWARE**　組み合わせとしての追尾・走行・停止は実機未検証です。会場で走行する前に実機試験ガイドの項目を確認してください。
