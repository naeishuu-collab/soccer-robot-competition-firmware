# 診断ガイド（Phase 1 実装範囲）

この手順は現在の` soccer_camera `系に対するものです。USBシリアルはMAIN/IR/MOTORを115200 baudで開きます。**実機へ書き込む前に、各基板へ書き込む対象が` soccer_camera/{main,ir,motor}`であることを確認してください。** `soccer_ir_main_line`、`chase_*`、`legacy_before_chase`、`line`は別実装・比較用です。LINE基板は競技接続しません。

## 現在使える診断操作

| 基板 | 操作 | 実際の動作 |
|---|---|---|
| MAIN | `i`（1文字） | build識別子、行動/停止理由/未成立条件mask、IR/カメラ鮮度、ゴール/白線、方向・出力・旋回、RS-485統計とイベント履歴を表示。ZIP追尾経路ではPD項とPWM段階比較も表示 |
| MAIN | `d`または`s` | 停止を保持 |
| MAIN | `g` | 停止保持を解除 |
| MAIN | `v` | viewer用VIZ行を切り替え |
| IR | `i` | 最後に受けた12入力の計数値とactive mask、角度・強度、サンプル時間/周期/角度処理時間、要求/応答/不正要求数を表示 |
| IR | `v` | filtered要求時の連続診断表示を切り替え。raw要求の通常構成では使わない |
| MOTOR | `i` + Enter | RX/CRC/BAD/timeout/age/target/current/appliedを表示。appliedはHブリッジへのPWM指令で、車輪回転センサーの測定値ではない |
| MOTOR | `diag` + Enter | 既存の車輪浮上用モーター診断に入る。詳細は`docs/UPGRADE_2026-11-26_JA.md`のモーター測定節 |
| OpenMV | `camera_calibration.py`の`DEBUG_STATS = True` | OpenMV USBコンソールへ2秒周期でFPS、capture/line、段階別時間、白線/ゴール状態、校正識別子を表示 |

MAIN/IRの`i`は行末不要です。MOTORの`i`は行末が必要です。USB未接続時に診断出力は保存されず、フラッシュへも書き込みません。`tests/compile_arduino.ps1`経由のArduinoビルドはHEAD SHAを埋め込みます。手動ビルドやOpenMVの表示が`git=unembedded`なら、SHA未設定です。OpenMVは`camera_calibration.py`で識別子を更新します。

通常時の例（センサー配置や時刻で値は変わります）:

```text
[MAIN] fw=0.1.0 git=unembedded uptime_ms=... mode=DEGRADED state=SAFE_STOP reason=camera link hold=0
[IR] valid=1 angle=32 strength=... age_ms=...
[MOTOR_TX] direction=0 power=0 turn=0 avoid=0
[LINKS] ir_poll=... ir_reply=... ir_timeout=... camera_crc_err=...
```

`reason=camera link`、`IR link lost`、`no ball detected`、`no safe direction`などが停止要因です。MAIN診断は拡張`ROBOT_UPGRADE=1`経路の詳しいスナップショットです。

## 症状から調べる

### ロボットが動かない／競技経路だけ動かない

1. 車輪を浮かせ、MAINの`i`で`hold`、`reason`、IR/カメラ`age_ms`を見ます。
2. `camera link`ならOpenMV USBコンソールのFPS、`ready`、`setup`、UART統計を確認します。`DEBUG_STATS=True`が必要です。
3. `IR link lost`ならIRの`i`で`req`と`tx`を比較し、MOTORの`i`でRX/CRC/ageを確認します。
4. MAINの`power`が0ならMAINの理由が先です。MAINが非0を送るのにMOTORの`APPLIED`が0なら、MOTORの受信・timeout・モードを確認します。
5. 現行初期設定は拡張経路（MAIN/MOTOR `ROBOT_UPGRADE=1`、OpenMV `PROTOCOL=2`）です。構成不一致をMAINが自動検出する機能はありません。3つの設定を合わせ、実機未検証構成は床上で使わないでください。

### 1輪だけ動かない／回転方向がおかしい

1. MOTORのUSBで`diag`を入力し、`docs/UPGRADE_2026-11-26_JA.md`にある既存の`p M1 + 160`等で1輪ずつ試します。
2. 表示された`APPLIED`はPWM指令です。実際に回ったかは目視・記録し、ログ値を回転測定と取り違えないでください。
3. 車輪番号・GPIO・極性・モーター配線を照合します。コードのGPIO/順序を根拠なく変えません。診断終了後は新しいMAIN指令を待つため、必要ならMAINを停止して再開します。

### ボールを追尾しない／方向が逆／追尾が振動する

1. IRの`i`で12入力値と`angle`を確認し、ボールを機体の前後左右へ置いて方位を記録します。
2. MAINの`i`でIR角度・鮮度・強度と送信方向を確認します。IRのセンサー順や角度補正を確認するまで変更しないでください。
3. `soccer_camera/tests/test_zip_chase_pd.cpp`はZIP基準PDの10,000時系列入力比較です。これはセンサー極性や実機応答を検証しません。
4. 現行通常経路はZIP直接PWMの`chaseLoop()`ではなく拡張`upgradeLoop()`です。MAINの`i`表示で`PD active=0`なら拡張経路です。拡張制御とZIP PDは同じ方式として扱わないでください。

### 白線付近で止まる／白線を越える

1. OpenMVで`DEBUG_STATS=True`にし、`ready`、mask、escape、line、処理時間とミラー校正を確認します。
2. MAINの`i`でカメラage、ready、slow、line count、maskと停止理由を見ます。`mask=0000`や期限切れは走行許可が出ない状態です。
3. 白線方向、停止距離、ミラー死角を車輪を浮かせた観察の後で記録します。`GROUND_POINTS`、色閾値、ミラー、露出を未測定のまま変更しないでください。

### ゴールを認識しない／黄色だけ認識しない

1. OpenMVの2秒診断行で`Y=bearing/confidence`と`B=bearing/confidence`、設定した`TARGET_GOAL`を確認します。
2. 両ゴールが出ない場合はカメラ準備・ミラー・ROIと設定識別子を確認します。片方だけの場合はその色閾値と実際の照明を比較します。
3. PC合成画像テストはOpenMV実機のLAB値やFPSの証明になりません。校正変更は実機画像を記録してから行います。

### FPSが低い／RS-485またはUARTが不安定

1. OpenMVの`fps`と`capture_ms`を分けて確認します。45ms露出だけでもフレーム周期を制限します。`stats.report()`は段階時間のavg/max/p95を表示します。
2. MAINの`LINKS`でIR要求/応答/timeout、RTT、カメラCRCエラー、最大受信間隔を見る。MOTORの`MD`行ではRX、CRC、BAD、timeout、ageを確認します。
3. 半二重RS-485はMAINからのポーリングとIR応答の順で確認します。通信断を診断するために配線やGPIOを変更しないでください。

### 回り込みが動かない

MAINの`state`と`reason`、カメラの黄/青bearing・confidence・鮮度、mask、IR角度を同時に記録します。ゴール情報欠落や安全maskで停止する場合があります。回り込みのUSB個別モード切替コマンドは現時点で実装されていません。

## 現時点で未実装の診断

このガイドでは存在しないシリアルコマンドを案内しません。独立テストモード選択、基板間プロトコル互換性の自動照合、MAINが受け取るMOTOR状態返信、全停止条件の同時ビットマスク、IR/OpenMVを含む全基板共通イベント履歴、診断ログのCSV/JSON Lines切替はまだありません。競技実機検証も未実施です。
