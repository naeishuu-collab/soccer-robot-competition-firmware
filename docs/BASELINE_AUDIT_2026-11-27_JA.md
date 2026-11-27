# Phase 0 基準監査（2026-11-27）

## 復元点と検証状態

- upstream: `https://github.com/naeishuu-collab/soccer-robot-competition-firmware`
- 監査対象: `origin/main` = `44cd80b3fd2a5137721e3f97bdd7603d21474557`
- 作業ブランチ: `codex/diagnostics-phase1`（基準コミットから作成）
- 作業開始前に既存の他worktreeを確認。未コミット変更があるチェックアウトは変更していない。
- ZIP由来PDの復元・比較基準: `soccer_camera/main/zip_chase_pd.h` と `tests/test_zip_chase_pd.cpp`
- `origin/main` とこのブランチの共通基準SHAが完全な復元点。基準コードへ戻すには別worktreeで `git switch --detach 44cd80b3fd2a5137721e3f97bdd7603d21474557` を実行する。

**ハードウェア状態:** このリポジトリの文書はZIP由来の追尾など一部を過去に実機確認したと記録しているが、現在のmainにある拡張MAIN/MOTOR/OpenMV経路は実機未検証と明記されている。本監査でも実機確認はしていない。stable認定はしない。

## ソース構成と実行経路

| 部分 | 実装位置 | 起動時/呼び出し経路 |
|---|---|---|
| MAIN | `soccer_camera/main/main.ino`, `main_logic.h`, `zip_chase_pd.h` | `setup()` UART/GPIO初期化 → `loop()` がUSB停止・モード分岐 → `upgradeLoop()`, `zipLoop()`, `chaseLoop()` または従来経路 → RS-485送信 |
| IR | `soccer_camera/ir/ir.ino`, `ir_logic.h` | `loop()` がMAIN要求を解析 → `sampleAndReply()`（互換IR）または `sampleFiltered()` → 同じ半二重バスへ応答 |
| MOTOR | `soccer_camera/motor/motor.ino`, `motor_logic.h` | `loop()` → USB診断/冷却/`runModel()`または旧駆動 → `applyMotor()` → TB67H450 PWM |
| OpenMV | `soccer_camera/openmv/competition_vision.py` と `camera_calibration.py` | キャリブレーション値を読み、白線・ゴールを処理し、結果をMAINへUART送信。カメラは最終走行判断をしない |
| 共通・旧経路 | `soccer_camera/common/src/*`, `soccer_ir_main_line/*`, `soccer_camera/chase_*`, `legacy_before_chase/*` | 共通RS-485、安全制御や旧スケッチ・比較用のコード。ディレクトリ名だけでは実際に書き込む対象を選べない |

標準MAIN設定は `ROBOT_UPGRADE=1`、`ROBOT_ZIP_TRACK=0`、`ROBOT_ZIP_COMPAT=0`。よって `loop()` は拡張 `upgradeLoop()` へ分岐する。ZIP互換直接PWM追尾はコードとテストに残るが標準起動経路ではない。MOTORも`ROBOT_UPGRADE=1`、OpenMV校正は`PROTOCOL=2`。これらの組み合わせはリポジトリ文書で実機未検証とされている。

## 現行プロトコル

- MAIN→IR: 115200 baud RS-485、要求 `A5 5A 01 FE`（raw）または `A5 5A 02 FD`（filtered）。IRは要求後に1回だけ応答するため同一バス上の常時送信局ではない。
- IR raw応答: `AA, angle(16-bit BE), detectCount(16-bit BE), 55`。filtered応答は`ir_logic.h`の9-byte形式（開始`AB`、CRC-8）。
- MAIN→MOTOR: 115200 baud RS-485、12-byte `A5`開始・CRC-7。type 3は方向/出力/turn、type 4は4輪符号付きPWM（オフセット210）。
- OpenMV→MAIN: 校正`PROTOCOL=2`で115200 baud、main parserはversion 3拡張パケットを扱う。旧8-byteパケットも別経路で存在する。
- MOTOR→MAINの状態返信はない。MAIN表示だけでMOTORの実際の適用PWMやタイムアウト停止を判断できない。

## PWMと校正の基準

- `ZipChasePD`: P=`2.0 * error`、D=`0.2 * angular_rate`、基準175、各出力の非ゼロ絶対値140〜210。直接PWMは`chaseLoop()`がtype 4で送る。テストはZIP式を独立転記して10,000入力を比較する。
- 現行MOTOR: `PWM_MAX=210`、`OUTPUT_MIN_DUTY=140`、`MIN_RUNNING_DUTY=160`。拡張wheel modelの車輪テーブルは暫定値と記載されている。
- 拡張MAIN: `MOVE_POWER=160`、`AVOID_POWER=160`、`MAX_POWER=200`。さらに`ROBOT_TEST_MAX_PWM=210`がZIP経路の直接出力に適用される。
- よってZIP PDの基準は保存されている一方、通常起動経路は要求された175/140/210の基準PDそのものではない。既実機確認GPIO・車輪順・極性はこのPhaseでは変更しない。
- OpenMVの露出45ms、gain 6dB、白色バランス固定値、LAB色範囲、ミラー設定、床距離、ゴール色は校正ファイルを基準として保持。数値の正しさは現物カメラで確認していない。

## 既存診断とギャップ

- MAINには`d`/`s`停止、`g`再開、`v`viewer、周期的に縮約されたIR/カメラ/drive表示と状態理由がある。MAINの機能フラグ、理由コード、イベントリング履歴、PD P/D内訳、計算前後PWMの統一表示はない。
- IRには`v`でセンサー別raw値とfiltered角度/strength/confidence/faultsを表示する簡易診断がある。要求数、応答数、通信エラー、計測時間の集計は不足。
- MOTORは周期的なRX/CRC/BAD/timeout/age/direction/power/target/currentを表示し、`diag`のUSB個別試験を持つ。モーター適用状態や停止理由がMAINへ返らず、フレーム総数と不正種別の網羅性も不足。
- OpenMVは`DEBUG_STATS`、`VISION.status()`、各処理段のタイミング保持を持つが、実行ループからの出力・撮像時間・UART所要時間など一連の診断表示は限定的。
- `BNO_ENABLED`既定値は0。BNO読取経路は任意だが、全モードで未接続の独立性が保証された構成仕様は不足。LINE基板ソースは残り、OpenMV白線経路との混同を招く。
- 同じ機能を持つ旧コード/比較用スケッチが複数あり、どのファイルを各基板に書き込むかがREADMEだけでは十分明瞭でない。

## 既存テスト

2026-11-27に `soccer_camera/tests/run_tests.ps1` を実行し、終了コード0。ZIP PD 10,000比較、統合・loop互換、通信/安全/センサー/オムニ制御を含むホストC++群を実行。主要なログでは`test_core.cpp`が65ケース・5,165 checks、`test_integration.cpp`が14シナリオ・107 checksで失敗0。複数の既存変換警告（C4244、MSVC `sscanf`警告等）が出た。

同スクリプトは作業木内の`tests/results_last_run.txt`を書き換えるため、監査後にこの生成物だけ基準版へ戻した。ホストテストは実機のGPIO/PWM、RS-485バス、OpenMVのFPSを証明しない。

## 2026年規則の確認

公式[RoboCupJunior Soccer Rules 2026](https://robocup-junior.github.io/soccer-rules/master/rules.html)を確認。国際大会版ではInfraredロボットの寸法22 cm、高さ22 cm、重量上限1.5 kg、ボール保持域1.5 cm（6.2.1）、InfraredリーグのIR発光部品禁止（6.2.2）、2026年IRボール直径42 mm（3.8）が示されている。地域大会・エントリーリーグは主催者による差があるため、大会投入前に適用規則を確認する。規則適合性の実機検査は未実施。

## 次段階の判断

Phase 0監査と基準保存は完了。Phase 1では互換するRS-485フレームを変更せず、各基板のローカルUSB診断、停止理由の識別、低負荷イベント履歴を追加する。PD/車輪制御・OpenMV校正・GPIO・通信ピンはこの段階で変更しない。
