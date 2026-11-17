# テスト・ビルド結果

最終確認日: 2026-11-17（JST）

## 2026-11-17 起動時AUTO IR連続追跡化後の確認（最新）

| 検証 | 結果 | 範囲 |
|---|---|---|
| Python GPIO/ソース検査 | PASS、自己テスト12/12 | LINE 18ch使用/故障6ch除外、GPIO配列、MOTOR出力、コマンド処理、独自ヘッダー不使用を確認 |
| C++ `test_core.cpp` | PASS | MSVC C++17。60ケース/1,791チェック、0失敗。起動時の一度だけの自動ARM・連続追跡・IR消失停止を含む |
| C++ `test_integration.cpp` | COMPILE PASS / RUN PASS | MSVC C++17。14シナリオ/107チェック、0失敗 |
| 競技用Arduinoスケッチ | PASS 4/4 | Arduino CLI 1.6.0、Arduino-Pico 6.3.2。MAIN/IR RP2354A、LINE RP2354B、MOTOR RP2354A。各スケッチを直接コンパイル。アップロードなし |
| RS485単体チェック用スケッチ | PASS 2/2 | TX/RXエコーテスト。RP2350Aでコンパイル。アップロード・実機通信なし |
| MAIN→RS485→MOTOR→4輪 | ユーザー報告でPASS | 以前の手動実機確認。今回の作業では通電・書き込み・再試験なし |
| IR/LINE RS485、IR方位、BNO055、AUTO走行 | 未確認 | センサー実通信、方向符号、車輪合成方向、GPIO4/5 I2C、境界検出、床上走行は未実施 |

今回のArduino Flash/グローバルRAMはMAIN `72,992/10,924 B`、IR `60,044/14,840 B`、LINE `56,508/10,164 B`、MOTOR `57,636/10,204 B`でした。MOTORのRP2350AはZIP親基板BOMに合わせたビルド対象ですが、実装MCU刻印は未照合です。

`AUTO IR`は一時試験専用です。起動後はLINE照会なしでIRを待ち、最初の新鮮なボール検出で一度だけ自動ARMします。IRが有効なボール検出を続ける間は各輪PWM上限255/255で走行します。正面0°を含む30°区分で、正面だけ低速前進、他方向はその場旋回します。IR無応答/不正値/不完全フレーム、ボール消失、`S`で停止・解除し、自動再ARMしません。USBコマンドを入力すると起動時の自動ARM待ちは解除されます。通常`AUTO`/`AUTO FLOOR`の構成ゲートは引き続き閉じています。

共通5モジュールを4基板それぞれの`.ino`に直接統合しました。`common/src`はホストテスト用の原本として残し、Python検査で4つの統合コピーとの一致を確認します。追加インクルードパスは不要です。MOTOR PWM/方向実装は変更していません。Arduinoコアと外部ライブラリのヘッダーは引き続き使用します。

IR試験は未確認の車輪合成方向を使い、LINE境界検出を行いません。MAIN基板に物理E-stopはありません。ユーザー報告ではMOTOR基板の物理スイッチがモーター電源を切りますが、この作業では遮断を実測していません。車輪を浮かせ、スイッチをすぐ操作できる状態でのみ試してください。MOTORにはMAINへのACKがなく、通信断時の停止はMOTOR側300ms期限に依存します。今回、実機電源投入・モーター起動・ファームウェア書き込みは行っていません。

詳細記録: [Arduino 4基板コンパイル](../tests/arduino_compile_current.txt)、[C++テスト](../tests/results_last_run.txt)、[GPIO検査](../tests/gpio_check_current.txt)。

この節の結果が同日以前の検証記録より新しいものです。2026-11-15以下は履歴で、今回の合格件数として再利用しません。

## 2026-11-15 実行結果（履歴）

## 変更前の検証記録（履歴。今回のPASSとして再利用しない）

| 検証 | 結果 | 範囲 |
|---|---|---|
| GPIO checker自己試験 | PASS 8/8 | パッケージGPIO範囲、衝突検出、MOTOR旧版隔離、アクティブ設定検査 |
| GPIO/ソース検査 | PASS | IR/LINE設計候補マップ、MOTOR現行ルートUNKNOWN、MOTOR UART/PWMゲート閉、キッカー要求ポリシーfalse |
| 現行Arduinoスケッチ | PASS 4/4 | Arduino CLI 1.6.0、Arduino-Pico 6.3.2、Generic RP2350 A/B、2MB。コンパイルのみ |
| 旧単体スケッチ | PASS 6/6 | Arduino-Pico 6.3.2、RP2040 Pico。作業コピーからコンパイルのみ |
| C++ test_core | COMPILE PASS / RUN BLOCKED | MSVCで実行ファイル生成後、Windows Application Controlが起動を拒否。アサーションは実行されていない |
| C++ test_integration | COMPILE PASS / RUN NOT ATTEMPTED | MSVCでコンパイル。テスト実行は環境の実行ブロックを回避せず保留 |
| 実機検証 | NOT RUN | 書き込み・通電・UART・I2C・センサー・PWM・モーター・キッカー操作なし |

ホストテストランナーは一時実行ファイルを起動する段階でWindows Application Controlに拒否された。これはテストのassertion失敗ではない。セキュリティ制御を迂回していないため、今回のC++テスト実行結果は未取得。

## Arduinoコンパイル値

| スケッチ | 選択ターゲット | Flash | グローバルRAM | 結果 |
|---|---|---:|---:|---|
| MAIN | Generic RP2350A / 2MB | 70,144 B | 10,860 B | PASS |
| IR | Generic RP2350A / 2MB | 58,980 B | 14,836 B | PASS |
| LINE | Generic RP2350B / 2MB | 56,508 B | 10,164 B | PASS |
| MOTOR | Generic RP2350B / 2MB | 54,316 B | 10,184 B | PASS |

MOTORのRP2350B選択はコンパイル用ターゲットで、製造新版のMCU/packageを確定しない。4件ともアップロードなし。Flash/RAM値は今回のビルド出力で、スタックの最大使用量や実機ピークRAMを表さない。

## 旧スケッチのビルド

| スケッチ | Flash | グローバルRAM | 結果 |
|---|---:|---:|---|
| gyro/gyro.ino | 61,672 B | 9,136 B | PASS |
| gyro/gyro_callibration.ino | 61,576 B | 9,128 B | PASS |
| IR/main.ino | 57,140 B | 9,204 B | PASS |
| IR/sub.ino | 59,988 B | 9,824 B | PASS |
| line/sensor24pin.ino | 57,364 B | 9,112 B | PASS |
| motor/motor_test.ino | 56,448 B | 8,912 B | PASS |

旧motor_testは旧スケッチのコンパイル確認に限る。ピン値は製造MOTOR新版向け設定ではない。

## C++テストの履歴と今回の限界

最新のIR/LINE/MAIN統合変更以前に、隔離作業コピーで単体45ケース/323チェックと統合12シナリオ/102チェックがPASSした記録がある。これは旧時点の結果であり、今回変更後の実行結果として再利用しない。

今回のtest_coreはコンパイルまでは成功したが、実行時に拒否された。test_integrationもコンパイルのみ。したがって、IR方向コヒーレンス、LINE debounce、MAINのキッカー要求ゲートを含む最新C++アサーション件数・失敗件数は**未測定**。

## ZIP・KiCad静的監査の範囲

soccer.zipは33,932エントリー、SHA-256はD39977E63DE3C7CAC23FECA27BC9AF064628A34A17E64C260859601AC081DE4A。直接エントリーの.ino/.pde/.c/.cpp/.h/.hpp/.cc/.cxxは0件。名前が.zipの479エントリーを調べ、467件はZIPとして開け、入れ子内の同拡張子ソースは0件でした。残る12件は__MACOSXのAppleDouble ._*.zipメタデータで、ZIPアーカイブではありません。KiCad/製造ファイルを含むものは設計資料であり、実基板の検証ではありません。

既存監査記録のKiCad CLI 10.0.8静的DRC値を、版の根拠を分けて掲載します。件数はプロジェクト設定に基づくDRC違反レコード数と未接続パッド数です。全て設計ファイル上の結果で、製造後の基板状態を示しません。

| 設計ファイル | 分類 | DRCレコード | 未接続 |
|---|---|---:|---:|
| kick/kika | VERIFIED_DESIGN | 9 | 0 |
| soccer_line/line | VERIFIED_DESIGN | 25 | 2 |
| soccer_ui/soccer_ui | VERIFIED_DESIGN | 32 | 1 |
| soccer_md/soccer_md (MOTOR親) | LEGACY_REFERENCE | 79 | 3 |
| esc/esc | VERIFIED_DESIGN | 63 | 4 |
| soccer_main/main | VERIFIED_DESIGN | 12 | 2 |
| line2/line2 | 過去監査資料。作業対象には未存在/現行板と未照合 | 187 | 1 |
| soccer_ir/soccer_ir | VERIFIED_DESIGN | 111 | 10 |
| soccer_moter/soccer_moter (MOTOR子) | LEGACY_REFERENCE | 6 | 0 |
| ir&main/ir | VERIFIED_DESIGN | 29 | 4 |
| soccer_esc/soccer_dribbler | VERIFIED_DESIGN | 109 | 0 |
| soccer_power/soccer_power | VERIFIED_DESIGN | 53 | 0 |

MOTORの2行は旧版KiCad由来で、製造された改善新版のDRC結果ではありません。新版に同じ違反が存在するとも、解消済みとも判断していません。

## 安全上の実機境界

- MOTORは基準コードのTX/RX/DE候補でUARTを設定したが、実配線・RS485通信は未確認。MOTORの安全GPIOは未設定で、ユーザー報告の物理電源スイッチ遮断も実測していない。
- MAINに物理E-stopはない。`AUTO`/`AUTO FLOOR`の構成ゲートは閉じたまま。`AUTO IR`は起動後の最初の新鮮なIRボール検出で一度だけ自動ARMし、PWM 255上限の車輪浮上試験用。IR異常/ボール消失/`S`で停止後は自動再ARMしない。LINE境界検出はなく、手動`M`は従来どおり-255..255。
- キッカーの物理出力アダプターはなく、ハードウェア・要求ポリシー・姿勢窓は未設定。
- 60ms pulse / 1000ms cooldownはホストモック用で、実機設定値ではない。
- 実機で確認済みと扱うのはユーザー報告の過去の電源投入とLED点灯だけ。今回再現していない。

## 未実施

実MOTOR新版でのGPIO/電源/RS485導通、センサー出力、BNO055校正、停止回路、実PWM、競技走行、キッカー残留電圧・充電禁止・発射安全は未確認。ハードウェア判定はNO-GO。

