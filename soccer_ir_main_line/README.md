# soccer_ir_main_line（カメラ無し・IR/main/line 構成）

OpenMV カメラを使わず、**IR・MAIN・LINE（＋MAINが制御するMOTOR）**だけで動く構成です。
`soccer_camera/` から必要なファイルだけをコピーし、`main/main.ino` のキック判定を
IR方位ベースに書き換えています。元の `main/` `openmv/` などは変更していません。

## 含まれる基板

| フォルダ | 基板 | 役割 |
|---|---|---|
| `main/` | MAIN (RP2350) | IR統合・キック制御 |
| `motor/` | MOTOR (RP2350) | 4輪オムニ駆動（MAINから制御） |
| `ir/` | IR (RP2350) | 赤外線ボール方位 |
| `line/` | LINE (RP2350) | 白線検出 |
| `common/src/` | 共通 | スケッチ共通コード |
| `config/` | 設定 | GPIOプロファイル |

`openmv/` は含みません。

## カメラ版との違い

- `AUTO IR` はIRでボールを追い、ボール消失・IR通信断では停止します。LINEもカメラも使わず、境界を検知できません。
- キックには正面±5°とIR近接信号が必要です。`main/main.ino` の `kIrKickCloseSignal` は初期値0で、キックを無効にします。実球で測定し、キッカーを無効にした状態で値と判定を確認してから設定してください。ゴール方向は判別できません。
- 詳細は[動作ガイド](../soccer_camera/docs/CONTROL_GUIDE_JA.md)を参照してください。

## 書き込み手順

各フォルダを Arduino IDE で開き、対応する基板に書き込みます：

1. `ir/ir.ino` → IR基板
2. `line/line.ino` → LINE基板
3. `motor/motor.ino` → MOTOR基板
4. `main/main.ino` → MAIN基板

`common/src/` の内容は各スケッチに inlined されているため、個別に書き込む必要はありません。

## 補足

- カメラが無いため、壁／仮想白線／ペナルティエリアの推定は行いません
- `AUTO IR` は故障中の `line/` 基板を使いません
- キック近接しきい値は `main/main.ino` の `kIrKickCloseSignal` です

## chase.zip の実機確認結果の統合

- IRのサンプリングは2 ms、最低検出数は3、方位は実機確認コードに合わせて180°補正し、予測値ではなく計測した方位を送ります。RELは引き続き未校正の相対値です。
- AUTO IRは全方位に平行移動し、ボール消失やIR通信断では停止します。MOTORの4番目はIN1=GPIO8、IN2=GPIO9で、100 msの通信タイムアウトと5 PWM/10 msの加速・減速を使用します。
- chase.zipとは通信形式が異なります。MAIN・IR・MOTORはこのプロジェクトの組み合わせで更新してください。LINEは未使用のままです。
- 統合後の車輪方向・IR方位・反転待ち・キッカーおよび停止は未実機検証です。車輪浮上、キッカー無効、即座にMOTOR電源を遮断できる状態から確認してください。PWM上限16、競技AUTOゲート、既存のカメラ校正ゲートは維持しています。
