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

## 変更点（`soccer_camera/` との違い）

`soccer_ir_main_line/main/main.ino` のみ、次の2点を変更しました：

1. **キック判定をIR方位だけで行う**
   - IRが新鮮で、ボール方位が正面 **±5°** 以内のときだけキック
   - カメラの `flags bit4` / `kick` は使用しない
2. **カメラ指令による運動制御の上書きを削除**
   - `openMvFresh` の判定を外し、IR追従とジャイロ方位ホールドだけで動作

```cpp
const bool irBallFront = input.irFresh && input.ballFound && input.ballAngleDeg < 360 &&
    fabsf(shortestAngleError(static_cast<float>(input.ballAngleDeg), 0.0f)) <= 5.0f;
kickerInput.ballAligned = irBallFront;
kickerInput.request = irBallFront;
```

## 書き込み手順

各フォルダを Arduino IDE で開き、対応する基板に書き込みます：

1. `ir/ir.ino` → IR基板
2. `line/line.ino` → LINE基板
3. `motor/motor.ino` → MOTOR基板
4. `main/main.ino` → MAIN基板

`common/src/` の内容は各スケッチに inlined されているため、個別に書き込む必要はありません。

## 補足

- カメラが無いため、壁／仮想白線／ペナルティエリアの推定は行いません
- 白線は `line/` 基板の実センサーを使います
- キックの角度許容（±5°）は `main.ino` の `5.0f` を変更すると調整できます
