# 書き込みガイド

## ビルド

Arduino-Pico 6.3.2の Generic RP2350、2MB Flashを使用。既存手順のビルド設定はMAINがRP2350B、IRとMOTORがRP2350A。A/Bとフラッシュ容量は実機の刻印と基板資料で確認する。`competition/scripts/compile.ps1` は3基板をコンパイルする。Arduino IDEでは `competition/libraries/RobotCommon` をスケッチブックの `libraries` に入れ、対象基板の `.ino` を開く。

| 基板 | 書き込み対象 |
| --- | --- |
| MAIN | `competition/main/main.ino` |
| IR | `competition/ir/ir.ino` |
| MOTOR | `competition/motor/motor.ino` |
| OpenMV | `competition/openmv/main.py`、`vision.py`、`preview.py`、`camera_calibration.py` を同じディレクトリへ |

## 実機作業時

1. 各基板の現在の版、動作設定、OpenMVの現在のファイルと実機固有の校正値を記録・保存する。
2. 動力を切り、機体を固定する。Arduinoボードを個別にBOOTSELへ入れ、ボードに合うUF2を書き込む。
3. OpenMV IDEで4ファイルを本体へ保存する。電源の再投入で `main.py` が開始する。
4. MAINとMOTORは13バイト版の同じコミットにそろえる。OpenMVは同じ作業ツリーの4ファイルをまとめて保存する。IRの要求・応答はZIP互換だが、ベクトル相殺に対応する今回の3基板を組み合わせる。
5. USB診断から `i`、`s`、`g` を確認し、[実機確認手順](HARDWARE_TEST_GUIDE.md)を進める。

OpenMV UART3のピンとAPIは公式の [UART仕様](https://docs.openmv.io/v5.0.1/library/pyb.UART.html) を参照。


## 車輪を浮かせる診断用MAIN

現在の診断版は従来の記載とコマンド・制限が異なる。ビルドには `RCJ_SERIAL_TRIAL=0` と `RCJ_WHEELS_LIFTED_DIAGNOSTICS=1` の両方が必要。床上では使わない。最新の [Phase 3.1 校正・診断手順](PHASE31_CALIBRATION.md) を使用する。
