# サッカーロボット：カメラあり版

> 現行の動作・書き込み方法は [最終版のやさしい説明](docs/FINAL_GUIDE_JA.md) と [2026-11-26の改善・実機検査手順](docs/UPGRADE_2026-11-26_JA.md) を参照してください。

## 競技候補コード

MAINの既定実行経路はZIP PD追尾（`ROBOT_ZIP_TRACK=1`）です。カメラの新鮮な安全情報を要求し、白線退避を優先します。直線の0°/180°退避はIR未検出でも動作できます。元の `chase (4).zip` ファイルはこのリポジトリにないため、ZIP互換ヘッダーと独立転記テストを原本と同一視しません。実機検証は未実施です。

書き込みに使う現行版は `main/`・`ir/`・`motor/` の3基板と `openmv/goal_wall_navigation.py` です。OpenMVには `openmv/camera_calibration.py` と `openmv/competition_vision.py` も保存します。`line/` は今回の chase 構成では未使用です。

`chase_verified/` と `legacy_before_chase/` は比較・検査用の保存版で、書き込みには使いません。`common/`・`tests/` はホスト検査用、`docs/` は手順と記録、`config/` は検査設定です。

## VS Codeで開く

VS Codeの「フォルダーを開く」で `soccer_camera` を選びます。

## 書き込みと運転

- MAINは `main/main.ino`、MOTORは `motor/motor.ino` を組で書き込みます。OpenMVは `openmv/goal_wall_navigation.py` を `main.py` として保存し、実機の `camera_calibration.py` を同じ場所に置きます。今回のカメラとMAINは8方向の床情報を含む新しいUART形式で通信します。
- IRは `ir/ir.ino` または実機確認済み `chase (3).zip` のIRスケッチを使えます。MAINは30 ms間隔でIRに問い合わせ、IRは要求ごとに5 ms測定します。
- 電源投入後、カメラとIRの通信が有効になり、ボールと進行可能な床を確認すると自動で追跡します。未検出、進行先の床が確認できないとき、通信が途絶したときは停止します。ただし有効なライン退避は未検出より優先します。条件が戻ると自動で再開します。起動・復帰の動作確認は車輪を浮かせて行います。

## 役割分担

- IRがボールの有無と方位を測ります。カメラが緑床と白線を確認し、走行可能な方向と白線から離れる方向をMAINに送ります。
- MAINはカメラが許可した方向だけに進みます。床を確認できないとき、ボールを見失ったとき、通信が途絶えたときは停止します（ライン退避は除く）。キッカー出力GPIO11は起動時にLOWにします。
- MOTORは4輪出力を生成します。逆極性への切り替えには200 msの待ちを適用し、有効な指令が100 ms途絶えると出力を止めます。
- OpenMVの `CHASE_ONLY=True` はゴール色の探索関数を呼び出さず、床・白線安全処理とUART送信を続けます。旋回時の安全判定は全16セクターを要求する保守条件を維持します。機体寸法・停止距離・方向別安全判定が未検証のためです。

停止条件と復帰条件は[停止条件](../docs/STOP_CONDITIONS.md)、作業の限界とPENDING_HARDWAREは[最終検証記録](../docs/FINAL_VERIFICATION.md)を参照してください。

白線の回避距離25 cmは暫定値です。白線距離はミラー軸からの距離です。実機で前後左右の検出と停止距離、車輪極性、IR・カメラ・MOTORの通信断、復帰時の自動再開を確認してから、出力と距離を調整してください。 [OpenMV校正手順](docs/OPENMV_CALIBRATION_JA.md)

## 可視化と診断

[認識結果・移動方向の可視化の使い方](docs/VISUALIZATION_JA.md)
