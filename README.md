# サッカーロボット競技ファームウェア

現在のカメラあり版は [soccer_camera/README.md](soccer_camera/README.md) を参照してください。MAIN・IR・MOTORは同じ版を組み合わせ、AUTO・ARM操作を使わず、IRと有効なカメラ情報がそろったときに追跡します。LINEは現在未使用です。

VS Codeでは `サッカーロボット_カメラあり.code-workspace` を開くと、４基板・OpenMV・資料・テストが整理されて表示されます。動作条件と書き込み先は[変更記録](soccer_camera/docs/CHASE_2026-11-21_JA.md)を確認してください。

`soccer_ir_main_line/` と `soccer_ir_main_motor/` は過去のカメラなし版です。以前のAUTO制御を含み、現在のカメラあり版とは通信互換ではありません。`legacy_before_chase/` と `chase_camera/` も旧版・作業途中の保存であり、書き込み用ではありません。

カメラの校正が必要です。統合後の実機動作は未確認のため、車輪を浮かせ、動力をすぐ切れる状態で方向・停止を確認してください。
