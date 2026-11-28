# ロボットが動かないとき

動力を切れる状態で車輪を浮かせ、基板ごとのUSBログを開く。MAINで `i` を送ると次の空き時間に詳細が出る。USBで `s` は停止保持、`g` は解除。

1. **MAINが計算しているか。** `PWM_CALCULATED` がゼロなら `IR_LINK`、`IR_ANGLE`、`IR_STRENGTH`、`IR_AGE_MS` を確認。正の強度で角度が -1 ならセンサーの合成ベクトルが相殺されている可能性がある。IRのUSBで `i` を送り、12個の `COUNTS`、`VECTOR`、要求数と応答数を確認する。
2. **安全判定で止まったか。** `STOP_REASON` と `STOP_DETAIL` を見る。`PWM_CALCULATED` と `PWM_AFTER_SAFETY` の差が安全判定後の変更。 `PATH_BLOCKED` は `REQUIRED_MASK` のうち `BLOCKED_MASK` に入った方向が閉じている。 `NO_SAFE_ESCAPE` では前後に安全な退避がない。 `CAMERA_NOT_READY` は画像サイズや校正、`CAMERA_SLOW` は100ms超のフレーム、`CAMERA_TIMEOUT` はCRCが通り新しい連番のパケットを200ms受理できていない状態。
3. **MAINから送信したか。** `PWM_TRANSMITTED` と `MOTOR_TX_COUNT` を見る。停止時・起動時もゼロ指令を送る。送信数はMAIN側の完了数。
4. **MOTORで受信したか。** MOTORにUSBをつなぎ `i`。`RX_COUNT` が増えず `CRC_ERROR_COUNT` や `INVALID_FRAME_COUNT` が増える場合はフレームと配線を確認する。全部が0なら115200設定、RS485のA/B・GND・DE、ボードの組み合わせ、MAIN送信を順に確認。
5. **MOTORが出力を設定したか。** `PWM_RECEIVED` は最後の妥当な受信値、`PWM_TARGET` は期限確認後の目標、`PWM_APPLIED` は実際にGPIOへ設定した符号付きPWM。実際の車輪回転は目視や計測器で確認する。 `REVERSE_MASK` は反転待ちの原因チャネル、`WATCHDOG_STOP` は指令期限切れ。 `ZERO_COMMAND` はMAINからのゼロ指令。
6. **OpenMVの認識を確認。** OpenMV USBで `i`。 `LINE`、`SAFE_MASK`、`READY`、`SLOW`、`FAST`、`UART_TX`、`UART_PARTIAL`、`FRAME_LATE` を見る。床を見ているのにマスクが閉じる場合はしきい値とミラー方向、USBの計測値が良好なのにMAINで期限切れなら P4→GP3 とGND、版を確認する。両ゴールが -1 でも他の条件がそろえば走行する。

各Arduinoは通常約1秒ごとに診断を出力。MAINの `EVENT_MS` は状態・理由の変化。USBが詰まった場合は測定を止めず、キューのあふれは `EVENT_DROPPED` で分かる。MOTORからMAINへの返信はないため、受信・適用・逆転待機はMOTORのUSBで確認する。

## 最適化版の診断

MAINの PATH_MODE は ZIP、SWEEP、REDUCED_TURN、STRAIGHT_160、ESCAPE、STOP。FALLBACK_REASON は代替または停止の理由。STOP_COUNT／STOP_DURATION_MS は累計で、理由別の COUNT／DURATION_MS も出る。IR_POLL_INTERVAL_MS と IR_POLL_MAX_MS はIR周期。MOTOR の REVERSE_WAIT_COUNT／REVERSE_WAIT_MS は反転回数と累積時間。PWM_ALT_TRANSMITTED が0なら逆転待機中の代替出力は無効。

OpenMV v5.0.1で報告されたタプル展開の構文エラーと blob.rect() の型エラーは修正済み。旧Blobメソッドとv5のBlob属性の両方を扱う。カメラの main.py、vision.py、preview.py、camera_calibration.py は同じ版にそろえる。

GOAL_ERROR と DRAW_ERROR は、任意のゴール処理・描画で発生した最後の例外名と累積数。例外が起きても白線UARTを継続する。描画に失敗した場合はOFFへ切り替わる。
