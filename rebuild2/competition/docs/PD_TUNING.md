# PD追尾の調整（実機未検証）

`competition/main/chase_pd.h` の `PdConfig` が調整箇所。旧制御へ戻すには同ファイルの `RCJ_PD_LEGACY_DEFAULT` を `1` に変更してArduino IDEで再コンパイルする。停止中のシリアルパラメーター変更は実装していない。p/y/b/cはいずれも同じPDを使用し、y/bはその後にゴール方向補正を加える。

旧値は Kp 2.0、Kd 0.2、TURN ±250、基準175、非ゼロ最小140、最大210。新しい初期値はKp 1.8、Kd 0.08、Dを±50に制限してから係数0.30で平滑化、正面±4°デッドバンド、TURN ±180・毎秒2000の変化制限。直進基準175を旋回の大きさに応じて140まで下げる。最小140と最大210は維持。最小140と飽和により小さな左右差が消える場合がある。実機の起動PWMが未確認なので最小値は下げていない。角度の平滑化は行わず、`IR_FILTERED_ANGLE` は現在 `IR_RAW_ANGLE` と同じ。角度差分は円周角として計算し、喪失時に状態を消去する。Dは同じ側の誤差に逆らって単独で反対側にTURNを出さないよう制限するが、スルーレートの過渡期間は直前の符号が残りうる。

IRのポーリング目標は30ms、IR基板の取得窓は5ms。PDは有効な新しいIR応答が来たときのみ更新し、実際の間隔で微分する。`IR_POLL_INTERVAL_MS` と `IR_POLL_MAX_MS` は実際の送信間隔を示す。MOTORの100ms通信監視、200ms逆転待ちは変更していない。MOTORには通常の加減速ランプはなく、TURNの変化制限も個々のPWMの滑らかさを保証しない。

115200bpsで `i` を送ると1秒周期と同じ診断が要求される。`IR_RAW_ANGLE`、`IR_FILTERED_ANGLE`、`PD_ERROR`、`PD_P`、`PD_D_RAW`（制限前）、`PD_D_FILTERED`、`PD_TURN_RAW`（Pと平滑化Dの和）、`PD_TURN_FILTERED`（変化制限後）、`BASE_PWM`、`PWM_CALCULATED`、`PWM_AFTER_SAFETY`、`PWM_TRANSMITTED`、`IR_POLL_MAX_MS` を記録する。診断は従来通り非同期の32バイト分割送信だが、追加書式処理の実機所要時間は未測定。`PD_LEGACY=1` は旧制御を表す。

実機調整は、まず動力遮断でIRと診断を確認し、車輪を浮かせてPWM方向、停止、通信断、逆転待ちを確認する。白線を無視する試験モードは障害物・段差から離れ、外部動力遮断担当者を配置する。シリアルモニターを開き、`p`、新しいIR検出を確認して `g`。`s` は停止。現行仕様では時間制限はない。短い試験から開始し、波形と実際の挙動を記録して設定を一度に一つだけ変え、旧版とも比較する。実際のタイヤ・床・重心と制動距離を測るまでは、ドリフト改善や安全性は未確認。

ホスト比較は `competition/tests/pd_comparison.cpp`。指標は合成入力に対するTURN符号反転回数、角度とTURNの符号不一致回数、TURNと4輪PWMのサンプル間絶対変化の合計。ステップ応答は0→60°でTURNが54以上に初めて到達するまでの時間。ロボットの閉ループ性能や実測時間ではない。

## 合成入力の比較結果

`old/new` は旧制御／新制御。`wrong` は実入力角とTURNの符号が逆のサンプル数。`turn_delta` と `pwm_delta` は初回をゼロからの変化として含む。移動入力の符号反転にはボール自身の左右移動も含まれる。

```text
stationary old/new flips=0/0 wrong=0/0 turn_delta=40/36 pwm_delta=700/688 response_ms=-1/-1
moving old/new flips=4/4 wrong=12/0 turn_delta=1348/1067 pwm_delta=1820/1820 response_ms=-1/-1
front_noise old/new flips=79/0 wrong=0/0 turn_delta=5968/0 pwm_delta=21896/700 response_ms=-1/-1
step_0_to_60 old/new flips=0/0 wrong=0/0 turn_delta=380/128 pwm_delta=1960/840 response_ms=0/0
wrap_359_0 old/new flips=39/0 wrong=20/0 turn_delta=1232/0 pwm_delta=5612/700 response_ms=-1/-1
loss_reacquire old/new flips=0/0 wrong=0/0 turn_delta=240/216 pwm_delta=1400/1400 response_ms=-1/-1
irregular old/new flips=0/0 wrong=0/0 turn_delta=11559/2156 pwm_delta=19292/700 response_ms=-1/-1
goal_10deg old/new PWM=115,210/114,210
```
