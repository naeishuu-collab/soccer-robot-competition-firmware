# 本番 OpenMV の修復と実機比較

## 調査結果

作業前のDraft PR #7、作業ツリー、ローカルHEADは f8908c500d2b671d34dcca5a18daa449f5b114c2。OPENMV (E:) から4ファイルを読み戻した。vision.py と preview.py は当時のコミットとハッシュ一致。main.py はインデント・コメント等の差があり、構文木で処理が一致した。camera_calibration.py の差は VISUALIZATION_MODE が実機では FULL だったこと。現物の4ファイルはPCへ退避した。

OpenMV v5の描画座標はタプルを渡す。旧本番は最初に draw_string(2, 2, ...) を呼んだため、そこで座標の型エラーが起きると黄青の枠へ進まず、呼び出し側が表示モードをOFFに変えていた。公式のv5移行仕様と、利用者がタプルの blob.rect を渡して枠を描けた事実が一致する。v5の Blob の rect、cx、cy、x、y、pixels、area、code は属性。校正上の x/y/w/h は rect から取得し、面積の検査は w*h と pixels の双方を使う。旧版のメソッド形式は引き続き扱う。現物で debug_mode_enabled の値は採取していないが、旧コードでは False で描画自体をスキップした。 [v5移行仕様](https://docs.openmv.io/v5.0.0/changelog/firmware/v5.0.0.html#image-module-signature-overhaul-major) · [v5.0.1 Blob仕様](https://docs.openmv.io/v5.0.1/library/omv.image.Blob.html)

新しい描画はRECT、STRING、CROSS、CIRCLE、LINE、ARROWをすべてタプル座標で呼ぶ。枠を補助文字より先に描き、例外を起こした描画関数だけを一時停止する。DRAW_ERROR に関数・例外型・メッセージを記録。USBの0/1/2で描画状態を再試行できる。IDE接続検出は診断出力経路の選択にだけ使う。IDEには約2秒ごとの標準出力、一般USB端末にはUSB_VCP送信を使う。BUILD、MODE、4ファイルの読み込み元を起動時に表示する。 [USB VCP仕様](https://docs.openmv.io/v5.0.1/library/pyb.USB_VCP.html)

取得とUARTの実績がある sensor API を使用。非推奨警告は移行情報。JPEG・RAW設定と画像の実際の転送周期はIDE側にも依存する。描画は新しい画像を使い、白線とゴールの認識・UART送信の完了後だけ。現在フレームの枠は常に描き、FULLの床・ミラー・セクター描画を100ms間隔にする。古い方位は cached と表示する。 [IDEプレビュー仕様](https://docs.openmv.io/v5.0.1/openmvcam/tutorial/vision/csi/ide-preview.html)

## ゴールと速度

本番のLAB（黄5と青2）、サイズ、中心、露光45ms、ゲイン6dBを維持。既定の g は同一フレームの1回の find_blobs にまとめ、code ビットを用いて異色の結合を禁止する。最小画素数・面積、リング、角幅、密度を評価する。縦長や候補の外側に緑が多い場合は信頼度を下げる。探索帯の内縁で切れた候補は全周探索で再評価する。初回と4回に1度は全リング探索、通常は帯または直近の両ゴールを含むROIを使う。探索に12ms超を使った場合は間隔を45msから160msへ広げる。カメラUARTのゴール欄は次の白線パケットで更新する。b は従来の色別2回探索、h は共同の全リング探索、l は白線のみ。d は小さい黄候補を診断する。未検出や信頼度は走行判定への入力にならない。

BENCH_BASELINE=True とすると、利用者が成功した広い暫定LAB、全画像、最低30画素、暖機後のゲイン・白バランス固定、露光自動で比較できる。READYは送信せず、モーターを駆動する有効なカメラ情報を作らない。通常の短露光比較は BENCH_BASELINE=False で BENCH_EXPOSURE_US を 45000、30000、20000、15000、10000、5000 の順に切り替える。同じ照明・鏡位置でOFF/LIGHT/FULLの各モード、IDE転送の有無を比較し、白線・床・黄・青の正解数と誤検出数を記録する。タイミング統計は再利用し、色別の時間は b、共同探索の時間は GOALS_BOTH で確認する。純粋な画像取得＋IDE比較は BENCH_BASELINE=True と OFF を組み合わせる。

利用者による単体取得は約46.75FPS。本番の実際の露光時間・取得時間はログで確認する。45msが実露光なら約22.2FPSが撮像の上限目安。修正後の本番FPS、IDE遅延、検出精度は実機未測定。

## SAFE_MASKとIRの切り分け

ログの NO_SAFE_ESCAPE は、近接白候補のために退避条件へ入り、前進・後退とも SAFE_MASK に無いことを示す。旧実装は各放射線で2連続の白を白線扱いとし、各方向に必要な全放射線で35cm以内の緑が連続している場合だけ床を確認済みとした。18cmは距離表の開始点なので、その値が出た場合はミラー中心、白い機体部品、反射、本物の境界を画像で照合する。手元のログから実際の物体や各セクターの拒否理由を確定できない。

- RAYS は前方から時計回りに15°ずつ。Gは連続した緑床、Wは隣の放射線と距離が8cm以内で一致する白候補、wは単独の白候補、?は確認できない観測。FULLでは緑床、白候補、ミラー中心・リング・前方を表示する。
- FLOOR_MASK は床を確認済みの16方向。UNKNOWN_MASK はその補集合。LINE_MASK は白候補が閉鎖した方向。NEAR_AMBIG_MASK は単独の近接候補から十分離れない方向。隣の放射線による裏付けがあるときだけ新たに退避を開始する。単独の近接候補もその方向への進入を閉鎖する。裏付けによって未知の床を許可することはない。
- ESCAPE_AWAY_MASK は幾何上離れる前後方向。ESCAPE_MASK はさらに床を確認できた退避方向。近接閾値25cm、解放30cm、接近方向の閉鎖35cm、退避時の最低角差105°は維持。支持のない近接候補では、105°以上離れていて床を確認できる既存経路だけが残る。
- IR_STRENGTH=0 は有効なIR応答内の12受光値合計がゼロという意味。IR_ANGLE=0 を前方の検出と解釈しない。IR基板のUSBで i を送り、ボールなしと発光中のボールを近づけた場合の COUNTS、REQUEST_COUNT、RESPONSE_COUNT を比較する。全チャネルがゼロのままなら送信球、受光器電源・GND、GP18–29のLOWパルス、基板版を確認する。MAINのIR健全表示は通信を示す。既存のIR 4,096パターン、ZIP PD、MOTOR経路をホストで再検証した。

## 承認後の導入と確認

1. 動力を切った状態で、現物の4ファイルを別の場所へ保存する。
2. competition/openmv の main.py、vision.py、preview.py、camera_calibration.py を一組として、OpenMV IDEまたはOPENMVボリュームへコピーする。端末で FULL を使う場合は現物の設定を維持するか USB 2 で切り替える。変更済みの現物校正値がある場合はコピー前に比較する。
3. IDEで実行し、BUILD が rcj-openmv-production-2、読み込み元が意図したもの、MODE が LIGHT または FULL であることを確認する。ROIや処理時間は i、ゴール比較は b/g/h、黄候補は d。最新フレームの黄青枠と DRAW_ERROR、GOAL_ERROR を確認する。
4. 緑床のみ、後方の白線、前方の白線、固定の白い配線と反射、ボールなし／ありを順に確認する。MAINの RX 鮮度、SAFE_MASK、STOP_REASON、IR_STRENGTH をカメラのログと同時に記録する。走行確認は車輪を浮かせた状態から始める。
5. 継続して NO_SAFE_ESCAPE となる場合、まず FLOOR_MASK と ESCAPE_AWAY_MASK の共通ビットを確認し、FULL画像と RAYS を照合してミラー校正とLABを確認する。修正前へ戻す場合は退避した4ファイルを復元する。

修正した通信は従来と同じカメラUARTフレームを使用する。MAIN、IR、MOTORのロジックとタイムアウトは今回の変更対象外。
