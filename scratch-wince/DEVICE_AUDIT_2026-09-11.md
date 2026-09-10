# i.MX28 周辺モデル横断監査 — 追加バグ修正記録（2026-09-11）

タッチパネル監査（`TOUCH_PANEL_AUDIT.md` 第5節）に続き、`qemu-brain` の
i.MX28 機種モデル全般（pinctrl / icoll / timrot / ssp / auart / i2c / pwm /
gpmi / dcp / perfmon / syscon / dflpt / usb / apbh / apbx / pxp / bch / saif /
etm / reserved / rob / lcdif / lradc / pl011）を追加監査し、以下 6 件を根本修正した。

## 1. 修正実施項目

| # | 対象 | 不具合 | 修正 |
|---|---|---|---|
| 1 | `hw/gpio/mxs_pinctrl.c` | CTRL 書き込みで `SFTRST` が立っている間 `CLKGATE` を常時再セット（`val \|= (1u<<30)`）。`mxs_bank.h` が明記する「CLKGATE は SFTRST の立ち上がりエッジでのみ強制」に反し、SFTRST 保持中の CLKGATE クリア（Linux `stmp_reset_block()` / WinCE BSP 共通のリリース手順）でゲストが永久スピンし得る | `mxs_bank_sftrst(old, val)` に置換（立ち上がりエッジ検出） |
| 2 | `hw/dma/mxs_apbh.c` | 上記と同一パターン（`APBH_CTRL0`） | `mxs_bank_sftrst()` に置換 |
| 3 | `hw/dma/mxs_apbx.c` | 上記と同一パターン（`APBX_CTRL0`） | `mxs_bank_sftrst()` に置換 |
| 4 | `hw/char/mxs_auart.c` | `AUART_DATA` への `size==3` 書き込みで `n=3` バイト送信するが `ch[0]/ch[1]` しか初期化せず、`ch[2]` がスタックの未初期化値を送出 | バイト数を `value` からリトルエンディアン順に全バイト初期化（`size<4` は `size` バイト、`size>=4` は従来通り 1 文字） |
| 5 | `hw/misc/mxs_i2c.c` | 読み出し転送で最終バイトの **NACK を `i2c_recv()` 後**に発行していた。QEMU の i2c API は「直前の ACK/NACK を次の `i2c_recv()` で適用」するため、最終バイトが ACK され、NACK が次トランザクションの先頭に漏れる | 2 箇所の読み出しループとも `i2c_nack()` を最終 `i2c_recv()` の**前**に発行。冗長だった後追い `i2c_ack()` を削除 |
| 6 | `hw/misc/mxs_syscon.c` | `RTC_MILLISECONDS` が起動からの通算ミリ秒をそのまま返していた（実機は 1 kHz カウンタで 0..999 を毎秒ロール） | `qemu_clock_get_ms(...) % 1000` に修正（`RTC_SECONDS` の秒数と整合する「秒未満」を返す） |

### 修正理由の補足

- **#1–#3** は既存モデル群（icoll / ssp / saif / pxp / bch / dcp / perfmon /
  syscon / i2c / timrot / rob / pwm）がすべて `mxs_bank_sftrst()` を使っており、
  pinctrl と apbh/apbx の 3 箇所だけが「SFTRST セット中は CLKGATE を再ピン」する
  旧実装のまま残っていた。`mxs_bank.h` の注記はこのデッドロックを明示的に警告して
  いる（「CLKGATE は SFTRST の *立ち上がりエッジ* でのみ強制しなければならない」）。
- **#5** は `i2c_recv()` が「呼び出し前に設定された ACK/NACK をその受信に適用し、
  呼び出し後に ACK 状態へリセットする」という QEMU core の契約に反していた。
  従来コードは NACK を最終受信の後に発行していたため、(a) 最終バイトが誤って ACK
  され、(b) その NACK が次トランザクションの先頭バイトに誤適用される、という
  二重の不具合があった。ack-all スレーブ（各アドレスに常駐）に対しては実害が
  出ないため起動は正常だったが、SGTL5000 等の実スレーブが読まれる経路では
  プロトコル違反になり得る。

## 2. 監査で「正常」と判断した主な項目（参考）

- 各ブロックの SFTRST/CLKGATE 処理: icoll / ssp / saif / pxp / bch / dcp /
  perfmon / syscon / i2c / timrot / rob は `mxs_bank_sftrst()` または等価な
  エッジ検出済み。CTRL の SET/CLR/TOG エイリアス（+0x4/+0x8/+0xc）まで含めて
  正しく `(offset & ~0xfu) == 0` / `idx == 0` 判定されていた。
- pwm は CTRL を「自己クリアする SFTRST/CLKGATE」として読み出し時にマスクする
  特殊モデルで、BSP の `PWMGetChannelPresentMask()`（0x50 期待）との整合を
  コメント含めて維持。
- icoll は 128 ソース・4 優先度・ARM_RSE ベクタ読み出し・LEVELACK・SFTRST による
  intr クリアを実装済みで、優先度/ネスティング計算も整合。
- timrot は 4 カウンタ・match/fixed 両モード・W1C の TIMCTRL.IRQ 処理・
  match 発火の「1 プログラミングにつき 1 回」モデルを実装済み。
- rob / dflpt / etm / reserved は「レジスタのみ」ブロックとして reset 値・
  R/O マスク・SFTRST エッジを実装済み。
- ssp は RUN エッジ起動・`mxs_bank_sftrst`・END_CMD ビットの A/B 検証コメントを
  含め実装済み。

## 3. 見送り（今回は修正しないと判断した項目）

- **mxs_ssp vmstate が `timing`/`ddr_ctrl`/`dll_ctrl`/FIFO/データフェーズ状態を
  保存しない**: マイグレーション（save/load）を使用した場合のみ現れる非完備。
  データフェーズ進行中（`data_remaining`/`data_read`/`multiblock`/`fifo`）の保存
  追加は vmstate バージョン変更と互換処理を伴い、本機種（手持ち端末の全系
  エミュレーション）でマイグレーション実績がないため、リスクに対して利益が
  小さいと判断。将来マイグレーション対応を行う際に v2 化して追加する。
- **mxs_dflpt のサブワード書き込みがバイトレーンを無視**（read は `offset&3`
  を考慮するのに write は未シフト）: ゲスト（ARM926 MMU 設定）は 32 ビット
  ストアでページテーブルを書くため実害なし。read/write の対称性の問題として
  記録に留める。
- **ICOLL_STAT のアイドル値 0x7f**: ソース 127（GPIO0）と誤解され得るが、BSP は
  IRQ 線アサート中にしか STAT を読まない（かつ ARM_RSE モードでは VECTOR を読む）
  ため実害なし。RM の厳密なアイドル値が未確認のため触らない。
- **apbh/apbx の `ctrl0` SFTRST リセット時のチャネル状態**: 実機の SFTRST は
  チャネルレジスタを初期化するが、本モデルは CTRL0 のビット操作のみでチャネルを
  触らない。ゲストのリセットシーケンスでは CHANNEL_CTRL の RESET_CHANNEL が
  明示的に使われるため、現状の挙動で成立していると判断。

## 4. 検証

- 修正 6 件はいずれも「既存モデル群との同一性を確認した正規形（`mxs_bank_sftrst`）」
  「QEMU core の API 契約」「i.MX28 RM のレジスタ定義」という裏付けに基づく。
- `mxs_bank_sftrst` のシグネチャ（`(uint32_t old, uint32_t new_val)`）と引数順を
  全置換箇所で確認。
- auart の `value >> (8*i)` は `value` が `uint64_t` のため size==3 のシフト 16 まで
  安全。C99 の for 初期化子宣言は QEMU（gnu99）で許容。
- 完全ビルドは前回同様サンドボックスのネットワーク制約（deb.debian.org 到達不能、
  glib/pixman dev ヘッダ導入不可）により未実施。静的確認のみ。

## 5. 追加修正（第2弾）

| # | 対象 | 不具合 | 修正 |
|---|---|---|---|
| 7 | `hw/misc/mxs_pwm.c` | `HW_PWM_VERSION`（バイトオフセット 0x110）の読み出し判定が `MXS_PWM_VERSION_OFF >> 2`（=0x44）になっており、VERSION が誤ったオフセット 0x440 で返り、本来の 0x110 は `regs[0x11]`（ゼロ）を返していた。他ブロックは全て `VERSION_OFF >> 4`（`MXS_BANK_INDEX` と同基軸） | `>> 2` を `>> 4` に修正 |
| 8 | `hw/misc/mxs_pwm.c` | ファイル冒頭コメントが「PRESENT は PWM4\|PWM6 のみ（0x50 期待）」と説明する一方、コードは `BRAIN_PRESENT_MASK` で全 8 チャネル（0xFF）を報告しており、記述が実装と矛盾 | 冒頭コメントを実装（`BSP_PRESENT_CH_MASK = 0xFF` 期待）に合わせて書き直し |
| 9 | `hw/misc/mxs_saif.c` | `saif_dma_xfer()` の書き込み方向ループが `for (i = 0; i + 3 < len + 3; i += 4)`（実質 `i < len`）で、`len` が 4 の倍数でないとき最終反復が `buf[i+1..i+3]` をバッファ末尾越えで読む（OOB read）。読み出し方向は `i + 3 < len` で正しくガード済み（`dcp_swap_buf` も同形式） | ループ境界を `i + 3 < len` に修正（4 バイト整ワードのみ処理） |

### 追加の見送り

- **`hw/misc/mxs_gpmi.c` の default 書き込みパスがバイトレーンを無視**
  （`s->regs[off >> 2] = (uint32_t)value`。read は `mxs_bank_extract` でバイトレーン対応）:
  GPMI レジスタは 4 バイト境界に 32 ビットストアされるのが BSP の実使用であり、
  GPMI_DATA（コマンド/ステータスバイト経路）へのバイト書き込みも lane 0 のみで
  現状の実装で成立する。DFLPT と同種の read/write 非対称として記録に留める。

## 6. 追加修正（第3弾）

| # | 対象 | 不具合 | 修正 |
|---|---|---|---|
| 10 | `hw/arm/mxs_fat.c` | `mxs_fat_get_entry()` の非 FAT32 経路が **FAT12 の 1.5 バイト歩幅**（`(cluster*3)/1024`、`(cluster*3)&1023`）で位置を計算しながら **2 バイト（FAT16 幅）** のエントリを読んでいた。FAT16 ボリュームでは全クラスタチェーンが 1 バイトずれて歩かれ、全ファイル読み出しが誤る | 非 FAT32 経路を正しい FAT16（2 バイト/エントリ）に修正。FAT12（総クラスタ < 4085）は本リーダーの文書化済み対象外（FAT16/32 のみ）なので open 時に明示的に拒否 |
| 11 | `hw/misc/mxs_dcp.c` | SEMA 書き込みの実行ループが **フル 32 ビット書き込み値 `v`** を発行パケット数として使っていた（SEMA は 8 ビットカウンタで、加算するのは `v & 0xff` のみ）。上位ビットが立った書き込みで最大 2^32 個の幻パケットが発行され得た | `added = v & 0xff` をループ境界に使用 |
| 12 | `hw/misc/mxs_gpmi.c` | `GPMI_CTL_ECC_STEP`/`GPMI_CTL_ECC_POS` が **シフトされていない `0x3`** で、ファイル冒頭コメントの「bits[11:10] / bits[15:14]」と矛盾。書き込みマスクが RUN/READ ビットにエイリアスし、ECC_STEP/ECC_POS フィールドは書き込み時に常に 0 に落ちていた | `(0x3u << 10)` / `(0x3u << 14)` に修正 |

### 追加で確認し「正常」と判断した箇所（第3弾）

- `hw/arm/mxs.c` の機器配線（ICOLL/APBH/APBX/SSP/LRADC/LCDIF/AUART/SAIF/
  I2C0 コーデック/EDNA2 mailbox/タッチキー）を全文確認。IRQ 番号（ICOLL 33 =
  EDNA2 attention、59/58 = SAIF0/1、41 = GPMI/BCH、13 = HSADC 等）は i.MX28
  割り込み表と一致。EDNA2 mailbox の doorbell(+0x3C)/command(+0xE8)/done/
  touchkey(+0x404) の段階モデルもコメント含め整合。
- `hw/input/brain_kbd.c`（キーマトリクス・MRSensor・タッチキー・EDNA2 attention
  パルス）は全 1005 行を確認。行列デコード・W1C タイミング・スキャン周期モデルに
  不整合なし。vmstate v5（touchkey_want/pub を v5 フィールド化、min v4）も正しい
  後方互換パターン。
- `hw/display/mxs_pxp.c` はブリッターとしてサーフェスサイズを PXP_MAX_SURFACE
  (16 MiB) で上限付けしており、`pxp_fetch` が NULL を返した場合の早期復帰も実装
  済み。回転/反転/スケールの座標マップも整合。
- `hw/misc/mxs_saif.c`（FIFO レベル判定・codec 配線・W1C STAT 処理）、
  `hw/misc/mxs_gpmi.c`（コマンド別 busy タイミング・2 相プログラム・BCH エンコード
  レイアウト）、`hw/misc/mxs_dcp.c`（AES/SHA/CRC パケット処理・キー RAM・割り込み
  ルーティング）に追加の不整合なし。
- `accel/tcg/brain_stats.c` / `include/brain_stats.h` のイベントリング
  （BRAIN_EVENT_RING=256、2 の冪）とダンプ処理（`start = pos - n`、n<=total なので
  アンダーフローなし）は正常。
- `hw/char/pl011.c` は標準 PL011 + BRAIN_SERTRACE デバッグ補助のみで、brain 固有の
  変更は最小限（`mxs_trace_guest_pc()` の外部参照とゲスト PC 表示）。

## 7. 追加修正（第4弾）: mxs_i2c

| # | 不具合 | 修正 |
|---|---|---|
| 13 | 複数バイト PIO 送信（書き込み）が完了できない。DATA 書き込みハンドラは送信先アドレス判定に CTRL0 の PRE_SEND_START を使うが、このビットはゲストが消さないため全 DATA バイトが「アドレス」扱いで再 START され、継続バイト経路（`else if (RUN)`）は到達不能。さらに完了判定が「書き換えられない CTRL0.XFER_COUNT を毎回読み直す」ため永遠に 0 にならず、`count<=1` でしか完了しなかった | 送信状態 `xfer_started` / 残りバイト数 `xfer_left` を追加。先頭 DATA = アドレス（`i2c_start_send`）、XFER_COUNT はアドレス含みなので `xfer_left = count-1`。以降の DATA は `xfer_left` を減算し、0 で STOP + `DATA_ENGINE_CMPLT` 完了。`finish()` / `reset()` で状態を初期化 |
| 14 | CTRL1（+0x40）書き込みの扱いが破綻。スイッチは `case 0x48>>4`（idx 4）で、+0x40 直書き込み・+0x44 SET・+0x48 CLR・+0x4c TOG を区別せず、`regs[4]=val` の直後に `regs[4] &= ~val` を実行するため **CTRL1 ワード全体が常に 0 に潰れた**（SET エイリアスで enable を立てても即消滅。直書き込みは status ビットのみの場合に偶然クリアできただけ）。IRQ 更新も SET/CLR/TOG では意味をなさなかった | `MXS_BANK_OP(off)` でエイリアスを区別。+0x40 直書き込みは status ビット [7:0] と CLR_GOT_A_NAK(bit28) を **W1C**（書いた 1 のビットをクリア）として `(val & ~W1C) | (old & ~val & W1C)` にし、SET/CLR/TOG は `mxs_bank_apply()` の結果をそのまま維持。いずれも `mxs_i2c_update_irq()` を呼ぶ |

- CTRL1 のレイアウトは Linux `drivers/i2c/busses/i2c-mxs.c` と一致することを確認
  （`MXS_I2C_CTRL1_CLR = 0x48`、`DATA_ENGINE_CMPLT_IRQ=0x40`、`NO_SLAVE_ACK_IRQ=0x20`、
  各 *_IRQ ビットは write-1-to-clear）。BSP が直接 0x78 / 0x40 を書くのは ack（クリア）。
- vmstate は regs[] のみ（`xfer_left`/`xfer_started` は一時状態のため他 mxs 機器と同じ
  慣例で含めない）。

## 8. 追加修正（第5弾）: 定義・コメントの不整合

| # | 対象 | 不整合 | 修正 |
|---|---|---|---|
| 15 | `hw/misc/mxs_gpmi.c` | ファイル冒頭コメントは `GPMI_CONFIG` の BCH_BYTES を「bits[2:1]」としているが、マクロは **シフトされていない `0x3u`**（bits[1:0]）で矛盾（ECC_STEP と同じクラス。現状未使用マクロだが後続修正の誤誘導源） | `(0x3u << 1)` に修正 |
| 16 | `hw/misc/mxs_syscon.c` | HW_DIGCTL_MPTEn_LOC のコメントが「DIGCTL base + 0x500 + **4*n**」と誤記。実際は **0x10 間隔**（0x500/0x510/.../0x5f0、u-boot `regs-digctl.h` の構造体配置と一致。実装 `DIG_MPTE0 + n` も 0x10 間隔で、コードは正しい） | コメントを「0x10*n」に修正 |

### 第5弾で全文確認し「正常」と判断した箇所

- `hw/intc/mxs_icoll.c` — 割り込み優先度・in-service レベル・RSE モードの VECTOR 読み取り
  ack・LEVELACK・SFTRST 時の intr 全クリア、autorelease（発信源消滅時の in-service 解放）。
  不整合なし。
- `hw/timer/mxs_timrot.c` — レジスタ配置（ROTCTRL 0x00 / TIMCTRLn 0x20+0x40n / VERSION
  0x120）を u-boot `regs-timrot.h`（mxs_reg_32 展開で各レジスタ 0x10）と照合し一致。
  SELECT/PRESCALE デコード（0xb=32k,0xc=8k,0xd=4k,0xe=1k,0xf=24M）も一致。TIMCTRLn.IRQ の
  W1C 処理は実装済み。MATCH モードの fired_match による再発火抑止も正しい。
- `hw/dma/mxs_apbh.c` / `hw/dma/mxs_apbx.c` — CCW チェーン歩行・PIO ワード・SEMA（低 8 ビット
  加算＋SEMAPHORE フラグで減算）・CHAIN/IRQONCMPLT・RESET_CHANNEL 自己クリア、両者一致。
  不整合なし（vmstate 未登録は WinCE 用途では実害なし、と判断）。
- `hw/sd/mxs_ssp.c` — CTRL1 の IRQ ペア（奇数=status, 偶数=enable）走査・BLOCK_COUNT/
  BLOCK_LOG2 デコード・複数ブロック STOP_TRANSMISSION 合成・FIFO ストリーミング・
  END_CMD の「新コマンド開始でクリア」を確認。不整合なし。
- `hw/misc/mxs_perfmon.c` — SNAP/CLR 自己クリア、SFTRST 復帰、統計 shadow レジスタ、
  24 MHz 換算の ACTIVE_CYCLE 積算。不整合なし。
- `hw/misc/mxs_dflpt.c` / DIGCTL MPTE — SPAN(26:24)/LOC(11:0)/DIS(31) のデコードと
  syscon 側の説明が一致。DFLPT の fixed PTE 2048（0x80000C12、AP/DOMAIN/B のみ書き込み可）も
  RM 記述と整合。
- `hw/misc/mxs_etm.c` / `hw/misc/mxs_rob.c` / `hw/misc/mxs_reserved.c` — リセット値配列の
  サイズと `.nwords` が一致（hsadc 12, spdif 7, dram 190, can 608, enet 418, swi 8192,
  audioout 1）。RO マスク・word/bank 両スタイルの使い分けも正しい。
- `hw/gpio/mxs_pinctrl.c` — IRQSTAT/IRQEN/PIN2IRQ の三重 AND でバンク割り込みを駆動、
  edge/level 両対応、PIN2IRQ クリア時の IRQ 再評価。BSP 逆アセンブル由来のコメントと整合。
- `hw/audio/sgtl5000.c` — 偶数アドレス 16 ビットレジスタ（`regs[reg >> 1]`）、
  `reg > SGTL_MAX_REG` ガード、defaults テーブルは全エントリ 0x013a 未満。不整合なし。

## 9. 追加修正（第6弾）: 音声 codec（brain 実機 = BU26154）

ユーザー指摘の確認: brain マシンの実機 codec は **LAPIS/ROHM BU26154MUV**（CE レジストリ
WaveDev = `wavedev2_BU26154.dll`、I2C0 @ 0x1a）。SGTL5000（0x0a）は Linux/Brainux DTS 用の
別構成で、マシンの既定 wiring は既に `bu26154`（`brain_codec_select()` のデフォルト）。
前回の audit で SGTL5000 側だけ読んでいたのを是正し、今回は実機側の `bu26154.c` 全文
（1330 行）を audit した。

| # | 対象 | 不具合 | 修正 |
|---|---|---|---|
| 17 | `hw/audio/bu26154.c` `bu26154_in_cb()` | ホスト音声バックエンドからの ADC 取り込みループが、**書き込みポインタをループ内で進めずに `in_len` から毎回同じスロットを計算**していた（`wp = (in_start + in_len) % BU_RING` がループ中不変）。1 回の `audio_be_read()` で得たチャンク全体が **1 スロットに潰れて**書き込まれ、`in_len` だけ `got` 分増えるため、再生側には「実データ 1 バイト + (got-1) バイトのゴミ」が流れた | ループ変数 `i` を使って `(in_start + in_len + i) % BU_RING` に各バイトを書き込む |
| 18 | `hw/audio/sgtl5000.c` `sgtl5000_in_cb()` | 上記と**同一のバグ**（同じ実装を流用）。SGTL5000 は `-machine … codec=sgtl5000` で選択可能なため残存する構成であり、同様に修正 | 同修正 |

### 追加: SGTL5000 への陳腐化した参照の是正（brain 実機は BU26154）

- `hw/arm/mxs.c` — `BrainMachineState.sgtl5000` フィールド名が誤解を招くため
  `codec_dev` に改名（中身は `brain_codec_select()` が選んだ codec。既定 BU26154）。
  `hmp_brain_i2c`/`brain_micfill`/`brain_sgtl` と `brain_init()` の参照も追随。
- `hw/misc/mxs_i2c.c` — struct と realize のコメントが「board codec = SGTL5000 @ 0x0a」
  と誤記していたのを「BU26154 @ 0x1a（Brain）／ SGTL5000 @ 0x0a（Linux-DTS 構成）」に是正。
- `include/hw/arm/mxs.h` — `mxs_i2c_codec_device()` のコメントを「BU26154 on the Brain,
  or SGTL5000 for the Linux-DTS wiring」に是正。
- `include/hw/arm/mxs_saif.h` — 「Link a SAIF to the board's SGTL5000 codec」という
  コメントを、実機 BU26154 既定・型によるディスパッチ（`mxs_saif.c` の
  `object_dynamic_cast(TYPE_BU26154/TYPE_SGTL5000)`）の説明に是正。

### bu26154.c を全文 audit して「正常」と判断した箇所

- I2C プロトコル: 8bit レジスタインデックス、even=read / odd=write アドレス、
  連続転送でインデックス +2、START/FINISH での `want_idx` 管理 — データシート記述と一致。
- MAPCON（0x1c/0x1d）グローバル選択、0x3 禁止（p.48）を拒否。SOFTRST（0x11 bit0）は
  CPU インターフェース＋自レジスタのみリセットでレジスタファイルは維持（p.46）。
- OSRSEL 0x3 禁止、RECPLAY の「stop を経由しない状態遷移禁止」と MCTIME 充電窓
  （40/fs + 128/fs/step）— いずれも実装済み。
- ゲイン則: PDATT/RDVOL/Effect 共通 0.5dB 減衰則（0x00..0x6E 禁止→mute、0x6F mute、
  0x70..0xFF −71.5..0dB）、AVVOL[5:0]（0x00 mute, 0x01..0x09 −28..−2dB, 0x0a 0dB,
  0x0b..0x19 +2..+18dB, 0x1a.. 未定義→+18dB クランプ）、PGAATT 0/−9dB、MINVOL 6..27dB。
- 電源/クロックゲート: VMIDCON、DACPW(DACREN|DACLEN)、AINPW(ADCEN/PGAEN/PGAATT)、
  AREFPW(MICBEN)、CLKEN/CLKIO の PLLOE+PLLEN/MCLKEN 規則、MCTIME 窓、SPPW b02 ハード固定。
- データ経路: DAC は mono（L+R 平均）→ PDATT→Effect→AVVOL の Q15 ゲイン連鎖を適用して
  stereo S16LE フレームでステージング、SAIF 側へ 48 kHz でプッシュ。ADC は host 入力→
  PGA→MICVOL→RDVOL のゲイン連鎖、mic bias/amp 停止時・DVMUTE 時は無音。整合。
- vmstate は regs/map/cur_idx/want_idx/リング状態を保持、post_load で dac_on/adc_on を
  レジスタファイルから再計算。正しいパターン。
