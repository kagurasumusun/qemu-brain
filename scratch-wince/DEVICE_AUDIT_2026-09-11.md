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
