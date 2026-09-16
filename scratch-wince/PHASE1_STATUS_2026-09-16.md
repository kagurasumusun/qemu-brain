# Phase 1 現状確認報告 (2026-09-16)

qemu-brain の現状調査・ビルド・実 eMMC イメージによるテスト起動の結果。
ソースの機能変更は行っていない (Phase 1 制約)。ビルド環境の整備と
検証ログの取得のみ実施。

## 1. 対象・基準

- リポジトリ: `kagurasumusun/qemu-brain` ブランチ `arena/01a08cb7-qemu-brain`
- HEAD: `597b513 Add files via upload` (作業開始時点、未コミット変更なし)
- ベース: QEMU 11.0.3 + i.MX28 (ARM926EJ-S) / SHARP Brain (PW-AJ2, PW-SH6系)
  マシン定義 `-M brain`、RAM 128 MiB
- 使用イメージ: `kagurasumusun/wince` リリース **`emmc-setup-2026-08-26`**
(「emmc repaired4 setup-boot (QEMU)」。5 分割 6,605,935,104 bytes を結合。
結合直後の sha256: `893c76af78b3ba91b43c99c9826d9eff17298f64416f7549c4a9fbcb860eb33a`)

## 2. 作業環境

- 2 vCPU / 1.9 GiB RAM / 25 GB ディスク (Debian 13 trixie, gcc 14.2)
- Python 3.13: pip で meson 1.12.0 / ninja 1.13.2 を導入
- apt で `libfdt-dev`, `device-tree-compiler` を導入 (QEMU の FDT 依存。
  前セッションの監査メモでは apt 不可だったが本環境では deb.debian.org 到達可)
- glib 2.84.4 / pixman 0.44.0 / zlib 1.3.1 / gnutls 3.8.9 は既存

## 3. 既存実装の調査結果 (ソース・ドキュメント)

### マシン構成 (`hw/arm/mxs.c` 他)

- CPU: arm926 (ARM926EJ-S) 固定、1 コア。DRAM 128 MiB @0x40000000。
  OCRAM 128 KiB @0 (0xa8000000 にエイリアス)、ブート ROM 窓 @0xC0000000。
- ブート ROM モデルは i.MX28 ROM の SB ブートストリーム処理を再実装:
  MBR → type 0x53 パーティション → BCB (magic 0x00112233, drive/tag/sector/count)
  → SB ヘッダ ("STMP" v1.1, DEK 復号) → LOAD/CALL 列を辿り、
  既定 `boot-mode=eboot` で EBOOT (0x40060000) に直入り
  (`full` は XLDR→EBOOT チェーン+トランポリン)。
- 実装済み周辺 (監査ドキュメント §2 由来 + Kconfig/meson 確認):
  icoll / pinctrl / apbh / apbx / ssp(1..3?)/ timrot / rtc / pwm / i2c /
  lradc(タッチ) / lcdif / pxp / gpmi / bch / dcp / perfmon / syscon(digctl) /
  dflpt / etm / rob / reserved(hsadc,spdif,dram,can,enet,swi,audioout) /
  saif(0,1) / clkctrl / power / ocotp / auart / duart(PL011+SERTRACE) /
  usbctrl / mxs_sd (sd.c 改造含む) / brain_kbd (キーマトリクス+MRSensor+
  タッチキー) / EDNA2 mailbox・doorbell / bu26154 (実機 codec, I2C0@0x1a) /
  sgtl5000 (0x0a, Linux 構成用) / brain_stats イベントリング /
  reg-log 系 (reg-log, reg-log-ring, reg-log-tick) / mxs_fat (SD 読み)。
- マシンプロパティ: codec / boot-mode(eboot|full) / rom-verbose / reg-log /
  strict-hw / gpmi-nand(+file) / aid-* / reg-log-tick / reg-log-ring /
  lcd-width/height/rotate。
- eMMC は `-drive if=sd,file=...,format=raw` (index 0)、microSD は index 1。
- 既知の見送り事項 (監査記録): ssp vmstate 非完備 (migration 未対応)、
  dflpt/gpmi の write バイトレーン無視 (read と非対称・32bit access では実害なし)、
  ICOLL_STAT アイドル値 0x7f 根拠未確認、apbh/apbx SFTRST 時チャネル状態。

### ドキュメント

- `AGENTS.md`: 最終目的 7 項目 (1:1 完全仮想化 / エラー無し起動 / ログ正常 /
  タッチ完全動作 / モデル仕様一致 / スタブ排除 / 全ドライバ対応)。
- `DEVICE_AUDIT_2026-09-11.md`: 周辺モデル横断監査 + 修正 18 件の記録。
- `TOUCH_PANEL_AUDIT.md`: タッチ座標パイプライン照合 (プレート定数は
  ドライバと一致と結論。picture-box 追跡が実質デッドコード等の指摘あり)。
- `PINCTRL_MAP_ANALYSIS.md`: 実機 pinctrl マップ確定値 / keybd_EDNA2
  7x7 マトリクス (PS/2 Set1)。
- `brainのキーコード.txt`: キーコード資料。

## 4. eMMC イメージ構造 (`emmc_setup_2026-08-26` 結合物・観測)

- MBR: 55aa 正常。part0 FAT32(0x0b,boot flag) LBA 1624073 × 11278144
  (≒5.4 GiB 辞書領域) / part1 type 0x53 LBA 256 × 1024 (SB ブート領域) /
  part2 type 0x10 LBA 2304 × 65536 (32 MiB) / part3 FAT16(0x0e) LBA 598031
  × 1026033 (≒501 MiB)。
- 0x53 領域: 先頭に BCB (drive 0 tag 1 sector 260 count 335)。
  SB イメージ (STMP v1.1, 171,440 bytes, 1 key) が +0x800 / +0x80800 に二重化。
- sec2 BootConfig: "SHARP E-DICTIONARY BOOT CONFIG", flags=0,
  sum16=0x0a2e / nsum=0xf5d1 (repaired4 記載値と一致)。
- sec16 FactorySetting 先頭: `80 01 00 00 37 30 ...` (repaired4 記載パターン相当)。
- Nand2 (FAT16, sector 0x27800 起): 辞書メタ `.BOX` 群・SYSTEM.BOX 等が
  正常に列挙可能。イベントログ領域 (sector 0x27000 起、"LOG1" マジック) に
  前回記録 (VMC_Flush(2002/10/2 ...) 等) が残存。

## 5. ビルド結果

構成:

```
../configure --target-list=arm-softmmu \
  --disable-docs --disable-guest-agent --disable-user \
  --disable-sdl --disable-gtk --disable-vnc \
  --disable-slirp --disable-capstone --disable-libnfs --disable-curl \
  --disable-glusterfs --disable-tpm --disable-vte --disable-brlapi \
  --disable-spice --disable-opengl --disable-virglrenderer \
  --disable-xen --disable-kvm --disable-werror
```

- 結果: **成功** (`qemu-system-arm` ≈ 98 MiB。qemu-img/qemu-io も確認)。
- 注意点: ソースツリー直下の `pyvenv/meson.build` は QEMU 11 の追跡対象
  ファイル。誤って消すと configure が `subdir('pyvenv')` で失敗する
  (今回 1 度消去→`git checkout -- pyvenv` で復旧)。
- 監査メモにあった「ビルド未実施」状態を解消した。

## 6. テスト起動結果 (実施済み・観測事実)

起動コマンド例 (初回は `-snapshot` 付きで検証、その後 raw 直書きでも確認):

```
./qemu-system-arm -M brain,rom-verbose=on -m 128M \
  -drive if=sd,file=emmc_setup.img,format=raw \
  -display none -serial stdio   # 他: -monitor unix:... で HMP 操作
```

### ブートチェーン (serial ログ実測)

1. QEMU mxs-rom: BCB 解析 → SB v1.1 復号 (DEK) → LOAD×4/CALL×2 →
   EBOOT 0x40060000 起動 (rom-verbose ログで確認)。
2. EBOOT: WinCE Bootloader Common Library 1.4 → SDMMC_Init
   (MMC High Density / 8bit) → OEMLaunch → NK 0x40200000。
3. WinCE 6.0 カーネル (Built May 7 2012): OEMInit → RTC/PM/USB/I2C/
   TPDriver/Backlight/MRSensor/SDHC/PXP/DSK(FMD)/表示
   (LCDIF PixFreq=240000, VMem 8 MiB)/EDNA2 キーボード/LayMgr/touch
   attach まで一連のスレッド起動。`Debug UART disable` 後は仕様通り静粛化。
   ハング・リセット・クラッシュの兆候なく 150 秒 + α 稼働継続
   (VM status: running)。

### 画面 (HMP screendump 実測)

- 854×480 (回転後ランドスケープ)。 **PW-AJ2 初回セットアップ
  「日付と時刻の設定」画面** (20[00]年[01]月[01]日 / [12]時[00]分 /
  AM午前・PM午後 / 決定・戻る / 操作ヒント表示)。リリース記載の
  期待状態と一致。→ WinCE メイン OS の起動完了を画面レベルで確認。
- 表示の乱れ (要調査): 全面に水平ストライプ + 左端に大きな黒三角帯。
  実機との差異として Phase 2 で照合が必要 (screendump 固有か、
  LCDIF/PXP モデルの描画問題かは未切り分け)。

### ゲスト書き込み (グラウンドトゥルース)

- raw イメージ直接ブート後に pristine (リリース品 sha256 記録済) と
  全領域比較: **変更は 11 セクタのみ、全て Nand2 近傍 (78..80 MiB)**。
  暴走書き込みなし。
- ログ領域 (abs 0x4ec4000) に今回のブート由来の追記を確認:
  **`VMC_INIT(2000/1/1 0:00:13)` / `EdLogThread: START!` / `UE[A=0]` /
  `BE[L=0 1004000]`** — リリース記載の実機同等ログそのもの。
  併せて Nand2 FAT16 のディレクトリ項目更新 (タイムスタンプ) も観測。
  → ストレージ読み書き・FMD/NAND リージョン経路は少なくとも
  ブート・初期化ログの記録まで実動作。

### 入力 (部分検証・要調査あり)

- brain_kbd は QemuInputHandler 登録済み。HMP `sendkey` で
  `[brain-kbd] event qcode=.. cell=r,c down=..` が発火し、
  マトリクス座標へのマッピング自体は動作。
- ただし BRAIN_KBD_DEBUG の `refresh` ログ上、ゲストの行読み出し
  (`din2`/`din4`) は常時 all-high のままで、キー押下がゲストから
  読み取れる形に現れない。画面ピクセルも送 key 前後で不変。
  キースキャンループ (列個別駆動→7f アイドル) はゲスト側で稼働。
  → **ゲストにキー入力が到達しない状態** (モデル側の行読み出し経路・
  タイミングの問題の疑い。要 Phase 2 調査)。
- タッチ: マウス #1 として「MXS LRADC touchscreen (absolute)」登録済み。
  HMP mouse_move/mouse_button での画面変化は今回未観測 (要調査)。

## 7. 問題点・分類 (起動ログの ERROR 行)

| 症状 | 段階 | 暫定分類 | 備考 |
|---|---|---|---|
| `Failed to read MBR from SDHC` / `g_bSDHCExist[0]=1 [1]=0` | EBOOT SDMMC_Init | 仕様確認中 | SD 未装着時の実機挙動と要照合 |
| `LoadBootCFG: failed to load configuration` / `flash initialization failed - loading bootloader defaults` | EBOOT | 実機一致の可能性 (要照合) | sec2 BootConfig は存在。毎回セットアップ画面に行く実機挙動と整合する可能性 |
| `ERROR: VMCopy ... sub2.c line 587: Failed in createFile() for PMI1:` | カーネル | 要調査 | VMCopy ドライバの PMI1 生成失敗 |
| `FMD Init Get Region Error` (Region 6=DSK3 / Region 16=DSK2 / Region count Region 1=DSK5) | カーネル DSK_Init | 仕様確認中 | 現レイアウトに無い領域の問い合わせの可能性 |
| `GetBlockSize: Failed to get the alignment value [Error:0x2]` / `GetOverlayAlign: fails, size is 8!` / `GetDisplayGuid: ` (空) | カーネル 表示系 | 要調査 | 既定フォールバックあり。実挙動との照合要 |
| RTC が 2000/1/1 にリセットされる | RTC モデル | 要調査 (軽微) | 画面の 20[00] 年と一致。実機は保持されるか |
| 画面ストライプ・左端黒三角 | 表示 | 要調査 | 上記 |
| キー入力がゲストに届かない | brain_kbd/入力 | **要対応 (Phase 2 優先)** | 上記 |
| タッチの画面反応未確認 | lradc/入力 | 要調査 (Phase 2) | 上記 |

## 8. 現在地まとめ (完成度分類)

- ブートチェーン (ROM→SB→EBOOT→NK→メイン OS セットアップ画面): **実装済み・
  QEMU 起動確認済み** (画面 + ゲストログ + 書き込みの 3 面で実測)
- ストレージ (eMMC 読み書き・Nand リージョン): 実装済み・本 Phase で
  読み書き実測。領域問い合わせ系エラーは仕様確認中。
- 表示 (LCDIF/PXP): 初期化・フレーム出力まで動作 (観測)。
  画質の乱れは要調査。
- シリアル (DUART/auart): 動作確認済み (全ブートログ取得)。
- キーボード入力: 部分実装の疑い (イベント受領まで。ゲスト到達確認が取れない)。
- タッチ入力: 検証未完了 (マウス登録は確認)。
- RTC/電源/PM/I2C/codec: ドライバ初期化ログベースでは起動。継続検証は今後。
- 既知の残件 (監査記録): vmstate 系、dflpt/gpmi バイトレーンなど。

## 9. 次 Phase の作業 (優先順位順)

1. **キー入力がゲストに届かない問題の解析・修正** (brain_kbd refresh 経路、
   行読み出し・列極性・タイミング。セッタップ画面操作には必須)。
2. タッチ入力の end-to-end 検証 (HMP/将来的には SDL/VNC 付きビルドでの
   双方向確認) 。
3. 画面表示の乱れ (水平ストライプ・左端黒帯) の切り分け
   (screendump 固有か LCDIF/PXP モデルか。実機画面情報と照合)。
4. EBOOT 系エラー (`LoadBootCFG`/`SDHC MBR`) が実機でも出るかの照合
   (edsh6-tools / DiagOS ログと比較)。
5. VMCopy `PMI1:` createFile 失敗・`GetBlockSize`/`GetOverlayAlign`/
   `GetDisplayGuid` 空の原因特定 (BSP ドライバ期待値との照合)。
6. RTC 初期値 (2000/1/1) の実機挙動確認。
7. セットアップ画面の操作 (日時設定→決定) による WinCE 通常画面
   (ホーム/辞書アプリ) への到達確認 — 入力修正後に実施。

## 10. 環境メモ (復元用)

- eMMC 取得: リリース 5 パートを GitHub asset API
  (`/repos/.../releases/assets/{id}`, `Accept: application/octet-stream`) から
  curl で DL → cat 結合 (browser_download_url の直 GET では 404 になる事象あり)。
- `curl --netrc` は引数を取らない (取ると URL として解釈される)。
  明示する場合は `--netrc-file` を使う。
- `qemu-img dd` (本ビルド) は範囲指定で 0 バイト出力になる不具合を観測
  (skip 指定が効かない)。領域切り出しはホスト `dd` を使用。
- ビルド生成物は `build/` (スナップショット対象外)。ソースは Git 管理済みの
  ため再現手順は本書 §5 + libfdt-dev 導入で復元可能。
