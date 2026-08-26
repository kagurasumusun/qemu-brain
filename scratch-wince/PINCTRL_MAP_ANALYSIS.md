# 実機 pinctrl レジスタマップ確定 (2026-08-12)

## 方法

SHARP Brain (i.MX28 / WinCE6) の DiagOS から `cspddk.dll` (vbase 0xc0640000) を
抽出し、GPIO ドライバ関数を逆アセンブルして実機のレジスタマップを確定した。
Linux ドライバ (arch/arm/mach-mx28/gpio.c, gpio-mxs.c) の定義とも一致。

対象エクスポート (cspddk.dll):
- DDKGpioConfig          RVA 0x4330
- DDKGpioWriteDataPin    RVA 0x49d0
- DDKGpioReadDataPin     RVA 0x4af8
- DDKGpioReadIntrPin     RVA 0x4ba8
- DDKGpioClearIntrPin    RVA 0x4c2c
- DDKGpioEnableDataPin   RVA 0x48a8

## 確定したレジスタマップ

各機能レジスタは 0x10 バイト間隔で 5 バンク (bank*0x10)、
SET/CLR/TOG は +0x4/+0x8/+0xC (MXS_BANK 形式)。

| 機能 | ベース | bank 間隔 | SET(+4) | CLR(+8) | ビット幅 |
|---|---|---|---|---|---|
| CTRL    | 0x000  | —      | —    | —    | —    |
| MUXSEL  | 0x300  | 0x10   | 0x304 | 0x308 | 4bit/ピン |
| DRIVE   | 0x600  | 0x10   | 0x604 | 0x608 | 2bit/ピン |
| DOUT    | 0x700  | 0x10   | 0x704 | 0x708 | 1bit/ピン |
| DIN     | 0x900  | 0x10   | —    | —    | 1bit/ピン (read) |
| DOE     | 0xB00  | 0x10   | 0xB04 | 0xB08 | 1bit/ピン |
| IRQEN   | 0x1000 | 0x10   | 0x1004 | 0x1008 | 1bit/ピン |
| PIN2IRQ | 0x1100 | 0x10   | 0x1104 | 0x1108 | 1bit/ピン |
| IRQLEVEL| 0x1200 | 0x10   | 0x1204 | 0x1208 | 1bit/ピン |
| IRQPOL  | 0x1300 | 0x10   | 0x1304 | 0x1308 | 1bit/ピン |
| IRQSTAT | 0x1400 | 0x10   | 0x1404 | 0x1408 | 1bit/ピン (read 0x1400) |

## DDKGpioConfig のフラグ (r1)

- bit0: DOE (出力許可) — 1: SET (出力), 0: CLR (入力)
- bit1: IRQEN (割り込み許可) — 1: SET + IRQSTAT CLR, 0: CLR + PIN2IRQ CLR
- bit3: IRQLEVEL (0x1200) — レベル/エッジ
- bit4: IRQPOL (0x1300) — 極性 (0: アクティブロー, 1: アクティブハイ)

## ピン番号

- pin = bank*32 + bit  (0-223)
- DDKGpio* は pin > 0xE0 (224) をエラーとする

## keybd_EDNA2 のキーマトリクス (逆アセンブルで確定)

keybd_EDNA2.dll (0xc08a0000):
- 列 (出力, 駆動): GPIO4 ピン 0,1,2,3,4,6,7
- 行 (入力, 読み): GPIO2 ピン 16,17,18,19,20,21 (行0-5), GPIO4 ピン 8 (行6)
- キーマップ: 7x7 = PS/2 Set 1 スキャンコード (0xc08a2cc0, 49 バイト)
- スキャン: 全列駆動 (アイドル/行検出) + 列を 1 本ずつ駆動 (列特定)
- 行ピンの変化で GPIO 割り込み (IRQEN[2] ピン16-21, IRQEN[4] ピン8, レベル+アクティブロー)

## 既存 QEMU モデルとの差分 (修正前)

既存 mxs_pinctrl.c は Linux mxs pinctrl オフセット (0x70/0x90/0xB0/0xF0/0x150) を
使用し、WinCE BSP のアクセス (0x300/0x600/0x700/0x900/0xB00/0x1000/0x1400 系) を
すべて無視していた。GPIO 入出力/割り込みが機能せず、タッチ/キーボード非動作の
根本原因。
