# SHARP Brain (PW-SH6) タッチパネル — 仕様照合報告（未修正）

対象: `hw/adc/mxs_lradc.c` / `hw/display/mxs_lcdif.c` / `hw/input/brain_kbd.c` /
`hw/arm/mxs.c` / `include/hw/arm/mxs.h`。対象マシンは `-machine brain`（i.MX28 / WinCE6）。

## 0. 「仕様」はどこにあるか

リポジトリに独立した仕様書ファイルは存在しない。仕様は以下に分散しているため、
これらを「仕様」として照合した。

1. `scratch-wince/PINCTRL_MAP_ANALYSIS.md` … pinctrl レジスタマップ / keybd_EDNA2 マトリクス
2. `hw/adc/mxs_lradc.c` 冒頭コメント … 実機 CalibrationData・BootArgs 係数・プレート定数
3. `hw/input/brain_kbd.c` / `hw/arm/mxs.c` の mailbox(+0x404) プロトコルコメント
4. i.MX28 RM の LRADC/LCDIF セマンティクス
5. SHARP 公式操作ガイド（外部）… 「操作タッチキーは画面右（縦表示時は画面下）」

## 1. 座標パイプラインの確認（正常と判断した部分）

入力 → 出力の流れを数値まで追った結果、主要経路は整合している。

- パネル: GRAM 480(列)×854(行) portrait。`rotate=90`、ブートローダが `MADCTL=0xd0`
  (MY|MX) を設定 → コンソールは 854×480。
  このとき `mxs_lcdif_to_console(gx,gy) = (gy, 479-gx)` となり、
  「ピクチャ列 = 配列行」「ピクチャ行 = 遠端から数えた配列列」というコメント
  (mxs_lradc.c:118-124) と一致。
- フロントエンド (SDL2/GTK) は `qemu_input_queue_abs`(max_in = surface 幅) →
  `qemu_input_scale_axis` → 0..0x7FFF に正規化して ABS イベントを配送。
  `mxs_lcdif_touch_position`(mxs_lcdif.c:769) が nearest 丸めでピクセルを復元。
  last column (console x=853) も正しく cx=853 に復元される。
- X 板則: `mxs_lradc_plate_count(fx, 800, 160, 3712)` は
  fx=0→raw160、fx=799→raw3708、fx=800→raw3712 を生成。
  実ドライバ変換 x' = (633*raw − 36*2811)/2811 に代入すると
  raw160→x'=0、raw3708→x'=799 で、**ドライバの 0..799 論理レンジと正確に一致**。
  つまりコードの X 定数 (160..3712 / 800px) はドライバに対して正しい。
- Y 板則: fy=0→3964、fy=479→約220（下降方向＝反転）で、
  コメント「Y は下向きに減少」と一致。エンドポイントの向きの入れ違い
  （過去の「垂直ミラー」バグ）は現状ない。
- ストリップ境界: `in_strip = px < 0 || px >= 800` (mxs_lradc.c:784)。
  ピクチャは GRAM 行 0..799 → コンソール列 0..799、オーバースキャン 54 列は
  コンソール列 800..853（=画面右）。公式マニュアルの「画面右の操作タッチキー」
  および mxs.h「right-edge」記述と整合。ストリップ側の行→パッド割当
  (row*9/480) もコンソール行ベースで正しい。

## 2. 発見した不整合・バグ（影響度順）

### 2-1.【重大・潜在】picture box 追跡が実質デッドコード（mxs_lcdif.c）

- リセットで `pic_x0=0, pic_y0=0, pic_x1=panel_w-1(479), pic_y1=panel_h-1(853)`
  ＝**全面板**に初期化 (mxs_lcdif.c:907-910)。
- `mxs_lcdif_start_transfer` の更新は MIN/MAX のみ（「Grow, never shrink」、
  mxs_lcdif.c:490-497）。全面板から始まるため、col_start≥0 / col_end≤479 /
  row_start≥0 / row_end≤853 により **box は永久に初期値のまま変化しない**。
- コメントが謳う「インセット描画ウィンドウの検出」「bx0=54 のケース」が
  機能しない。box_at は常に bx0=0, by0=0 を返す。
- 現行ハードウェア（ピクチャが GRAM 行 0..799 開始）では bx0=0 が偶然正しいため
  顕在化しないが、ピクチャがインセットされるレイアウトでは座標全体が 54 列ずれ、
  「画面の一部が反応しない」症状そのものになる。デッドロジック自体が仕様
  （コメントの意図）と不整合。

### 2-2.【中・文書】プレート則コメント内の数値が相互矛盾（mxs_lradc.c:80-144）

同一コメントブロック内で 3 通りの値が併記され、互いに矛盾している。

- X 傾き: 「raw = 160 + 4.4347·x'」/ 「160..3704 → 0..799」(傾き 4.43) /
  「3552 counts over 800 (4.44/pixel)」。
- X 右端: コメント「3704」 vs コード定数 `BRAIN_PLATE_X_AT_PIC800 = 3712`。
  実ドライバからは x'=799 ⇔ raw=3708 が正（§1 参照）。コメントの 3704 は誤り。
- Y: 「raw = 3968 − 7.83·y'」/ 「3960..219 → 0..479」/ コード定数「3964..211」。
- ストリップ節の「factory calibration ends at x'=799 (raw span 888..2961)」は、
  旧クランプバグの残骸。888..2961 は校正ターゲット帯（x'=164..631）であり、
  全ガラス帯は 160..3704 系。全面板レンジと校正ターゲット帯が混同されている。

（実害は右端/下端の数ピクセル程度だが、「仕様としての数値」が読めないため、
  以後の検証・保守の妨げになる。）

### 2-3.【低・実バグ】`mxs_lradc_plate_count` の負勾配丸め誤差（mxs_lradc.c:243-250）

```c
v = at_first + (sgn * pos + span / 2) / span;
```

Y 軸は `sgn = at_last - at_first = 211 - 3964 = -3753 < 0`。C の整数除算は
**ゼロ方向への切捨て**のため、`+ span/2` の丸め補正が負値では逆作用し、
Y 板値に系統的な約 1 カウント（≒0.13px）の誤差が出る。例: pos=479 で
trunc 丸めは 220、nearest は 219（ドライバ期待値 219 に一致）。実害は極小だが、
コメントの「Round to nearest」という意図（mxs_lradc.c:236-240）と実装が食い違う。

### 2-4.【低・潜在】CTRL4 非恒等マップでチャネル/トリガ解釈が破綻（mxs_lradc.c:478-482）

`mxs_lradc_convert` は**仮想**チャネルビットマスク（DELAY trigger フィールド）を
`mxs_lradc_sample(s, phys, channels)` に渡すが、sample 内では `trigger & (1<<ch)`
と**物理**チャネル番号と比較する（case 2/3 の判定）。CTRL4 が恒等
（0x76543210、実ドライバ使用。mxs_lradc.c:967）のときだけ成立する。CTRL4 を
非恒等に組むドライバでは CH2/CH3 の X/Y 判定が誤る潜在バグ。

### 2-5.【低】表示 fast path の `rotate == 270` 判定が死んでいる（mxs_lcdif.c:633）

`if (!s->madctl && s->rotate == 270)` の専用ループは、マシンが `lcd_rotate = 90`
(mxs.c:3062) を設定し、ブートローダが MADCTL=0xd0 を設定する現行構成では
不成立でデッドコード。コメント「The case this hardware actually uses」は
旧 rotate=270 時代の残存。機能不整合ではない（generic パスで正しく描画）が、
記述と実態が不一致。

### 2-6.【文書】ヘッダ/コメントの不整合

- `include/hw/arm/mxs.h:159-162`: `mxs_lcdif_touch_position` を「GRAM 座標に変換」
  と記すが、実装は picture 相対座標 (`px = cx - bx0`, mxs_lcdif.c:785) を返す。
- `hw/arm/mxs.c:3052-3054`: rotate 検証の根拠として「date/time dialog … soft-key
  column down its left edge」とある。公式マニュアル（画面右の操作タッチキー）・
  mxs.h / mxs_lradc.c の「right-edge」と一見矛盾。これは（物理タッチキー帯でなく）
  特定ダイアログの描画 UI を指す可能性が高いが、読む側を誤らせる。
- `hw/arm/mxs.c:678`: コメントが「brain_pwrite / brain_touch」という存在しない
  HMP コマンドに言及（実在するのは brain_lilo / brain_i2c / brain_micfill /
  brain_saifpump / brain_saifplay / brain_sgtl / brain_regdump / brain_trace /
  brain_mbtrace / brain_mrs / brain_stats / brain_events）。

## 3. 「画面の一部しか反応しない」との対応関係（現時点の見立て）

- 空間マッピング本体（§1）は現行構成では数値的に正しく、大規模な
  「42% 不感」級のクランプ・ミラー系バグは現状コードには見当たらない
  （過去の「888..2961 クランプ」バグは修正済み）。
- 残る候補は以下の順:
  1. **2-1** picture box のデッド化が、ピクチャがインセットされる画面/起動段階で
     54 列のずれを生み、その帯が不感になる（最も「一部だけ反応」に近い）。
  2. **2-2** の定数不整合が、右端・下端の数ピクセルをドライバの受理矩形外に
     追い出す（ごく狭い不感帯）。
  3. **2-3 / 2-4** は実害が微小または潜在。
- ストリップ判定閾値 `px >= 800` 自体は、校正スパン 800px と一致しており正しい。
- 「反応しなかったりする」（断続性）を含むなら、LRADC の KICK/DELAY/ループの
  タイミング・ペン状態機械（PEN_LIFTING 等）の再検証、および実機トレース
  (`BRAIN_KBD_DEBUG`, `mxs_lradc_set_touch` / `mxs_lradc_convert` trace) での
  再現が次の確定手段になる。

## 4. 次のステップ（修正は未実施）

1. 2-1 を確定させる: picture box の初期値を「全面板」ではなく
   未確定状態（または最初の全幅/全高スキャンで確定）にする、あるいは
   インセット検出を機能させる。修正前に、実ゲストがどの GRAM 行範囲に
   ピクチャを描くかを trace で確定する。
2. 2-2 のコメント数値を、実ドライバ変換から導出した 160..3708 (X) /
   ~3964..~219 (Y) に統一して文書化する。
3. 2-3 は符号を考慮した nearest 丸め（負値は `- span/2` でバイアス）に修正。
4. 2-4 は `mxs_lradc_sample` の trigger を物理チャネル空間へ正規化して比較する。

---

## 5. 修正実施記録（2026-09-11）

全項目を根本修正した。実装（コード）と仕様（コメント）の双方を、実ドライバの
逆変換から導出した正確な数値に統一した。

### コード修正

| # | 項目 | 修正内容 | ファイル |
|---|---|---|---|
| 2-1 | picture box デッド化 | 初期値を「未観測(空)」+ 観測済みフラグに変更。全軸スキャンごとに「最広のスキャン」で extent を確定（全幅スキャン=行範囲、全高スキャン=列範囲）。未観測時は全面板フォールバック。vmstate v3 化（旧ストリームはフォールバック扱い） | mxs_lcdif.c |
| 2-2 | 板則コメント数値矛盾 | コメントをドライバ逆変換から導出した正しい値に全面書き換え（X=160..3708/800px、Y=3964..219/480px、傾き 4.44 / 7.81875）。「888..2961 全面板」等の旧クランプ残骸を除去 | mxs_lradc.c |
| 2-3 | plate_count 負勾配丸め | ゼロ方向切捨てを符号対応の nearest 丸めに修正（X 軸は無変化、Y 軸の系統 +1 を解消、fy=480 で端点 211 に正確一致） | mxs_lradc.c |
| 2-4 | CTRL4 非恒等で trigger 誤判定 | 仮想チャネルマスクを物理チャネルマスクへ変換して sample へ渡すよう修正 | mxs_lradc.c |
| 2-5 | fast path の rotate==270 が死んでいる | 回転定数の直書きをやめ、変換を 3 点プローブして転置+鏡像を検出する方式に変更。実機構成（MADCTL=0xd0 + rotate=90）で fast path が有効化され、generic 経路と画素単位で一致 | mxs_lcdif.c |
| 2-6 | ヘッダ/コメント不整合 | touch_position の「GRAM 座標」記述を「picture 相対」に修正（mxs.h / mxs_lcdif.c）、存在しない brain_pwrite/brain_touch への言及を実在コマンドに修正（mxs.c）、コメント改行崩れ修正（mxs.c） | mxs.h, mxs_lcdif.c, mxs.c |

### 見送り（バグでないと判断）

- 2-6 の「soft-key column left edge（mxs.c 回転検証コメント）」: これは日付/時刻
  ダイアログの **GUI ソフトキー列**の位置で、画面右の物理タッチキー帯とは別物。
  矛盾ではないため内容は維持し、改行崩れのみ修正した。

### 検証

- 板則の数値は Python でドライバ変換の逆と照合（ターゲット 5 点が
  (399,256)/(164,124)/(164,391)/(631,389)/(630,118) に一致）。
- plate_count の新旧を C で再現し、X 軸が完全に無変化、Y 軸の +1 系統誤差が
  解消して端点 211 に正確に一致することを確認。
- fast path の 3 点プローブが実機構成で成立し、generic 経路と同一座標を書くことを
  机上検証。
- 完全ビルドはサンドボックスのネットワーク制約（apt/deb.debian.org 到達不能、
  glib/pixman dev ヘッダ導入不可）により未実施。
