#!/usr/bin/env python3
"""Init ハイブ解析 (最終版): バイト列パターン (文字+00 の LE ペア) で
値名/キー名を検索し、Nand0X キーの Region/Order/Index を抽出する。"""
import struct, re

nk = open('/home/user/dist/nk.bin','rb').read()
base = 0x1bf0000
area = nk[base:0x1c03000]

def u16(s):
    """'abc' -> b'a\\x00b\\x00c\\x00' (観測された LE 文字ペア形式)"""
    out = bytearray()
    for ch in s:
        out += ch.encode('utf-16le')
    return bytes(out)

def find_all(hay, pat):
    res = []
    i = 0
    while True:
        j = hay.find(pat, i)
        if j < 0:
            break
        res.append(j)
        i = j + 1
    return res

# 1) キー名 "Drivers\BuiltIn\Nand0X" (と NandDic) の位置
keypos = []  # (name, rel_offset)
for name in ['Nand00','Nand01','Nand02','Nand03','Nand04','Nand05','Nand06','Nand07','Nand08','NandDic']:
    pat = u16('Drivers\\BuiltIn\\' + name)
    for j in find_all(area, pat):
        keypos.append((name, j))
keypos.sort(key=lambda x: x[1])

# 2) 各キーについて、そのキー名の直後〜次のキー名の前の範囲で値を抽出
VALNAMES = ['Region', 'Order', 'Index', 'Prefix', 'Dll', 'Profile', 'CFTimeOut']

def read_data(after, vname):
    """値名+終端NUL の直後のバイト列からデータを読む。"""
    if vname in ('Region', 'Order', 'Index', 'CFTimeOut'):
        if len(after) >= 4:
            d = struct.unpack_from('<I', after, 0)[0]
            return d if d < 0x10000 else None
        return None
    # 文字列: (文字,00) ペアを読む
    out = []
    i = 0
    while i + 1 < len(after) and after[i] != 0:
        out.append(chr(after[i]))
        i += 2
    return ''.join(out) if out else None

print(f"{'key':10s} {'Region':>7s} {'Order':>6s} {'Index':>6s} {'CFTimeOut':>9s}  Dll / Profile")
print("-" * 80)
results = {}
for idx, (name, pos) in enumerate(keypos):
    end = keypos[idx+1][1] if idx+1 < len(keypos) else len(area)
    chunk = area[pos:end]
    vals = {}
    for vname in VALNAMES:
        j = chunk.find(u16(vname) + b'\x00\x00')
        if j < 0:
            continue
        after = chunk[j + len(u16(vname)) + 2:]
        v = read_data(after, vname)
        if v is not None:
            vals[vname] = v
    results[name] = vals
    print(f"{name:10s} {str(vals.get('Region')):>7s} {str(vals.get('Order')):>6s} "
          f"{str(vals.get('Index')):>6s} {str(vals.get('CFTimeOut')):>9s}  "
          f"{vals.get('Dll','')} / {vals.get('Profile','')}")

print()
print("== シリアル実測: DSK_Init 呼び出し (Order 昇順) ==")
serial = [('Region=6','DSK3'), ('Region=16','DSK2'), ('Region=4','DSK6'),
          ('Region=1','DSK5'), ('Region=0','DSK8'), ('Region=2','DSK9'), ('Region=3','DSK7')]
for r, dsk in serial:
    print(f"  {r} -> {dsk}")

print()
print("== Index(=DSK番号) での対応 ==")
for name, vals in sorted(results.items(), key=lambda kv: kv[1].get('Index', 999)):
    print(f"  {name}: Index={vals.get('Index')} Region={vals.get('Region')} "
          f"Order={vals.get('Order')} Profile={vals.get('Profile')}")
