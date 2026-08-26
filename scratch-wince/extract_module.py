#!/usr/bin/env python3
"""DiagOS NK から XIP モジュールを抽出する"""
import struct, sys

def main(nk_path, module_name, out_path):
    data = open(nk_path, "rb").read()
    physfirst = 0x80200000
    physlast = 0x80d9437c
    romoff = 0xb92cf4
    nummods = struct.unpack_from('<I', data, romoff+16)[0]
    for i in range(nummods):
        te = data[romoff+84+i*32: romoff+84+i*32+32]
        lpszName, e32off, o32off, loadoff = struct.unpack_from('<IIII', te, 16)
        noff = lpszName - physfirst
        name = data[noff:data.find(b'\x00', noff)].decode('latin1','replace')
        if name.lower() != module_name.lower():
            continue
        e32f = e32off - physfirst
        objcnt, entryrva = struct.unpack_from('<HI', data, e32f)
        vbase, _ = struct.unpack_from('<II', data, e32f+8)
        vsize = struct.unpack_from('<I', data, e32f+20)[0]
        secs = []
        o32f = o32off - physfirst
        for j in range(objcnt):
            osz, orva, opsz, odptr, oreal, oflags = struct.unpack_from('<IIIIII', data, o32f+j*24)
            foff = odptr - physfirst if physfirst <= odptr < physlast else 0
            secs.append((orva, osz, foff))
        img = bytearray(vsize)
        for orva, osz, foff in secs:
            if osz <= 0: continue
            if orva + osz > len(img): continue
            img[orva:orva+osz] = data[foff:foff+osz]
        open(out_path, "wb").write(bytes(img))
        print(f"{name}: vbase=0x{vbase:08x} vsize=0x{vsize:x} -> {out_path}")
        return
    print(f"{module_name}: not found")

if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2], sys.argv[3])
