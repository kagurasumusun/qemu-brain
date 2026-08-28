# Source from MSYS2 GitHub Actions steps (MINGW64).
# MinGW Python/Meson uses Win32 CreateProcess:
#   /mingw64/bin/gcc.exe  -> WinError 2 (not a Windows path)
#   /usr/bin/cc           -> cc1 CreateProcess fail (-m64 vs MSYS gcc)
# Use cygpath -m (D:/.../gcc.exe) for --cc, and a real cc.exe PE file.
export PATH="/mingw64/bin:/usr/bin"
export CCACHE_DISABLE=1
unset CC CXX CCACHE_PATH
rm -f /mingw64/lib/ccache/bin/cc.exe /mingw64/lib/ccache/bin/gcc.exe \
      /mingw64/lib/ccache/bin/cc /mingw64/lib/ccache/bin/gcc
# Copy, do not symlink: ninja CreateProcess does not follow MSYS symlinks.
cp -f /mingw64/bin/gcc.exe /mingw64/bin/cc.exe
cp -f /mingw64/bin/g++.exe /mingw64/bin/c++.exe
hash -r
CI_CC=$(cygpath -m /mingw64/bin/gcc.exe)
CI_CXX=$(cygpath -m /mingw64/bin/g++.exe)
export CI_CC CI_CXX
