let
  moz_overlay = import (builtins.fetchTarball
    "https://github.com/mozilla/nixpkgs-mozilla/archive/9b11a87c0cc54e308fa83aac5b4ee1816d5418a2.tar.gz");
  nixpkgs = import <nixpkgs> { overlays = [ moz_overlay ]; };
in with nixpkgs;
let
  dlopen-libs = with xorg; [ vulkan-loader libX11 libXcursor libXrandr libXi libxkbcommon ];
in mkShell.override {
  stdenv = pkgs.stdenvAdapters.useMoldLinker pkgs.stdenv;
} {
  nativeBuildInputs = with pkgs; [
    rustChannels.stable.rust
    pkg-config
    zstd
    protobuf
  ];
  shellHook = ''
    export RUST_BACKTRACE=1
    export ZSTD_SYS_USE_PKG_CONFIG=1
    export SHADERC_LIB_DIR="${shaderc.static}/lib"
    export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:${lib.makeLibraryPath dlopen-libs}"
    #export VK_INSTANCE_LAYERS=VK_LAYER_KHRONOS_validation
    export XDG_DATA_DIRS="$XDG_DATA_DIRS:${vulkan-validation-layers}/share"
    export RUST_LOG=client=trace,server=trace,common=trace,vulkan=info
  '';
}
