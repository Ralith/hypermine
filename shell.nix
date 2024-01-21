with import <nixpkgs> { };
let
  dlopen-libs = with xorg; [ vulkan-loader libX11 libXcursor libXrandr libXi ];
in mkShell {
  nativeBuildInputs = with pkgs; [ rustChannels.stable.rust pkg-config zstd protobuf ];
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
