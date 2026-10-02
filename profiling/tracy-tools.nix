# Tracy viewer (`tracy`), headless recorder (`tracy-capture`) and `tracy-csvexport`, built from the same Tracy commit
# that evitado_common vendors (foreign/tracy), so they speak the client's network/file protocol (72). The release
# packaged in nixpkgs (0.11.1, protocol 69) cannot read traces from that client.
#
# Build: nix build -f profiling/tracy-tools.nix -o profiling/tracy-tools
{
  pkgs ? import (builtins.fetchTarball {
    # nixpkgs-unstable as locked in the workspace flake
    url = "https://github.com/nixos/nixpkgs/archive/9b008d60392981ad674e04016d25619281550a9d.tar.gz";
  }) { },
}:
let
  # keep in sync with submodule_version in evitado_common/package.nix
  tracySrc = pkgs.fetchFromGitHub {
    owner = "wolfpld";
    repo = "tracy";
    rev = "9074461ffbb44ad77244a493c1090eeaa6f20322";
    hash = "sha256-q+cA2Bqo2for4KrUhumJAuMb0W1OmtQ23I2Iwxl6mws=";
  };

  # dependencies Tracy's CMake would otherwise download through CPM (no network in the nix sandbox)
  imgui = pkgs.applyPatches {
    name = "imgui-tracy";
    src = pkgs.fetchFromGitHub {
      owner = "ocornut";
      repo = "imgui";
      rev = "v1.91.8-docking";
      hash = "sha256-zm/7c2kYXdsgqAWbLVG55sIiXWqcGkMRjuSVAKdpVkc=";
    };
    patches = [
      "${tracySrc}/cmake/imgui-emscripten.patch"
      "${tracySrc}/cmake/imgui-loader.patch"
    ];
  };
  # this Tracy revision needs capstone 6 (nixpkgs has 5)
  capstone = pkgs.fetchFromGitHub {
    owner = "capstone-engine";
    repo = "capstone";
    rev = "6.0.0-Alpha1";
    hash = "sha256-oKRu3P1inWueEMIpL0uI2ayCMHZ9FIVotil4sqwLqH4=";
  };
  ppqsort = pkgs.fetchFromGitHub {
    owner = "GabTux";
    repo = "PPQSort";
    rev = "v1.0.4";
    hash = "sha256-VantHB1VDsY1CJIhgfaJRPAWnj+d7Q+7VH2BNXI0akM=";
  };
in
# X11 (GLFW) viewer: it also runs under XWayland, while the wayland-only build cannot start on X11 desktops
(pkgs.tracy.override { withWayland = false; }).overrideAttrs (old: {
  version = "0.11.1-unstable-2025-03-10";
  src = tracySrc;

  nativeBuildInputs = old.nativeBuildInputs ++ [ pkgs.python3 ];
  buildInputs = (builtins.filter (p: p != pkgs.capstone) old.buildInputs) ++ [ pkgs.zstd ];
  # build capstone 6 from the local copy instead of downloading it
  cmakeFlags = [
    "-DDOWNLOAD_CAPSTONE=on"
    "-DCPM_capstone_SOURCE=${capstone}"
    "-DTRACY_STATIC=off"
    "-DLEGACY=on"
  ];

  postPatch = (old.postPatch or "") + ''
    python3 - <<'EOF'
    import re

    def replace_block(path, name, replacement):
        src = open(path).read()
        new, n = re.subn(r"CPMAddPackage\(\s*NAME " + re.escape(name) + r"\b.*?\n[ \t]*\)\n", replacement, src, flags=re.S)
        assert n == 1, (path, name)
        open(path, "w").write(new)

    replace_block("cmake/vendor.cmake", "zstd", """
    pkg_check_modules(ZSTD REQUIRED libzstd)
    add_library(libzstd INTERFACE)
    target_include_directories(libzstd INTERFACE ''${ZSTD_INCLUDE_DIRS})
    target_link_libraries(libzstd INTERFACE ''${ZSTD_LINK_LIBRARIES})
    """)
    replace_block("cmake/vendor.cmake", "ImGui", "set(ImGui_SOURCE_DIR ${imgui})\n")
    replace_block("cmake/vendor.cmake", "PPQSort", """
    find_package(Threads REQUIRED)
    add_library(PPQSort INTERFACE)
    target_include_directories(PPQSort INTERFACE ${ppqsort}/include)
    target_link_libraries(PPQSort INTERFACE Threads::Threads)
    add_library(PPQSort::PPQSort ALIAS PPQSort)
    """)
    # no git checkout in the sandbox: stamp the pinned commit instead of asking git
    path = "extra/git-ref.py"
    src = open(path).read()
    old = 'ref = subprocess.run(["git", "rev-parse", "--short", rev], check=True, capture_output=True).stdout.decode().strip()'
    assert old in src
    open(path, "w").write(src.replace(old, 'ref = "9074461"'))

    replace_block("profiler/CMakeLists.txt", "wayland-protocols",
                  "set(wayland-protocols_SOURCE_DIR ${pkgs.wayland-protocols}/share/wayland-protocols)\n")
    EOF
  '';
})
