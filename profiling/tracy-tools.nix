# Tracy viewer (`tracy`), headless recorder (`tracy-capture`) and `tracy-csvexport` for the desktop, built from the
# same Tracy commit as the client faster-lio compiles in. The build is defined in ../package.nix (passthru.tracyTools),
# which the vehicle package uses for its recorder.
#
# Desktop tools: nix build -f profiling/tracy-tools.nix -o profiling/tracy-tools
{
  pkgs ? null,
  captureOnly ? false,
}:
let
  package = import ../package.nix;
  # only the Tracy part of the package is evaluated: the ROS dependencies (and lib, used in meta) are never touched
  faster-lio = package (
    builtins.mapAttrs (_: _: null) (builtins.functionArgs package)
    // {
      buildRosPackage = attrs: attrs;
    }
  );
in
faster-lio.passthru.tracyTools (
  { inherit captureOnly; } // (if pkgs != null then { inherit pkgs; } else { })
)
