#!/usr/bin/env bash
# Record a Tracy profile of a faster-lio run into a .tracy file, then print a summary.
#
#   profiling/record.sh [-o out.tracy] [-s seconds] -- <command that starts the profiled node>
#   e.g. profiling/record.sh -- roslaunch faster_lio evitado.launch
#
# The node must be built with -DFASTER_LIO_TRACY=ON. Stop the run with Ctrl-C (or let -s expire); the trace is
# written when the node disconnects. Open the file in a Tracy viewer >= 0.12.
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
out="$here/../Log/faster_lio_$(date +%F_%H-%M-%S).tracy"
seconds=""
while getopts "o:s:" opt; do
    case $opt in
        o) out="$OPTARG" ;;
        s) seconds="$OPTARG" ;;
        *) sed -n '2,9p' "${BASH_SOURCE[0]}"; exit 1 ;;
    esac
done
shift $((OPTIND - 1))
[[ "${1:-}" == "--" ]] && shift
if [[ $# -eq 0 ]]; then
    sed -n '2,9p' "${BASH_SOURCE[0]}"
    exit 1
fi

capture="${TRACY_CAPTURE:-$here/tracy-tools/bin/tracy-capture}"
if [[ ! -x "$capture" ]]; then
    echo "tracy-capture not found, build it with: nix build -f $here/tracy-tools.nix -o $here/tracy-tools" >&2
    exit 1
fi
mkdir -p "$(dirname "$out")"

# tracy-capture waits until the profiled process appears on localhost:8086 and records until it disconnects
"$capture" -o "$out" -f ${seconds:+-s "$seconds"} > "${out%.tracy}.capture.log" 2>&1 &
capture_pid=$!

# make the profiled process wait at exit until all data is sent
export TRACY_NO_EXIT=1
"$@" || true

wait "$capture_pid" || true
echo "trace: $out"
python3 "$here/summarize.py" "$out" || true
