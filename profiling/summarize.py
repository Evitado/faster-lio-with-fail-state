#!/usr/bin/env python3
"""Text summary of a faster-lio .tracy file: time per scan of each zone, and cpu / memory plots.

    python3 profiling/summarize.py run.tracy

Uses tracy-csvexport from profiling/tracy-tools (or $TRACY_CSVEXPORT).
"""
import csv
import io
import os
import statistics
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
CSVEXPORT = os.environ.get("TRACY_CSVEXPORT", os.path.join(HERE, "tracy-tools", "bin", "tracy-csvexport"))


def export(args):
    out = subprocess.run([CSVEXPORT] + args, check=True, capture_output=True, text=True).stdout
    return list(csv.DictReader(io.StringIO(out)))


def mib(v):
    return f"{v / 2**20:8.1f} MiB"


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        return 1
    trace = sys.argv[1]

    # plot samples: one row per sample (name, timestamp, value); the zone filter matches nothing so that only plots
    # are exported (unwrapping every zone event of a long run is very slow)
    plots = {}
    for row in export(["-u", "-p", "-f", "@@plots-only@@", trace]):
        name = row.get("name")
        try:
            plots.setdefault(name, []).append(float(row["value"]))
        except (KeyError, TypeError, ValueError):
            pass
    scans = len(plots.get("IEKF iterations", []))

    zones = export([trace])
    zones = [z for z in zones if z.get("name")]
    zones.sort(key=lambda z: -float(z["total_ns"]))

    print(f"\n{trace}\nprocessed scans: {scans}\n")
    print(f"{'zone':32} {'calls':>8} {'mean ms':>9} {'max ms':>9} {'ms/scan':>9}")
    for z in zones:
        if z["name"] in ("idle (rate.sleep)",):
            continue
        total_ms = float(z["total_ns"]) / 1e6
        per_scan = total_ms / scans if scans else float("nan")
        print(f"{z['name'][:32]:32} {int(z['counts']):8d} {float(z['mean_ns']) / 1e6:9.3f} "
              f"{float(z['max_ns']) / 1e6:9.3f} {per_scan:9.3f}")
    print("  (zones nest and tbb zones run in parallel on worker threads, so ms/scan does not add up to cpu time)")

    def stat(name, fmt):
        values = plots.get(name)
        if not values:
            return
        values_sorted = sorted(values)
        p95 = values_sorted[min(len(values_sorted) - 1, int(0.95 * len(values_sorted)))]
        print(f"{name:28} mean {fmt(statistics.fmean(values))}  p95 {fmt(p95)}  max {fmt(max(values))}  "
              f"min {fmt(min(values))}  last {fmt(values[-1])}")

    print()
    stat("process cpu [cores]", lambda v: f"{v:6.2f}")
    stat("process rss", mib)
    stat("system mem available", mib)
    stat("process threads", lambda v: f"{v:6.0f}")
    stat("map voxels", lambda v: f"{v:8.0f}")
    stat("scan points (downsampled)", lambda v: f"{v:6.0f}")
    stat("IEKF iterations", lambda v: f"{v:6.1f}")
    print("  process cpu [cores]: 1.0 = one fully busy core")
    return 0


if __name__ == "__main__":
    sys.exit(main())
