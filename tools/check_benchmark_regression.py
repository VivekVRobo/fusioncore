#!/usr/bin/env python3
"""Catch benchmark regressions before they ship.

The problem this solves: FusionCore has many coupled tuning knobs. A change made
to win one sequence (say a long GPS blackout) can quietly worsen another, and
without a check nobody notices for weeks. This compares fresh run metrics against
a recorded baseline (tools/benchmark_baseline.json) and fails loudly if
FusionCore's error grew by more than the allowed threshold.

Feed it metrics.json files produced by tools/evaluate.py --json:

    python3 tools/check_benchmark_regression.py \\
        benchmarks/nclt/2013-04-05/results_fast/metrics.json

    # or a whole set at once
    python3 tools/check_benchmark_regression.py \\
        --glob 'benchmarks/nclt/*/results_fast/metrics.json'

Exit code is non-zero if any sequence regressed, so it can gate CI or a release.
"""

import argparse
import glob as globmod
import json
import math
import os
import sys

# metric we regression-gate on: XY ATE is what matters for a ground robot
METRIC = 'ate_rmse_xy'
BASELINE_KEY = 'fusioncore_ate_rmse_xy'


def load_json_object(path, description):
    """Load a JSON object, turning file/parse failures into actionable errors."""
    try:
        with open(path, encoding='utf-8') as f:
            data = json.load(f)
    except (OSError, UnicodeError, json.JSONDecodeError) as e:
        raise ValueError(f'{path}: cannot read {description}: {e}') from e
    if not isinstance(data, dict):
        raise ValueError(f'{path}: expected a JSON object')
    return data


def load_metrics(path):
    data = load_json_object(path, 'valid metrics JSON')
    seq = data.get('sequence')
    if not isinstance(seq, str) or not seq.strip():
        raise ValueError(f'{path}: missing non-empty string "sequence"')

    filters = data.get('filters')
    if not isinstance(filters, dict):
        raise ValueError(f'{path}: missing object "filters"')
    fc = filters.get('FusionCore')
    if not isinstance(fc, dict) or METRIC not in fc:
        raise ValueError(
            f'{path}: no FusionCore/{METRIC} '
            '(is this a tools/evaluate.py --json output?)'
        )

    cur = fc[METRIC]
    if (isinstance(cur, bool) or not isinstance(cur, (int, float))
            or not math.isfinite(cur) or cur < 0):
        raise ValueError(
            f'{path}: FusionCore/{METRIC} must be a finite, non-negative number'
        )
    return seq, float(cur)


def load_baseline(path, threshold_override):
    base = load_json_object(path, 'valid baseline JSON')
    seqs = base.get('sequences')
    if not isinstance(seqs, dict):
        raise ValueError(f'{path}: missing object "sequences"')

    threshold = (threshold_override if threshold_override is not None
                 else base.get('regression_threshold_pct', 10.0))
    if (isinstance(threshold, bool) or not isinstance(threshold, (int, float))
            or not math.isfinite(threshold) or threshold < 0):
        raise ValueError('regression threshold must be a finite, non-negative number')
    return base, seqs, float(threshold)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('metrics', nargs='*', help='metrics.json file(s) from evaluate.py --json')
    ap.add_argument('--glob', help='glob pattern for metrics.json files')
    ap.add_argument('--baseline', default=os.path.join(os.path.dirname(__file__), 'benchmark_baseline.json'))
    ap.add_argument('--threshold', type=float, default=None,
                    help='allowed %% increase in XY ATE before it is a regression (default: baseline value)')
    args = ap.parse_args()

    try:
        base, seqs, threshold = load_baseline(args.baseline, args.threshold)
    except ValueError as e:
        print(f'Invalid baseline: {e}', file=sys.stderr)
        return 2

    paths = list(args.metrics)
    if args.glob:
        paths += sorted(globmod.glob(args.glob))
    if not paths:
        print('No metrics.json files given. Pass paths or --glob.', file=sys.stderr)
        return 2

    print(f'Regression gate: FusionCore XY ATE, threshold +{threshold:.1f}%  '
          f'(baseline {base.get("baseline_commit", "?")}, recorded {base.get("recorded", "?")})')
    print(f'{"sequence":<14}{"baseline":>10}{"current":>10}{"change":>10}   status')
    print('-' * 58)

    regressions, unknown, invalid = [], [], []
    for p in paths:
        try:
            seq, cur = load_metrics(p)
        except ValueError as e:
            print(f'  invalid: {e}', file=sys.stderr)
            invalid.append(p)
            continue

        if seq not in seqs:
            print(f'{seq:<14}{"-":>10}{cur:>10.3f}{"-":>10}   NEW (no baseline)')
            unknown.append(seq)
            continue

        entry = seqs[seq]
        b = entry.get(BASELINE_KEY) if isinstance(entry, dict) else None
        if (isinstance(b, bool) or not isinstance(b, (int, float))
                or not math.isfinite(b) or b <= 0):
            message = (
                f'{p}: baseline entry for sequence {seq!r} must contain a '
                f'finite, positive {BASELINE_KEY}'
            )
            print(f'  invalid: {message}', file=sys.stderr)
            invalid.append(p)
            continue

        b = float(b)
        pct = (cur - b) / b * 100.0
        if pct > threshold:
            status = 'REGRESSION'
            regressions.append((seq, b, cur, pct))
        elif pct < -threshold:
            status = 'improved'
        else:
            status = 'ok'
        verified = '' if entry.get('verified') else '  (baseline provisional)'
        print(f'{seq:<14}{b:>10.3f}{cur:>10.3f}{pct:>+9.1f}%   {status}{verified}')

    print('-' * 58)
    if regressions:
        print(f'FAIL: {len(regressions)} sequence(s) regressed beyond +{threshold:.1f}%:')
        for seq, b, cur, pct in regressions:
            print(f'  {seq}: {b:.3f} m -> {cur:.3f} m ({pct:+.1f}%)')

    if invalid:
        print(
            f'FAIL: {len(invalid)} input(s) invalid; refusing to pass the regression gate.',
            file=sys.stderr,
        )
        return 2

    if regressions:
        return 1

    print('PASS: no FusionCore XY ATE regressions beyond threshold.')
    if unknown:
        print(f'Note: {len(unknown)} sequence(s) had no baseline entry (add them to '
              f'{os.path.basename(args.baseline)} once verified).')
    return 0


if __name__ == '__main__':
    sys.exit(main())
