#!/usr/bin/env python3
"""Read local MoveIt/press evidence only; no ROS, network, or control commands."""
import argparse
import json
import math
from pathlib import Path
import re

MAX_TRACE_BYTES = 8 * 1024 * 1024
MAX_TAIL_BYTES = 64 * 1024


def newest(paths):
    records = []
    for path in paths:
        try:
            if path.is_file() and not path.is_symlink():
                records.append((path.stat().st_mtime_ns, path))
        except OSError:
            continue
    return max(records, default=(0, None))[1]


def moveit_log(root):
    directory = root / 'logs' / 'lyl_cell'
    if not directory.is_dir():
        return None
    runs = sorted((p for p in directory.iterdir() if p.is_dir() and not p.is_symlink()),
                  key=lambda p: p.stat().st_mtime_ns, reverse=True)[:32]
    return newest(p for run in runs for p in run.glob('*_moveit_*.log'))


def safe_tail(path):
    with path.open('rb') as source:
        source.seek(0, 2)
        size = source.tell()
        source.seek(max(0, size - MAX_TAIL_BYTES))
        raw = source.read(MAX_TAIL_BYTES).decode('utf-8', errors='replace')
    lines = raw.splitlines()
    if size > MAX_TAIL_BYTES:
        lines = lines[1:]
    selected = []
    for line in lines[-60:]:
        # Only motion diagnostics; never return arbitrary prompts, keys or headers.
        if re.search(r'api.?key|authorization|bearer|sk-ant-|sk-[A-Za-z0-9]|password|token|https?://', line, re.I):
            continue
        if re.match(r'^\s*(?:\[(?:vertical|alignment|trajectory|pickup|action|executor)\]|>>> \[(?:MoveIt|중단|복구)|MOVEIT PICK&PLACE)', line):
            selected.append(line[:1200])
    return selected


def number(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    return value if math.isfinite(value) else None


def coord(value):
    return number(value[2]) if isinstance(value, list) and len(value) >= 3 else None


def metrics(value):
    value = value if isinstance(value, dict) else {}
    return {key: number(value.get(key)) for key in ('xy_m', 'orientation_deg', 'edge_bound_m')}


def sample_summary(sample, target_z):
    sample = sample if isinstance(sample, dict) else {}
    desired_z, actual_z = coord(sample.get('desired_tcp')), coord(sample.get('actual_tcp'))
    return {'stamp': number(sample.get('stamp')), 'desired': metrics(sample.get('desired')),
            'actual': metrics(sample.get('actual')), 'desired_z_m': desired_z,
            'actual_z_m': actual_z, 'target_z_m': target_z,
            'actual_z_error_m': actual_z - target_z if actual_z is not None and target_z is not None else None}


def press_summary(path):
    if path.stat().st_size > MAX_TRACE_BYTES:
        raise ValueError('압착 기록이 8 MiB를 초과해 자동으로 읽지 않았습니다.')
    with path.open('rb') as source:
        raw = source.read(MAX_TRACE_BYTES + 1)
    if len(raw) > MAX_TRACE_BYTES:
        raise ValueError('압착 기록이 읽는 동안 커져 다시 수집해야 합니다.')
    data = json.loads(raw)
    if not isinstance(data, dict) or not str(data.get('step', '')).startswith('press'):
        raise ValueError('압착 단계의 JSON 기록이 아닙니다.')
    samples = [s for s in data.get('samples', []) if isinstance(s, dict)]
    planned = data.get('planned') or []
    target_z = coord(planned[-1].get('tcp')) if planned and isinstance(planned[-1], dict) else None
    def edge(sample):
        actual = sample.get('actual')
        result = number(actual.get('edge_bound_m')) if isinstance(actual, dict) else None
        return result if result is not None else -1
    peak = max(samples, key=edge, default={})
    limits = data.get('limits_m') or {}
    return {'file': str(path), 'modified_ns': path.stat().st_mtime_ns,
            'completed': data.get('completed') if isinstance(data.get('completed'), bool) else None,
            'guard_failed': bool(data.get('guard_failure')), 'sample_count': len(samples),
            'radius_m': number(data.get('radius_m')), 'plan_max_edge_m': number(data.get('plan_max_edge_m')),
            'execution_limit_m': number(limits.get('execution')) if isinstance(limits, dict) else None,
            'end_z_error_m': number(data.get('end_z_error_m')),
            'peak_actual_edge_sample': sample_summary(peak, target_z),
            'last_sample': sample_summary(samples[-1] if samples else {}, target_z)}


def collect(root):
    root = Path(root).expanduser().resolve()
    output = {'workcell': str(root), 'read_only': True, 'errors': [],
              'note': '로그와 압착 기록은 각각 최신 파일입니다. 같은 실행인지 파일 경로와 시각을 확인하세요.'}
    try:
        log = moveit_log(root)
        if log is None:
            output['errors'].append('logs/lyl_cell 아래 MoveIt 실행 로그가 없습니다.')
        else:
            output['moveit_log'] = {'file': str(log), 'modified_ns': log.stat().st_mtime_ns,
                                    'filtered_last_60_lines': safe_tail(log)}
    except OSError:
        output['errors'].append('MoveIt 로그를 읽을 수 없습니다. 경로와 읽기 권한을 확인하세요.')
    try:
        trace = newest((root / 'logs' / 'vertical_v46').glob('vertical_*_press*.json'))
        if trace is None:
            output['errors'].append('logs/vertical_v46 아래 완료 저장된 압착 JSON이 없습니다. 아직 실행 중일 수 있습니다.')
        else:
            output['press'] = press_summary(trace)
    except (OSError, ValueError, TypeError, AttributeError):
        output['errors'].append('압착 JSON을 읽지 못했습니다. 기록 중이거나 손상됐거나 크기 제한을 넘었습니다.')
    output['status'] = 'ok' if not output['errors'] else 'incomplete'
    return output


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workcell', type=Path, default=Path.home() / 'AHEAD/pac2026_integrated')
    args = parser.parse_args(argv)
    result = collect(args.workcell)
    print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
    return 0 if result['status'] == 'ok' else 1


if __name__ == '__main__':
    raise SystemExit(main())
