#!/usr/bin/env python3
"""Scenario sweep runner for the niihan patrol robot.

Reads config/scenarios.yaml, expands each scenario across its sweep axes,
launches Gazebo Harmonic headless per run for a bounded duration, and
writes a per-run pass/fail report plus a summary.

Scenarios whose `requires:` list references robot subsystems that do NOT
yet exist in this workspace (fault_manager, battery_model, estop_subsystem,
teleop_watchdog, nav_stack, patrol_controller) are marked INCONCLUSIVE
with an honest note rather than faked PASS.

Usage:
    python3 run_scenario_sweep.py [--scenarios id1,id2] [--dry-run]
                                  [--per-run-timeout 30]

Outputs:
    reports/sweep_<timestamp>/summary.json
    reports/sweep_<timestamp>/summary.md
    reports/sweep_<timestamp>/<scenario_id>/<run_id>.json
"""
import argparse
import itertools
import json
import os
import shutil
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

try:
    import yaml
except ImportError:
    print('ERROR: PyYAML is required. Install with: sudo apt install python3-yaml', file=sys.stderr)
    sys.exit(2)

HERE = Path(__file__).resolve().parent
PKG_ROOT = HERE.parent  # niihan_description
SCENARIOS_YAML = PKG_ROOT / 'config' / 'scenarios.yaml'
BASE_WORLD = PKG_ROOT / 'worlds' / 'niihan_patrol_base.world'
REPORTS_ROOT = PKG_ROOT / 'reports'

# Subsystems that don't yet exist in this workspace. Scenarios that
# require any of these are marked INCONCLUSIVE (not PASS, not FAIL).
MISSING_SUBSYSTEMS = {
    'fault_manager',
    'battery_model',
    'estop_subsystem',
    'teleop_watchdog',
    'nav_stack',
    'dock_nav',
    'pedestrian_actor',
    'localization_confidence_estimator',
    'geofence_monitor',
}

# Sweep axes that are not yet wired into world/robot generation. Scenarios
# that sweep these axes are marked INCONCLUSIVE instead of falsely PASS,
# since the generated world never actually changes for those params.
UNSUPPORTED_SWEEP_AXES = {
    'distance_m',
    'speed_mps',
    'angle_deg',
    'pitch_deg',
    'roll_deg',
}


def load_scenarios():
    if not SCENARIOS_YAML.exists():
        print(f'ERROR: scenarios file not found: {SCENARIOS_YAML}', file=sys.stderr)
        sys.exit(2)
    with SCENARIOS_YAML.open() as f:
        data = yaml.safe_load(f)
    return data


def expand_axes(sweep_axes):
    """Expand sweep_axes dict into a list of parameter combinations.

    sweep_axes: {'lighting': ['day','night'], 'density': [0,1,2]} ->
        [{'lighting':'day','density':0}, {'lighting':'day','density':1}, ...]
    """
    if not sweep_axes:
        return [{}]
    keys = list(sweep_axes.keys())
    value_lists = [sweep_axes[k] if isinstance(sweep_axes[k], list) else [sweep_axes[k]] for k in keys]
    combos = []
    for values in itertools.product(*value_lists):
        combos.append(dict(zip(keys, values)))
    return combos


def build_variant_world(base_world_path, params, out_path):
    """Copy base world and inject variant-specific overlays.

    Only geometry/lighting-level parameters can be encoded in the .world
    itself (obstacle density, lighting mode, dock obstruction). Behavior-
    level parameters (GNSS noise, sensor dropout, battery drain) are
    passed to the (not-yet-existing) robot control stack via env vars
    and recorded in the run report.
    """
    shutil.copy(base_world_path, out_path)

    # Lighting overlay: inject a night ambient or IR-mode ambient by
    # rewriting the <scene><ambient> tag if a lighting axis is set.
    lighting = params.get('lighting')
    if lighting:
        text = out_path.read_text()
        if lighting == 'night':
            text = text.replace(
                '<ambient>0.5 0.5 0.5 1</ambient>',
                '<ambient>0.05 0.05 0.08 1</ambient>')
            text = text.replace(
                '<background>0.6 0.7 0.8 1</background>',
                '<background>0.02 0.02 0.05 1</background>')
        elif lighting == 'ir_on':
            text = text.replace(
                '<ambient>0.5 0.5 0.5 1</ambient>',
                '<ambient>0.15 0.10 0.10 1</ambient>')
        out_path.write_text(text)
    density = params.get('density')
    if isinstance(density, (int, float)) and density > 0:
        # density is obstacles-per-meter along the corridor; convert to a
        # concrete obstacle count instead of silently truncating floats to 0.
        corridor_length = 10.0
        num_obstacles = max(0, round(density * corridor_length))
        boxes = []
        for i in range(num_obstacles):
            x = 2.0 + i * 1.5
            y = (-1.0) ** i * 0.4
            boxes.append(f'''
    <model name='obs_{i}'>
      <static>true</static>
      <pose>{x} {y} 0.3 0 0 0</pose>
      <link name='link'>
        <collision name='col'><geometry><box><size>0.4 0.4 0.6</size></box></geometry></collision>
        <visual name='vis'><geometry><box><size>0.4 0.4 0.6</size></box></geometry>
          <material><ambient>0.5 0.2 0.2 1</ambient><diffuse>0.6 0.3 0.3 1</diffuse></material></visual>
      </link>
    </model>''')
        text = out_path.read_text()
        text = text.replace('</world>', ''.join(boxes) + '\n</world>')
        out_path.write_text(text)


def scenario_gate(scenario):
    """Return (executable, reason) — executable=False means INCONCLUSIVE."""
    requires = scenario.get('requires', []) or []
    axes = scenario.get('axes', {}) or {}
    found = [k for k in axes if k in UNSUPPORTED_SWEEP_AXES]
    if found:
        return False, f'sweep axis not yet implemented in world/robot generation: {sorted(found)}'
    missing = [r for r in requires if r in MISSING_SUBSYSTEMS]
    if missing:
        return False, f'requires missing subsystems: {sorted(set(missing))}'
    return True, ''


def run_one(scenario, params, run_id, run_dir, per_run_timeout, dry_run, headless=False):
    scenario_id = scenario['id']
    executable, gate_reason = scenario_gate(scenario)

    run_report = {
        'scenario_id': scenario_id,
        'run_id': run_id,
        'params': params,
        'started_at': datetime.utcnow().isoformat() + 'Z',
        'status': None,
        'failure_mode': None,
        'notes': [],
    }

    if not executable:
        run_report['status'] = 'INCONCLUSIVE'
        run_report['failure_mode'] = gate_reason
        run_report['notes'].append(
            'Scenario declared prerequisites that do not exist in this workspace; '
            'geometry variant was generated but no behavioral check was possible.')

    # Build variant world regardless (so the artifact is on disk for inspection).
    variant_world = run_dir / f'{run_id}.world'
    try:
        build_variant_world(BASE_WORLD, params, variant_world)
        run_report['world_file'] = str(variant_world.relative_to(PKG_ROOT))
    except Exception as e:
        run_report['status'] = 'FAIL'
        run_report['failure_mode'] = f'world_generation_error: {e}'
        run_report['finished_at'] = datetime.utcnow().isoformat() + 'Z'
        (run_dir / f'{run_id}.json').write_text(json.dumps(run_report, indent=2))
        return run_report

    if executable and dry_run:
        run_report['status'] = 'DRY_RUN_OK'
        run_report['notes'].append(
            'Scenario is executable but --dry-run was requested; only the '
            'variant world was generated, gz sim was not launched.')

    if not executable or dry_run:
        run_report['finished_at'] = datetime.utcnow().isoformat() + 'Z'
        (run_dir / f'{run_id}.json').write_text(json.dumps(run_report, indent=2))
        return run_report

    # Executable path: launch gz sim with the variant world.
    # The sweep owns its own gz sim process (its own process group) and
    # only ever signals that process/group on timeout or exit -- it never
    # does a global pkill that could kill someone else's simulation.
    log_path = run_dir / f'{run_id}.log'
    if headless:
        cmd = ['gz', 'sim', '-s', '-r', '-v', '4', str(variant_world)]
    else:
        cmd = ['gz', 'sim', '-r', '-v', '4', str(variant_world)]
    try:
        with log_path.open('wb') as logf:
            proc = subprocess.Popen(cmd, stdout=logf, stderr=subprocess.STDOUT,
                                     start_new_session=True)
            t0 = time.time()
            timed_out = False
            while time.time() - t0 < per_run_timeout:
                if proc.poll() is not None:
                    break
                time.sleep(0.5)
            else:
                timed_out = True
            if proc.poll() is None:
                timed_out = True
            if timed_out:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                except ProcessLookupError:
                    pass
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    proc.wait(timeout=5)
                run_report['status'] = 'PASS'
                run_report['notes'].append(
                    f'gz sim ran cleanly for {per_run_timeout}s; only world-level '
                    'validation (loads, physics steps, no crash).')
            else:
                # Exited early -> inspect return code.
                if proc.returncode == 0:
                    run_report['status'] = 'PASS'
                    run_report['notes'].append('gz sim exited cleanly before timeout.')
                else:
                    run_report['status'] = 'FAIL'
                    run_report['failure_mode'] = f'gz_sim_exit_{proc.returncode}'
                    run_report['notes'].append(f'See log: {log_path.name}')
    except FileNotFoundError:
        run_report['status'] = 'FAIL'
        run_report['failure_mode'] = 'gz_sim_not_found'
    except Exception as e:
        run_report['status'] = 'FAIL'
        run_report['failure_mode'] = f'launch_error: {e}'

    run_report['finished_at'] = datetime.utcnow().isoformat() + 'Z'
    (run_dir / f'{run_id}.json').write_text(json.dumps(run_report, indent=2))
    return run_report


def summarize(all_reports, sweep_dir):
    by_scenario = {}
    for r in all_reports:
        by_scenario.setdefault(r['scenario_id'], []).append(r)

    summary = {'generated_at': datetime.utcnow().isoformat() + 'Z', 'scenarios': {}}
    md_lines = ['# Niihan Scenario Sweep Summary', '',
                f'Generated: {summary["generated_at"]}', '',
                '| Scenario | Runs | PASS | FAIL | INCONCLUSIVE | Dominant failure mode |',
                '|---|---|---|---|---|---|']
    for sid, runs in sorted(by_scenario.items()):
        p = sum(1 for r in runs if r['status'] == 'PASS')
        f = sum(1 for r in runs if r['status'] == 'FAIL')
        i = sum(1 for r in runs if r['status'] == 'INCONCLUSIVE')
        fmodes = [r['failure_mode'] for r in runs if r['failure_mode']]
        dominant = max(set(fmodes), key=fmodes.count) if fmodes else '-'
        summary['scenarios'][sid] = {
            'total': len(runs), 'pass': p, 'fail': f, 'inconclusive': i,
            'dominant_failure_mode': dominant,
        }
        md_lines.append(f'| {sid} | {len(runs)} | {p} | {f} | {i} | {dominant} |')

    (sweep_dir / 'summary.json').write_text(json.dumps(summary, indent=2))
    (sweep_dir / 'summary.md').write_text('\n'.join(md_lines) + '\n')
    return summary


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--scenarios', default='', help='comma-separated scenario IDs to run (default: all)')
    ap.add_argument('--dry-run', action='store_true', help='generate variant worlds but do not launch Gazebo')
    ap.add_argument('--headless', action='store_true', help='run Gazebo in headless mode without GUI (default: false)')
    ap.add_argument('--per-run-timeout', type=int, default=20, help='seconds to run each Gazebo launch')
    ap.add_argument('--max-runs-per-scenario', type=int, default=4,
                    help='cap on axis expansions per scenario (keeps sweep bounded)')
    args = ap.parse_args()

    data = load_scenarios()
    scenarios = data.get('scenarios', [])
    if args.scenarios:
        wanted = set(args.scenarios.split(','))
        scenarios = [s for s in scenarios if s['id'] in wanted]
    if not scenarios:
        print('No scenarios matched.', file=sys.stderr)
        sys.exit(1)

    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    sweep_dir = REPORTS_ROOT / f'sweep_{ts}'
    sweep_dir.mkdir(parents=True, exist_ok=True)

    all_reports = []
    for scenario in scenarios:
        sid = scenario['id']
        scen_dir = sweep_dir / sid
        scen_dir.mkdir(exist_ok=True)
        combos = expand_axes(scenario.get('axes', {}))
        if len(combos) > args.max_runs_per_scenario:
            combos = combos[:args.max_runs_per_scenario]
        for idx, params in enumerate(combos):
            run_id = f'{sid}_r{idx:02d}'
            print(f'[{run_id}] params={params}')
            r = run_one(scenario, params, run_id, scen_dir,
                        args.per_run_timeout, args.dry_run,
                        headless=args.headless)
            print(f'  -> {r["status"]} ({r.get("failure_mode") or ""})')
            all_reports.append(r)

    summary = summarize(all_reports, sweep_dir)
    print()
    print(f'Sweep written to: {sweep_dir}')
    print(f'  summary.json / summary.md')
    print()
    for sid, s in summary['scenarios'].items():
        print(f'  {sid}: {s["pass"]} PASS / {s["fail"]} FAIL / {s["inconclusive"]} INCONCLUSIVE  (dominant: {s["dominant_failure_mode"]})')


if __name__ == '__main__':
    main()
