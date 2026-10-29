#!/usr/bin/env python3
"""Open-loop chassis velocity sequences. Preview by default; no map or localization."""
import argparse
import json
import math
import os
import select
import signal
import sys
from pathlib import Path

from g2_chassis import ChassisController, SessionLock, Velocity, validate_timing

TEMPLATE = {"segments": [{"vx": "m/s", "vy": "m/s", "wz": "rad/s", "seconds": "s"}]}
NOTE = ('Nominal displacement is velocity × time in the commanded body frame; '
        'it is an estimate, not measured displacement or closed-loop arrival. '
        'Axis signs and frame semantics require physical validation.')


def _number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{name} must be a finite number')
    return float(value)


def parse_config(config):
    if not isinstance(config, dict) or set(config) != {'segments'}:
        raise ValueError('config must contain only segments')
    segments = config['segments']
    if not isinstance(segments, list) or not segments:
        raise ValueError('segments must be a nonempty list')
    result = []
    for entry in segments:
        if not isinstance(entry, dict) or set(entry) != {'vx', 'vy', 'wz', 'seconds'}:
            raise ValueError('each segment must contain exactly vx, vy, wz, seconds')
        vx, vy, wz, seconds = (_number(entry[n], n) for n in ('vx', 'vy', 'wz', 'seconds'))
        velocity = Velocity(vx, vy, wz)
        validate_timing(seconds)
        result.append((velocity, seconds))
    return result


def relative_segment(dx, dy, speed):
    dx, dy, speed = (_number(v, n) for v, n in ((dx, 'dx'), (dy, 'dy'), (speed, 'speed')))
    distance = math.hypot(dx, dy)
    if distance == 0 or speed <= 0:
        raise ValueError('relative displacement and speed must be positive in magnitude')
    seconds = distance / speed
    velocity = Velocity(speed * dx / distance, speed * dy / distance, 0.0)
    validate_timing(seconds)
    return [(velocity, seconds)]


def describe(segments):
    return [{'vx': v.vx, 'vy': v.vy, 'wz': v.wz, 'seconds': t,
             'nominal_body_displacement_m': [v.vx * t, v.vy * t],
             'nominal_yaw_rad': v.wz * t} for v, t in segments]


def run_segments(controller, segments):
    try:
        for velocity, seconds in segments:
            controller.run_velocity(velocity, seconds)
    finally:
        controller.stop()


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', type=Path, help='JSON file with explicit velocity segments')
    p.add_argument('--dx', type=float, help='nominal displacement in current body x, metres')
    p.add_argument('--dy', type=float, help='nominal displacement in current body y, metres')
    p.add_argument('--speed', type=float, help='planar speed, m/s')
    p.add_argument('--execute', action='store_true', help='request control and send commands')
    p.add_argument('--handoff-task-id', type=int, help='ID of the existing remote task to hand off, if any')
    p.add_argument('--watch-stdin', action='store_true', help=argparse.SUPPRESS)
    args = p.parse_args(argv)
    relative = (args.dx, args.dy, args.speed)
    try:
        if args.handoff_task_id is not None and not 0 < args.handoff_task_id <= 4294967295:
            raise ValueError('--handoff-task-id must be a positive uint32')
        if args.config and any(v is not None for v in relative):
            raise ValueError('choose either --config or --dx/--dy/--speed')
        if args.config:
            segments = parse_config(json.loads(args.config.read_text()))
        elif any(v is not None for v in relative):
            if any(v is None for v in relative):
                raise ValueError('--dx, --dy and --speed are all required')
            segments = relative_segment(*relative)
        else:
            if args.execute:
                raise ValueError('execution requires explicit segments or relative displacement')
            print(json.dumps({'mode': 'template', 'sdk_imported': False,
                              'config_template': TEMPLATE, 'note': NOTE}, indent=2))
            return 0
    except (ValueError, OSError, json.JSONDecodeError) as exc:
        p.error(str(exc))
    if not args.execute:
        print(json.dumps({'mode': 'preview', 'sdk_imported': False, 'segments': describe(segments),
                          'note': NOTE, 'physical_motion_tested': False}, indent=2))
        return 0
    cancelled = [False]
    def is_cancelled():
        if not cancelled[0] and args.watch_stdin:
            ready, _, _ = select.select([sys.stdin.fileno()], [], [], 0)
            if ready and not os.read(sys.stdin.fileno(), 1024):
                cancelled[0] = True
        return cancelled[0]
    signals = [signal.SIGINT, signal.SIGTERM]
    if hasattr(signal, 'SIGHUP'):
        signals.append(signal.SIGHUP)
    previous = {s: signal.signal(s, lambda *_: cancelled.__setitem__(0, True)) for s in signals}
    session = None
    controller = ChassisController(execute=True, cancelled=is_cancelled,
                                   handoff_task_id=args.handoff_task_id)
    status, error = 'commands_finished', None
    try:
        session = SessionLock()
        if not session.acquired:
            raise RuntimeError('a chassis worker is already active; stop it first')
        session.record_owner()
        with controller:
            run_segments(controller, segments)
    except (KeyboardInterrupt, InterruptedError) as exc:
        status, error = 'cancelled', str(exc)
    except Exception as exc:
        status, error = 'failed', str(exc)
    finally:
        if session is not None:
            session.close()
        for s, handler in previous.items():
            signal.signal(s, handler)
    if controller.cleanup_errors and status == 'commands_finished':
        status = 'cleanup_failed'
    if controller.requested and controller.last_stop_sent == 0:
        status = 'stop_unconfirmed'
    print(json.dumps({'status': status, 'error': error, 'handoff_task_id': args.handoff_task_id,
                      'control_request_sent': controller.control_request_sent,
                      'owned_task_id': controller.owned_task_id,
                      'released_task_id': controller.released_task_id,
                      'unassociated_request_may_remain_active': controller.control_request_sent and controller.owned_task_id is None,
                      'exclusive_control_verified': False,
                      'sent': controller.sent, 'stop_sent': controller.stop_sent,
                      'final_stop_sent': controller.last_stop_sent, 'stop_errors': controller.stop_errors,
                      'cleanup_errors': controller.cleanup_errors, 'physical_motion_verified': False}))
    return 0 if status == 'commands_finished' else 1


if __name__ == '__main__':
    raise SystemExit(main())
