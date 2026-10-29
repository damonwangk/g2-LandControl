#!/usr/bin/env python3
"""A(current robot pose) -> B in an existing, localized SLAM map. Preview default."""
import argparse
import json
import math
import os
from pathlib import Path
import select
import signal
import sys
import time

from g2_chassis import SessionLock

TEMPLATE = {'map_id': None, 'target': {'x': None, 'y': None, 'yaw_rad': None}, 'timeout_s': 120}
TERMINAL = (7, 8, 9)


def validate_config(value):
    if not isinstance(value, dict) or set(value) != {'map_id', 'target', 'timeout_s'}:
        raise ValueError('config requires map_id, target and timeout_s')
    mid = value['map_id']
    if isinstance(mid, bool) or not isinstance(mid, int) or not 0 <= mid <= 255:
        raise ValueError('map_id must be an integer in 0..255')
    target = value['target']
    if not isinstance(target, dict) or set(target) != {'x', 'y', 'yaw_rad'}:
        raise ValueError('target requires x, y, yaw_rad in the selected map frame')
    for key, number in list(target.items()) + [('timeout_s', value['timeout_s'])]:
        if isinstance(number, bool) or not isinstance(number, (int, float)) or not math.isfinite(number):
            raise ValueError(key+' must be finite; fill the unconfirmed point before execution')
    if not 0 < value['timeout_s'] <= 3600:
        raise ValueError('timeout_s must be in (0, 3600]')
    return value


def snapshot(pnc):
    s = pnc.get_task_state()
    return dict(id=int(s.id), state=int(s.state), type=int(s.type), message=str(getattr(s, 'message', '')))


def cancel_owned(pnc, task_id):
    """Do not cancel a replacement task or another navigation mode."""
    if task_id is None:
        return False
    state = snapshot(pnc)
    if state['id'] == task_id and state['type'] == 1 and state['state'] in (1, 2, 3, 4, 5, 6):
        pnc.cancel_task(task_id)
        return True
    return False


def navigate(pnc, maps, sdk, config, *, clock=time.monotonic, sleep=time.sleep,
             cancelled=lambda: False, progress=lambda state: None):
    """Use the public SDK, without Python DDS callbacks.

    The SDK does not return a task ID. We require an idle/terminal initial task,
    then a new ordinary-navigation ID after submission. This assumes the operator
    has stopped other task producers; it does not provide cross-client exclusion.
    """
    config = validate_config(config)
    current_map = maps.get_curr_map()
    if int(current_map.id) != config['map_id']:
        raise RuntimeError('select the configured map and relocalize on the Pad before navigating')
    before = snapshot(pnc)
    if before['state'] not in (0, 7, 8, 9):
        raise RuntimeError('PNC has an active task; no task was cancelled: '+str(before))
    req = sdk.NaviReq()
    target = config['target']
    req.target.position.x, req.target.position.y, req.target.position.z = target['x'], target['y'], 0.0
    req.target.orientation.x = req.target.orientation.y = 0.0
    req.target.orientation.z = math.sin(target['yaw_rad']/2)
    req.target.orientation.w = math.cos(target['yaw_rad']/2)
    req.timestamp_ns = time.time_ns()
    if cancelled():
        raise InterruptedError('cancelled before submitting navigation')
    pnc.normal_navi(req)
    start = clock()
    association_deadline = start + min(5.0, config['timeout_s'])
    finish = start + config['timeout_s']
    task_id = None
    try:
        while True:
            if cancelled():
                raise InterruptedError('navigation cancelled')
            state = snapshot(pnc)
            progress(state)
            if task_id is None:
                if state['type'] == 1 and state['id'] > 0 and state['id'] != before['id']:
                    task_id = state['id']
                elif state['state'] not in (0, 7, 8, 9) and state['type'] not in (0, 1):
                    raise RuntimeError('a different task appeared after navigation submission: '+str(state))
                elif clock() >= association_deadline:
                    raise RuntimeError('request sent but a new navigation task ID could not be associated; no unknown task was cancelled')
            elif state['id'] != task_id or state['type'] != 1:
                raise RuntimeError('navigation task changed; replacement task was not cancelled: '+str(state))
            if task_id is not None and state['id'] == task_id:
                if state['state'] in TERMINAL:
                    return dict(status={7:'cancelled',8:'failed',9:'navigation_reported_success'}[state['state']],
                                task_id=task_id, task=state, target=config,
                                task_id_association='new navigation ID after submission',
                                exclusive_control_verified=False, physical_arrival_verified=False)
            if clock() >= finish:
                return dict(status='timeout', task_id=task_id,
                            cancel_requested=cancel_owned(pnc, task_id), physical_arrival_verified=False)
            sleep(max(0.0, min(0.1, finish-clock())))
    except BaseException as exc:
        exc.navigation_request_sent = True
        exc.task_id = task_id
        try:
            exc.cancel_requested = cancel_owned(pnc, task_id)
        except Exception as cancel_error:
            exc.cancel_requested = False
            exc.cancel_error = str(cancel_error)
        raise


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', type=Path)
    p.add_argument('--map-id', type=int)
    p.add_argument('--x', type=float); p.add_argument('--y', type=float); p.add_argument('--yaw', type=float)
    p.add_argument('--timeout', type=float, default=120)
    p.add_argument('--localized', action='store_true', help='operator has selected this map and completed Pad relocalization')
    p.add_argument('--execute', action='store_true')
    p.add_argument('--watch-stdin', action='store_true', help=argparse.SUPPRESS)
    args = p.parse_args(argv)
    try:
        numbers = [args.map_id, args.x, args.y, args.yaw]
        if args.config:
            if any(x is not None for x in numbers): raise ValueError('choose --config or map/pose arguments')
            config = validate_config(json.loads(args.config.read_text()))
        elif any(x is not None for x in numbers):
            if any(x is None for x in numbers): raise ValueError('map-id, x, y, yaw are all required')
            config = validate_config(dict(map_id=args.map_id,target=dict(x=args.x,y=args.y,yaw_rad=args.yaw),timeout_s=args.timeout))
        else:
            if args.execute: raise ValueError('no A/B point has been configured')
            print(json.dumps(dict(mode='template', sdk_imported=False, config_template=TEMPLATE), indent=2))
            return 0
    except (ValueError, OSError) as exc:
        p.error(str(exc))
    if not args.execute:
        print(json.dumps(dict(mode='preview', sdk_imported=False, configuration=config,
                              requires_existing_map_and_pad_relocalization=True), indent=2))
        return 0
    if not args.localized:
        p.error('complete Pad relocalization first, then specify --localized')
    cancelled = [False]
    def is_cancelled():
        if args.watch_stdin:
            ready, _, _ = select.select([sys.stdin.fileno()], [], [], 0)
            if ready and not os.read(sys.stdin.fileno(), 1024): cancelled[0] = True
        return cancelled[0]
    signals = [signal.SIGINT, signal.SIGTERM, signal.SIGHUP]
    previous = {s:signal.signal(s, lambda *_:cancelled.__setitem__(0, True)) for s in signals}
    lock = None; sdk = None; initialized = False; cleanup = []
    result = dict(status='failed', physical_arrival_verified=False)
    try:
        lock = SessionLock()
        if not lock.acquired: raise RuntimeError('a chassis/navigation worker is already active')
        lock.record_owner()
        if sys.version_info[:2] != (3, 10): raise RuntimeError('use robot-native Python 3.10 and its SDK environment')
        import agibot_gdk as sdk
        if sdk.gdk_init() != sdk.GDKRes.kSuccess: raise RuntimeError('gdk_init failed')
        initialized = True
        maps, pnc = sdk.Map(), sdk.Pnc()
        time.sleep(2)
        result = navigate(pnc, maps, sdk, config, cancelled=is_cancelled,
                          progress=lambda s:print('NAV_TASK '+json.dumps(s), file=sys.stderr, flush=True))
    except (KeyboardInterrupt, InterruptedError) as exc:
        result.update(status='cancelled', error=str(exc))
        result.update({k:getattr(exc,k) for k in ('navigation_request_sent','task_id','cancel_requested','cancel_error') if hasattr(exc,k)})
    except Exception as exc:
        result.update(status='failed', error=str(exc))
        result.update({k:getattr(exc,k) for k in ('navigation_request_sent','task_id','cancel_requested','cancel_error') if hasattr(exc,k)})
    finally:
        if initialized:
            for name in ('release_dds', 'gdk_release'):
                try:
                    code = getattr(sdk, name)()
                    if name == 'gdk_release' and code != sdk.GDKRes.kSuccess: cleanup.append(name+': '+str(code))
                except Exception as exc: cleanup.append(name+': '+str(exc))
        if lock: lock.close()
        for s, h in previous.items(): signal.signal(s, h)
    result['cleanup_errors'] = cleanup
    print(json.dumps(result, indent=2))
    return 0 if result['status'] == 'navigation_reported_success' and not cleanup else 1


if __name__ == '__main__':
    raise SystemExit(main())
