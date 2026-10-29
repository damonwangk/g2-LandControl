#!/usr/bin/env python3
"""Chassis-only G2 velocity control. Preview by default; --execute sends commands."""
import argparse
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import select
import signal
import sys
import time


@dataclass(frozen=True)
class Velocity:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0

    def __post_init__(self):
        if not all(math.isfinite(v) for v in (self.vx, self.vy, self.wz)):
            raise ValueError('velocity must be finite')
        if math.hypot(self.vx, self.vy) > 0.1000000001:
            raise ValueError('this program limits planar speed to 0.10 m/s')
        if abs(self.wz) > 0.3000000001:
            raise ValueError('this program limits angular speed to 0.30 rad/s')
        if self.vy != 0.0 and self.wz != 0.0:
            raise ValueError('lateral translation plus yaw rotation is not verified by this SDK contract')

    def axes(self):
        return (self.vx, self.vy, 0.0, 0.0, 0.0, self.wz)

    def mode(self):
        return 'crab' if self.vy else 'forward_turn_or_stop'


ZERO = Velocity()


class ChassisController:
    """Reusable base API. update_velocity renews a short application lease.

    tick must run regularly. An expired lease sends zero; this is a software
    safeguard, not proof of a robot-side command timeout after process death.
    """
    def __init__(self, execute=False, gdk=None, clock=time.monotonic,
                 sleep=time.sleep, cancelled=lambda: False, preflight=None,
                 before_release=None, handoff_task_id=None, manage_tasks=True,
                 task_timeout=3.0, release_task=True, stop_only=False):
        if handoff_task_id is not None and (isinstance(handoff_task_id, bool)
                or not isinstance(handoff_task_id, int) or not 0 < handoff_task_id <= 4294967295):
            raise ValueError('handoff_task_id must be a positive uint32')
        if not math.isfinite(task_timeout) or not 0 < task_timeout <= 30:
            raise ValueError('task_timeout must be in (0, 30] seconds')
        self.execute = execute
        self.gdk = gdk
        self.clock, self.sleep, self.cancelled = clock, sleep, cancelled
        self.preflight, self.before_release = preflight, before_release
        self.pnc = None
        self.initialized = self.requested = self.opened = False
        self.command = ZERO
        self.expires = 0.0
        self.previous_tick = None
        self.sent = self.nonzero_sent = 0
        self.stop_sent = self.last_stop_sent = 0
        self.stop_errors, self.cleanup_errors = [], []
        self.handoff_task_id, self.manage_tasks = handoff_task_id, manage_tasks
        self.task_timeout, self.release_task, self.stop_only = task_timeout, release_task, stop_only
        self.initial_task = None
        self.pre_request_task = None
        self.owned_task_id = self.released_task_id = None
        self.control_request_sent = False

    def __enter__(self):
        if self.opened:
            raise RuntimeError('controller is already open')
        self.opened = True
        self.initialized = False
        self.pnc = None
        self.owned_task_id = self.released_task_id = None
        self.requested = self.control_request_sent = False
        self.initial_task = self.pre_request_task = None
        try:
            if self.execute:
                if self.gdk is None:
                    if sys.version_info[:2] != (3, 10):
                        raise RuntimeError('use robot-native Python 3.10 and /home/agi/app/env.sh')
                    import agibot_gdk
                    self.gdk = agibot_gdk
                result = self.gdk.gdk_init()
                self.initialized = result == self.gdk.GDKRes.kSuccess
                if not self.initialized:
                    raise RuntimeError('gdk_init failed: ' + str(result))
                if self.preflight is not None:
                    self.preflight()
                self.pnc = self.gdk.Pnc()
                self.sleep(2.0)
                if self.manage_tasks:
                    self._prepare_control()
                    if self.stop_only and self.initial_task['state'] == 2 and self.initial_task['type'] == 3:
                        self.requested = True
                        return self
                    self.pre_request_task = self.task_state()
                    if self.pre_request_task['state'] not in (0, 7, 8, 9):
                        raise RuntimeError('PNC task changed before control request')
                if self.cancelled():
                    raise InterruptedError('cancelled before requesting chassis control')
                self.requested = True
                self.control_request_sent = True
                self.pnc.request_chassis_control(0)
                self.sleep(0.5)
                if self.manage_tasks:
                    state = self._wait_task(lambda s: s['state'] == 2 and s['type'] == 3
                                            and s['id'] > 0 and s['id'] != self.pre_request_task['id'])
                    self.owned_task_id = state['id']
            return self
        except BaseException:
            self.close()
            raise

    def __exit__(self, *_):
        self.close()

    def task_state(self):
        if self.pnc is None:
            raise RuntimeError('PNC is not open')
        state = self.pnc.get_task_state()
        return dict(id=int(state.id), state=int(state.state), type=int(state.type),
                    message=str(getattr(state, 'message', '')))

    def _wait_task(self, ready, expected_id=None):
        deadline = self.clock() + self.task_timeout
        while True:
            if self.cancelled():
                raise InterruptedError('cancelled during control handoff')
            state = self.task_state()
            if expected_id is not None and not (state['state'] == 0 and state['type'] == 0):
                if state['id'] != expected_id or state['type'] != 3:
                    raise RuntimeError('PNC task changed during handoff: ' + str(state))
            if ready(state):
                return state
            if state['type'] not in (0, 3) and state['state'] not in (0, 7, 8, 9):
                raise RuntimeError('another PNC navigation task is active: ' + str(state))
            if self.clock() >= deadline:
                raise TimeoutError('PNC task transition timed out: ' + str(state))
            self.sleep(max(0.0, min(0.05, deadline-self.clock())))

    def _prepare_control(self):
        state = self.task_state()
        self.initial_task = state
        if state['state'] in (0, 7, 8, 9):
            return  # The vendor accepts idle and terminal tasks without cancellation.
        if state['type'] != 3 or state['state'] != 2:
            raise RuntimeError('PNC is busy; no task was cancelled: ' + str(state))
        if self.stop_only:
            return  # Zero an existing remote-control stream without claiming its task.
        if state['id'] != self.handoff_task_id:
            raise RuntimeError('remote control is active; use --handoff-task-id ' + str(state['id']))
        current = self.task_state()
        if (current['id'], current['type'], current['state']) != (state['id'], 3, 2):
            raise RuntimeError('remote task changed before cancellation')
        self.pnc.cancel_task(state['id'])
        self._wait_task(lambda s: (s['state'] == 0 and s['type'] == 0)
                        or (s['id'] == state['id'] and s['type'] == 3 and s['state'] in (7, 8, 9)),
                        expected_id=state['id'])

    def _send(self, value):
        if self.execute:
            twist = self.gdk.Twist()
            twist.linear = self.gdk.Vector3()
            twist.angular = self.gdk.Vector3()
            (twist.linear.x, twist.linear.y, twist.linear.z,
             twist.angular.x, twist.angular.y, twist.angular.z) = value.axes()
            self.pnc.move_chassis(twist)
        self.sent += 1
        self.nonzero_sent += int(value != ZERO)

    def update_velocity(self, vx=0.0, vy=0.0, wz=0.0, lease=0.35):
        if not self.opened:
            raise RuntimeError('use controller as a context manager')
        if not math.isfinite(lease) or not 0 < lease <= 0.5:
            raise ValueError('application lease must be in (0, 0.5] seconds')
        command = Velocity(vx, vy, wz)
        if self.stop_only and command != ZERO:
            raise ValueError('a stop-only controller cannot send nonzero velocity')
        self.command = command
        self.expires = self.clock() + lease

    def tick(self):
        if not self.opened:
            raise RuntimeError('controller is closed')
        if self.cancelled():
            raise InterruptedError('cancelled')
        now = self.clock()
        if self.previous_tick is not None and now - self.previous_tick > 0.25:
            raise RuntimeError('control loop stalled; motion will not resume automatically')
        self.previous_tick = now
        value = self.command if now < self.expires else ZERO
        self._send(value)
        return value

    def stop(self):
        self.command = ZERO
        self.expires = 0.0
        self.previous_tick = None
        self.last_stop_sent = 0
        if not self.execute:
            return
        if self.pnc is not None and self.requested:
            for _ in range(5):
                try:
                    self._send(ZERO)
                    self.stop_sent += 1
                    self.last_stop_sent += 1
                except BaseException as exc:
                    self.stop_errors.append(str(exc) or type(exc).__name__)
                self.sleep(0.04)

    def close(self):
        if not self.opened:
            return
        try:
            self.stop()
        finally:
            if self.manage_tasks and self.release_task and self.owned_task_id is not None and self.pnc is not None:
                try:
                    state = self.task_state()
                    if state['id'] == self.owned_task_id and state['type'] == 3 and state['state'] in (1, 2, 3, 4, 5, 6):
                        self.pnc.cancel_task(self.owned_task_id)
                        self.released_task_id = self.owned_task_id
                except BaseException as exc:
                    self.cleanup_errors.append('release_owned_remote_task: ' + str(exc))
            self.pnc = None
            if self.before_release is not None:
                try:
                    self.before_release()
                except BaseException as exc:
                    self.cleanup_errors.append('before_release: ' + str(exc))
            if self.initialized:
                for name in ('release_dds', 'gdk_release'):
                    try:
                        fn = getattr(self.gdk, name, None)
                        if fn is not None:
                            result = fn()
                            if name == 'gdk_release' and result != self.gdk.GDKRes.kSuccess:
                                self.cleanup_errors.append(name + ' returned ' + str(result))
                    except BaseException as exc:
                        self.cleanup_errors.append(name + ': ' + str(exc))
            self.opened = False

    def run_velocity(self, velocity, duration, hz=50.0):
        validate_timing(duration, hz)
        start = self.clock()
        finish = start + duration
        deadline = start
        while True:
            now = self.clock()
            if now >= finish:
                if (self.previous_tick is not None and now-self.previous_tick > 0.25) or now-finish > 0.10:
                    raise RuntimeError('control loop exceeded the finish deadline')
                break
            self.update_velocity(velocity.vx, velocity.vy, velocity.wz)
            self.tick()
            deadline += 1.0 / hz
            now = self.clock()
            if deadline <= now:
                deadline = now + 1.0 / hz
            self.sleep(max(0.0, min(deadline, finish) - now))
        self.stop()


def validate_timing(duration, hz=50.0):
    if not math.isfinite(duration) or not 0 < duration <= 120:
        raise ValueError('duration must be in (0, 120] seconds')
    if not math.isfinite(hz) or not 10 <= hz <= 100:
        raise ValueError('hz must be in [10, 100]')


def keyboard_velocity(key, speed, turn_speed):
    d = speed / math.sqrt(2)
    values = {'w': (speed, 0, 0), 's': (-speed, 0, 0),
              'a': (0, speed, 0), 'd': (0, -speed, 0),
              'q': (d, d, 0), 'e': (d, -d, 0),
              'z': (-d, d, 0), 'c': (-d, -d, 0),
              'j': (0, 0, turn_speed), 'l': (0, 0, -turn_speed),
              'u': (speed, 0, turn_speed), 'o': (speed, 0, -turn_speed),
              'n': (-speed, 0, turn_speed), 'm': (-speed, 0, -turn_speed),
              ' ': (0, 0, 0)}
    return Velocity(*values[key]) if key in values else None


def keyboard_loop(controller, speed, turn_speed, max_seconds):
    import termios
    import tty
    fd = sys.stdin.fileno()
    if not os.isatty(fd):
        raise RuntimeError('keyboard mode needs an interactive TTY')
    saved = termios.tcgetattr(fd)
    start = controller.clock()
    escape = False
    print('W/S forward/back; A/D left/right; Q/E/Z/C diagonals; J/L yaw command;', flush=True)
    print('U/O forward turn; N/M reverse turn; SPACE stop; X exit. Hold/repeat key to move.', flush=True)
    try:
        tty.setcbreak(fd)
        while controller.clock() - start < max_seconds:
            ready, _, _ = select.select([fd], [], [], 0)
            if ready:
                data = os.read(fd, 128)
                if not data:
                    break
                for byte in data:
                    if byte == 27:
                        escape = True
                        continue
                    if escape:
                        if byte not in (91, 79) and 64 <= byte <= 126:
                            escape = False
                        continue
                    key = chr(byte).lower()
                    if key == 'x' or byte in (3, 4):
                        return
                    value = keyboard_velocity(key, speed, turn_speed)
                    if value is not None:
                        controller.update_velocity(value.vx, value.vy, value.wz)
            controller.tick()
            controller.sleep(0.02)
    finally:
        termios.tcsetattr(fd, termios.TCSANOW, saved)


def build_parser():
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest='action', required=True)
    v = sub.add_parser('velocity', help='bounded velocity command')
    for name in ('vx', 'vy', 'wz'):
        v.add_argument('--' + name, type=float, default=0.0)
    v.add_argument('--seconds', type=float, required=True)
    k = sub.add_parser('keyboard', help='interactive keyboard control')
    k.add_argument('--speed', type=float, default=0.05)
    k.add_argument('--turn-speed', type=float, default=0.10)
    k.add_argument('--max-seconds', type=float, default=3600)
    sub.add_parser('stop', help='request chassis control and send zero velocity')
    for item in (v, k, sub.choices['stop']):
        item.add_argument('--execute', action='store_true', help='actually use the robot SDK')
        item.add_argument('--handoff-task-id', type=int, help='explicit ID of an existing remote-control task to hand off')
        item.add_argument('--watch-stdin', action='store_true', help=argparse.SUPPRESS)
    return p


def validate_args(args):
    if getattr(args, 'handoff_task_id', None) is not None and not 0 < args.handoff_task_id <= 4294967295:
        raise ValueError('handoff task ID must be a positive uint32')
    if args.action == 'velocity':
        Velocity(args.vx, args.vy, args.wz)
        validate_timing(args.seconds)
    elif args.action == 'keyboard':
        if args.speed <= 0 or args.turn_speed <= 0:
            raise ValueError('keyboard speeds must be positive')
        Velocity(args.speed, 0, args.turn_speed)
        if not math.isfinite(args.max_seconds) or not 0 < args.max_seconds <= 86400:
            raise ValueError('keyboard max-seconds must be in (0, 86400]')


class SessionLock:
    """One active instance of this program per robot account."""
    def __init__(self, path=None):
        import fcntl
        self.fcntl = fcntl
        self.path = Path(path) if path is not None else Path('/tmp') / ('g2-chassis-control-' + str(os.getuid()) + '.lock')
        flags = os.O_RDWR | os.O_CREAT | getattr(os, 'O_NOFOLLOW', 0)
        self.fd = os.open(str(self.path), flags, 0o600)
        info = os.fstat(self.fd)
        if info.st_uid != os.getuid() or info.st_mode & 0o077:
            os.close(self.fd)
            raise RuntimeError('unexpected control lock ownership or permissions')
        try:
            fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self.acquired = True
        except BlockingIOError:
            self.acquired = False

    def record_owner(self):
        os.ftruncate(self.fd, 0)
        os.lseek(self.fd, 0, os.SEEK_SET)
        os.write(self.fd, str(os.getpid()).encode())
        os.fsync(self.fd)

    def cancel_owner(self):
        os.lseek(self.fd, 0, os.SEEK_SET)
        raw = os.read(self.fd, 64).strip()
        if not raw.isdigit():
            raise RuntimeError('worker is starting; retry stop')
        pid = int(raw)
        proc = Path('/proc') / str(pid)
        if pid < 2 or proc.stat().st_uid != os.getuid():
            raise RuntimeError('unexpected worker process')
        args = (proc / 'cmdline').read_bytes().split(b'\0')
        cwd = (proc / 'cwd').resolve()
        expected = {Path(__file__).resolve(),
                    Path(__file__).with_name('bounded_lateral_test.py').resolve(),
                    Path(__file__).with_name('polling_lateral_test.py').resolve(),
                    Path(__file__).with_name('ab_no_map.py').resolve(),
                    Path(__file__).with_name('ab_map.py').resolve()}
        matching = False
        for arg in args[1:]:
            if not arg or not arg.endswith(b'.py'):
                continue
            path = Path(os.fsdecode(arg))
            if not path.is_absolute():
                path = cwd / path
            matching |= path.resolve() in expected
        if not matching:
            raise RuntimeError('lock process is not this chassis worker')
        os.kill(pid, signal.SIGTERM)
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            try:
                self.fcntl.flock(self.fd, self.fcntl.LOCK_EX | self.fcntl.LOCK_NB)
                return pid
            except BlockingIOError:
                time.sleep(0.05)
        raise RuntimeError('active worker has not exited; physical stop is unconfirmed')

    def close(self):
        os.close(self.fd)


def execute_main(args):
    cancelled = [False]
    def is_cancelled():
        if not cancelled[0] and args.watch_stdin and args.action != 'keyboard':
            ready, _, _ = select.select([sys.stdin.fileno()], [], [], 0)
            if ready and not os.read(sys.stdin.fileno(), 1024):
                cancelled[0] = True
        return cancelled[0]
    signals = [signal.SIGINT, signal.SIGTERM]
    if hasattr(signal, 'SIGHUP'):
        signals.append(signal.SIGHUP)
    previous = {s: signal.signal(s, lambda *_: cancelled.__setitem__(0, True))
                for s in signals}
    controller = ChassisController(execute=True, cancelled=is_cancelled,
                                  handoff_task_id=args.handoff_task_id,
                                  stop_only=args.action == 'stop')
    status, error = 'commands_finished', None
    try:
        with controller:
            if args.action == 'velocity':
                controller.run_velocity(Velocity(args.vx, args.vy, args.wz), args.seconds)
            elif args.action == 'keyboard':
                keyboard_loop(controller, args.speed, args.turn_speed, args.max_seconds)
            else:
                controller.stop()
    except (KeyboardInterrupt, InterruptedError) as exc:
        status, error = 'cancelled', str(exc)
    except Exception as exc:
        status, error = 'failed', str(exc)
    finally:
        for s, handler in previous.items():
            signal.signal(s, handler)
    if controller.cleanup_errors and status == 'commands_finished':
        status = 'cleanup_failed'
    if controller.requested and controller.last_stop_sent == 0:
        status = 'stop_unconfirmed'
    print(json.dumps(dict(status=status, error=error, sent=controller.sent,
                          stop_sent=controller.stop_sent, final_stop_sent=controller.last_stop_sent,
                          stop_errors=controller.stop_errors,
                          cleanup_errors=controller.cleanup_errors,
                          control_request_sent=controller.control_request_sent,
                          owned_task_id=controller.owned_task_id,
                          released_task_id=controller.released_task_id,
                          exclusive_control_verified=False,
                          unassociated_request_may_remain_active=(controller.control_request_sent
                                                                 and controller.owned_task_id is None),
                          physical_motion_verified=False), indent=2))
    return 0 if status == 'commands_finished' else 1


def main(argv=None):
    p = build_parser()
    args = p.parse_args(argv)
    try:
        validate_args(args)
    except ValueError as exc:
        p.error(str(exc))
    if not args.execute:
        result = dict(mode='preview', sdk_imported=False, options=vars(args),
                      physical_motion_tested=False)
        if args.action == 'velocity':
            result['command_axes'] = Velocity(args.vx, args.vy, args.wz).axes()
            result['nominal_displacement_m'] = [args.vx * args.seconds, args.vy * args.seconds]
            result['distance_is_command_estimate'] = True
        print(json.dumps(result, indent=2))
        return 0
    session = None
    try:
        session = SessionLock()
        if not session.acquired:
            if args.action != 'stop':
                raise RuntimeError('a chassis worker is already active; stop it first')
            pid = session.cancel_owner()
            print(json.dumps(dict(status='active_worker_exited', worker_pid=pid,
                                  physical_motion_verified=False)))
            return 0
        session.record_owner()
        return execute_main(args)
    except Exception as exc:
        print(json.dumps(dict(status='failed', error=str(exc), physical_motion_verified=False)))
        return 1
    finally:
        if session is not None:
            session.close()


if __name__ == '__main__':
    raise SystemExit(main())
