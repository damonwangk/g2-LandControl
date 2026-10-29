#!/usr/bin/env python3
"""Use Ubuntu's ssh g2 alias to run only the chassis worker on the robot."""
import shlex
import argparse
import importlib
import signal
import subprocess
import sys
from g2_chassis import build_parser, validate_args

WORKER = '/home/agi/Project_codex/g2-chassis-control/g2_chassis.py'


def stop_child(process):
    if process.stdin is not None and not process.stdin.closed:
        process.stdin.close()
    else:
        process.terminate()
    try:
        process.wait(timeout=3)
    except subprocess.TimeoutExpired:
        process.terminate()
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    selector = argparse.ArgumentParser(add_help=False)
    selector.add_argument('--program', choices=['chassis', 'no-map', 'map'], default='chassis')
    selected, argv = selector.parse_known_args(argv)
    module_name = {'chassis':'g2_chassis','no-map':'ab_no_map','map':'ab_map'}[selected.program]
    execute = '--execute' in argv
    if not execute:
        # All default previews are local: no SSH, SDK import or robot contact.
        return importlib.import_module(module_name).main(argv)
    if selected.program == 'chassis':
        p = build_parser()
        args = p.parse_args(argv)
        try:
            validate_args(args)
        except ValueError as exc:
            p.error(str(exc))
        keyboard = args.action == 'keyboard'
    else:
        keyboard = False
    worker = WORKER if selected.program == 'chassis' else WORKER.rsplit('/',1)[0]+'/'+module_name+'.py'
    worker_args = argv + (['--watch-stdin'] if execute and not keyboard else [])
    command = ' '.join(shlex.quote(arg) for arg in ['python3', worker] + worker_args)
    if execute:
        command = 'source /home/agi/app/env.sh /home/agi/app >/dev/null && exec ' + command
    remote = ' '.join(shlex.quote(arg) for arg in ['bash', '-lc', command])
    tty_flag = '-tt' if keyboard else '-T'
    # Keep the remote pipe open for noninteractive execution; EOF asks the
    # worker to stop when this launcher exits. Keyboard uses SSH's remote TTY.
    stdin = subprocess.PIPE if not keyboard else None
    process = subprocess.Popen(['ssh', tty_flag, '-o', 'BatchMode=yes', 'g2', remote], stdin=stdin)
    signals = [signal.SIGTERM]
    if hasattr(signal, 'SIGHUP'):
        signals.append(signal.SIGHUP)
    def interrupted(*_):
        raise KeyboardInterrupt
    previous = {s: signal.signal(s, interrupted) for s in signals}
    try:
        return process.wait()
    except KeyboardInterrupt:
        # Ignore repeat termination signals during bounded child cleanup.
        for s in signals + [signal.SIGINT]:
            if s not in previous:
                previous[s] = signal.getsignal(s)
            signal.signal(s, signal.SIG_IGN)
        stop_child(process)
        return 130
    finally:
        if process.stdin is not None and not process.stdin.closed:
            process.stdin.close()
        for s, handler in previous.items():
            signal.signal(s, handler)


if __name__ == '__main__':
    raise SystemExit(main())
