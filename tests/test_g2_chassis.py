import contextlib
import io
import math
import types
import tempfile
from pathlib import Path
import unittest
from unittest.mock import patch
import g2_chassis as m
import control_from_ubuntu as launcher
import subprocess
import shlex


class Clock:
    def __init__(self):
        self.now = 0.0
        self.fail_stop_sleep = False
    def clock(self):
        return self.now
    def sleep(self, seconds):
        if self.fail_stop_sleep and seconds == 0.04:
            raise RuntimeError('stop sleep interrupted')
        self.now += seconds


class FakeGDK:
    GDKRes = types.SimpleNamespace(kSuccess=0)
    class Vector3: pass
    class Twist: pass
    def __init__(self, clock):
        self.clock = clock
        self.values, self.times, self.calls = [], [], []
        self.fail_motion = self.fail_stop = self.fail_request = False
        self.fail_dds_release = self.stall_once = False
        self.release_result = 0
        self.current_task = types.SimpleNamespace(id=0, state=0, type=0, message='')
    def gdk_init(self):
        self.calls.append('init')
        return 0
    def Pnc(self): return self
    def request_chassis_control(self, value):
        self.calls.append(('request', value))
        if self.fail_request: raise RuntimeError('request uncertain')
        self.current_task = types.SimpleNamespace(id=100, state=2, type=3, message='')
    def get_task_state(self): return self.current_task
    def cancel_task(self, task_id):
        self.calls.append(('cancel', task_id))
        if self.current_task.id != task_id: raise RuntimeError('wrong task')
        self.current_task = types.SimpleNamespace(id=task_id, state=7, type=3, message='')
    def move_chassis(self, v):
        axes = (v.linear.x, v.linear.y, v.linear.z, v.angular.x, v.angular.y, v.angular.z)
        self.values.append(axes)
        self.times.append(self.clock.now)
        if (self.fail_motion and any(axes)) or (self.fail_stop and not any(axes)):
            raise RuntimeError('send failed')
        if self.stall_once:
            self.clock.now += 0.12
            self.stall_once = False
    def release_dds(self):
        self.calls.append('release_dds')
        if self.fail_dds_release: raise RuntimeError('release failed')
    def gdk_release(self):
        self.calls.append('gdk_release')
        return self.release_result


class ChassisTests(unittest.TestCase):
    def fixture(self):
        clock = Clock()
        gdk = FakeGDK(clock)
        return clock, gdk, m.ChassisController(True, gdk, clock.clock, clock.sleep)

    def test_velocity_contract(self):
        for v in [m.Velocity(0.05, 0.05), m.Velocity(-0.05, 0, 0.1), m.Velocity(0, 0, 0.1), m.ZERO]:
            self.assertEqual(v.axes()[2:5], (0, 0, 0))
        for v in [(0, 0.05, 0.1), (0.1, 0.1, 0), (0, 0, 0.31)]:
            with self.assertRaises(ValueError): m.Velocity(*v)

    def test_nonfinite_inputs(self):
        for value in [float('nan'), float('inf'), -float('inf')]:
            for axis in range(3):
                v = [0.0, 0.0, 0.0]; v[axis] = value
                with self.assertRaises(ValueError): m.Velocity(*v)

    def test_keyboard_geometry(self):
        for key in 'wsadqezc':
            v = m.keyboard_velocity(key, 0.05, 0.1)
            self.assertAlmostEqual(math.hypot(v.vx, v.vy), 0.05)
            self.assertEqual(v.wz, 0)
        self.assertGreater(m.keyboard_velocity('a', 0.05, 0.1).vy, 0)
        self.assertLess(m.keyboard_velocity('d', 0.05, 0.1).vy, 0)
        self.assertEqual(m.keyboard_velocity(' ', 0.05, 0.1), m.ZERO)
        self.assertIsNone(m.keyboard_velocity('?', 0.05, 0.1))

    def test_lease_expiry_and_renewal(self):
        clock, gdk, c = self.fixture()
        with c:
            c.update_velocity(vy=0.05); self.assertEqual(c.tick().vy, 0.05)
            clock.now += 0.2; c.tick()
            clock.now += 0.16; self.assertEqual(c.tick(), m.ZERO)
            c.update_velocity(vy=-0.05); self.assertEqual(c.tick().vy, -0.05)
        self.assertEqual(gdk.values[-5:], [(0, 0, 0, 0, 0, 0)] * 5)

    def test_application_error_stops_and_releases(self):
        _, gdk, c = self.fixture()
        with self.assertRaises(RuntimeError):
            with c:
                c.update_velocity(vx=0.05); c.tick()
                raise RuntimeError('application failed')
        self.assertEqual(c.last_stop_sent, 5)
        self.assertEqual(gdk.calls[-2:], ['release_dds', 'gdk_release'])

    def test_uncertain_control_request_only_sends_zero(self):
        _, gdk, c = self.fixture(); gdk.fail_request = True
        with self.assertRaises(RuntimeError):
            with c: pass
        self.assertEqual(gdk.values, [(0, 0, 0, 0, 0, 0)] * 5)
        self.assertEqual(gdk.calls[-2:], ['release_dds', 'gdk_release'])

    def test_stop_interruption_still_releases(self):
        clock, gdk, c = self.fixture()
        with self.assertRaisesRegex(RuntimeError, 'interrupted'):
            with c: clock.fail_stop_sleep = True
        self.assertEqual(gdk.calls[-2:], ['release_dds', 'gdk_release'])
        self.assertFalse(c.opened)

    def test_last_stop_failure_not_masked_by_previous_stop(self):
        _, gdk, c = self.fixture()
        with c:
            c.stop(); self.assertEqual(c.last_stop_sent, 5)
            gdk.fail_stop = True
        self.assertEqual(c.last_stop_sent, 0)
        self.assertEqual(len(c.stop_errors), 5)

    def test_cleanup_errors_recorded_and_other_release_attempted(self):
        _, gdk, c = self.fixture()
        gdk.fail_dds_release = True; gdk.release_result = 1
        with c: pass
        self.assertEqual(gdk.calls[-2:], ['release_dds', 'gdk_release'])
        self.assertEqual(len(c.cleanup_errors), 2)

    def test_missed_ticks_not_replayed_as_burst(self):
        _, gdk, c = self.fixture()
        with c:
            gdk.stall_once = True
            c.run_velocity(m.Velocity(vy=0.05), 0.5)
            active = [t for t, v in zip(gdk.times, gdk.values) if any(v)]
            self.assertTrue(all(b-a >= 0.019 for a, b in zip(active, active[1:])))

    def test_large_stall_aborts(self):
        clock, _, c = self.fixture()
        with self.assertRaisesRegex(RuntimeError, 'stalled'):
            with c:
                c.update_velocity(vy=0.05); c.tick()
                clock.now += 0.3; c.tick()
        self.assertEqual(c.last_stop_sent, 5)

    def test_timing_validation(self):
        for value in [0, -1, 121, float('nan')]:
            with self.assertRaises(ValueError): m.validate_timing(value)

    def test_all_previews_without_sdk(self):
        for args in [['velocity', '--vy', '0.05', '--seconds', '2'], ['keyboard'], ['stop']]:
            with patch.dict('sys.modules', {'agibot_gdk': None}), contextlib.redirect_stdout(io.StringIO()) as out:
                self.assertEqual(m.main(args), 0)
            self.assertIn('"sdk_imported": false', out.getvalue())

    def test_tick_requires_open_context(self):
        with self.assertRaises(RuntimeError): m.ChassisController().tick()

    def test_explicit_stop_allows_later_new_motion(self):
        clock, _, c = self.fixture()
        with c:
            c.run_velocity(m.Velocity(vx=0.05), 0.1)
            clock.now += 2.0
            c.run_velocity(m.Velocity(vy=0.05), 0.1)
            self.assertEqual(c.last_stop_sent, 5)

    def test_finish_is_not_delayed_to_an_extra_tick(self):
        clock,gdk,c=self.fixture()
        with c:
            start=clock.now
            c.run_velocity(m.Velocity(vy=0.02),2.5)
            stop_index=next(i for i,v in enumerate(gdk.values) if not any(v))
            self.assertAlmostEqual(gdk.times[stop_index]-start,2.5,places=8)

    def test_stall_crossing_finish_is_reported(self):
        clock,gdk,c=self.fixture()
        with self.assertRaisesRegex(RuntimeError,'finish deadline'):
            with c:
                original=clock.sleep
                def delayed(seconds):
                    original(seconds)
                    if 4.9<clock.now<5.01:clock.now+=0.6
                c.sleep=delayed
                c.run_velocity(m.Velocity(vy=0.02),2.5)
        self.assertGreater(c.last_stop_sent,0)

    def test_preflight_failure_requests_no_control_and_no_velocity(self):
        clock,gdk,c=self.fixture()
        constructed=[]
        def pnc():
            constructed.append(True)
            return gdk
        gdk.Pnc=pnc
        def bad():raise RuntimeError('feedback unavailable')
        c.preflight=bad
        with self.assertRaisesRegex(RuntimeError,'feedback'):
            with c:pass
        self.assertFalse(c.requested)
        self.assertEqual(constructed,[])
        self.assertEqual(gdk.values,[])
        self.assertNotIn(('request',0),gdk.calls)

    def test_busy_remote_requires_exact_explicit_handoff(self):
        _,gdk,c=self.fixture()
        gdk.current_task=types.SimpleNamespace(id=22,state=2,type=3,message='remote')
        with self.assertRaisesRegex(RuntimeError,'handoff-task-id'):
            with c:pass
        self.assertEqual(gdk.values,[])
        self.assertNotIn(('request',0),gdk.calls)
        self.assertNotIn(('cancel',22),gdk.calls)

    def test_exact_handoff_and_own_task_release(self):
        _,gdk,c=self.fixture();c.handoff_task_id=22
        gdk.current_task=types.SimpleNamespace(id=22,state=2,type=3,message='remote')
        with c:
            c.run_velocity(m.Velocity(vy=.03),.1)
        self.assertLess(gdk.calls.index(('cancel',22)),gdk.calls.index(('request',0)))
        self.assertIn(('cancel',100),gdk.calls)
        self.assertEqual(c.released_task_id,100)

    def test_navigation_cannot_be_cancelled_by_remote_handoff(self):
        _,gdk,c=self.fixture();c.handoff_task_id=55
        gdk.current_task=types.SimpleNamespace(id=55,state=2,type=1,message='navigation')
        with self.assertRaisesRegex(RuntimeError,'busy'):
            with c:pass
        self.assertEqual(gdk.values,[])
        self.assertFalse(any(isinstance(x,tuple) and x[0]=='cancel' for x in gdk.calls))

    def test_terminal_task_does_not_need_another_cancel(self):
        _,gdk,c=self.fixture()
        gdk.current_task=types.SimpleNamespace(id=22,state=7,type=3,message='')
        with c:pass
        self.assertNotIn(('cancel',22),gdk.calls)
        self.assertIn(('request',0),gdk.calls)

    def test_cleanup_does_not_cancel_replacement_task(self):
        _,gdk,c=self.fixture()
        with c:
            gdk.current_task=types.SimpleNamespace(id=300,state=2,type=1,message='other')
        self.assertNotIn(('cancel',300),gdk.calls)
        self.assertNotIn(('cancel',100),gdk.calls)

    def test_reused_or_stale_id_is_not_claimed_or_cancelled(self):
        _,gdk,c=self.fixture()
        gdk.current_task=types.SimpleNamespace(id=100,state=7,type=3,message='old')
        with self.assertRaises(TimeoutError):
            with c:pass
        self.assertIsNone(c.owned_task_id)
        self.assertNotIn(('cancel',100),gdk.calls)

    def test_stop_can_zero_running_remote_without_requesting_it_again(self):
        clock,gdk,_=self.fixture()
        gdk.current_task=types.SimpleNamespace(id=22,state=2,type=3,message='')
        with m.ChassisController(True,gdk,clock.clock,clock.sleep,stop_only=True) as c:
            with self.assertRaises(ValueError):c.update_velocity(vy=.02)
            self.assertEqual(c.command,m.ZERO)
            c.stop()
        self.assertTrue(gdk.values)
        self.assertTrue(all(not any(v) for v in gdk.values))
        self.assertNotIn(('request',0),gdk.calls)
        self.assertNotIn(('cancel',22),gdk.calls)

    def test_launcher_previews_all_programs_without_network(self):
        for argv in [['velocity','--vy','.03','--seconds','1'],
                     ['--program','no-map'],['--program','map']]:
            with patch.object(launcher.subprocess,'Popen',side_effect=AssertionError('network')), \
                 patch.dict('sys.modules',{'agibot_gdk':None}), contextlib.redirect_stdout(io.StringIO()) as out:
                self.assertEqual(launcher.main(argv),0)
            self.assertIn('"sdk_imported": false',out.getvalue())

    def test_execute_command_building_without_shlex_join(self):
        # Python 3.7 has shlex.quote but no shlex.join. Exercise the real
        # execute branch with SSH mocked, including nested shell quoting.
        old_shlex=types.SimpleNamespace(quote=shlex.quote)
        config="/tmp/robot points/it's $(literal); data.json"
        cases=[(['keyboard','--speed','.03','--turn-speed','.10','--execute'],
                'g2_chassis.py','-tt',None,False),
               (['velocity','--vy','.03','--seconds','1','--execute'],
                'g2_chassis.py','-T',subprocess.PIPE,True),
               (['--program','map','--config',config,'--localized','--execute'],
                'ab_map.py','-T',subprocess.PIPE,True)]
        for argv,worker,tty,stdin,watch in cases:
            with self.subTest(argv=argv),patch.object(launcher,'shlex',old_shlex), \
                 patch.object(launcher.subprocess,'Popen') as popen, \
                 patch.dict('sys.modules',{'agibot_gdk':None}):
                popen.return_value.wait.return_value=0
                popen.return_value.stdin=None
                self.assertEqual(launcher.main(argv),0)
                ssh=popen.call_args[0][0]
                self.assertEqual(ssh[:5],['ssh',tty,'-o','BatchMode=yes','g2'])
                self.assertEqual(len(ssh),6)
                outer=shlex.split(ssh[-1])
                self.assertEqual(outer[:2],['bash','-lc'])
                prefix='source /home/agi/app/env.sh /home/agi/app >/dev/null && exec '
                self.assertTrue(outer[2].startswith(prefix))
                command=shlex.split(outer[2][len(prefix):])
                self.assertEqual(command[:2],['python3',launcher.WORKER.rsplit('/',1)[0]+'/'+worker])
                self.assertEqual('--watch-stdin' in command,watch)
                self.assertEqual(popen.call_args[1]['stdin'],stdin)
                if '--config' in argv:self.assertEqual(command[command.index('--config')+1],config)

    def test_only_one_worker_can_own_session(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'session.lock'
            one = m.SessionLock(path)
            two = m.SessionLock(path)
            try:
                self.assertTrue(one.acquired)
                self.assertFalse(two.acquired)
                one.record_owner()
                self.assertTrue(path.read_text().isdigit())
            finally:
                two.close(); one.close()
            three = m.SessionLock(path)
            self.assertTrue(three.acquired)
            three.close()

    def test_launcher_reaps_stubborn_child(self):
        class Pipe:
            closed = False
            def close(self): self.closed = True
        class Process:
            def __init__(self, pipe):
                self.stdin = Pipe() if pipe else None
                self.calls = []
                self.killed = False
            def wait(self, timeout=None):
                self.calls.append(('wait', timeout))
                if not self.killed: raise subprocess.TimeoutExpired('ssh', timeout)
                return 0
            def terminate(self): self.calls.append('terminate')
            def kill(self): self.calls.append('kill'); self.killed = True
        for pipe in (True, False):
            process = Process(pipe)
            launcher.stop_child(process)
            self.assertEqual(process.calls[-2:], ['kill', ('wait', None)])
            if pipe: self.assertTrue(process.stdin.closed)


if __name__ == '__main__': unittest.main()
