import contextlib
import io
import types
import unittest
from unittest.mock import patch
import ab_map as m

NS=types.SimpleNamespace
CONFIG={'map_id':4,'target':{'x':1.,'y':2.,'yaw_rad':.5},'timeout_s':1.}


class Clock:
    def __init__(self):self.t=0.
    def now(self):return self.t
    def sleep(self,d):self.t+=d


class Pnc:
    def __init__(self,states):self.states=list(states);self.index=0;self.requests=[];self.cancels=[]
    def get_task_state(self):
        x=self.states[min(self.index,len(self.states)-1)];self.index+=1
        return NS(id=x[0],state=x[1],type=x[2],message='')
    def normal_navi(self,request):self.requests.append(request)
    def cancel_task(self,task_id):self.cancels.append(task_id)


def request():return NS(target=NS(position=NS(),orientation=NS()))


class MapTests(unittest.TestCase):
    def run_nav(self,states,config=CONFIG,cancelled=lambda:False,map_id=4):
        clock=Clock();p=Pnc(states)
        result=m.navigate(p,NS(get_curr_map=lambda:NS(id=map_id)),NS(NaviReq=request),config,
                          clock=clock.now,sleep=clock.sleep,cancelled=cancelled)
        return result,p

    def test_template_preview_cannot_import_sdk_or_open_lock(self):
        with patch.dict('sys.modules',{'agibot_gdk':None}),patch.object(m,'SessionLock',side_effect=AssertionError()), \
             contextlib.redirect_stdout(io.StringIO()) as out:
            self.assertEqual(m.main([]),0)
            self.assertIn('"mode": "template"',out.getvalue())

    def test_config_rejects_unconfirmed_or_invalid_points(self):
        with self.assertRaises(ValueError):m.validate_config(m.TEMPLATE)
        with self.assertRaises(ValueError):m.validate_config(dict(CONFIG,map_id=True))
        with self.assertRaises(ValueError):m.validate_config(dict(CONFIG,target={'x':float('nan'),'y':0,'yaw_rad':0}))

    def test_new_id_associated_then_success_and_pose_fields(self):
        result,p=self.run_nav([(10,9,1),(10,9,1),(11,1,1),(11,2,1),(11,9,1)])
        self.assertEqual(result['status'],'navigation_reported_success')
        self.assertEqual(result['task_id'],11)
        self.assertEqual(p.requests[0].target.position.x,1)
        self.assertAlmostEqual(p.requests[0].target.orientation.z,.24740395925452294)
        self.assertEqual(p.cancels,[])

    def test_map_mismatch_and_busy_task_send_nothing(self):
        for states,map_id in [([(0,0,0)],9),([(10,2,3)],4)]:
            p=Pnc(states)
            with self.assertRaises(RuntimeError):m.navigate(p,NS(get_curr_map=lambda:NS(id=map_id)),NS(NaviReq=request),CONFIG)
            self.assertEqual(p.requests,[]);self.assertEqual(p.cancels,[])

    def test_failure_and_cancelled_are_not_arrival(self):
        for state,expected in [(8,'failed'),(7,'cancelled')]:
            r,_=self.run_nav([(0,0,0),(42,state,1)])
            self.assertEqual(r['status'],expected)

    def test_timeout_cancels_only_same_associated_navigation(self):
        r,p=self.run_nav([(0,0,0),(42,2,1)])
        self.assertEqual(r['status'],'timeout');self.assertEqual(p.cancels,[42])

    def test_deadline_crossed_before_sleep_does_not_sleep_negative(self):
        values=iter([0., .99, 1.01, 1.02])
        sleeps=[]
        p=Pnc([(0,0,0),(42,2,1)])
        result=m.navigate(p,NS(get_curr_map=lambda:NS(id=4)),NS(NaviReq=request),CONFIG,
                          clock=lambda:next(values),sleep=sleeps.append)
        self.assertEqual(result['status'],'timeout')
        self.assertEqual(sleeps,[0.])
        self.assertEqual(p.cancels,[42])

    def test_replacement_task_is_not_cancelled(self):
        clock=Clock();p=Pnc([(0,0,0),(42,2,1),(99,2,1)])
        with self.assertRaisesRegex(RuntimeError,'changed'):
            m.navigate(p,NS(get_curr_map=lambda:NS(id=4)),NS(NaviReq=request),CONFIG,
                       clock=clock.now,sleep=clock.sleep)
        self.assertEqual(p.cancels,[])

    def test_missing_association_does_not_cancel_unknown_task(self):
        clock=Clock();p=Pnc([(0,0,0)])
        with self.assertRaisesRegex(RuntimeError,'could not be associated') as failure:
            m.navigate(p,NS(get_curr_map=lambda:NS(id=4)),NS(NaviReq=request),CONFIG,
                       clock=clock.now,sleep=clock.sleep)
        self.assertEqual(p.cancels,[])
        self.assertTrue(failure.exception.navigation_request_sent)
        self.assertIsNone(failure.exception.task_id)
        self.assertFalse(failure.exception.cancel_requested)


if __name__=='__main__':unittest.main()
