import contextlib
import io
import unittest
from unittest.mock import patch
import mapping as m


class Slam:
    def __init__(self):self.calls=[]
    def start_mapping(self):self.calls.append('start')
    def stop_mapping(self):self.calls.append('save')
    def cancel_mapping(self):self.calls.append('cancel')


class MappingTests(unittest.TestCase):
    def test_record_saves_after_explicit_finish(self):
        s=Slam();m.record_map(s,finish_prompt=lambda text:None)
        self.assertEqual(s.calls,['start','save'])

    def test_interrupted_record_cancels_unsaved_mapping(self):
        s=Slam()
        def interrupted(text):raise KeyboardInterrupt()
        with self.assertRaises(KeyboardInterrupt):m.record_map(s,finish_prompt=interrupted)
        self.assertEqual(s.calls,['start','cancel'])

    def test_default_and_record_preview_import_no_sdk(self):
        for args in [[],['record'],['switch','--map-id','4']]:
            with patch.dict('sys.modules',{'agibot_gdk':None}),contextlib.redirect_stdout(io.StringIO()) as out:
                self.assertEqual(m.main(args),0)
                self.assertIn('"sdk_imported": false',out.getvalue())


if __name__=='__main__':unittest.main()
