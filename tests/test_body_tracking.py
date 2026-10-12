import asyncio
import json
import math
from unittest.mock import Mock
import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'quest'))
from body_tracking import arm_fields, skeleton_packet, skeleton_diagnostics


class BodyTrackingTests(unittest.TestCase):
    def arm(self):
        return {'shoulder':[0,1.4,0], 'elbow':[0,1.1,0], 'wrist':[0,1.1,-.25]}

    def test_directions_use_gravity_correct_legacy_coordinates(self):
        values=arm_fields(self.arm())
        self.assertEqual(values[:6],[0,1,0,0,0,1])
        self.assertAlmostEqual(values[6],.3)
        self.assertAlmostEqual(values[7],.25)
        self.assertEqual(values[8],1)

    def test_translation_does_not_change_bone_directions(self):
        arm=self.arm()
        translated={key:[v+offset for v,offset in zip(point,[3,2,-4])] for key,point in arm.items()}
        for a,b in zip(arm_fields(arm),arm_fields(translated)):
            self.assertAlmostEqual(a,b)

    def test_degenerate_nonfinite_and_implausible_points_are_invalid(self):
        for point in ([0,1.4,0],[math.nan,1,0],[2,1,0],[0,1],[True,1,0]):
            arm=self.arm();arm['elbow']=point
            self.assertEqual(arm_fields(arm),[0]*9)

    def test_one_lost_arm_does_not_invalidate_other_arm(self):
        body={'source':'webxr','timestamp':123000,'left':None,'right':self.arm()}
        fields=skeleton_packet(body,123000).decode().split(',')
        self.assertEqual(len(fields),20)
        self.assertEqual(fields[:2],['123','1'])
        self.assertEqual(float(fields[10]),0)
        self.assertEqual(float(fields[19]),1)

    def test_missing_wrong_source_or_mixed_frame_clears_tracking(self):
        for body in (None,{}, {'source':'inferred','timestamp':1,'left':self.arm()},
                     {'source':'webxr','timestamp':0,'left':self.arm()}):
            fields=skeleton_packet(body,1).decode().split(',')
            self.assertEqual(fields[1],'0')
            self.assertTrue(all(float(v)==0 for v in fields[2:]))

    def test_diagnostics_distinguish_missing_payload_and_rejected_poses(self):
        self.assertEqual(skeleton_diagnostics(None, 1), ('missing_body_payload',)*2)
        body = {'source':'webxr', 'timestamp':1, 'left':None, 'right':self.arm(),
                'diagnostics':{'left':'emulated_position'}}
        self.assertEqual(skeleton_diagnostics(body,1), ('emulated_position','valid'))
        body['left'] = self.arm()
        body['left']['elbow'] = [9,9,9]
        self.assertEqual(skeleton_diagnostics(body,1), ('invalid_geometry','valid'))
        self.assertEqual(skeleton_diagnostics(body,2), ('timestamp_mismatch',)*2)

    def test_diagnostics_do_not_trust_client_log_text_or_valid_flag(self):
        body = {'source':'webxr', 'timestamp':1, 'left':None, 'right':None,
                'diagnostics':{'left':'valid','right':'injected\nmessage'}}
        self.assertEqual(skeleton_diagnostics(body,1), ('missing_arm',)*2)

    def test_bridge_forwards_body_and_rejects_duplicate_frames(self):
        from quest_bridge import QuestBridge
        body={'source':'webxr','timestamp':123000,'left':self.arm(),'right':None}
        messages=[{'timestamp':123000,'bodyTracking':body},
                  {'timestamp':123000,'bodyTracking':body},
                  {'timestamp':124000,'bodyTracking':None}]
        class Connection:
            remote_address=('test',0)
            def __aiter__(self):
                async def iterate():
                    for message in messages: yield json.dumps(message)
                return iterate()
        bridge=QuestBridge.__new__(QuestBridge)
        bridge.ws_clients=set();bridge.client_connected=False
        bridge.bimanual=True;bridge.skeleton_sock=Mock();bridge.skeleton_target=('127.0.0.1',47951)
        bridge.left_forwarder=Mock();bridge.right_forwarder=Mock()
        bridge.left_stats=Mock();bridge.right_stats=Mock()
        asyncio.run(bridge.handle_client(Connection()))
        calls=bridge.skeleton_sock.sendto.call_args_list
        self.assertEqual(len(calls),2)
        self.assertEqual(calls[0].args[0].decode().split(',')[1],'1')
        self.assertEqual(calls[1].args[0].decode().split(',')[1],'0')
        self.assertFalse(bridge.client_connected)


if __name__=='__main__': unittest.main()
