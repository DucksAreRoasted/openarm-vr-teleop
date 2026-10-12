"""KER mapping, malformed input and stale USB bridge protection, without hardware."""
import copy
from contextlib import ExitStack
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "script"))
from ker_bridge import load_mapping, make_packet, map_angles
import ker_bridge


class KerMappingTests(unittest.TestCase):
    def setUp(self):
        self.mapping = load_mapping(ROOT / "config/ker.yaml")

    def test_channel_order_direction_offset_and_gripper(self):
        raw = [0.] * 16
        raw[0] = 90
        raw[8] = 45
        raw[7] = -60
        raw[15] = 30
        self.mapping["left"]["signs"][0] = -1
        self.mapping["left"]["zero_degrees"][0] = 15
        self.mapping["left"]["offset_rad"][0] = .1
        target = map_angles(raw, self.mapping)
        self.assertAlmostEqual(target[0], -3.141592653589793/6+.1)
        self.assertAlmostEqual(target[8], 3.141592653589793/2)
        self.assertAlmostEqual(target[7], -.18)
        self.assertAlmostEqual(target[15], .62)

    def test_reject_missing_or_nonfinite_encoder(self):
        for raw in ([0.] * 15, [float("nan")] * 16, [float("inf")] * 16):
            with self.assertRaises(ValueError):
                map_angles(raw, self.mapping)

    def test_trigger_threshold_uses_each_calibrated_gripper_direction(self):
        raw = [0.] * 16
        raw[7], raw[15] = -53.9, 53.9
        self.assertFalse(ker_bridge.trigger_pressed(raw, self.mapping, "left"))
        self.assertFalse(ker_bridge.trigger_pressed(raw, self.mapping, "right"))
        raw[7], raw[15] = -54, 54
        self.assertTrue(ker_bridge.trigger_pressed(raw, self.mapping, "left"))
        self.assertTrue(ker_bridge.trigger_pressed(raw, self.mapping, "right"))

    def test_real_requires_calibration_and_valid_config(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "ker.yaml"
            for change in ("uncalibrated", "duplicate", "sign", "nonfinite", "stroke"):
                mapping = copy.deepcopy(self.mapping)
                mapping["calibrated"] = change != "uncalibrated"
                if change == "duplicate": mapping["left"]["channels"][0] = 0
                if change == "sign": mapping["left"]["signs"][0] = 0
                if change == "nonfinite": mapping["left"]["offset_rad"][0] = float("nan")
                if change == "stroke": mapping["left"]["gripper_closed_degrees"] = 0
                path.write_text(yaml.safe_dump(mapping))
                with self.assertRaises(ValueError, msg=change):
                    load_mapping(path, real=True)
            mapping = copy.deepcopy(self.mapping)
            mapping["calibrated"] = True
            path.write_text(yaml.safe_dump(mapping))
            self.assertTrue(load_mapping(path, real=True)["calibrated"])


class KerTriggerCalibrationTests(unittest.TestCase):
    def test_measured_short_travel_and_opposite_directions(self):
        mapping = load_mapping(ROOT / "config/ker.yaml")
        released, pressed = [0.]*16, [0.]*16
        released[15], released[7] = 2., -3.
        pressed[15], pressed[7] = 37.0689, -34.6849
        result = ker_bridge.trigger_endpoints(mapping, [released]*10, [pressed]*10)
        self.assertFalse(result["calibrated"])
        self.assertEqual(mapping["left"]["gripper_closed_degrees"], 60)
        for side in ("left", "right"):
            self.assertFalse(ker_bridge.trigger_pressed(released, result, side))
            self.assertTrue(ker_bridge.trigger_pressed(pressed, result, side))
        self.assertAlmostEqual(map_angles(pressed, result)[7], .62)
        self.assertAlmostEqual(map_angles(pressed, result)[15], .62)

    def test_static_or_unstable_channels_cannot_calibrate(self):
        mapping = load_mapping(ROOT / "config/ker.yaml")
        released, pressed = [0.]*16, [0.]*16
        pressed[15], pressed[7] = 37., -35.
        with self.assertRaisesRegex(ValueError, "行程"):
            ker_bridge.trigger_endpoints(mapping, [pressed]*10, [pressed]*10)
        noisy = [pressed.copy() for _ in range(10)]
        noisy[-1][15] += 3
        with self.assertRaisesRegex(ValueError, "不稳定"):
            ker_bridge.trigger_endpoints(mapping, [released]*10, noisy)
        with self.assertRaisesRegex(ValueError, "五个"):
            ker_bridge.trigger_endpoints(mapping, [released], [pressed])

    def test_inspection_outputs_fresh_frames_without_udp(self):
        import io
        import json
        journal = io.StringIO()
        stream = Mock()
        stream.recv.side_effect = [
            {"timestamp":1, "angles":[0.]*16},
            {"timestamp":1, "angles":[0.]*16},
            {"timestamp":2, "angles":[1.]*16},
        ]
        with patch("ker_bridge.socket.socket") as socket_mock, \
             patch("ker_bridge.time.sleep"), \
             patch("ker_bridge.time.monotonic", side_effect=[100, 100.1, 100.2, 100.3]), \
             patch("builtins.print") as output:
            ker_bridge.inspect_stream(stream, Mock(side_effect=[True, True, True, False]), output=journal)
        socket_mock.assert_not_called()
        self.assertEqual(output.call_count, 2)
        records = [json.loads(line) for line in journal.getvalue().splitlines()]
        self.assertEqual([r["timestamp"] for r in records], [1, 2])
        self.assertEqual(records[-1]["angles"], [1.]*16)
        self.assertAlmostEqual(records[-1]["pc_monotonic"], 100.3)


class KerDeviceInputTests(unittest.TestCase):
    def test_device_duplicates_do_not_become_fresh_commands(self):
        decoder = ker_bridge.KerDeviceInput()
        frame = {"seq":10, "angles":[0.]*16, "error_mask":0}
        self.assertIsNotNone(decoder.accept(frame))
        self.assertIsNone(decoder.accept(frame))
        self.assertIsNone(decoder.accept({**frame, "angles":[1.]*16}))
        self.assertIsNotNone(decoder.accept({**frame, "seq":11}))

    def test_encoder_faults_rejected_in_both_sdk_formats(self):
        for extra in ({"error_mask":1 << 7}, {"errors":[False]*15+[True]}):
            with self.assertRaisesRegex(RuntimeError, "编码器故障通道"):
                ker_bridge.KerDeviceInput().accept({"angles":[0.]*16, **extra})

    def test_sequence_wrap_allowed_but_device_restart_rejected(self):
        decoder = ker_bridge.KerDeviceInput()
        for seq in (0xfffffffe, 0xffffffff, 0, 1):
            self.assertIsNotNone(decoder.accept({"seq":seq,"angles":[0.]*16}))
        with self.assertRaisesRegex(RuntimeError,"设备重启"):
            decoder.accept({"seq":0,"angles":[0.]*16})

    def test_fw2_timestamp_duplicates_do_not_refresh_watchdog(self):
        decoder = ker_bridge.KerDeviceInput()
        frame = {"timestamp":1234,"angles":[0.]*16,"errors":[False]*16}
        self.assertIsNotNone(decoder.accept(frame))
        self.assertIsNone(decoder.accept(frame))
        self.assertIsNotNone(decoder.accept({**frame,"timestamp":1235}))
        for stamp in (True, -1, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                decoder.accept({**frame,"timestamp":stamp})

    def test_compact_angles_and_malformed_health_metadata(self):
        decoder = ker_bridge.KerDeviceInput()
        self.assertEqual(decoder.accept({"angles_cd":[125]*16})["angles"], [1.25]*16)
        for extra in ({"error_mask":-1}, {"error_mask":65536}, {"errors":[False]},
                      {"seq":True}, {"seq":-1}):
            with self.assertRaises(ValueError):
                ker_bridge.KerDeviceInput().accept({"angles":[0.]*16,**extra})


class KerUsbPacketReadTests(unittest.TestCase):
    def test_sdk_preserves_independent_gripper_channel_updates(self):
        try:
            from openarm_ker.ker_stream import KERStream
        except ImportError:
            self.skipTest("requires official SDK")
        import struct
        stream = KERStream(transport="usb")
        stream._fmt = "<I16f16?"
        stream._fields = [{"key":"timestamp","count":1}, {"key":"angles","count":16},
                          {"key":"errors","count":16}]
        decoder = ker_bridge.KerDeviceInput()
        previous = [0.]*16
        for seq, channel in enumerate((7, 15, 7, 15), 1):
            raw = previous.copy()
            raw[channel] += seq
            payload = struct.pack(stream._fmt, seq, *raw, *([False]*16))
            checksum = 0
            for byte in payload:
                checksum ^= byte
            frame = stream._parse_stream_packet(b"\xa5\x5a"+payload+bytes([checksum]))
            accepted = decoder.accept(frame)
            self.assertEqual(accepted["angles"], raw)
            changed = [i for i in range(16) if accepted["angles"][i] != previous[i]]
            self.assertEqual(changed, [channel])
            previous = raw

    def test_reads_endpoint_packets_and_keeps_serial_unchanged(self):
        import types
        stream = types.SimpleNamespace(_read_raw=Mock(return_value=b"data"),
                                       _ep_in=types.SimpleNamespace(wMaxPacketSize=64))
        original = stream._read_raw
        ker_bridge.configure_usb_packet_reads(stream,"usb")
        self.assertEqual(stream._read_raw(4096),b"data")
        original.assert_called_once_with(64)
        stream._read_raw(12)
        original.assert_called_with(12)
        serial = types.SimpleNamespace(_read_raw=Mock())
        original = serial._read_raw
        ker_bridge.configure_usb_packet_reads(serial,"serial")
        self.assertIs(serial._read_raw,original)

    def test_sdk_usb_chunks_reach_two_second_cpp_gate(self):
        # Run with .venv-ker/bin/python to exercise the pinned SDK itself.
        try:
            from openarm_ker.ker_stream import KERStream
        except ImportError:
            self.skipTest("run this SDK regression with .venv-ker/bin/python")
        import struct
        import types
        released, pressed = [0.]*16, [0.]*16
        pressed[7], pressed[15] = -34.6849, 37.0689
        mapping = ker_bridge.trigger_endpoints(load_mapping(ROOT / "config/ker.yaml"),
                                              [released]*10, [pressed]*10)
        wire = bytearray()
        for n in range(300):
            angles = pressed if n >= 10 else released
            payload = struct.pack("<I16f16?", n*10,*angles,*([False]*16))
            checksum = 0
            for value in payload: checksum ^= value
            wire.extend(b"\xa5\x5a"+payload+bytes([checksum]))
        class Usb:
            def read(self, endpoint, size, timeout):
                # Model a device supplying full USB packets. A large request can
                # time out instead of returning those partial packets to PyUSB.
                if size > 64:
                    raise AssertionError("large bulk read can lose partial timeout data")
                data = bytes(wire[:size]); del wire[:size]
                return data
        stream = KERStream(transport="usb")
        stream._dev = Usb()
        stream._ep_in = types.SimpleNamespace(bEndpointAddress=0x81,wMaxPacketSize=64)
        stream._fmt = "<I16f16?"
        stream._fields = [{"key":"timestamp","count":1},{"key":"angles","count":16},
                          {"key":"errors","count":16}]
        stream._packet_size = 2+struct.calcsize(stream._fmt)+1
        ker_bridge.configure_usb_packet_reads(stream,"usb")
        frames = []
        while wire: frames.extend(stream._read_all())
        self.assertEqual(len(frames),300)
        decoder = ker_bridge.KerDeviceInput()
        packets = []
        for n, frame in enumerate(frames):
            data = decoder.accept(frame)
            self.assertIsNotNone(data)
            packets.append(make_packet(n,100+n*.01,map_angles(data["angles"],mapping),
                left_pressed=ker_bridge.trigger_pressed(data["angles"],mapping,"left"),
                right_pressed=ker_bridge.trigger_pressed(data["angles"],mapping,"right")))
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory)/"gate.cpp"
            binary = Path(directory)/"gate"
            source.write_text(r'''#include <io/ker_joint_input.hpp>
#include <cassert>
#include <iostream>
int main() {
    KerJointInput gate;
    KerJointInput::Pose lo,hi;lo.fill(-3.14);hi.fill(3.14);
    std::string packet;unsigned n=0;
    while(std::getline(std::cin,packet)) {
        assert(gate.accept(packet,100+n*.01,lo,hi,true));
        if(n<210) assert(!gate.armed());
        if(n>211) assert(gate.armed());
        ++n;
    }
    assert(n==300 && gate.armed());
}
''')
            subprocess.run(["c++","-std=c++17","-I",str(ROOT/"src"),str(source),"-o",str(binary)],check=True)
            subprocess.run([str(binary)],input=b"\n".join(packets)+b"\n",check=True)


class KerProtocolTests(unittest.TestCase):
    def test_controller_requires_continuous_dual_trigger_hold(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "arming.cpp"
            binary = Path(directory) / "arming"
            source.write_text(r'''
#include <io/ker_joint_input.hpp>
#include <cassert>
#include <iomanip>
std::string packet(unsigned seq,double time,bool left,bool right) {
    std::ostringstream out;out<<std::setprecision(15)<<"KER2,"<<seq<<","<<time<<","<<left<<","<<right;
    for(int i=0;i<16;++i) out<<","<<(i%8==7 ? -.98 : 0);
    return out.str();
}
int main() {
    KerJointInput::Pose lo,hi;lo.fill(-2);hi.fill(2);
    auto step=[&](KerJointInput& gate,unsigned seq,double t,bool l,bool r) {
        assert(gate.accept(packet(seq,t,l,r),t,lo,hi,false));
    };
    KerJointInput gate;
    for(unsigned n=0;n<16;++n) {step(gate,n,100+n*.125,true,true);assert(!gate.armed());}
    assert(gate.arming_hold_seconds(101.875)==1.875);
    assert(gate.arming_hold_seconds(102.2)==0); // stale input must not show a progressing hold
    assert(!gate.accept(packet(15,101.875,false,false),101.9,lo,hi,false));
    step(gate,16,101.999,true,true);assert(!gate.armed());
    step(gate,17,102,true,true);assert(gate.armed()); // exactly two seconds
    step(gate,18,102.125,false,false);assert(gate.armed()); // startup latch, not a deadman switch
    KerJointInput single;
    for(unsigned n=0;n<40;++n) {step(single,n,200+n*.125,true,false);assert(!single.armed());}
    KerJointInput interrupted;
    for(unsigned n=0;n<16;++n) step(interrupted,n,300+n*.125,true,true);
    step(interrupted,16,302,true,false);assert(!interrupted.armed());
    for(unsigned n=0;n<16;++n) {step(interrupted,n+17,302.125+n*.125,true,true);assert(!interrupted.armed());}
    step(interrupted,33,304.125,true,true);assert(interrupted.armed());
    KerJointInput gap;
    for(unsigned n=0;n<9;++n) step(gap,n,400+n*.125,true,true);
    for(unsigned n=0;n<16;++n) {step(gap,n+9,402+n*.125,true,true);assert(!gap.armed());}
    step(gap,25,404,true,true);assert(gap.armed());
    KerJointInput restarted;assert(!restarted.armed());
    auto legacy=packet(1,500,true,true);legacy.replace(0,4,"KER1");
    bool rejected=false;
    try {restarted.accept(legacy,500,lo,hi,false);}catch(const std::exception&) {rejected=true;}
    assert(rejected && !restarted.armed());
}
''')
            subprocess.run(["c++", "-std=c++17", "-I", str(ROOT / "src"), str(source), "-o", str(binary)], check=True)
            subprocess.run([str(binary)], check=True)

    def test_packet_validation_units_and_watchdog(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "ker.cpp"
            binary = Path(directory) / "ker"
            source.write_text(r'''
#include <io/ker_joint_input.hpp>
#include <cassert>
#include <iostream>
int main() {
    KerJointInput::Pose lo,hi;lo.fill(-2);hi.fill(2);
    lo[7]=lo[15]=0;hi[7]=hi[15]=.044;
    KerJointInput input;
    std::string packet;std::getline(std::cin,packet);
    assert(input.accept(packet,100,lo,hi,true));
    assert(std::abs(input.target()[7]-.044)<1e-9);
    assert(std::abs(input.target()[15])<1e-9);
    assert(input.fresh(100.2));assert(!input.fresh(100.251));
    assert(!input.accept(packet,100.1,lo,hi,true)); // duplicate cannot renew watchdog
    KerJointInput old;assert(!old.accept(packet,101,lo,hi,true));
    KerJointInput future;assert(!future.accept(packet,99,lo,hi,true));
    for (const auto& bad : {packet+",",packet+",0",std::string("KER1,0,nan"),
                           std::string("KER2,0,100"),std::string("KER1,-1,100")}) {
        bool rejected=false;
        try {KerJointInput test;test.accept(bad,100,lo,hi,true);}
        catch(const std::exception&) {rejected=true;}
        assert(rejected);
    }
    hi[0]=-.1;
    KerJointInput standby;
    assert(standby.accept(packet,100,lo,hi,true));
    assert(!standby.pose_valid() && !standby.armed());
    assert(standby.pose_error().find("left joint 1 q=")!=std::string::npos);
}
''')
            subprocess.run(["c++", "-std=c++17", "-I", str(ROOT / "src"), str(source), "-o", str(binary)], check=True)
            target = [0.] * 16
            target[7], target[15] = .62, -.98
            subprocess.run([str(binary)], input=make_packet(1, 100, target)+b"\n", check=True)

    def test_invalid_startup_pose_resets_hold_and_active_violation_stops(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "limits.cpp"
            binary = Path(directory) / "limits"
            source.write_text(r'''
#include <io/ker_joint_input.hpp>
#include <cassert>
#include <iomanip>
std::string packet(unsigned seq,double time,double left,double right) {
    std::ostringstream out;out<<std::setprecision(15)<<"KER2,"<<seq<<","<<time<<",1,1";
    for(int i=0;i<16;++i) out<<","<<(i==1 ? left : i==8 ? right : i%8==7 ? -.98 : 0);
    return out.str();
}
int main() {
    KerJointInput::Pose lo,hi;lo.fill(-2);hi.fill(2);hi[1]=.17453267320510335;
    KerJointInput input;
    // An unpowered out-of-range preparation pose must stay alive without arming.
    for(unsigned n=0;n<20;++n) {
        assert(input.accept(packet(n,100+n*.125,.8,2.5),100+n*.125,lo,hi,false));
        assert(!input.armed() && !input.pose_valid());
    }
    assert(input.pose_error().find("left joint 2 q=0.8")!=std::string::npos);
    assert(input.pose_error().find("right joint 1 q=2.5")!=std::string::npos);
    assert(input.pose_error().find("limits=[")!=std::string::npos);
    // Invalid samples also interrupt a partially completed two-second confirmation.
    for(unsigned n=0;n<9;++n) {
        input.accept(packet(n+20,103+n*.125,0,0),103+n*.125,lo,hi,false);
        assert(!input.armed());
    }
    input.accept(packet(29,104.125,.8,0),104.125,lo,hi,false);assert(!input.armed());
    for(unsigned n=0;n<16;++n) {
        input.accept(packet(n+30,104.25+n*.125,0,0),104.25+n*.125,lo,hi,false);
        assert(input.pose_valid() && input.pose_error().empty() && !input.armed());
    }
    input.accept(packet(46,106.25,0,0),106.25,lo,hi,false);assert(input.armed());
    bool rejected=false;
    try {input.accept(packet(47,106.375,.8,0),106.375,lo,hi,false);}
    catch(const std::exception& error) {
        rejected=true;
        assert(std::string(error.what()).find("left joint 2 q=0.8")!=std::string::npos);
    }
    assert(rejected);
}
''')
            subprocess.run(["c++", "-std=c++17", "-I", str(ROOT / "src"), str(source), "-o", str(binary)], check=True)
            subprocess.run([str(binary)], check=True)


class KerUsbBridgeTests(unittest.TestCase):
    def test_device_probe_closes_without_streaming_or_opening_udp(self):
        import types
        stream = Mock()
        stream.metadata = {"hw": "KER-test"}
        sdk = types.ModuleType("openarm_ker.ker_stream")
        sdk.KERStream = Mock(return_value=stream)
        sdk.CMD_STREAM = b"\x02"
        with patch.dict(sys.modules, {"openarm_ker": types.ModuleType("openarm_ker"),
                                      "openarm_ker.ker_stream": sdk}), \
             patch.object(sys, "argv", ["ker_bridge", "--probe"]), \
             patch.object(ker_bridge.sys, "version_info", (3, 12)), \
             patch("ker_bridge.socket.socket") as socket_factory, \
             patch("ker_bridge.signal.signal"):
            self.assertEqual(ker_bridge.main(), 0)
        stream.connect.assert_called_once()
        stream.close.assert_called_once()
        stream.send_command.assert_not_called()
        socket_factory.assert_not_called()

    def test_usb_permission_error_has_actionable_fix(self):
        stream = Mock()
        stream.connect.side_effect = PermissionError(13, "Access denied (insufficient permissions)")
        with self.assertRaisesRegex(RuntimeError, "sudo bash scripts/configure_ker_usb.sh"):
            ker_bridge.connect_stream(stream, "usb")
        stream.send_command.assert_not_called()

    def test_other_device_errors_are_preserved(self):
        stream = Mock()
        error = OSError(19, "No such device")
        stream.connect.side_effect = error
        with self.assertRaises(OSError) as caught:
            ker_bridge.connect_stream(stream, "usb")
        self.assertIs(caught.exception, error)
        stream.connect.side_effect = PermissionError(13, "serial permission denied")
        with self.assertRaises(PermissionError):
            ker_bridge.connect_stream(stream, "serial")

    def test_usb_reads_newest_packet_and_closes_on_disconnect(self):
        # Exercise the bridge loop without opening USB or UDP in the test environment.
        import types
        stream = Mock()
        stream.metadata = {"hw": "test"}
        old = {"angles": [0.] * 16}
        new = {"angles": [10.] * 16}
        new["angles"][7] = -30
        new["angles"][15] = 30
        stream.recv.side_effect = [old, new, None, None]
        sdk = types.ModuleType("openarm_ker.ker_stream")
        sdk.KERStream = Mock(return_value=stream)
        sdk.CMD_STREAM = b"\x02"
        sender = Mock()
        sender.__enter__ = Mock(return_value=sender)
        sender.__exit__ = Mock(return_value=False)
        with ExitStack() as stack:
            stack.enter_context(patch.dict(sys.modules, {"openarm_ker": types.ModuleType("openarm_ker"),
                                                         "openarm_ker.ker_stream": sdk}))
            stack.enter_context(patch.object(sys, "argv", ["ker_bridge", "--udp-port", "47932"]))
            stack.enter_context(patch.object(ker_bridge.sys, "version_info", (3, 12)))
            stack.enter_context(patch("ker_bridge.socket.socket", return_value=sender))
            stack.enter_context(patch("ker_bridge.signal.signal"))
            stack.enter_context(patch("ker_bridge.time.monotonic", side_effect=[100, 100.1, 102]))
            stack.enter_context(patch("ker_bridge.time.sleep"))
            self.assertEqual(ker_bridge.main(), 1)
        sdk.KERStream.assert_called_once_with(transport="usb", port="/dev/ttyACM0", auto_resume=False)
        stream.connect.assert_called_once()
        stream.close.assert_called_once()
        sender.sendto.assert_called_once()
        packet, destination = sender.sendto.call_args.args
        self.assertEqual(destination, ("127.0.0.1", 47932))
        expected = make_packet(0, 100.1, map_angles(new["angles"], load_mapping(ROOT / "config/ker.yaml")))
        self.assertEqual(packet, expected)

    def test_draining_usb_queue_preserves_brief_trigger_release(self):
        import types
        stream = Mock()
        stream.metadata = {}
        released = {"angles": [0.] * 16}
        pressed = {"angles": [0.] * 16}
        pressed["angles"][7], pressed["angles"][15] = -60, 60
        stream.recv.side_effect = [released, pressed, None, None]
        sdk = types.ModuleType("openarm_ker.ker_stream")
        sdk.KERStream = Mock(return_value=stream)
        sdk.CMD_STREAM = b"\x02"
        sender = Mock()
        sender.__enter__ = Mock(return_value=sender)
        sender.__exit__ = Mock(return_value=False)
        with patch.dict(sys.modules, {"openarm_ker": types.ModuleType("openarm_ker"),
                                      "openarm_ker.ker_stream": sdk}), \
             patch.object(sys, "argv", ["ker_bridge"]), \
             patch.object(ker_bridge.sys, "version_info", (3, 12)), \
             patch("ker_bridge.socket.socket", return_value=sender), \
             patch("ker_bridge.signal.signal"), \
             patch("ker_bridge.time.monotonic", side_effect=[100, 100.1, 102]), \
             patch("ker_bridge.time.sleep"):
            self.assertEqual(ker_bridge.main(), 1)
        packet = sender.sendto.call_args.args[0].decode().split(",")
        self.assertEqual(packet[3:5], ["0", "0"])
        self.assertAlmostEqual(float(packet[12]), .62) # newest pose retained


if __name__ == "__main__":
    unittest.main()
