"""Exercise sensor identity checks with synthetic identities and no I/O."""
from pathlib import Path
import struct
import sys
import unittest
from unittest.mock import patch

RUNTIME = Path(__file__).resolve().parents[1] / "runtime"
sys.path.insert(0, str(RUNTIME))
sys.path.insert(0, str(RUNTIME / "sensors"))
import hardware
import radar_device


class LocalIdentity(unittest.TestCase):
    def test_live_preflight_requires_each_identity(self):
        for missing, present in (("FAST_LIVO2_RADAR_SERIAL", "FAST_LIVO2_CAMERA_SERIAL"),
                                 ("FAST_LIVO2_CAMERA_SERIAL", "FAST_LIVO2_RADAR_SERIAL")):
            with self.subTest(missing=missing), patch.dict("os.environ", {present: "fixture"}, clear=True):
                with self.assertRaisesRegex(RuntimeError, missing):
                    hardware.preflight()

    def test_unconfigured_radar_opens_no_socket(self):
        with patch.object(radar_device, "SERIAL", ""), patch("socket.socket") as socket:
            with self.assertRaisesRegex(RuntimeError, "FAST_LIVO2_RADAR_SERIAL"):
                radar_device.Device()
            socket.assert_not_called()

    def test_radar_serial_mismatch_and_masking(self):
        value = b"fixture-radar\0"
        reply = struct.pack("<BHHH", 0, 1, 0x8000, len(value)) + value
        device = object.__new__(radar_device.Device)
        with patch.object(device, "request", return_value=reply):
            with patch.object(radar_device, "SERIAL", "fixture-other"):
                with self.assertRaisesRegex(RuntimeError, "Wrong radar serial"):
                    device.query([0x8000])
            with patch.object(radar_device, "SERIAL", "fixture-radar"):
                self.assertEqual(device.query([0x8000])["values"]["serial"], "verified")


if __name__ == "__main__":
    unittest.main()
