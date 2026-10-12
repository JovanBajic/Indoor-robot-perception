"""Test pipeline dataset validation, failed-stage logging and detection of occupied

viewer ports.
"""

import importlib.util
import json
import os
from pathlib import Path
import socket
import sys
import tempfile
import unittest

import numpy as np
from PIL import Image

spec = importlib.util.spec_from_file_location(
    'room_pipeline', Path(__file__).parents[1] / 'scripts/room_pipeline.py')
pipeline = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pipeline)


class PipelineTest(unittest.TestCase):
    def test_preflight_rejects_rgb_only_and_mismatched_depth(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            segment = root / 'd455/segment_000'
            segment.mkdir(parents=True)
            manifest = segment / 'frames.jsonl'
            record = {'stamp_ns': 1}
            manifest.write_text(json.dumps(record))
            with self.assertRaisesRegex(ValueError, 'RGB-only'):
                pipeline.validate_capture(root)
            record.update(depth='depth.npy', depth_stamp_ns=2)
            manifest.write_text(json.dumps(record))
            with self.assertRaisesRegex(ValueError, 'timestamps'):
                pipeline.validate_capture(root)
            np.save(segment / 'depth.npy', np.ones((16, 16), dtype=np.float32))
            Image.new('RGB', (16, 16)).save(segment / 'rgb.png')
            record.update(depth_stamp_ns=1, height=16, width=16, rgb='rgb.png',
                          K=[10, 0, 8, 0, 10, 8, 0, 0, 1], D=[0] * 5,
                          world_frame='world', position_m=[0, 0, 0],
                          quaternion_xyzw=[0, 0, 0, 1])
            manifest.write_text('\n'.join(json.dumps(record) for _ in range(8)))
            self.assertEqual(pipeline.validate_capture(root), 8)

    def test_failed_stage_preserves_output_and_stops(self):
        with tempfile.TemporaryDirectory() as tmp:
            log = Path(tmp) / 'progress.log'
            with self.assertRaisesRegex(RuntimeError, 'exit 7'):
                pipeline.run_stage([sys.executable, '-c',
                                    'print("diagnostic"); raise SystemExit(7)'], log, os.environ)
            self.assertIn('diagnostic', log.read_text())

    def test_busy_viewer_port(self):
        with socket.socket() as sock:
            sock.bind(('127.0.0.1', 0))
            with self.assertRaisesRegex(ValueError, 'busy'):
                pipeline.check_port(sock.getsockname()[1])


if __name__ == '__main__':
    unittest.main()
