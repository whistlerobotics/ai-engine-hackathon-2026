import base64
import importlib.util
import json
import pathlib
import threading
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.error import HTTPError
from urllib.request import Request, urlopen
from unittest.mock import patch

spec = importlib.util.spec_from_file_location("three_view_proxy", pathlib.Path(__file__).with_name("three_view_proxy.py"))
proxy = importlib.util.module_from_spec(spec)
spec.loader.exec_module(proxy)


def jpeg(width=640, height=360):
    return (b'\xff\xd8\xff\xc0\x00\x0b\x08' + height.to_bytes(2, 'big')
            + width.to_bytes(2, 'big') + b'\x01\x01\x11\x00\xff\xd9')


class CameraTests(unittest.TestCase):
    def test_sizes(self):
        self.assertEqual(proxy.jpeg_size(jpeg()), (640, 360))
        self.assertEqual(proxy.jpeg_size(jpeg(640, 480)), (640, 480))
        for invalid in (b'', b'not a jpeg', b'\xff\xd8\xff\xc0\x00\xff'):
            with self.assertRaises(ValueError):
                proxy.jpeg_size(invalid)

    def test_missing_and_stale_views_fail_closed(self):
        frames = proxy.Frames(.5)
        with patch.object(proxy.time, 'monotonic', return_value=10):
            frames.update('wrist', jpeg())
            with self.assertRaisesRegex(ValueError, 'left'):
                frames.snapshot()
            frames.update('left', jpeg())
            frames.update('right', jpeg())
            self.assertEqual(list(frames.snapshot()), ['wrist', 'left', 'right'])
        with patch.object(proxy.time, 'monotonic', return_value=10.51):
            with self.assertRaisesRegex(ValueError, 'stale'):
                frames.snapshot()

    def test_translation_preserves_other_observation_fields(self):
        frames = proxy.Frames(1)
        for name in proxy.VIEWS:
            frames.update(name, jpeg())
        request = {'episode_id': 'e', 'step': 3, 'images': {'top': 'old'},
                   'proprio': {'joint_pos': [1, 2, 3, 4, 5, 6]}, 'task_id': 't'}
        result = json.loads(proxy.translate(json.dumps(request).encode(), frames))
        self.assertEqual(result['proprio'], request['proprio'])
        self.assertEqual(result['episode_id'], 'e')
        self.assertEqual(result['step'], 3)
        self.assertEqual(set(result['images']), set(proxy.VIEWS))
        self.assertEqual(base64.b64decode(result['images']['wrist']), jpeg())

    def test_bad_frame_does_not_refresh_previous_frame(self):
        frames = proxy.Frames(.5)
        with patch.object(proxy.time, 'monotonic', return_value=10):
            for view in proxy.VIEWS:
                frames.update(view, jpeg())
        with patch.object(proxy.time, 'monotonic', return_value=10.51):
            with self.assertRaises(ValueError):
                frames.update('wrist', jpeg(640, 480))
            with self.assertRaisesRegex(ValueError, 'wrist'):
                frames.snapshot()

    def test_proxy_passes_response_and_echo_headers(self):
        received = []
        class Policy(BaseHTTPRequestHandler):
            def do_POST(self):
                received.append(json.loads(self.rfile.read(int(self.headers['Content-Length']))))
                body = b'{"episode_id":"e","step":3,"action":{"type":"joint_delta","values":[0,0,0,0,0,0],"gripper":1}}'
                self.send_response(200)
                self.send_header('X-RC-Episode-Id', 'e')
                self.send_header('X-RC-Step', '3')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)

        upstream = ThreadingHTTPServer(('127.0.0.1', 0), Policy)
        frames = proxy.Frames(5)
        server = proxy.create_server('127.0.0.1', 0, f'http://127.0.0.1:{upstream.server_port}', frames)
        for instance in (upstream, server):
            threading.Thread(target=instance.serve_forever, daemon=True).start()
        request = Request(f'http://127.0.0.1:{server.server_port}/act',
                          data=json.dumps({'episode_id': 'e', 'step': 3, 'images': {'top': 'legacy'}}).encode(),
                          headers={'Content-Type': 'application/json'})
        try:
            with self.assertRaises(HTTPError) as error:
                urlopen(request)
            self.assertEqual(error.exception.code, 503)
            error.exception.close()
            self.assertEqual(received, [])
            for view in proxy.VIEWS:
                frames.update(view, jpeg())
            with urlopen(request) as response:
                self.assertEqual(response.headers['X-RC-Episode-Id'], 'e')
                self.assertEqual(response.headers['X-RC-Step'], '3')
                self.assertEqual(json.load(response)['action']['gripper'], 1)
            self.assertEqual(set(received[0]['images']), set(proxy.VIEWS))
        finally:
            for instance in (server, upstream):
                instance.shutdown()
                instance.server_close()


if __name__ == '__main__':
    unittest.main()
