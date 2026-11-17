"""Local ESP32-CAM snapshot service. Python 3 standard library only."""
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from urllib.request import Request, urlopen
from urllib.parse import urlparse, parse_qs
from pathlib import Path
from datetime import datetime
import json, re, os

CAMERA_CAPTURE_URL = 'http://172.20.10.4/capture'  # Change here if ESP32-CAM IP changes
SAVE_DIR = Path(r'C:\AnhAnNinh') if os.name == 'nt' else Path.home() / 'AnhAnNinh'
HOST, PORT = '127.0.0.1', 8765
SAVE_DIR.mkdir(parents=True, exist_ok=True)

class Handler(BaseHTTPRequestHandler):
    def reply(self, status, data, content_type='application/json; charset=utf-8'):
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.end_headers()
        self.wfile.write(data if isinstance(data, bytes) else json.dumps(data, ensure_ascii=False).encode('utf-8'))
    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()
    def do_GET(self):
        url = urlparse(self.path)
        args = parse_qs(url.query)
        if url.path == '/health':
            return self.reply(200, {'ok': True, 'folder': str(SAVE_DIR)})
        if url.path == '/capture':
            category = re.sub('[^a-zA-Z0-9_-]', '_', args.get('type', ['ALERT'])[0])[:32]
            try:
                req = Request(CAMERA_CAPTURE_URL, headers={'User-Agent': 'Nhom20-Snapshot/1.0'})
                with urlopen(req, timeout=12) as response:
                    content = response.read(5 * 1024 * 1024 + 1)
                if len(content) > 5 * 1024 * 1024 or not content.startswith(b'\xff\xd8'):
                    raise ValueError('Camera did not return a valid JPEG')
                filename = f'{datetime.now():%Y%m%d_%H%M%S_%f}_{category}.jpg'
                (SAVE_DIR / filename).write_bytes(content)
                return self.reply(200, {'ok': True, 'filename': filename, 'path': str(SAVE_DIR / filename), 'url': f'http://{HOST}:{PORT}/image?file={filename}'})
            except Exception as exc:
                return self.reply(502, {'ok': False, 'error': str(exc)})
        if url.path == '/image':
            filename = args.get('file', [''])[0]
            if not re.fullmatch(r'[a-zA-Z0-9_-]+\.jpg', filename):
                return self.reply(400, {'error': 'Invalid filename'})
            file = SAVE_DIR / filename
            if not file.is_file():
                return self.reply(404, {'error': 'Image not found'})
            return self.reply(200, file.read_bytes(), 'image/jpeg')
        return self.reply(404, {'error': 'Not found'})

if __name__ == '__main__':
    print('NHOM 20 - AUTO SNAPSHOT SERVER')
    print('Camera:', CAMERA_CAPTURE_URL)
    print('Save to:', SAVE_DIR)
    print(f'Listening: http://{HOST}:{PORT}/health')
    print('Keep this window open while using Dashboard. Ctrl+C to stop.')
    ThreadingHTTPServer((HOST, PORT), Handler).serve_forever()
