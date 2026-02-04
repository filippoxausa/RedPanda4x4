from http.server import BaseHTTPRequestHandler, HTTPServer
import time

PAGE = """<!doctype html>
<html>
<head><meta name="viewport" content="width=device-width, initial-scale=1"/></head>
<body style="margin:0;display:flex;align-items:center;justify-content:center;height:100vh;font-family:system-ui;background:#222;color:#fff;">
  <div>
    <h2 style="margin:0;">FAKE CAMERA STREAM</h2>
    <p id="t" style="opacity:.8"></p>
  </div>
<script>
  setInterval(()=>document.getElementById('t').innerText = 'Time: ' + new Date().toLocaleTimeString(), 200);
</script>
</body></html>"""

class H(BaseHTTPRequestHandler):
    def do_GET(self):
        self.send_response(200)
        self.send_header("Content-Type","text/html; charset=utf-8")
        self.end_headers()
        self.wfile.write(PAGE.encode("utf-8"))

if __name__ == "__main__":
    print("Fake cam on http://0.0.0.0:8081")
    HTTPServer(("0.0.0.0", 8081), H).serve_forever()
