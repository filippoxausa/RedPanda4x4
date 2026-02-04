from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import urlparse, parse_qs
import time

last = {"x": 0, "y": 0, "z": 0, "t": time.time()}

INDEX_HTML = r"""<!doctype html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1"/>
  <style>
    body{font-family:system-ui;margin:0;display:flex;justify-content:center;}
    .wrap{width:min(900px, 100%); padding:12px; display:flex; flex-direction:column; gap:12px;}
    .cam{
      width:100%;
      aspect-ratio: 16 / 9;
      border-radius:14px;
      overflow:hidden;
      box-shadow:0 6px 18px rgba(0,0,0,.15);
      background:#111;
    }
    iframe{width:100%; height:100%; border:0;}
    .pad{display:grid; grid-template-columns:110px 110px 110px; grid-template-rows:110px 110px 110px; gap:14px; justify-content:center;}
    button{
      font-size:42px; border-radius:22px; border:none;
      box-shadow:0 6px 18px rgba(0,0,0,.15);
      user-select:none; -webkit-user-select:none;
      touch-action:none;
    }
    .empty{visibility:hidden;}
    #status{margin:0; font-size:14px; opacity:.75; text-align:center;}
    .hint{font-size:12px; opacity:.6; text-align:center;}
  </style>
</head>
<body>
  <div class="wrap">
    <div class="cam">
      <iframe id="cam"></iframe>
    </div>

    <div class="pad">
      <div class="empty"></div>
      <button id="up">↑</button>
      <div class="empty"></div>

      <button id="left">←</button>
      <div class="empty"></div>
      <button id="right">→</button>

      <div class="empty"></div>
      <button id="down">↓</button>
      <div class="empty"></div>
    </div>

    <p id="status">X=0 Y=0 Z=0</p>
    <div class="hint">Tieni premuto per muovere, rilascia per fermare.</div>
  </div>

<script>
  const FULL = 16000;

  // ✅ Fake-cam: usa lo stesso host della pagina ma porta 8081
  // Esempio: se apri http://172.20.10.3:8080, allora iframe -> http://172.20.10.3:8081
  document.getElementById("cam").src = "http://" + location.hostname + ":8081";

  async function sendXYZ(x, y, z=0){
    document.getElementById("status").innerText = `X=${x} Y=${y} Z=${z}`;
    const url = `/cmd?x=${encodeURIComponent(x)}&y=${encodeURIComponent(y)}&z=${encodeURIComponent(z)}`;
    try { await fetch(url, {cache:"no-store"}); } catch(e){}
  }

  function bindHoldXYZ(id, x, y){
    const el = document.getElementById(id);

    const down = (e)=>{ e.preventDefault(); sendXYZ(x, y, 0); };
    const up   = (e)=>{ e.preventDefault(); sendXYZ(0, 0, 0); };

    el.addEventListener("touchstart", down, {passive:false});
    el.addEventListener("touchend",   up,   {passive:false});
    el.addEventListener("touchcancel",up,   {passive:false});

    el.addEventListener("mousedown", down);
    el.addEventListener("mouseup",   up);
    el.addEventListener("mouseleave",up);
  }

  // Mapping direzioni -> (X,Y)
  bindHoldXYZ("up",    0,  FULL);
  bindHoldXYZ("down",  0, -FULL);
  bindHoldXYZ("left", -FULL, 0);
  bindHoldXYZ("right", FULL, 0);
</script>
</body>
</html>"""

class H(BaseHTTPRequestHandler):
    def do_GET(self):
        p = urlparse(self.path)
        if p.path == "/":
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(INDEX_HTML.encode("utf-8"))
            return

        if p.path == "/cmd":
            q = parse_qs(p.query)
            x = int(q.get("x", ["0"])[0])
            y = int(q.get("y", ["0"])[0])
            z = int(q.get("z", ["0"])[0])
            last["x"], last["y"], last["z"] = x, y, z
            last["t"] = time.time()
            print(f"CMD: x={x} y={y} z={z}")
            self.send_response(200); self.end_headers()
            self.wfile.write(b"ok")
            return

        if p.path == "/status":
            age = int((time.time() - last["t"]) * 1000)
            body = f'{{"x":{last["x"]},"y":{last["y"]},"z":{last["z"]},"age_ms":{age}}}'
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(body.encode("utf-8"))
            return

        self.send_response(404); self.end_headers()

if __name__ == "__main__":
    print("Fake receiver on http://0.0.0.0:8080")
    HTTPServer(("0.0.0.0", 8080), H).serve_forever()
