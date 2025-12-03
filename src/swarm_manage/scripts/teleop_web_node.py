#!/usr/bin/env python3
"""
teleop_web_node.py
------------------
Mobile-friendly web UI to teleoperate the bot by publishing simple string
commands on a ROS topic (default: /bot/cmd). It also exposes a tiny REST API.

Routes:
- GET /                -> Serves the teleop HTML UI
- GET /api/cmd?c=XXX   -> Publish command (e.g., forward, left, right, stop)
- POST /api/cmd        -> JSON {"cmd":"forward"}
- GET /api/status      -> Returns last JSON seen on /bot/status_raw (if any)

Params:
- ~host (str)          -> HTTP bind host, default: 0.0.0.0
- ~port (int)          -> HTTP bind port, default: 8080
- ~cmd_topic (str)     -> Command topic, default: /bot/cmd
- ~status_topic (str)  -> Status topic to mirror, default: /bot/status_raw

No external web framework required. Uses Python's http.server.
"""
import json
import threading
import urllib.parse
from http.server import BaseHTTPRequestHandler, HTTPServer

import rospy
from std_msgs.msg import String, Int32


PUB = None
PUB_ALARM = None
PUB_TIMER = None
LAST_STATUS = None
LOCK = threading.Lock()


HTML_PAGE = """<!DOCTYPE html>
<html lang=\"en\">
<head>
  <meta charset=\"UTF-8\" />
  <meta name=\"viewport\" content=\"width=device-width, initial-scale=1, user-scalable=no\" />
  <title>Bot Teleop</title>
  <style>
    :root { --bg:#0f172a; --fg:#e2e8f0; --accent:#38bdf8; --btn:#1e293b; }
    body { margin:0; font-family: system-ui, -apple-system, Segoe UI, Roboto, sans-serif; background:var(--bg); color:var(--fg); }
    .wrap { display:flex; flex-direction:column; align-items:center; justify-content:center; min-height:100vh; gap:16px; padding:16px; }
    h1 { font-size:20px; margin:0 0 8px; color:var(--accent); }
    .grid { display:grid; grid-template-columns: 80px 80px 80px; grid-template-rows: 80px 80px 80px; gap:12px; }
    button { background:var(--btn); color:var(--fg); border:none; border-radius:14px; font-size:16px; padding:12px; box-shadow: 0 2px 8px rgba(0,0,0,.25); }
    button:active { transform: scale(0.98); }
    .full { width:100%; max-width:360px; display:flex; gap:12px; }
    .card { background: #0b1220; padding:12px; border-radius:12px; width:100%; max-width:380px; }
    .row { display:flex; gap:8px; align-items:center; justify-content:space-between; }
    .status { font-family: ui-monospace, SFMono-Regular, Menlo, monospace; font-size:12px; white-space:pre-wrap; word-break:break-word; }
  </style>
  <script>
    async function sendCmd(cmd){
      try {
        const r = await fetch(`/api/cmd?c=${encodeURIComponent(cmd)}`, { method:'GET' });
        const j = await r.json();
        console.log('cmd', cmd, j);
      } catch (e) { console.error(e); }
    }
    async function refreshStatus(){
      try {
        const r = await fetch('/api/status');
        const t = await r.text();
        document.getElementById('status').textContent = t || '—';
      } catch (e) { console.error(e); }
    }
    setInterval(refreshStatus, 1200);
    window.addEventListener('load', refreshStatus);

    async function setAlarm(){
      try {
        const v = document.getElementById('alarm').value; // 'HH:MM'
        const r = await fetch('/api/scheduler/alarm', { method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify({ time: v })});
        console.log('alarm set', await r.json());
      } catch(e){ console.error(e); }
    }
    async function disableAlarm(){
      try {
        const r = await fetch('/api/scheduler/alarm', { method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify({ time: 'off' })});
        console.log('alarm off', await r.json());
      } catch(e){ console.error(e); }
    }
    async function setTimer(){
      try {
        const s = parseInt(document.getElementById('timer').value||'0');
        const r = await fetch('/api/scheduler/timer', { method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify({ seconds: s })});
        console.log('timer set', await r.json());
      } catch(e){ console.error(e); }
    }
  </script>
  </head>
  <body>
    <div class=\"wrap\">
      <div class=\"card\">
        <h1>Teleop</h1>
        <div class=\"grid\">
          <div></div>
          <button onclick=\"sendCmd('forward')\">▲</button>
          <div></div>
          <button onclick=\"sendCmd('left')\">◀</button>
          <button onclick=\"sendCmd('stop')\">■</button>
          <button onclick=\"sendCmd('right')\">▶</button>
          <div></div>
          <button onclick=\"sendCmd('backward')\">▼</button>
          <div></div>
        </div>
        <div class=\"full\" style=\"margin-top:12px\">
          <button style=\"flex:1\" onclick=\"sendCmd('status')\">Status</button>
          <button style=\"flex:1\" onclick=\"sendCmd('start')\">Start Clean</button>
          <button style=\"flex:1\" onclick=\"sendCmd('stop_clean')\">Stop Clean</button>
        </div>
      </div>
      <div class=\"card\">
        <h1>Scheduler</h1>
        <div class=\"row\" style=\"margin-bottom:8px\"><strong>Daily alarm</strong> <span style=\"font-size:12px;opacity:.8\">(HH:MM)</span></div>
        <div class=\"full\">
          <input id=\"alarm\" type=\"time\" style=\"flex:1; background:#0b1220; color:#e2e8f0; border:1px solid #1e293b; border-radius:10px; padding:8px\"/>
          <button style=\"flex:1\" onclick=\"setAlarm()\">Save</button>
          <button style=\"flex:1\" onclick=\"disableAlarm()\">Disable</button>
        </div>
        <div class=\"row\" style=\"margin:12px 0 8px\"><strong>Countdown</strong> <span style=\"font-size:12px;opacity:.8\">(seconds)</span></div>
        <div class=\"full\">
          <input id=\"timer\" type=\"number\" min=\"1\" step=\"1\" placeholder=\"2\" style=\"flex:1; background:#0b1220; color:#e2e8f0; border:1px solid #1e293b; border-radius:10px; padding:8px\"/>
          <button style=\"flex:1\" onclick=\"setTimer()\">Start</button>
        </div>
      </div>
      <div class=\"card\">
        <div class=\"row\"><strong>Last status</strong><button onclick=\"refreshStatus()\">Refresh</button></div>
        <div id=\"status\" class=\"status\" style=\"margin-top:8px\">—</div>
      </div>
    </div>
  </body>
</html>
"""


def publish_cmd(text: str):
    if not text:
        return
    # Map UI special buttons to topics/services if needed
    lower = text.strip().lower()
    if lower == 'start':
        try:
            rospy.wait_for_service('/start_cleaning', timeout=0.5)
            from std_srvs.srv import Trigger
            start = rospy.ServiceProxy('/start_cleaning', Trigger)
            start()
            return
        except Exception:
            pass
    if lower in ('stop_clean', 'stopclean', 'stop_cleaning'):
        try:
            rospy.wait_for_service('/stop_cleaning', timeout=0.5)
            from std_srvs.srv import Trigger
            stop = rospy.ServiceProxy('/stop_cleaning', Trigger)
            stop()
            return
        except Exception:
            pass

    if PUB is not None:
        PUB.publish(String(data=lower))


class TeleopHandler(BaseHTTPRequestHandler):
    server_version = "TeleopHTTP/1.0"

    def _set_headers(self, status=200, content_type='text/html; charset=utf-8'):
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Cache-Control', 'no-store')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.end_headers()

    def do_OPTIONS(self):
        self._set_headers(200, 'text/plain')

    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        if parsed.path == '/' or parsed.path == '/index.html':
            self._set_headers(200, 'text/html; charset=utf-8')
            self.wfile.write(HTML_PAGE.encode('utf-8'))
            return
        if parsed.path == '/api/cmd':
            query = urllib.parse.parse_qs(parsed.query)
            cmd = None
            if 'c' in query and len(query['c']):
                cmd = query['c'][0]
            publish_cmd(cmd)
            self._set_headers(200, 'application/json')
            self.wfile.write(json.dumps({'ok': True, 'cmd': cmd}).encode('utf-8'))
            return
        if parsed.path == '/api/status':
            with LOCK:
                status_text = LAST_STATUS if LAST_STATUS is not None else ''
            self._set_headers(200, 'text/plain; charset=utf-8')
            self.wfile.write((status_text or '').encode('utf-8'))
            return
        if parsed.path == '/favicon.ico':
            self._set_headers(204, 'text/plain')
            return
        self._set_headers(404, 'text/plain')
        self.wfile.write(b'Not found')

    def do_POST(self):
        parsed = urllib.parse.urlparse(self.path)
        if parsed.path == '/api/cmd':
            length = int(self.headers.get('Content-Length') or 0)
            body = self.rfile.read(length) if length > 0 else b''
            cmd = None
            try:
                data = json.loads(body.decode('utf-8')) if body else {}
                cmd = data.get('cmd')
            except Exception:
                pass
            publish_cmd(cmd)
            self._set_headers(200, 'application/json')
            self.wfile.write(json.dumps({'ok': True, 'cmd': cmd}).encode('utf-8'))
            return
        if parsed.path == '/api/scheduler/alarm':
            length = int(self.headers.get('Content-Length') or 0)
            body = self.rfile.read(length) if length > 0 else b''
            time_str = ''
            try:
                data = json.loads(body.decode('utf-8')) if body else {}
                time_str = (data.get('time') or '').strip()
            except Exception:
                pass
            if PUB_ALARM is not None and time_str:
                PUB_ALARM.publish(String(data=time_str))
            self._set_headers(200, 'application/json')
            self.wfile.write(json.dumps({'ok': True, 'time': time_str}).encode('utf-8'))
            return
        if parsed.path == '/api/scheduler/timer':
            length = int(self.headers.get('Content-Length') or 0)
            body = self.rfile.read(length) if length > 0 else b''
            seconds = 0
            try:
                data = json.loads(body.decode('utf-8')) if body else {}
                seconds = int(data.get('seconds') or 0)
            except Exception:
                pass
            if PUB_TIMER is not None and seconds > 0:
                PUB_TIMER.publish(Int32(data=seconds))
            self._set_headers(200, 'application/json')
            self.wfile.write(json.dumps({'ok': True, 'seconds': seconds}).encode('utf-8'))
            return
        self._set_headers(404, 'text/plain')
        self.wfile.write(b'Not found')


def status_cb(msg: String):
    text = msg.data if isinstance(msg, String) else str(msg)
    with LOCK:
        global LAST_STATUS
        LAST_STATUS = text


def run_server(server: HTTPServer):
    try:
        server.serve_forever(poll_interval=0.5)
    except KeyboardInterrupt:
        pass


def main():
    rospy.init_node('teleop_web_node')
    host = rospy.get_param('~host', '0.0.0.0')
    port = int(rospy.get_param('~port', 8080))
    cmd_topic = rospy.get_param('~cmd_topic', '/bot/cmd')
    status_topic = rospy.get_param('~status_topic', '/bot/status_raw')

    global PUB, PUB_ALARM, PUB_TIMER
    PUB = rospy.Publisher(cmd_topic, String, queue_size=10)
    rospy.Subscriber(status_topic, String, status_cb)
    PUB_ALARM = rospy.Publisher('/scheduler/set_daily_alarm', String, queue_size=10)
    PUB_TIMER = rospy.Publisher('/scheduler/set_timer', Int32, queue_size=10)

    server = HTTPServer((host, port), TeleopHandler)

    t = threading.Thread(target=run_server, args=(server,), daemon=True)
    t.start()

    rospy.loginfo(f"Teleop web UI available at http://{host}:{port}  (open from your phone on the same network)")
    rospy.loginfo("Use the on-screen arrows to send: forward/backward/left/right/stop. Buttons also call status or cleaning services if available. Scheduler section allows setting daily alarm and countdown timer.")

    def on_shutdown():
        try:
            server.shutdown()
            server.server_close()
        except Exception:
            pass

    rospy.on_shutdown(on_shutdown)
    rospy.spin()


if __name__ == '__main__':
    main()
