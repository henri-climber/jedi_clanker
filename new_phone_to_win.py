#!/usr/bin/env python3
"""
Phone IMU Sensor Web Server -> UDP bridge
- Double-integrates phone acceleration (rough) to get position
- Uses second position value pos[1] to control robot joint 1 (index 0)
- Sends UDP JSON packets your BBOS bridge expects

Run:
  export BBOS_NET_TOKEN='secret'     # must match bridge
  python3 phone_to_win.py --host 0.0.0.0 --port 8000
"""

import argparse, asyncio, json, math, os, socket, time
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
import uvicorn

# ---------------- Config ----------------
BRIDGE_HOST   = os.environ.get("BBOS_BRIDGE_HOST", "10.42.0.1")  # robot Wi-Fi IP
UDP_PORT      = int(os.environ.get("BBOS_UDP_PORT", "5005"))
AUTH_TOKEN    = os.environ.get("BBOS_NET_TOKEN", "secret")
ROBOT_DOF     = int(os.environ.get("BBOS_ROBOT_DOF", "6"))
K_RAD_PER_M   = float(os.environ.get("BBOS_GAIN_RAD_PER_M", "1.5"))  # meters -> radians
JOINT_LIMITS  = (-math.pi, math.pi)  # clamp joint 1
SEND_HZ       = 20                       # loop rate
DEADBAND_MPS2 = 0.03                     # accel deadband (m/s^2)
GRAVITY       = 9.80665
K_RAD_PER_DEG   = float(os.environ.get("BBOS_GAIN_RAD_PER_DEG", "0.02"))  # radians per degree
DEADZONE_DEG    = float(os.environ.get("BBOS_DEADZONE_DEG", "2.0"))       # ignore tiny tilts
SLEW_RAD_PER_S  = float(os.environ.get("BBOS_SLEW_RAD_PER_S", "1.5"))     # max change rate

# ---------------- Globals ----------------
app = FastAPI()
latest_imu_data = {}
control_active = False
need_calibration = False
pos = [0.0, 0.0, 0.0]
vel = [0.0, 0.0, 0.0]
_last_t = None
_sock = None
_torque_on_remote = False
_last_sent_pos = [0.0]*ROBOT_DOF
_alpha_smooth = 0.2  # EMA for joint command

# ---------------- HTML ----------------
@app.get("/", response_class=HTMLResponse)
async def root():
    return HTMLResponse("""<!doctype html><html><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Phone IMU Sensor</title>
<style>
 body{font-family:Arial,sans-serif;text-align:center;margin:20px;background:#f0f0f0}
 .container{max-width:600px;margin:0 auto;background:#fff;padding:20px;border-radius:10px;box-shadow:0 2px 10px rgba(0,0,0,.1)}
 .status{padding:15px;margin:10px 0;border-radius:5px;font-weight:bold}
 .connected{background:#d4edda;color:#155724}.disconnected{background:#f8d7da;color:#721c24}
 .button{background:#007bff;color:#fff;border:none;padding:15px 30px;font-size:18px;border-radius:5px;cursor:pointer;margin:10px}
 .button:hover{background:#0056b3}.button:disabled{background:#6c757d;cursor:not-allowed}
 .data{background:#f8f9fa;padding:15px;margin:10px 0;border-radius:5px;text-align:left;font-family:monospace;font-size:12px}
 .controls{display:flex;justify-content:space-around;margin:20px 0}.active{background:#28a745!important}
</style></head><body>
<div class="container">
 <h2>📱 Phone IMU Sensor</h2>
 <div id="status" class="status disconnected">Click "Connect" to start</div>
 <div class="controls">
  <button id="connectBtn" class="button">Connect IMU</button>
  <button id="controlBtn" class="button" disabled>Start Control</button>
  <button id="calibrateBtn" class="button" disabled>Calibrate</button>
 </div>
 <div id="data" class="data">Waiting for IMU data...</div>
</div>
<script>
 let ws=null, imu={}, isControlling=false;
 const statusDiv=document.getElementById('status'), dataDiv=document.getElementById('data');
 const connectBtn=document.getElementById('connectBtn'), controlBtn=document.getElementById('controlBtn'), calibrateBtn=document.getElementById('calibrateBtn');
 const fmt=n=>((n??0).toFixed(3));
 function updateStatus(msg,ok){statusDiv.textContent=msg;statusDiv.className=`status ${ok?'connected':'disconnected'}`;}
 function updateData(){dataDiv.innerHTML=`Orientation: [${fmt(imu.alpha)}, ${fmt(imu.beta)}, ${fmt(imu.gamma)}]°<br>
Motion: ax=${fmt(imu.ax)}, ay=${fmt(imu.ay)}, az=${fmt(imu.az)}<br>
Rotation: rx=${fmt(imu.rx)}, ry=${fmt(imu.ry)}, rz=${fmt(imu.rz)}<br>
Control: ${isControlling?'ACTIVE':'inactive'}`.trim();}
 async function requestPermissions(){
   if(typeof DeviceMotionEvent?.requestPermission==='function'){
     const m=await DeviceMotionEvent.requestPermission(), o=await DeviceOrientationEvent.requestPermission();
     if(m!=='granted'||o!=='granted') throw new Error('Motion permissions denied');
   }
 }
 function setupIMU(){
   window.addEventListener('deviceorientation',e=>Object.assign(imu,{alpha:e.alpha,beta:e.beta,gamma:e.gamma}));
   window.addEventListener('devicemotion',e=>Object.assign(imu,{
     ax:e.acceleration?.x, ay:e.acceleration?.y, az:e.acceleration?.z,
     rx:e.rotationRate?.alpha, ry:e.rotationRate?.beta, rz:e.rotationRate?.gamma
   }));
 }
 function startWebSocket(){
   const proto=location.protocol==='https:'?'wss:':'ws:';
   ws=new WebSocket(`${proto}//${location.host}/ws`);
   ws.onopen=()=>{updateStatus('✅ WebSocket connected',true);controlBtn.disabled=false;calibrateBtn.disabled=false;};
   ws.onclose=()=>{updateStatus('❌ WebSocket disconnected',false);controlBtn.disabled=true;calibrateBtn.disabled=true;connectBtn.disabled=false;isControlling=false;controlBtn.textContent='Start Control';controlBtn.classList.remove('active');};
   ws.onerror=()=>updateStatus('❌ Connection error',false);
 }
 function sendData(){ if(ws&&ws.readyState===WebSocket.OPEN){ ws.send(JSON.stringify(imu)); } updateData(); }
 connectBtn.addEventListener('click',async()=>{
   try{ connectBtn.disabled=true; updateStatus('Requesting permissions...',false);
        await requestPermissions(); updateStatus('Setting up IMU...',false);
        setupIMU(); startWebSocket(); setInterval(sendData, 50);
   }catch(e){ console.error(e); updateStatus(`❌ Setup failed: ${e.message}`,false); connectBtn.disabled=false; }
 });
 controlBtn.addEventListener('click',()=>{
   isControlling=!isControlling;
   controlBtn.textContent=isControlling?'Stop Control':'Start Control';
   controlBtn.classList.toggle('active', isControlling);
   updateStatus(isControlling?'🤖 Robot control ACTIVE':'✅ Connected (control paused)', true);
   if(ws&&ws.readyState===WebSocket.OPEN){ ws.send(JSON.stringify({...imu, control_active:isControlling})); }
 });
 calibrateBtn.addEventListener('click',()=>{
   if(ws&&ws.readyState===WebSocket.OPEN){
     ws.send(JSON.stringify({...imu, calibrate:true}));
     updateStatus('🔧 Calibrating...',true); setTimeout(()=>updateStatus('✅ Calibration complete',true),2000);
   }
 });
 updateData();
</script></body></html>""")

# ---------------- Math helpers ----------------
def _deg2rad(d): return (d or 0.0) * math.pi / 180.0
def _matmul(A,B): return [[sum(A[i][k]*B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
def _rot_matrix(alpha,beta,gamma):
    a,b,g = _deg2rad(alpha), _deg2rad(beta), _deg2rad(gamma)
    ca,sa,cb,sb,cg,sg = math.cos(a),math.sin(a),math.cos(b),math.sin(b),math.cos(g),math.sin(g)
    Rz=[[ca,-sa,0],[sa,ca,0],[0,0,1]]
    Rx=[[1,0,0],[0,cb,-sb],[0,sb,cb]]
    Ry=[[cg,0,sg],[0,1,0],[-sg,0,cg]]
    return _matmul(_matmul(Rz,Rx),Ry)
def _matvec(R,v): return [R[0][0]*v[0]+R[0][1]*v[1]+R[0][2]*v[2],
                           R[1][0]*v[0]+R[1][1]*v[1]+R[1][2]*v[2],
                           R[2][0]*v[0]+R[2][1]*v[1]+R[2][2]*v[2]]

# ---------------- UDP helpers ----------------
def _udp_sock():
    global _sock
    if _sock is None:
        _sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return _sock

def _send_udp(msg: dict):
    msg = dict(msg)  # copy to add seq/token
    msg["token"] = AUTH_TOKEN
    msg["seq"] = int(time.time()*1000)
    data = json.dumps(msg).encode("utf-8")
    _udp_sock().sendto(data, (BRIDGE_HOST, UDP_PORT))

def _set_torque(enable: bool):
    global _torque_on_remote
    if _torque_on_remote == enable:  # avoid spam
        return
    _send_udp({"type": "torque", "enable": bool(enable)})
    _torque_on_remote = enable
    print(f"[CONTROL] torque {'ON' if enable else 'OFF'}")

_prev_cmd = 0.0  # remember last command for slew limiting

def _slew_limit(target: float, dt: float, max_rate: float):
    global _prev_cmd
    if dt <= 0:
        return _prev_cmd
    max_step = max_rate * dt
    delta = target - _prev_cmd
    if delta >  max_step: delta =  max_step
    if delta < -max_step: delta = -max_step
    _prev_cmd += delta
    return _prev_cmd

def _send_arm_pos(joint0_rad: float):
    # Build full pos vector, only joint 0 is commanded; others hold last value
    global _last_sent_pos
    joint0 = float(max(JOINT_LIMITS[0], min(JOINT_LIMITS[1], joint0_rad)))
    # EMA smoothing
    joint0 = _alpha_smooth*joint0 + (1.0-_alpha_smooth)*_last_sent_pos[0]
    vec = list(_last_sent_pos)
    vec[0] = joint0
    _send_udp({"type": "arm_cmd", "pos": vec})
    _last_sent_pos = vec
    # helper: simple slew-rate limiter for smooth/safe motion

# === new config/vars up top (near your other knobs) ===
CENTER_RAD = float(os.environ.get("BBOS_CENTER_RAD", "nan"))  # if NaN -> will auto-set to midpoint
_prev_cmd = 0.0

def _slew_limit(target: float, dt: float, max_rate: float):
    global _prev_cmd
    if dt <= 0:
        return _prev_cmd
    max_step = max_rate * dt
    delta = target - _prev_cmd
    if   delta >  max_step: delta =  max_step
    elif delta < -max_step: delta = -max_step
    _prev_cmd += delta
    return _prev_cmd

def _clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v

# === updated control loop ===
async def control_loop():
    global control_active, need_calibration, _last_t, _prev_cmd, CENTER_RAD, _last_sent_pos
    print("[CONTROL] Loop started (beta° → joint 1 around CENTER_RAD).")
    period = 1.0 / SEND_HZ
    _prev_cmd = 0.0
    _last_t = None

    # limits & options
    MAX_DEG = float(os.environ.get("BBOS_MAX_DEG", "60"))
    INVERT_SIGN = os.environ.get("BBOS_INVERT_SIGN", "0") in ("1", "true", "True")

    # pick a center if not provided: midpoint of JOINT_LIMITS
    if math.isnan(CENTER_RAD):
        CENTER_RAD = 0.5 * (JOINT_LIMITS[0] + JOINT_LIMITS[1])
        print(f"[CONTROL] CENTER_RAD auto-set to midpoint: {CENTER_RAD:.3f} rad")

    while True:
        t0 = time.time()

        if need_calibration:
            # set the current command as the new center (so “neutral phone” -> neutral joint)
            CENTER_RAD = _last_sent_pos[0]
            _prev_cmd = CENTER_RAD
            _last_t = None
            need_calibration = False
            print(f"[CONTROL] Calibrated: CENTER_RAD := {CENTER_RAD:.3f} rad")

        if not control_active:
            _last_t = None
            _set_torque(False)
        else:
            _set_torque(True)

            imu = latest_imu_data or {}
            beta_deg_raw = imu.get("beta")  # phone tilt (degrees)

            now = time.time()
            if _last_t is None:
                _last_t = now
            dt = now - _last_t
            _last_t = now

            if beta_deg_raw is not None and 0.0 < dt <= 0.25:
                # normalize to [-180, 180)
                beta_deg = ((float(beta_deg_raw) + 180.0) % 360.0) - 180.0
                if INVERT_SIGN:
                    beta_deg = -beta_deg

                # clamp usable tilt window
                beta_deg = _clamp(beta_deg, -MAX_DEG, MAX_DEG)

                # deadzone with sign
                if abs(beta_deg) < DEADZONE_DEG:
                    beta_used = 0.0
                else:
                    beta_used = math.copysign(abs(beta_deg) - DEADZONE_DEG, beta_deg)

                # map degrees → radians offset, then add CENTER
                target_rad = CENTER_RAD + (K_RAD_PER_DEG * beta_used)

                # slew limit, clamp to joint limits, send
                cmd_rad = _slew_limit(target_rad, dt, SLEW_RAD_PER_S)
                cmd_rad = _clamp(cmd_rad, JOINT_LIMITS[0], JOINT_LIMITS[1])
                _send_arm_pos(cmd_rad)

                print(f"[CONTROL] beta_raw={beta_deg_raw:+6.2f}°  beta={beta_deg:+6.1f}°  used={beta_used:+6.1f}°"
                      f" -> target={target_rad:+.3f}  cmd={cmd_rad:+.3f} rad"
                      f" | CENTER={CENTER_RAD:.3f}  lim[{JOINT_LIMITS[0]:.2f},{JOINT_LIMITS[1]:.2f}]")

        # loop rate
        elapsed = time.time() - t0
        await asyncio.sleep(max(0.0, period - elapsed))

# ---------------- WebSocket ----------------
@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    global latest_imu_data, control_active, need_calibration
    try:
        while True:
            data = await websocket.receive_text()
            try:
                imu_data = json.loads(data)
                latest_imu_data = imu_data
                if "control_active" in imu_data:
                    control_active = bool(imu_data.get("control_active"))
                if imu_data.get("calibrate"):
                    need_calibration = True
            except Exception:
                pass
    except WebSocketDisconnect:
        pass
    finally:
        _set_torque(False)

@app.on_event("startup")
async def _startup():
    asyncio.create_task(control_loop())

# ---------------- Main ----------------
if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", type=int, default=8000)
    p.add_argument("--ssl-key", dest="ssl_key", default=None)
    p.add_argument("--ssl-cert", dest="ssl_cert", default=None)
    args = p.parse_args()
    uv_kwargs = dict(host=args.host, port=args.port, reload=False)
    if args.ssl_key and args.ssl_cert:
        uv_kwargs["ssl_keyfile"] = args.ssl_key
        uv_kwargs["ssl_certfile"] = args.ssl_cert
    uvicorn.run("phone_to_win:app", **uv_kwargs)