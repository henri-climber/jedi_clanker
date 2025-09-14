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
BRIDGE_HOST = os.environ.get("BBOS_BRIDGE_HOST", "10.42.0.1")  # robot Wi-Fi IP
UDP_PORT = int(os.environ.get("BBOS_UDP_PORT", "5005"))
AUTH_TOKEN = os.environ.get("BBOS_NET_TOKEN", "secret")
ROBOT_DOF = int(os.environ.get("BBOS_ROBOT_DOF", "6"))
K_RAD_PER_M = float(os.environ.get("BBOS_GAIN_RAD_PER_M", "1.5"))  # meters -> radians
JOINT_LIMITS = (-math.pi, math.pi)  # clamp joint 1
SEND_HZ = 20  # loop rate
DEADBAND_MPS2 = 0.03  # accel deadband (m/s^2)
GRAVITY = 9.80665
K_RAD_PER_DEG = float(os.environ.get("BBOS_GAIN_RAD_PER_DEG", "0.02"))  # radians per degree
DEADZONE_DEG = float(os.environ.get("BBOS_DEADZONE_DEG", "2.0"))  # ignore tiny tilts
SLEW_RAD_PER_S = float(os.environ.get("BBOS_SLEW_RAD_PER_S", ".15"))  # max change rate

# --- J2 & J3 mapping from a second phone axis (default: gamma/roll) ---
K2_RAD_PER_DEG = float(os.environ.get("BBOS_GAIN2_RAD_PER_DEG", "0.02"))
DEADZONE2_DEG = float(os.environ.get("BBOS_DEADZONE2_DEG", "2.0"))
SLEW2_RAD_PER_S = float(os.environ.get("BBOS_SLEW2_RAD_PER_S", ".15"))
SLEW3_RAD_PER_S = float(os.environ.get("BBOS_SLEW3_RAD_PER_S", ".15"))
MAX2_DEG = float(os.environ.get("BBOS_MAX2_DEG", "60"))
INVERT_SIGN_J23 = os.environ.get("BBOS_INVERT_SIGN_J23", "0") in ("1", "true", "True")

# per-joint centers (auto midpoint if unset)
CENTER2_RAD = float(os.environ.get("BBOS_CENTER2_RAD", "nan"))  # joint 2 (index 1)
CENTER3_RAD = float(os.environ.get("BBOS_CENTER3_RAD", "nan"))  # joint 3 (index 2)

# optional: separate limits for j2/j3; defaults to global JOINT_LIMITS if not set
J2_MIN = float(os.environ.get("BBOS_J2_MIN_RAD", str(JOINT_LIMITS[0])))
J2_MAX = float(os.environ.get("BBOS_J2_MAX_RAD", str(JOINT_LIMITS[1])))
J3_MIN = float(os.environ.get("BBOS_J3_MIN_RAD", str(JOINT_LIMITS[0])))
J3_MAX = float(os.environ.get("BBOS_J3_MAX_RAD", str(JOINT_LIMITS[1])))

# --- Joint 5 from pos[0] (meters) ---
K5_RAD_PER_M = float(os.environ.get("BBOS_GAIN5_RAD_PER_M", "1.5"))  # m -> rad
DEADBAND_M = float(os.environ.get("BBOS_DEADBAND_M", "0.005"))  # ignore tiny drift (m)
SLEW5_RAD_PER_S = float(os.environ.get("BBOS_SLEW5_RAD_PER_S", ".15"))
INVERT_SIGN_J5 = os.environ.get("BBOS_INVERT_SIGN_J5", "0") in ("1", "true", "True")
CENTER5_DEG = float(os.environ.get("BBOS_CENTER5_DEG", "nan"))  # neutral yaw (in degrees)
K5_RAD_PER_DEG = float(os.environ.get("BBOS_GAIN5_RAD_PER_DEG", "0.015"))  # gentler wrist gain
# center & limits for joint 5 (index 4)
CENTER5_RAD = float(os.environ.get("BBOS_CENTER5_RAD", "0"))
J5_MIN = float(os.environ.get("BBOS_J5_MIN_RAD", str(JOINT_LIMITS[0])))
J5_MAX = float(os.environ.get("BBOS_J5_MAX_RAD", str(JOINT_LIMITS[1])))

# --- Drive (mobile base) joystick ---
DRIVE_HZ = int(os.environ.get("BBOS_DRIVE_HZ", "20"))
DRIVE_MAX_V = float(os.environ.get("BBOS_DRIVE_MAX_MPS", "0.15"))  # forward/back (m/s)
DRIVE_MAX_W = float(os.environ.get("BBOS_DRIVE_MAX_RADPS", "0.3"))  # yaw rate (rad/s)
DRIVE_DEADZONE = float(os.environ.get("BBOS_DRIVE_DEADZONE", "0.03"))  # 0..1 (joystick radius)
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
_last_sent_pos = [0.0] * ROBOT_DOF
_alpha_smooth = 0.2  # EMA for joint command


# ---------------- HTML ----------------
@app.get("/", response_class=HTMLResponse)
async def root():
    return HTMLResponse("""<!doctype html><html><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Phone IMU Sensor</title>
<style>
 body{font-family:Arial,sans-serif;text-align:center;margin:20px;background:#f0f0f0}
 .container{max-width:900px;margin:0 auto;background:#fff;padding:20px;border-radius:12px;box-shadow:0 2px 10px rgba(0,0,0,.1)}
 .status{padding:15px;margin:10px 0;border-radius:8px;font-weight:bold}
 .connected{background:#d4edda;color:#155724}.disconnected{background:#f8d7da;color:#721c24}
 .button{background:#007bff;color:#fff;border:none;padding:12px 22px;font-size:16px;border-radius:8px;cursor:pointer;margin:10px}
 .button:hover{background:#0056b3}.button:disabled{background:#6c757d;cursor:not-allowed}
 .data{background:#f8f9fa;padding:15px;margin:10px 0;border-radius:8px;text-align:left;font-family:monospace;font-size:12px}
 .controls{display:flex;flex-wrap:wrap;gap:10px;justify-content:center;margin:12px 0}
 .active{background:#28a745!important}

 /* joystick */
 .joy-wrap{display:flex;gap:24px;flex-wrap:wrap;justify-content:center;align-items:center;margin-top:18px}
 .joy-card{flex:1 1 360px;max-width:520px;background:#fafafa;border:1px solid #eee;border-radius:12px;padding:16px}
 .joy-title{font-weight:600;margin:0 0 8px 0}
 .joy-sub{color:#666;font-size:12px;margin:0 0 14px 0}
 #joypad{
   width:100%;max-width:420px;aspect-ratio:1/1;margin:0 auto;border-radius:50%;
   background:radial-gradient(circle at 50% 50%, #f3f7ff 0%, #e8eefc 60%, #dde7fb 100%);
   border:1px solid #d2dcfb;position:relative;touch-action:none;user-select:none;
 }
 .joy-center{
   position:absolute;left:50%;top:50%;transform:translate(-50%,-50%);
   width:18px;height:18px;border-radius:50%;background:#aab8ff;border:2px solid #6d83ff;
   pointer-events:none;opacity:.9;
 }
 .joy-stick{
   position:absolute;left:50%;top:50%;transform:translate(-50%,-50%);
   width:90px;height:90px;border-radius:50%;
   background:radial-gradient(circle at 35% 35%, #ffffff 0%, #f0f3ff 60%, #e2e9ff 100%);
   border:2px solid #91a6ff;box-shadow:0 8px 18px rgba(109,131,255,.25);
   pointer-events:none;opacity:.95;
 }
 .joy-row{display:flex;gap:12px;justify-content:center;margin-top:10px;flex-wrap:wrap}
 .joy-pill{
   background:#eef3ff;border:1px solid #d2dcfb;border-radius:999px;padding:6px 10px;font-size:12px;color:#4252a4
 }
</style></head><body>
<div class="container">
 <h2>📱 Phone IMU Sensor + 🕹️ Drive Joystick</h2>
 <div id="status" class="status disconnected">Click "Connect" to start</div>

 <div class="controls">
  <button id="connectBtn" class="button">Connect IMU</button>
  <button id="controlBtn" class="button" disabled>Start Control</button>
  <button id="calibrateBtn" class="button" disabled>Calibrate</button>
 </div>

 <div id="data" class="data">Waiting for IMU data...</div>

 <div class="joy-wrap">
   <div class="joy-card">
     <p class="joy-title">Base Drive Joystick</p>
     <p class="joy-sub">Drag the stick: up/down = forward/back, left/right = turn. Release to stop.</p>
     <div id="joypad">
       <div class="joy-center"></div>
       <div class="joy-stick" id="joystick"></div>
     </div>
     <div class="joy-row">
       <span class="joy-pill">Max v set by env: <code>BBOS_DRIVE_MAX_MPS</code></span>
       <span class="joy-pill">Max w set by env: <code>BBOS_DRIVE_MAX_RADPS</code></span>
       <span class="joy-pill">Deadzone: <code>BBOS_DRIVE_DEADZONE</code></span>
     </div>
   </div>
 </div>
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

 // ---------- Joystick logic ----------
 const pad = document.getElementById('joypad');
 const stick = document.getElementById('joystick');
 let dragging=false;

 function sendDrive(x, y){ // x,y in [-1,1], y up is +1
   if(ws && ws.readyState===WebSocket.OPEN){
     ws.send(JSON.stringify({drive:{x, y}}));
   }
 }
 function setStick(px, py){ stick.style.left = px + 'px'; stick.style.top = py + 'px'; }

 function pointerPos(e){
   const rect = pad.getBoundingClientRect();
   const cx = rect.left + rect.width/2;
   const cy = rect.top  + rect.height/2;
   const t = ('touches' in e) ? e.touches[0] : e;
   return { x: t.clientX - cx, y: t.clientY - cy, R: rect };
 }
 function clamp2(x,y,maxr){
   const m = Math.hypot(x,y);
   if(m<=maxr) return {x,y};
   const s = maxr/m; return {x:x*s,y:y*s};
 }
 function toUnit(x,y,maxr){
   const u = (maxr>0)? {x:x/maxr, y:y/maxr}:{x:0,y:0};
   // y: up positive (invert DOM y)
   return {x: u.x, y: -u.y};
 }

 function startDrag(e){ e.preventDefault(); dragging=true; handleMove(e); }
 function endDrag(){ dragging=false;
   // center stick & send zero
   const rect = pad.getBoundingClientRect();
   setStick(rect.width/2, rect.height/2);
   sendDrive(0,0);
 }
 function handleMove(e){
   if(!dragging) return;
   const p = pointerPos(e);
   const radius = Math.min(p.R.width, p.R.height)/2 * 0.9; // margin in pad
   const cl = clamp2(p.x, p.y, radius);
   setStick(p.R.width/2 + cl.x, p.R.height/2 + cl.y);
   const u = toUnit(cl.x, cl.y, radius);
   sendDrive(u.x, u.y);
 }

 pad.addEventListener('mousedown', startDrag);
 window.addEventListener('mousemove', handleMove);
 window.addEventListener('mouseup', endDrag);
 pad.addEventListener('touchstart', startDrag, {passive:false});
 window.addEventListener('touchmove', handleMove, {passive:false});
 window.addEventListener('touchend', endDrag, {passive:false});

 // init stick at center
 (function(){
   const rect = pad.getBoundingClientRect();
   setStick(rect.width/2, rect.height/2);
 })();

 updateData();
</script>
</body></html>""")


# ---------------- Math helpers ----------------
def _deg2rad(d): return (d or 0.0) * math.pi / 180.0


def _matmul(A, B): return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _rot_matrix(alpha, beta, gamma):
    a, b, g = _deg2rad(alpha), _deg2rad(beta), _deg2rad(gamma)
    ca, sa, cb, sb, cg, sg = math.cos(a), math.sin(a), math.cos(b), math.sin(b), math.cos(g), math.sin(g)
    Rz = [[ca, -sa, 0], [sa, ca, 0], [0, 0, 1]]
    Rx = [[1, 0, 0], [0, cb, -sb], [0, sb, cb]]
    Ry = [[cg, 0, sg], [0, 1, 0], [-sg, 0, cg]]
    return _matmul(_matmul(Rz, Rx), Ry)


def _matvec(R, v): return [R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2],
                           R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2],
                           R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2]]


# ---------------- UDP helpers ----------------
def _udp_sock():
    global _sock
    if _sock is None:
        _sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return _sock


def _send_udp(msg: dict):
    msg = dict(msg)  # copy to add seq/token
    msg["token"] = AUTH_TOKEN
    msg["seq"] = int(time.time() * 1000)
    data = json.dumps(msg).encode("utf-8")
    _udp_sock().sendto(data, (BRIDGE_HOST, UDP_PORT))


def _set_torque(enable: bool):
    global _torque_on_remote
    if _torque_on_remote == enable:  # avoid spam
        return
    _send_udp({"type": "torque", "enable": bool(enable)})
    _torque_on_remote = enable
    print(f"[CONTROL] torque {'ON' if enable else 'OFF'}")


def _send_drive(v: float, w: float):
    _send_udp({"type": "drive", "twist": [float(v), float(w)]})


_prev_cmd = 0.0  # remember last command for slew limiting


def _slew_limit(target: float, dt: float, max_rate: float):
    global _prev_cmd
    if dt <= 0:
        return _prev_cmd
    max_step = max_rate * dt
    delta = target - _prev_cmd
    if delta > max_step: delta = max_step
    if delta < -max_step: delta = -max_step
    _prev_cmd += delta
    return _prev_cmd


def _send_arm_pos(joint0_rad: float):
    # Build full pos vector, only joint 0 is commanded; others hold last value
    global _last_sent_pos
    joint0 = float(max(JOINT_LIMITS[0], min(JOINT_LIMITS[1], joint0_rad)))
    # EMA smoothing
    joint0 = _alpha_smooth * joint0 + (1.0 - _alpha_smooth) * _last_sent_pos[0]
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
    if delta > max_step:
        delta = max_step
    elif delta < -max_step:
        delta = -max_step
    _prev_cmd += delta
    return _prev_cmd


def _clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def _norm_deg180(x):
    """map any degrees to [-180, 180)"""
    if x is None: return None
    return ((float(x) + 180.0) % 360.0) - 180.0


# keep separate last-commands for slew per joint
_prev_cmd_by_joint = {0: 0.0, 1: 0.0, 2: 0.0}


def _slew_limit_joint(jidx: int, target: float, dt: float, max_rate: float):
    p = _prev_cmd_by_joint.get(jidx, 0.0)
    if dt <= 0.0:
        return p
    step = max_rate * dt
    delta = target - p
    if delta > step:
        delta = step
    elif delta < -step:
        delta = -step
    p = p + delta
    _prev_cmd_by_joint[jidx] = p
    return p


def _wrap_deg180(x: float) -> float:
    """Map any degrees to [-180, 180)."""
    return ((float(x) + 180.0) % 360.0) - 180.0


def _send_joint_updates(updates: dict):
    """
    updates: {index: angle_rad, ...}
    Only the provided joints are changed; others keep last value.
    """
    global _last_sent_pos
    vec = list(_last_sent_pos)
    for jidx, val in updates.items():
        vec[int(jidx)] = float(val)
    _send_udp({"type": "arm_cmd", "pos": vec})
    _last_sent_pos = vec


async def control_loop():
    """
    J1 (0): beta (deg) around CENTER_RAD
    J2 (1): gamma (deg) around CENTER2_RAD
    J3 (2): 0.5 * (J2 - CENTER2) + CENTER3
    J4 (3): fixed (never written)
    J5 (4): pos[0] (m) with LPF, deadband, slow slew, drift auto-bias
    """
    import math, time

    global control_active, need_calibration, _last_t
    global CENTER_RAD, CENTER2_RAD, CENTER3_RAD, CENTER5_RAD, CENTER5_DEG
    global pos, vel

    period = 1.0 / SEND_HZ
    _last_t = None

    # --- auto centers if unset (NaN) ---
    if math.isnan(CENTER_RAD):
        CENTER_RAD = 0.5 * (JOINT_LIMITS[0] + JOINT_LIMITS[1])
    if math.isnan(CENTER2_RAD):
        CENTER2_RAD = 0.5 * (J2_MIN + J2_MAX)
    if math.isnan(CENTER3_RAD):
        CENTER3_RAD = 0.5 * (J3_MIN + J3_MAX)
    if math.isnan(CENTER5_DEG):
        CENTER5_DEG = 0.0  # placeholder; we’ll set it on first sample or on calibrate

    print("[CONTROL] Loop (j1:beta, j2/j3:gamma, j5:pos[0] slow & filtered).")

    while True:
        tick_start = time.time()

        if need_calibration:
            # Re-center j1; zero integrators for pos[]
            CENTER_RAD = _last_sent_pos[0]
            pos[:] = [0.0, 0.0, 0.0]
            vel[:] = [0.0, 0.0, 0.0]
            j5_x_filt = 0.0
            j5_bias = 0.0
            _last_t = None
            need_calibration = False
            # capture current yaw as neutral for wrist (degrees source)
            theta_deg_now = latest_imu_data.get("alpha")  # or whatever provides 0..360 for J5
            if theta_deg_now is not None:
                CENTER5_DEG = float(theta_deg_now)
            print(f"[CONTROL] J5 yaw center set to {CENTER5_DEG:.1f}°")
            print(f"[CONTROL] Calibrated → CENTER_RAD={CENTER_RAD:.3f} rad; pos/vel reset; J5 filters reset")

        if not control_active:
            _last_t = None
            _set_torque(False)
        else:
            _set_torque(True)

            imu = latest_imu_data or {}
            ax, ay, az = imu.get("ax"), imu.get("ay"), imu.get("az")
            alpha, beta_raw, gamma_raw = imu.get("alpha"), imu.get("beta"), imu.get("gamma")

            now = time.time()
            if _last_t is None:
                _last_t = now
            dt = now - _last_t
            _last_t = now

            updates = {}

            # --- integrate acceleration -> pos[] (requires full sensor set) ---
            if None not in (ax, ay, az, alpha, beta_raw, gamma_raw) and 0.0 < dt <= 0.25:
                R = _rot_matrix(alpha, beta_raw, gamma_raw)  # phone→world
                a_world = _matvec(R, [ax, ay, az])
                a_world[2] -= GRAVITY  # remove gravity on world-Z

                # accel deadband to limit drift
                for i in range(3):
                    if abs(a_world[i]) < DEADBAND_MPS2:
                        a_world[i] = 0.0

                # integrate to velocity and position
                for i in range(3):
                    vel[i] += a_world[i] * dt
                    pos[i] += vel[i] * dt

                a_world_last = a_world

            # =========================
            # J1 from beta (degrees)
            # =========================
            if beta_raw is not None and 0.0 < dt <= 0.25:
                beta = _norm_deg180(beta_raw)
                if os.environ.get("BBOS_INVERT_SIGN", "0") in ("1", "true", "True"):
                    beta = -beta
                beta = max(-MAX2_DEG, min(MAX2_DEG, beta))

                if abs(beta) < DEADZONE_DEG:
                    beta_used = 0.0
                else:
                    beta_used = math.copysign(abs(beta) - DEADZONE_DEG, beta)

                target0 = CENTER_RAD + K_RAD_PER_DEG * beta_used
                cmd0 = _slew_limit_joint(0, target0, dt, SLEW_RAD_PER_S)
                cmd0 = max(JOINT_LIMITS[0], min(JOINT_LIMITS[1], cmd0))
                updates[0] = cmd0

            # =========================
            # J2 & J3 from gamma (degrees), J3 = 0.5*(J2-CENTER2)+CENTER3
            # =========================
            if gamma_raw is not None and 0.0 < dt <= 0.25:
                g = _norm_deg180(gamma_raw)
                if INVERT_SIGN_J23:
                    g = -g
                g = max(-MAX2_DEG, min(MAX2_DEG, g))

                if abs(g) < DEADZONE2_DEG:
                    g_used = 0.0
                else:
                    g_used = math.copysign(abs(g) - DEADZONE2_DEG, g)

                # J2
                target2 = CENTER2_RAD + K2_RAD_PER_DEG * g_used
                cmd2 = _slew_limit_joint(1, target2, dt, SLEW2_RAD_PER_S)
                cmd2 = max(J2_MIN, min(J2_MAX, cmd2))

                # J3 follows at half angle about its own center
                target3 = CENTER3_RAD + 0.5 * (cmd2 - CENTER2_RAD)
                cmd3 = _slew_limit_joint(2, target3, dt, SLEW3_RAD_PER_S)
                cmd3 = max(J3_MIN, min(J3_MAX, cmd3))

                updates[1] = cmd2
                updates[2] = cmd3
                # index 3 (Joint 4) intentionally untouched

            # =========================
            # J5 (index 4) from a 0..360° angle (wrap-safe), e.g. alpha
            # =========================
            if 0.0 < dt <= 0.25:
                theta_deg_raw = imu.get("alpha")  # <-- the source that wraps at 360→0
                if theta_deg_raw is not None:
                    theta_deg = float(theta_deg_raw)

                    # If center not yet set (e.g., first sample), set it now
                    if math.isnan(CENTER5_DEG):
                        CENTER5_DEG = theta_deg
                        # fall through to compute error this tick

                    # signed error around neutral, in [-180, 180)
                    err_deg = _wrap_deg180(theta_deg - CENTER5_DEG)

                    # optional invert
                    if INVERT_SIGN_J5:
                        err_deg = -err_deg

                    # deadzone (deg)
                    if abs(err_deg) < float(os.environ.get("BBOS_DEADZONE5_DEG", "2.0")):
                        err_used_deg = 0.0
                    else:
                        err_used_deg = math.copysign(abs(err_deg) - float(os.environ.get("BBOS_DEADZONE5_DEG", "2.0")),
                                                     err_deg)

                    # map to radians about CENTER5_RAD (rad), with slow slew & clamp
                    target5 = CENTER5_RAD + K5_RAD_PER_DEG * err_used_deg
                    cmd5 = _slew_limit_joint(4, target5, dt, SLEW5_RAD_PER_S)  # keep SLEW5 low (e.g., 0.4–0.8)
                    cmd5 = max(J5_MIN, min(J5_MAX, cmd5))
                    updates[4] = cmd5

                    # debug:
                    # print(f"[J5] raw={theta_deg:6.1f}°, center={CENTER5_DEG:6.1f}°, err={err_deg:+6.1f}° used={err_used_deg:+6.1f}° -> cmd={cmd5:+.3f} rad")
            # --- send any joint changes (0/1/2/4) in one packet ---
            if updates:
                _send_joint_updates(updates)

        # maintain loop rate
        elapsed = time.time() - tick_start
        await asyncio.sleep(max(0.0, period - elapsed))


_last_twist = [0.0, 0.0]  # [v, w]


async def drive_loop():
    """Sends the latest twist at DRIVE_HZ."""
    period = 1.0 / DRIVE_HZ
    while True:
        v, w = _last_twist
        # Apply small deadzone to avoid micro-jitter from the UI
        mag = (v * v + w * w) ** 0.5
        if mag < 1e-6:
            # still send zeros to guarantee stop when user releases
            _send_drive(0.0, 0.0)
        else:
            _send_drive(v, w)
        await asyncio.sleep(period)


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

                # Joystick message from browser:
                # { drive: { x: -1..1 (left/right), y: -1..1 (up/down) } }
                drv = imu_data.get("drive")
                if isinstance(drv, dict):
                    # UI convention: y=+1 is UP (forward). Map to v,w with scaling.
                    x = float(drv.get("x", 0.0))  # left/right -> turn (w)
                    y = float(drv.get("y", 0.0))  # up/down    -> forward (v)
                    # radial deadzone
                    r = (x * x + y * y) ** 0.5
                    if r < DRIVE_DEADZONE:
                        x = 0.0;
                        y = 0.0
                    # scale to real units
                    v = y * DRIVE_MAX_V
                    w = -x * DRIVE_MAX_W  # left swipe (x<0) => positive yaw (CCW)
                    global _last_twist
                    _last_twist = [v, w]

            except Exception:
                pass
    except WebSocketDisconnect:
        pass
    finally:
        _set_torque(False)


@app.on_event("startup")
async def _startup():
    asyncio.create_task(control_loop())
    asyncio.create_task(drive_loop())


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
    uvicorn.run("new_phone_to_win:app", **uv_kwargs)
