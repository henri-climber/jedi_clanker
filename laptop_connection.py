# /// script
# dependencies = ["bbos"]
# [tool.uv.sources]
# bbos = { path = "/home/bracketbot/BracketBotOS", editable = true }
# ///
import json, socket, time, signal, os
from threading import Event
import numpy as np
from bbos import Writer, Config, Type
from jedi_clanker import sfx

CFG = Config("so101")
UDP_PORT = 5005
AUTH_TOKEN = os.environ.get("BBOS_NET_TOKEN", "secret")
IDLE_TIMEOUT = 0.7  # seconds (watchdog -> torque off)

def main():
    stop = Event()
    def _sig(*a): stop.set()
    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", UDP_PORT))
    sock.settimeout(0.05)

    print(f"[bridge] listening udp://0.0.0.0:{UDP_PORT}  dof={CFG.dof}")
    # Play ignition/startup SFX (non-blocking) if available
    sfx.play_startup()

    with Writer("so101.torque", Type("so101_torque")) as w_torque, \
         Writer("so101.ctrl",   Type("so101_ctrl"))   as w_ctrl:
        # start torque off until first valid cmd
        w_torque["enable"] = np.zeros(CFG.dof, dtype=np.bool_)
        last_ok = time.time()
        torque_on = False

        swing = sfx.SwingSounder()

        while not stop.is_set():
            now = time.time()
            try:
                data, addr = sock.recvfrom(65536)
            except socket.timeout:
                # watchdog
                if torque_on and (now - last_ok) > IDLE_TIMEOUT:
                    w_torque["enable"] = np.zeros(CFG.dof, dtype=np.bool_)
                    torque_on = False
                    print("[bridge] idle -> torque OFF")
                continue

            try:
                msg = json.loads(data.decode("utf-8"))
                if msg.get("token") != AUTH_TOKEN:
                    continue  # ignore unauthorized
                mtype = msg.get("type")
                if mtype == "arm_cmd":
                    pos = msg.get("pos", [])
                    if not isinstance(pos, list) or len(pos) != CFG.dof:
                        continue
                    # clamp optional limits here if you like
                    w_ctrl["pos"] = np.array(pos, dtype=np.float32)
                    last_ok = now
                    # Play swing SFX on motion when torque is engaged
                    if torque_on:
                        try:
                            swing.on_motion(pos)
                        except Exception:
                            pass
                elif mtype == "torque":
                    en = bool(msg.get("enable", False))
                    w_torque["enable"] = (np.ones if en else np.zeros)(CFG.dof, dtype=np.bool_)
                    # If torque transitions OFF->ON, play ignition/startup
                    if en and not torque_on:
                        sfx.play_startup()
                    torque_on = en
                    last_ok = now
                elif mtype == "ping":
                    last_ok = now
                # else ignore unknown
            except Exception as e:
                # swallow malformed packet, keep serving
                pass

        # on exit: torque off
        w_torque["enable"] = np.zeros(CFG.dof, dtype=np.bool_)
        print("[bridge] exit, torque OFF")

if __name__ == "__main__":
    main()
