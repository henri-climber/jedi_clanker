# backwards_first_seeded.py
import json, socket, time, numpy as np

ROBOT_IP, ROBOT_PORT, TOKEN = "10.42.0.1", 5005, "secret"
HZ   = 100
CENTER = 1.2        # pick a safe neutral INSIDE your real joint range
A_INIT = 0.15       # initial backwards excursion from CENTER
HOLD_PRE_MS = 300   # how long to pre-seed before torque ON
HOLD_POST_MS = 300  # hold after torque ON to “win” any filter

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
addr = (ROBOT_IP, ROBOT_PORT)

def send(m): m["token"]=TOKEN; m["seq"]=int(time.time()*1000); sock.sendto(json.dumps(m).encode(), addr)
def torque(en): send({"type":"torque","enable":bool(en)})
def arm_cmd(q): send({"type":"arm_cmd","pos":list(map(float,q))})

def main():
    dof = 6
    q = np.zeros(dof, np.float32)

    # 1) Pre-seed a target LOWER than center (so first motion is "backwards")
    target0 = CENTER - A_INIT
    q[:] = 0.0
    q[0] = target0
    t_end = time.time() + HOLD_PRE_MS/1000.0
    while time.time() < t_end:
        arm_cmd(q)
        time.sleep(1.0/HZ)

    # 2) Now enable torque and KEEP sending the same target briefly
    torque(True)
    t_end = time.time() + HOLD_POST_MS/1000.0
    while time.time() < t_end:
        arm_cmd(q)
        time.sleep(1.0/HZ)

    # 3) Return smoothly to CENTER
    steps = int(0.8 * HZ)
    for i in range(steps):
        w = (i+1)/steps
        q[0] = (1-w)*target0 + w*CENTER
        arm_cmd(q)
        time.sleep(1.0/HZ)

    # 4) Done
    q[0] = CENTER
    arm_cmd(q)
    time.sleep(0.1)
    torque(False)

if __name__ == "__main__":
    main()