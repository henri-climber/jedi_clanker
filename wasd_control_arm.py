# /// script
# dependencies = [
#   "bbos",
#   "numpy",
# ]
# [tool.uv.sources]
# bbos = { path = "/home/bracketbot/BracketBotOS", editable = true }
# ///
from bbos import Writer, Config, Type
import numpy as np
import sys, select, termios, tty, time, os

# ---------------- User settings ----------------
CFG                = Config("so101")         # arm config (provides dof)
CTRL_TOPIC         = "so101.ctrl"
CTRL_TYPE          = "so101_ctrl"
CTRL_POS_KEY       = "pos"                   # change if your schema uses "q_des", etc.
TORQUE_TOPIC       = "so101.torque"
TORQUE_TYPE        = "so101_torque"
TORQUE_KEY         = "enable"

HZ                 = 50                      # control loop rate (Hz)
COARSE_STEP_DEG    = 1.0                     # per-press step
BOOST_STEP_DEG     = 1.0                     # faster step when boosted
MAX_STEP_PER_TICK  = 1.0                     # safety rate limit (deg/tick)

# Key bindings: (negative_key, positive_key) per joint (case-insensitive)
KEY_BINDINGS = [
    ('q','a'),
    ('w','s'),
    ('e','d'),
    ('r','f'),
    ('t','g'),
    ('y','h'),
    ('u','j'),
    ('i','k'),
    ('o','l'),
    # add more pairs if your arm has more DOF
]

# Set realistic limits once you have vendor data (per-joint arrays allowed)
JOINT_MIN = np.full(CFG.dof, -np.pi, dtype=np.float32)
JOINT_MAX = np.full(CFG.dof,  np.pi, dtype=np.float32)
# ------------------------------------------------

def deg(x): return np.deg2rad(x).astype(np.float32)

def getch_nonblocking():
    """Return one character if available, else None (raw, non-blocking)."""
    dr, _, _ = select.select([sys.stdin], [], [], 0)
    if dr:
        return sys.stdin.read(1)
    return None

def setup_keyboard():
    """Put stdin into cbreak/raw-like mode and return old termios settings."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)  # char-at-a-time, no enter
    return old

def restore_keyboard(old_settings):
    try:
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old_settings)
    except Exception:
        pass

def clamp(v, lo, hi): return np.minimum(np.maximum(v, lo), hi)

def main():
    os.environ.setdefault("BBOS_WHOAMI", "wasd_jogger")

    dof = CFG.dof
    if len(KEY_BINDINGS) < dof:
        print(f"Warning: only {len(KEY_BINDINGS)} key pairs for {dof} joints. Extra joints will not move.")

    q_cmd   = np.zeros(dof, dtype=np.float32)  # last sent
    q_tgt   = q_cmd.copy()
    q_home  = q_cmd.copy()

    step_coarse = deg(COARSE_STEP_DEG)
    step_boost  = deg(BOOST_STEP_DEG)
    max_dt      = deg(MAX_STEP_PER_TICK)

    torque_on = False
    boosted = False  # hold 'b' to boost step size

    # Print help
    print("BracketBotOS Arm Jogger (key-per-joint)")
    print("=======================================")
    print("Per-joint keys (neg / pos):")
    for j in range(min(dof, len(KEY_BINDINGS))):
        neg,pos = KEY_BINDINGS[j]
        print(f"  j{j:02d}: {neg} / {pos}")
    print("\nGlobal controls:")
    print("  h = Home pose    t = Torque toggle    b = Boost (hold)    q = Quit")
    print(f"Running at {HZ} Hz. Limits: ±{MAX_STEP_PER_TICK:.1f}°/tick")

    old = setup_keyboard()
    try:
        with Writer(TORQUE_TOPIC, Type(TORQUE_TYPE)) as w_torque, \
             Writer(CTRL_TOPIC,   Type(CTRL_TYPE))   as w_ctrl:

            # Try torque ON initially
            try:
                w_torque[TORQUE_KEY] = np.ones(dof, dtype=np.bool_)
                torque_on = True
                print("Torque: ON")
            except KeyError as e:
                raise RuntimeError(
                    f"Torque message missing field '{TORQUE_KEY}'. "
                    f"Available keys: {list(getattr(w_torque, 'schema', {}).keys())}"
                ) from e

            period = 1.0 / HZ
            last = time.time()

            while True:
                # Keep loop rate
                now = time.time()
                dt = now - last
                if dt < period:
                    time.sleep(period - dt)
                    now = time.time()
                last = now

                # ---- keyboard handling (non-blocking) ----
                # Consume all available chars this cycle (auto-repeat supported)
                while True:
                    c = getch_nonblocking()
                    if not c:
                        break

                    lc = c.lower()

                    # Global controls
                    if lc == '#':
                        raise KeyboardInterrupt
                    elif lc == '0':
                        torque_on = not torque_on
                        w_torque[TORQUE_KEY] = (np.ones if torque_on else np.zeros)(dof, dtype=np.bool_)
                        print("Torque:", "ON" if torque_on else "OFF")
                        continue
                    elif lc == 'ä':
                        q_tgt = q_home.copy()
                        print("Go Home: applied")
                        continue
                    elif lc == 'b':
                        # Boost toggles while held; we can't detect keyup in raw mode,
                        # so treat any 'b' press as a temporary boost for this tick.
                        boosted = True
                        continue

                    # Per-joint jog keys
                    for j in range(min(dof, len(KEY_BINDINGS))):
                        neg, pos = KEY_BINDINGS[j]
                        if lc == neg:
                            q_tgt[j] -= (step_boost if boosted else step_coarse)
                            break
                        elif lc == pos:
                            q_tgt[j] += (step_boost if boosted else step_coarse)
                            break

                boosted = False  # boost only lasts this tick unless 'b' repeats

                # ---- safety ----
                q_tgt = clamp(q_tgt, JOINT_MIN, JOINT_MAX)

                # ---- per-joint rate limit towards target ----
                delta = q_tgt - q_cmd
                step  = np.clip(delta, -max_dt, max_dt)
                q_cmd = (q_cmd + step).astype(np.float32)

                # ---- send command ----
                try:
                    w_ctrl[CTRL_POS_KEY] = q_cmd
                except KeyError as e:
                    raise RuntimeError(
                        f"Control message missing field '{CTRL_POS_KEY}'. "
                        f"Available keys: {list(getattr(w_ctrl, 'schema', {}).keys())}"
                    ) from e

    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        # Disable torque on exit
        try:
            with Writer(TORQUE_TOPIC, Type(TORQUE_TYPE)) as w_torque:
                w_torque[TORQUE_KEY] = np.zeros(CFG.dof, dtype=np.bool_)
        except Exception:
            pass
        restore_keyboard(old)
        print("Done.")

if __name__ == "__main__":
    main()