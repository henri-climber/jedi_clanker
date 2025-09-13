# /// script
# dependencies = [
#   "bbos",
# ]
# [tool.uv.sources]
# bbos = { path = "/home/bracketbot/BracketBotOS", editable = true }
# ///
from bbos import Writer, Type
import numpy as np
import sys, select, termios, tty

# Configuration
TURN_SPEED = 0.5
SPEED = 0.3
# Global variables
writer = None

def getch_nonblocking():
    """Return a single character if available, else None."""
    dr, _, _ = select.select([sys.stdin], [], [], 0)
    if dr:
        return sys.stdin.read(1)
    return None

def setup_keyboard():
    tty.setcbreak(sys.stdin.fileno())
    return termios.tcgetattr(sys.stdin)

def restore_keyboard(old_settings):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def main():
    """Main control loop"""
    # Initialize the drive control writer

    old = setup_keyboard()
    with Writer("drive.ctrl", Type("drive_ctrl")) as w_drive:
        print("BracketBotOS WASD Robot Control + Camera Capture")
        print("===============================================")
        print("Controls:")
        print("  W - Move Forward")
        print("  S - Move Backward") 
        print("  A - Turn Left")
        print("  D - Turn Right")
        print("  Q - Quit")
        print("Press and hold keys to move, release to stop.")
        print("Ready for input...")

        twist = [0.0, 0.0]
        while True:
            c = getch_nonblocking()
            if c:
                if c.lower() == 'w':
                    twist = [SPEED, 0.0]
                elif c.lower() == 's':
                    twist = [-SPEED, 0.0]
                elif c.lower() == 'a':
                    twist = [0.0, TURN_SPEED]
                elif c.lower() == 'd':
                    twist = [0.0, -TURN_SPEED]
                elif c.lower() == 'q':
                    break
            else:
                twist = [0.0, 0.0]
            w_drive['twist'] = np.array(twist, dtype=np.float32)
    restore_keyboard(old)
if __name__ == "__main__":
    main()
        