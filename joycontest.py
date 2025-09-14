from pyjoycon import GyroTrackingJoyCon, get_L_id
import time

joycon_id = get_L_id()
joycon = GyroTrackingJoyCon(joycon_id)
for i in range(20):
    print("joycon pointer:  ", joycon.pointer)
    print("joycon rotation: ", joycon.rotation)
    print("joycon direction:", joycon.direction)
    print()
    time.sleep(0.05)
