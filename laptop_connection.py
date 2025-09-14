# /// script
# dependencies = ["bbos", "soundfile"]
# [tool.uv.sources]
# bbos = { path = "/home/bracketbot/BracketBotOS", editable = true }
# ///
import json, socket, time, signal, os, random
from threading import Event, Thread
import numpy as np
import soundfile as sf
from bbos import Writer, Config, Type

CFG = Config("so101")
SPEAKER_CFG = Config("speakerphone")
UDP_PORT = 5005
AUTH_TOKEN = os.environ.get("BBOS_NET_TOKEN", "secret")
IDLE_TIMEOUT = 0.7  # seconds (watchdog -> torque off)


def play_wav_file(w_speaker, file_path):
    """Play a WAV file through the speaker in a separate thread"""

    def _play():
        try:
            with sf.SoundFile(file_path, mode='r') as sf_reader:
                while True:
                    input_chunk = sf_reader.read(SPEAKER_CFG.speaker_chunk_size, dtype='int16')
                    if len(input_chunk) == 0:
                        break

                    # Reshape and pad/truncate to expected size
                    reshaped = input_chunk.reshape(-1, SPEAKER_CFG.speaker_channels)

                    # Pad or truncate to exact chunk size
                    if len(reshaped) < SPEAKER_CFG.speaker_chunk_size:
                        # Pad with zeros
                        padding = np.zeros(
                            (SPEAKER_CFG.speaker_chunk_size - len(reshaped), SPEAKER_CFG.speaker_channels),
                            dtype=np.int16)
                        reshaped = np.vstack([reshaped, padding])
                    elif len(reshaped) > SPEAKER_CFG.speaker_chunk_size:
                        # Truncate
                        reshaped = reshaped[:SPEAKER_CFG.speaker_chunk_size]

                    with w_speaker.buf() as b:
                        b['audio'] = reshaped
        except Exception as e:
            print(f"[bridge] Error playing {file_path}: {e}")

    thread = Thread(target=_play, daemon=True)
    thread.start()


def main():
    stop = Event()

    def _sig(*a):
        stop.set()

    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", UDP_PORT))
    sock.settimeout(0.05)

    print(f"[bridge] listening udp://0.0.0.0:{UDP_PORT}  dof={CFG.dof}")

    # Lightsaber swing sounds
    swing_sounds = [
        "audio/lightsaber-swing-1.wav",
        "audio/lightsaber-swing-2.wav",
        "audio/lightsaber-swing-3.wav",
        "audio/lightsaber-swing-4.wav",
        "audio/lightsaber-swing-5.wav",
        "audio/lightsaber-swing-6.wav",
        "audio/lightsaber-swing-7.wav"
    ]

    with Writer("so101.torque", Type("so101_torque")) as w_torque, \
            Writer("so101.ctrl", Type("so101_ctrl")) as w_ctrl, \
            Writer("drive.ctrl", Type("drive_ctrl")) as w_drive, \
            Writer("speakerphone.speaker", Type("speakerphone_speaker")) as w_speaker:
        # start torque off until first valid cmd
        w_torque["enable"] = np.zeros(CFG.dof, dtype=np.bool_)
        last_ok = time.time()
        torque_on = False

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

                # 1% chance to play random lightsaber swing sound
                if random.random() < 0.01:
                    sound_file = random.choice(swing_sounds)
                    print(f"[bridge] playing random lightsaber swing: {sound_file}")
                    play_wav_file(w_speaker, sound_file)

                continue

            try:
                msg = json.loads(data.decode("utf-8"))
                if msg.get("token") != AUTH_TOKEN:
                    continue  # ignore unauthorized
                mtype = msg.get("type")
                print(f"[bridge] message type: {mtype}")
                if mtype == "arm_cmd":
                    pos = msg.get("pos", [])
                    if not isinstance(pos, list) or len(pos) != CFG.dof:
                        continue
                    # clamp optional limits here if you like
                    w_ctrl["pos"] = np.array(pos, dtype=np.float32)
                    last_ok = now
                elif mtype == "torque":
                    en = bool(msg.get("enable", False))
                    w_torque["enable"] = (np.ones if en else np.zeros)(CFG.dof, dtype=np.bool_)
                    torque_on = en
                    last_ok = now
                if mtype == "drive":
                    twist = msg.get("twist", [0.0, 0.0])
                    if isinstance(twist, (list, tuple)) and len(twist) == 2:
                        w_drive["twist"] = np.array(twist, dtype=np.float32)
                        last_ok = now
                        print(f"[bridge] drive twist -> {twist}")  # DEBUG
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