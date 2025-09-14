import os
import time
import random
import glob
import shutil
import subprocess
from typing import Iterable, List, Optional, Sequence


def _default_audio_dirs() -> List[str]:
    here = os.path.dirname(__file__)
    parent = os.path.dirname(here)
    # Support multiple override paths separated by os.pathsep
    env_multi = []
    for key in ("JEDI_CLANKER_AUDIO_DIR", "AUDIO_DIR"):
        val = os.environ.get(key)
        if val:
            env_multi.extend([p for p in val.split(os.pathsep) if p])
    return [
        *env_multi,                                  # explicit override(s)
        os.path.join(here, "audio"),                # package-local audio
        os.path.join(parent, "audio"),              # repo-level audio
    ]

def _existing_audio_dirs() -> List[str]:
    dirs: List[str] = []
    for d in _default_audio_dirs():
        if d and os.path.isdir(d):
            dirs.append(d)
    return dirs or [os.path.join(os.path.dirname(__file__), "audio")]

# Mutable so callers/tests can override at runtime without re-importing
AUDIO_DIRS = _existing_audio_dirs()


def _choose_player() -> Optional[List[str]]:
    """Pick an available CLI player for .wav files.

    Preference order: aplay (ALSA), ffplay (ffmpeg), afplay (macOS).
    Returns the command prefix to execute, or None if none is available.
    """
    candidates = [
        ["aplay", "-q"],
        ["ffplay", "-v", "0", "-nodisp", "-autoexit"],
        ["afplay"],
    ]
    for cmd in candidates:
        if shutil.which(cmd[0]):
            return cmd
    return None


def _resolve_path(path: str) -> Optional[str]:
    """Resolve a path relative to known audio dirs unless already absolute."""
    if os.path.isabs(path):
        return path if os.path.exists(path) else None
    for adir in AUDIO_DIRS:
        p = os.path.join(adir, path)
        if os.path.exists(p):
            return p
    return None


def play_wav(path: str) -> bool:
    """Fire-and-forget playback of a .wav file. Returns True if spawned."""
    if not SFX_ENABLED:
        return False
    apath = _resolve_path(path)
    if not apath:
        return False
    player = _choose_player()
    if not player:
        return False
    try:
        subprocess.Popen(
            [*player, apath],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        return True
    except Exception:
        return False


def _glob_any(patterns: Iterable[str]) -> List[str]:
    files: List[str] = []
    seen = set()
    for adir in AUDIO_DIRS:
        for pat in patterns:
            for f in glob.glob(os.path.join(adir, pat)):
                if f not in seen:
                    files.append(f)
                    seen.add(f)
    return files


def set_audio_dirs(paths: Sequence[str]) -> None:
    """Override the search paths for audio at runtime."""
    global AUDIO_DIRS
    AUDIO_DIRS = [p for p in paths if p and os.path.isdir(p)] or AUDIO_DIRS


def set_enabled(enabled: bool) -> None:
    """Globally enable/disable SFX at runtime (overrides env)."""
    global SFX_ENABLED
    SFX_ENABLED = bool(enabled)


def is_enabled() -> bool:
    """Return current enable state for SFX."""
    return SFX_ENABLED


def is_player_available() -> bool:
    """True if a supported CLI audio player is available on PATH."""
    return _choose_player() is not None


def _pick_from_priority_globs(groups: Sequence[Sequence[str]]) -> Optional[str]:
    """Return a random file from the first non-empty glob group.

    Each group is a list/tuple of glob patterns. The first group with hits
    is used, and a single file is chosen at random.
    """
    for grp in groups:
        matches = _glob_any(grp)
        if matches:
            return random.choice(matches)
    return None


def play_startup() -> bool:
    """Play a lightsaber ignition/startup sound if one exists.

    Looks for common patterns like 'saber_on*.wav', 'lightsaber_on*.wav', '*on*.wav'.
    Returns True if a file was found and playback started.
    """
    # Prefer filenames that actually exist in the repo's audio folder
    # e.g. lightsaber-on.wav, lightsaber-ignition.wav, lightsaber-ignite-*.wav
    choice = _pick_from_priority_globs([
        ("lightsaber-on*.wav",),
        ("lightsaber-ignite*.wav", "lightsaber-ignition*.wav"),
        ("saber_on*.wav", "lightsaber_on*.wav", "power_on*.wav"),
        ("*-on*.wav",),
        ("*on*.wav",),
    ])
    return play_wav(choice) if choice else False


class SwingSounder:
    """Detects arm motion and plays swing SFX with a cooldown.

    - threshold: L2 norm threshold on position delta to trigger.
    - cooldown: minimum seconds between swings.
    - patterns: filename globs to discover swing samples.
    """

    def __init__(
        self,
        threshold: float = 0.05,
        cooldown: float = 0.20,
        patterns: Optional[Iterable[str]] = None,
    ) -> None:
        self.threshold = float(threshold)
        self.cooldown = float(cooldown)
        self.patterns = list(patterns) if patterns else [
            "lightsaber-swing-*.wav",
            "swing*.wav",
            "whoosh*.wav",
            "*swing*.wav",
            "*whoosh*.wav",
        ]
        self._last_pos: Optional[List[float]] = None
        self._last_time: float = 0.0
        self._swing_files: List[str] = self._discover()

    def _discover(self) -> List[str]:
        return _glob_any(self.patterns)

    def _maybe_reload(self) -> None:
        if not self._swing_files:
            self._swing_files = self._discover()

    def on_motion(self, pos: Iterable[float]) -> bool:
        """Update with the latest joint positions and maybe play a swing.

        Returns True if a swing sound was triggered.
        """
        try:
            cur = [float(x) for x in pos]
        except Exception:
            return False

        now = time.time()
        if self._last_pos is None:
def _env_disabled() -> bool:
    """Return True if the env requests SFX to be disabled.

    Recognizes: JEDI_CLANKER_SFX in {"0","false","off","no"} (case-insensitive).
    """
    val = os.environ.get("JEDI_CLANKER_SFX", "").strip().lower()
    return val in {"0", "false", "off", "no"}


# Global toggle (can be changed via set_enabled)
SFX_ENABLED = not _env_disabled()

            self._last_pos = cur
            self._last_time = now
            return False

        # L2 distance between last and current positions
        delta = sum((a - b) ** 2 for a, b in zip(cur, self._last_pos)) ** 0.5
        self._last_pos = cur

        if delta < self.threshold:
            return False
        if (now - self._last_time) < self.cooldown:
            return False

        self._last_time = now
        self._maybe_reload()
        if not self._swing_files:
            return False
        return play_wav(random.choice(self._swing_files))
