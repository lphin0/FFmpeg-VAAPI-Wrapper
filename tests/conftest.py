import importlib.util
import os
import shutil
import subprocess
import sys

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

_FFMPEG_WRAP_PATH = os.path.join(os.path.dirname(__file__), "..", "ffmpeg-wrap.py")

HW_DEVICE = "/dev/dri/renderD128"
needs_hw = pytest.mark.skipif(
    not os.path.exists(HW_DEVICE),
    reason="no VAAPI render device available",
)
needs_ffmpeg = pytest.mark.skipif(
    shutil.which("ffmpeg") is None,
    reason="ffmpeg not installed",
)


def _encoder_available(name: str) -> bool:
    if shutil.which("ffmpeg") is None:
        return False
    try:
        result = subprocess.run(
            ["ffmpeg", "-hide_banner", "-encoders"],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=10)
        return name in result.stdout.decode(errors="replace")
    except Exception:
        return False


@pytest.fixture(scope="session")
def m():
    """Load ffmpeg-wrap.py as an importable module (filename has a dash)."""
    spec = importlib.util.spec_from_file_location("ffmpeg_wrap", _FFMPEG_WRAP_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules["ffmpeg_wrap"] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def video_info_factory(m):
    def make(**kwargs):
        defaults = dict(
            duration=120.0,
            width=1920,
            height=1080,
            codec="h264",
            audio_count=1,
            audio_codec="aac",
            audio_codecs=["aac"],
            decodable_audio_indices=[0],
            is_prores=False,
            has_alpha=False,
            fps=60.0,
        )
        defaults.update(kwargs)
        return m.VideoInfo(**defaults)
    return make


@pytest.fixture
def params():
    """A representative EncoderWorker param dict (unit tests only; no real I/O)."""
    return {
        "mode": "quality",
        "size": "10",
        "crf": 24,
        "v_codec": "H.264",
        "use_hw": True,
        "use_hw_decode": True,
        "vulkan_available": False,
        "quality_preset": 2,
        "two_pass": False,
        "hw_decode_2pass": False,
        "res_choice": "Original",
        "ar_choice": "Original",
        "fps_choice": "Original",
        "algo": "VAAPI (HW)",
        "a_codec": "AAC",
        "a_bitrate": "128",
        "output_folder": None,
        "device": HW_DEVICE,
        "copy_data": True,
        "container": "MP4",
        "auto_scale": False,
        "hw_decoder_caps": {},
        "hw_encoder_opts": {},
        "audio_encoders": {"opus": False, "aac": False},
        "ffmpeg_path": "ffmpeg",
        "ffprobe_path": "ffprobe",
        "input": "/tmp/unit-test-non-existent.mp4",
    }


@pytest.fixture(scope="session")
def qapp():
    """Session-wide offscreen QApplication for GUI tests."""
    from PySide6.QtWidgets import QApplication
    app = QApplication.instance() or QApplication([])
    return app


@pytest.fixture
def window(qapp, m, monkeypatch):
    """MainWindow with probing/sleep side effects disabled and UI enabled."""
    monkeypatch.setattr(m.MainWindow, "check_sw_encoders", lambda self: None)
    monkeypatch.setattr(m.MainWindow, "check_audio_encoders", lambda self: None)
    monkeypatch.setattr(m.MainWindow, "initial_probe", lambda self: None)
    monkeypatch.setattr(m.MainWindow, "inhibit_sleep", lambda self: None)
    monkeypatch.setattr(m.MainWindow, "release_sleep", lambda self: None)

    w = m.MainWindow()
    w.available_sw_codecs = {"h264": True, "hevc": True}
    w._set_ui_enabled(True)
    yield w
    w.close()
    w.deleteLater()
    qapp.processEvents()
