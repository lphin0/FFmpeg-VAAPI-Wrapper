"""End-to-end tests that run real ffmpeg encodes via EncoderWorker.

These verify the whole pipeline the app builds actually works on the host.
All are marked `ffmpeg` and are skipped automatically when ffmpeg is missing.
"""
import glob
import subprocess

import pytest

from conftest import _encoder_available, needs_ffmpeg, needs_hw


class CaptureWorker:
    """Runs EncoderWorker synchronously and captures results/logs."""

    def __init__(self, m, params):
        self._m = m
        self.worker = m.EncoderWorker(params)
        self.result = None
        self.logs = []
        self.warnings = []
        self.worker.log_signal.connect(self.logs.append)
        self.worker.finished_signal.connect(lambda ok: setattr(self, "result", ok))
        self.worker.compatibility_warning_signal.connect(self.warnings.append)

    def run(self):
        self.worker._run_unsafe()
        return self.result

    @property
    def log_text(self):
        return "\n".join(self.logs)


@pytest.fixture(scope="session")
def sample_video(m, tmp_path_factory):
    """Generate a tiny h264+aac input once per session."""
    if not _encoder_available("libx264"):
        pytest.skip("libx264 not available")
    path = tmp_path_factory.mktemp("media") / "sample.mp4"
    cmd = [
        "ffmpeg", "-hide_banner", "-y",
        "-f", "lavfi", "-i", "testsrc=duration=2:size=640x480:rate=30",
        "-f", "lavfi", "-i", "sine=frequency=440:duration=2",
        "-c:v", "libx264", "-c:a", "aac", "-shortest", str(path),
    ]
    result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=60)
    if result.returncode != 0:
        pytest.skip("could not generate sample video")
    return path


def job_params(m, input_path, output_folder, **overrides):
    p = {
        "mode": "quality",
        "size": "10",
        "crf": 24,
        "v_codec": "H.264",
        "use_hw": False,
        "use_hw_decode": True,
        "vulkan_available": False,
        "quality_preset": 4,
        "two_pass": False,
        "res_choice": "Original",
        "ar_choice": "Original",
        "fps_choice": "Original",
        "algo": "Bicubic",
        "a_codec": "AAC",
        "a_bitrate": "128",
        "output_folder": str(output_folder),
        "device": "/dev/dri/renderD128",
        "copy_data": True,
        "container": "MP4",
        "auto_scale": False,
        "hw_decoder_caps": {},
        "hw_encoder_opts": {},
        "audio_encoders": {"opus": False, "aac": False},
        "ffmpeg_path": "ffmpeg",
        "ffprobe_path": "ffprobe",
        "input": str(input_path),
    }
    p.update(overrides)
    return p


def output_file(output_folder, prefix, v_tag, ext):
    matches = glob.glob(str(output_folder / f"{prefix}*{v_tag}{ext}"))
    return matches[0] if matches else None


@needs_ffmpeg
@pytest.mark.ffmpeg
class TestSoftwareEncode:
    def test_h264_quality(self, m, sample_video, tmp_path):
        w = CaptureWorker(m, job_params(m, sample_video, tmp_path))
        assert w.run() is True
        assert output_file(tmp_path, "sample", "_h264", ".mp4")

    def test_h264_size_2pass(self, m, sample_video, tmp_path):
        w = CaptureWorker(m, job_params(m, sample_video, tmp_path,
                                        mode="size", size="5", two_pass=True))
        assert w.run() is True
        assert output_file(tmp_path, "sample", "_h264", ".mp4")

    @pytest.mark.skipif(not _encoder_available("libsvtav1"), reason="libsvtav1 missing")
    def test_av1_quality(self, m, sample_video, tmp_path):
        w = CaptureWorker(m, job_params(m, sample_video, tmp_path, v_codec="AV1"))
        assert w.run() is True
        assert output_file(tmp_path, "sample", "_av1", ".mp4")

    @pytest.mark.skipif(not _encoder_available("libsvtav1"), reason="libsvtav1 missing")
    def test_av1_size_2pass_gop_constraint(self, m, tmp_path):
        # 31s clip: long enough to exercise the non-short size-mode path
        # (2-pass + gop-constraint-rc rate control for libsvtav1)
        src = tmp_path / "long_sample.mp4"
        result = subprocess.run(
            ["ffmpeg", "-hide_banner", "-y",
             "-f", "lavfi", "-i", "testsrc=duration=31:size=640x480:rate=30",
             "-c:v", "libx264", "-preset", "veryfast", "-crf", "20", str(src)],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=60)
        if result.returncode != 0:
            pytest.skip("could not generate long sample video")
        w = CaptureWorker(m, job_params(m, src, tmp_path,
                                        mode="size", size="5", v_codec="AV1",
                                        two_pass=True))
        assert w.run() is True
        assert output_file(tmp_path, "long_sample", "_av1", ".mp4")
        assert "-g 150" in w.log_text and "gop-constraint-rc=1" in w.log_text

    @pytest.mark.skipif(not _encoder_available("libvpx-vp9"), reason="libvpx-vp9 missing")
    def test_vp9_quality_auto_container(self, m, sample_video, tmp_path):
        w = CaptureWorker(m, job_params(m, sample_video, tmp_path,
                                        v_codec="VP9", container="Auto", a_codec="Opus",
                                        audio_encoders={"opus": True, "aac": False}))
        assert w.run() is True
        # Opus + VP9 -> WebM
        assert output_file(tmp_path, "sample", "_vp9", ".webm")

    def test_fps_change(self, m, sample_video, tmp_path):
        w = CaptureWorker(m, job_params(m, sample_video, tmp_path, fps_choice="24"))
        assert w.run() is True

    def test_compat_block_webm_hevc(self, m, sample_video, tmp_path):
        w = CaptureWorker(m, job_params(m, sample_video, tmp_path,
                                        v_codec="H.265", container="WEBM"))
        assert w.run() is False
        assert any("not compatible" in msg for msg in w.warnings)

    def test_pcm_mp4_blocked(self, m, sample_video, tmp_path):
        w = CaptureWorker(m, job_params(m, sample_video, tmp_path, a_codec="PCM"))
        assert w.run() is False
        assert any("PCM" in msg for msg in w.warnings)


@needs_hw
@needs_ffmpeg
@pytest.mark.ffmpeg
class TestHardwareEncode:
    def test_cpu_decode_hw_upload(self, m, sample_video, tmp_path):
        # Explicitly force the CPU-decode path via decoder caps
        w = CaptureWorker(m, job_params(m, sample_video, tmp_path,
                                        use_hw=True, algo="VAAPI (HW)",
                                        hw_decoder_caps={"h264": False, "gpu_vendor": "amd"}))
        assert w.run() is True
        assert "CPU Decode" in w.log_text

    def test_universal_pipeline_with_fallback(self, m, sample_video, tmp_path):
        # On hosts where the universal (hw surface passthrough) pipeline fails,
        # the worker must transparently retry with CPU decode. Either way the
        # job must succeed.
        w = CaptureWorker(m, job_params(m, sample_video, tmp_path,
                                        use_hw=True, algo="VAAPI (HW)",
                                        hw_decoder_caps={"h264": True, "gpu_vendor": "amd"}))
        assert w.run() is True
        assert output_file(tmp_path, "sample", "_h264", ".mp4")

    def test_hw_quality_mode_cqp(self, m, sample_video, tmp_path):
        w = CaptureWorker(m, job_params(m, sample_video, tmp_path,
                                        use_hw=True, algo="VAAPI (HW)", crf=25,
                                        hw_decoder_caps={"h264": False, "gpu_vendor": "amd"},
                                        hw_encoder_opts={"h264": {"rc_mode", "qp", "quality"}}))
        assert w.run() is True
        assert "-rc_mode" in w.log_text and "CQP" in w.log_text
