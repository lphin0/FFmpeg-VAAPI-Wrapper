"""Tests for ProbingCoordinator (ordered log flushing, timeout) and HW prober wiring."""
import time


class TestProbingCoordinator:
    def test_single_reset_flush(self, m):
        c = m.ProbingCoordinator()
        flushed = []
        c.reset(["sw_encoders"])
        assert c.submit_logs("sw_encoders", "sw logs", flushed.append) is True
        assert flushed == [["sw logs"]]

    def test_accumulated_resets_flush_once_in_order(self, m):
        # regression: reset() used to wipe earlier groups, so logs from
        # previously-registered probers were dropped or flushed out of order
        c = m.ProbingCoordinator()
        flushed = []
        c.reset(["sw_encoders"])
        c.reset(["audio_encoders"])
        c.reset(["hw_encoders", "hw_decoders"])
        assert c.submit_logs("hw_decoders", "d", flushed.append) is False
        assert c.submit_logs("sw_encoders", "s", flushed.append) is False
        assert c.submit_logs("hw_encoders", "h", flushed.append) is False
        assert c.submit_logs("audio_encoders", "a", flushed.append) is True
        # flushed in PROBER_ORDER regardless of completion order
        assert flushed == [["s", "a", "h", "d"]]

    def test_flush_respects_prober_order(self, m):
        c = m.ProbingCoordinator()
        flushed = []
        c.reset(["hw_decoders", "sw_encoders"])
        c.submit_logs("hw_decoders", "d", flushed.append)
        c.submit_logs("sw_encoders", "s", flushed.append)
        assert flushed == [["s", "d"]]

    def test_timeout_marks_pending_and_flushes(self, m):
        c = m.ProbingCoordinator()
        flushed = []
        timeouts = []
        c.set_timeout_callback(lambda el: timeouts.append(el))
        c.reset(["hw_encoders"])
        c.submit_logs("hw_encoders", "h", flushed.append)  # completes
        c.reset(["vulkan"])
        c.start_time = time.time() - 999
        assert c.check_timeout() is True
        assert flushed == [["h"], ["vulkan: TIMED OUT"]]
        assert timeouts and timeouts[0] > 999

    def test_no_timeout_before_deadline(self, m):
        c = m.ProbingCoordinator()
        c.reset(["vulkan"])
        c.start_time = time.time()
        assert c.check_timeout() is False

    def test_timeout_ignored_when_idle(self, m):
        c = m.ProbingCoordinator()
        assert c.check_timeout() is False


class TestHWDeviceProberWiring:
    def test_default_caps_contain_all_keys(self, m):
        prober = m.HWDeviceProber("ffmpeg", "/dev/dri/renderD128")
        assert prober.capabilities["vp9"] is False
        assert prober.capabilities["encoder_options"] == {}

    def test_query_encoder_options_with_real_ffmpeg(self, m):
        prober = m.HWDeviceProber("ffmpeg", "/dev/dri/renderD128")
        opts = prober._query_encoder_options("h264_vaapi")
        assert isinstance(opts, set)
        # h264_vaapi exposes these on every modern build
        assert "rc_mode" in opts and "qp" in opts
