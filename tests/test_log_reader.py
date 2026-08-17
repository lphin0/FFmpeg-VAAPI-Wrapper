"""Tests for EncoderOutputReader (log draining, progress/ETA parsing)."""
from io import StringIO


def make_reader(m, text, video_duration=100.0, encode_start_time=None, cancelled=None, **kw):
    logs, progress, etas = [], [], []
    reader = m.EncoderOutputReader(
        stream=StringIO(text),
        video_duration=video_duration,
        encode_start_time=encode_start_time,
        log_cb=logs.append,
        progress_cb=progress.append,
        eta_cb=etas.append,
        is_cancelled=cancelled or (lambda: False),
    )
    return reader, logs, progress, etas


class TestFiltering:
    def test_only_keyword_lines_forwarded(self, m):
        reader, logs, _, _ = make_reader(m, "frame=   10 fps=0.0\nsome noise line\nStream #0:0 -> #0:0\n")
        reader.read_loop()
        assert len(logs) == 2
        assert "frame=" in logs[0]
        assert "Stream #0" in logs[1]

    def test_error_lines_forwarded_case_insensitive(self, m):
        reader, logs, _, _ = make_reader(m, "Error opening input\n")
        reader.read_loop()
        assert logs == ["Error opening input"]


class TestProgressParsing:
    def test_progress_from_time_marker(self, m):
        reader, _, progress, _ = make_reader(m, "time=00:00:25.00 bitrate=N/A", video_duration=100.0)
        reader.read_loop()
        assert progress == [25]

    def test_progress_clamped(self, m):
        reader, _, progress, _ = make_reader(m, "time=00:02:00.00\n", video_duration=100.0)
        reader.read_loop()
        assert progress == [100]

    def test_no_progress_without_duration(self, m):
        reader, _, progress, _ = make_reader(m, "time=00:00:25.00\n", video_duration=0.0)
        reader.read_loop()
        assert progress == []

    def test_eta_computed(self, m, monkeypatch):
        reader, _, progress, etas = make_reader(
            m, "time=00:00:25.00\n", video_duration=100.0, encode_start_time=10.0)
        monkeypatch.setattr(m.time, "time", lambda: 20.0)  # 10s elapsed for 25s of video
        reader.read_loop()
        assert progress == [25]
        # rate = 25/10 = 2.5s video per second; remaining 75s -> 30s ETA
        assert etas == [30]


class TestNeverStopsReading:
    def test_reader_drains_past_emit_cap(self, m):
        # regression: the old loop broke after MAX_EMIT lines, letting the pipe
        # fill and blocking ffmpeg forever. The reader must keep draining.
        lines = "frame= 1\n" * (m.EncoderOutputReader.MAX_EMIT + 500)
        reader, logs, _, _ = make_reader(m, lines, video_duration=0.0)
        suppressed = reader.read_loop()
        assert suppressed == 500
        assert len(logs) >= m.EncoderOutputReader.MAX_EMIT
        assert any("suppressed" in l for l in logs)

    def test_cancellation_stops_loop(self, m):
        state = {"n": 0}

        def cancelled():
            state["n"] += 1
            return state["n"] > 2

        reader, logs, _, _ = make_reader(
            m, "frame= 1\nframe= 2\nframe= 3\n", video_duration=0.0, cancelled=cancelled)
        reader.read_loop()
        assert len(logs) < 3

    def test_empty_stream(self, m):
        reader, logs, progress, _ = make_reader(m, "")
        assert reader.read_loop() == 0
        assert logs == []
        assert progress == []
