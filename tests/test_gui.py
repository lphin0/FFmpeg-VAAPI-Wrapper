"""Offscreen GUI tests for MainWindow."""
import os

from PySide6.QtWidgets import QMessageBox


class TestContainerValidation:
    def test_webm_hevc_no_crash_and_tooltip(self, window):
        # regression: validate_container_codec raised NameError on 'v_codec'
        w = window
        w.container_combo.setCurrentText("WEBM")
        w.v_codec.setCurrentText("H.265")
        w.validate_container_codec()
        assert "HEVC" in w.container_combo.toolTip()
        assert "WEBM" in w.container_combo.toolTip()

    def test_mp4_h264_no_warning(self, window):
        w = window
        w.container_combo.setCurrentText("MP4")
        w.v_codec.setCurrentText("H.264")
        w.validate_container_codec()
        assert w.container_combo.toolTip() == ""

    def test_webm_aac_logs_note(self, window):
        w = window
        w.container_combo.setCurrentText("WEBM")
        w.v_codec.setCurrentText("AV1")
        w.a_codec.setCurrentText("AAC")
        w.validate_container_codec()
        assert any("AAC" in w.log.toPlainText() for _ in [0])


class TestPresetOptions:
    def test_av1_hw_presets_are_0_to_13(self, window):
        w = window
        w.v_codec.setCurrentText("AV1")
        w.chk_hw.setChecked(True)
        w.device_capabilities[w.device_combo.currentText()] = {
            "av1": True, "h264": True, "hevc": True, "vp9": False, "encoder_options": {}}
        w.update_preset_options()
        assert w.quality_combo.count() == 14
        assert w.quality_combo.currentData() == 7  # default

    def test_vp9_sw_best_preset_is_zero(self, window):
        w = window
        w.v_codec.setCurrentText("VP9")
        w.chk_hw.setChecked(False)
        w.update_preset_options()
        w.quality_combo.setCurrentIndex(0)  # "Best (Slowest)"
        assert w.quality_combo.currentData() == 0

    def test_preset_zero_survives_batch_params(self, window, m, monkeypatch, tmp_path):
        # regression: `currentData() or 3` turned preset 0 into 3
        w = window
        w.v_codec.setCurrentText("AV1")
        w.chk_hw.setChecked(False)
        w.update_preset_options()
        w.quality_combo.setCurrentIndex(0)
        captured = {}
        monkeypatch.setattr(m.MainWindow, "process_next_job", lambda self: captured.update(self.batch_params))
        f = tmp_path / "in.mp4"
        f.write_bytes(b"x")
        w.add_path_to_queue(str(f))
        w.start_queue()
        assert captured["quality_preset"] == 0


class TestQueue:
    def test_add_remove_clear(self, window, tmp_path):
        w = window
        f1 = tmp_path / "a.mp4"
        f2 = tmp_path / "b.mp4"
        f1.write_bytes(b"x")
        f2.write_bytes(b"x")
        w.add_path_to_queue(str(f1))
        w.add_path_to_queue(str(f2))
        assert w.queue_list.count() == 2
        # duplicates are rejected
        w.add_path_to_queue(str(f1))
        assert w.queue_list.count() == 2
        w.queue_list.item(0).setSelected(True)
        w.remove_from_queue()
        assert w.queue_list.count() == 1
        w.clear_queue()
        assert w.queue_list.count() == 0
        # removed files can be re-added
        w.add_path_to_queue(str(f1))
        assert w.queue_list.count() == 1

    def test_invalid_file_rejected(self, window):
        w = window
        w.add_path_to_queue("/definitely/missing/file.mp4")
        assert w.queue_list.count() == 0

    def test_placeholder_toggles(self, window, tmp_path):
        w = window
        w.update_queue_placeholder()
        assert w.queue_placeholder.isVisible() or not w.queue_placeholder.isHidden()
        f = tmp_path / "c.mp4"
        f.write_bytes(b"x")
        w.add_path_to_queue(str(f))
        assert w.queue_placeholder.isHidden()
        w.clear_queue()
        assert not w.queue_placeholder.isHidden()


class TestStartQueueParams:
    def test_batch_params_captured(self, window, m, monkeypatch, tmp_path):
        w = window
        captured = {}
        monkeypatch.setattr(m.MainWindow, "process_next_job", lambda self: captured.update(self.batch_params))
        f = tmp_path / "in.mp4"
        f.write_bytes(b"x")
        w.add_path_to_queue(str(f))
        w.v_codec.setCurrentText("H.264")
        w.chk_hw.setChecked(True)
        w.mode_combo.setCurrentText("Target Size")
        w.target_size.setText("25")
        w.start_queue()
        assert captured["mode"] == "size"
        assert captured["size"] == "25"
        assert captured["v_codec"] == "H.264"
        assert captured["use_hw"] is True
        assert "hw_encoder_opts" in captured
        assert "hw_decoder_caps" in captured

    def test_custom_fps_parsed(self, window, m, monkeypatch, tmp_path):
        w = window
        captured = {}
        monkeypatch.setattr(m.MainWindow, "process_next_job", lambda self: captured.update(self.batch_params))
        f = tmp_path / "in.mp4"
        f.write_bytes(b"x")
        w.add_path_to_queue(str(f))
        w.fps_combo.setCurrentText("Custom")
        w.fps_custom_input.setText("59.94")
        w.start_queue()
        assert captured["fps_choice"] == "59.94"


class TestJobOutcomeTracking:
    def test_failure_counted_and_reported(self, window, m, monkeypatch, tmp_path):
        w = window
        monkeypatch.setattr(m.MainWindow, "process_next_job", lambda self: None)
        f = tmp_path / "in.mp4"
        f.write_bytes(b"x")
        w.add_path_to_queue(str(f))
        w._total_jobs = 1
        w._failed_count = 0

        class FakeWorker:
            class _Lock:
                def __enter__(self):
                    return self

                def __exit__(self, *a):
                    return False
            _process_lock = _Lock()
            is_cancelled = False
            params = {"input": str(f)}

            def isRunning(self):
                return False

        w.worker = FakeWorker()
        w.job_finished(False)
        assert w._failed_count == 1
        assert w.queue_list.count() == 0
        assert any("failed" in w.log.toPlainText().lower() for _ in [0])

    def test_all_finished_reports_failures(self, window, monkeypatch):
        w = window
        w._total_jobs = 3
        w._failed_count = 2
        messages = []
        monkeypatch.setattr(QMessageBox, "information", lambda *a, **k: messages.append(a[2]) if len(a) > 2 else None)
        w.chk_no_popup.setChecked(False)
        w.all_finished()
        assert messages and "2/3" in messages[0]

    def test_all_finished_success_message(self, window, monkeypatch):
        w = window
        w._total_jobs = 3
        w._failed_count = 0
        messages = []
        monkeypatch.setattr(QMessageBox, "information", lambda *a, **k: messages.append(a[2]) if len(a) > 2 else None)
        w.chk_no_popup.setChecked(False)
        w.all_finished()
        assert messages and "successfully" in messages[0]


class TestProbingLifecycle:
    def test_probe_timeout_unblocks_ui(self, window):
        w = window
        w._probing_active = True
        w._set_ui_enabled(False)
        w._on_probe_timeout(31.0)
        assert w.btn_start.isEnabled()
        assert w._probing_active is False

    def test_generation_guard_drops_stale_results(self, window):
        w = window
        w.available_sw_codecs = {"h264": False, "hevc": False}
        w._probe_generation = 1
        w.on_sw_encoders_found({"h264": True, "hevc": True}, generation=0)
        assert w.available_sw_codecs == {"h264": False, "hevc": False}
        w.on_sw_encoders_found({"h264": True, "hevc": True}, generation=1)
        assert w.available_sw_codecs["h264"] is True

    def test_device_detection_fallback(self, window, m, monkeypatch):
        monkeypatch.setattr(m.glob, "glob", lambda pattern: [] if pattern.startswith("/dev/dri") else m.glob.glob(pattern))
        w = window
        w.detect_devices()
        assert w.device_combo.count() == 1
        assert w.device_combo.currentText() == "/dev/dri/renderD128"
