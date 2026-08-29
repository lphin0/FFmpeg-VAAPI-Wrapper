"""Offscreen GUI tests for MainWindow."""
from PySide6.QtCore import QThread
from PySide6.QtWidgets import QMessageBox


class TestHwDecodePassesCheckbox:
    def test_not_clipped_in_its_own_row(self, m, window):
        # regression: the checkbox was crammed into the width-capped
        # target-size row and its label was clipped to a few pixels
        w = window
        assert w.chk_hw_decode_passes.width() >= w.chk_hw_decode_passes.sizeHint().width()

    def test_only_in_size_mode(self, window):
        w = window
        w.mode_combo.setCurrentText("Target Size")
        assert w.chk_hw_decode_passes.isEnabled()
        w.mode_combo.setCurrentText("Quality")
        assert not w.chk_hw_decode_passes.isEnabled()
        # Not usable without 2-pass, so it must not stay checked
        assert not w.chk_hw_decode_passes.isChecked()

    def test_unchecks_when_2pass_disabled(self, window):
        w = window
        w.mode_combo.setCurrentText("Target Size")
        w.chk_hw.setChecked(False)
        w.chk_2pass.setEnabled(True)
        w.chk_2pass.setChecked(True)
        w.chk_hw_decode_passes.setChecked(True)
        assert w.chk_hw_decode_passes.isChecked()
        w.chk_2pass.setChecked(False)
        assert not w.chk_hw_decode_passes.isChecked()

    def test_batch_params_gate_on_2pass(self, m, window, monkeypatch, tmp_path):
        w = window
        captured = {}
        monkeypatch.setattr(m.MainWindow, "process_next_job", lambda self: captured.update(self.batch_params))
        f = tmp_path / "in_hw2p.mp4"
        f.write_bytes(b"x")
        w.add_path_to_queue(str(f))
        w.mode_combo.setCurrentText("Target Size")
        w.chk_hw.setChecked(False)
        w.chk_2pass.setChecked(True)
        w.chk_hw_decode_passes.setChecked(True)
        w.start_queue()
        assert captured["hw_decode_2pass"] is True

        # Without 2-pass the sub-option must not leak into the job
        w.clear_queue()
        f2 = tmp_path / "in_hw2p_off.mp4"
        f2.write_bytes(b"x")
        w.add_path_to_queue(str(f2))
        w.chk_2pass.setChecked(False)
        w.chk_hw_decode_passes.setChecked(True)
        w.start_queue()
        assert captured["hw_decode_2pass"] is False


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
            was_cancelled_flag = False
            params = {"input": str(f)}

            def was_cancelled(self):
                return self.was_cancelled_flag

            def isRunning(self):
                return False

        w.worker = FakeWorker()
        w.job_finished(False)
        assert w._failed_count == 1
        assert w.queue_list.count() == 0
        assert any("failed" in w.log.toPlainText().lower() for _ in [0])

    def test_cancelled_job_keeps_queue(self, window, m, monkeypatch, tmp_path):
        w = window
        monkeypatch.setattr(m.MainWindow, "process_next_job", lambda self: None)
        f = tmp_path / "in.mp4"
        f.write_bytes(b"x")
        w.add_path_to_queue(str(f))
        w._total_jobs = 1
        w._failed_count = 0

        class FakeWorker:
            params = {"input": str(f)}

            def was_cancelled(self):
                return True

            def isRunning(self):
                return False

        w.worker = FakeWorker()
        w.job_finished(False)
        assert w._failed_count == 0  # cancellations are not failures
        assert w.queue_list.count() == 1  # cancelled queue is preserved

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


class TestToggleMode:
    def test_leaving_passthrough_reenables_fps_controls(self, window):
        # regression: fps_combo/fps_custom_input stayed disabled after
        # selecting Passthrough mode and switching back
        w = window
        w.mode_combo.setCurrentText("Passthrough")
        assert not w.fps_combo.isEnabled()
        w.mode_combo.setCurrentText("Target Size")
        assert w.fps_combo.isEnabled()
        assert w.fps_custom_input.isEnabled()
        w.mode_combo.setCurrentText("Quality")
        assert w.fps_combo.isEnabled()
        assert w.fps_custom_input.isEnabled()


class TestProbingLifecycle:
    def test_probe_timeout_unblocks_ui(self, window):
        w = window
        w._probing_active = True
        w._set_ui_enabled(False)
        w._on_probe_timeout(31.0)
        assert w.btn_start.isEnabled()
        assert w._probing_active is False

    def test_probing_not_complete_before_device_probe_scheduled(self, window):
        # regression: SW/audio checks finishing before initial_probe ran
        # declared probing complete and stopped the timeout watchdog before
        # the HW/Vulkan probes had even been started
        w = window
        w._probing_active = True
        w.sw_encoder_checker = None
        w.audio_encoder_checker = None
        w.hw_device_prober = None
        w.hw_decoder_checker = None
        w._device_probe_started = False
        w._check_probing_complete()
        assert w._probing_active is True

        w._device_probe_started = True
        w._check_probing_complete()
        assert w._probing_active is False

    def test_vulkan_reprobed_after_flag_reset(self, window, m, monkeypatch):
        # regression: on_ffmpeg_path_changed reset _vulkan_probed to False but
        # probe_device checked hasattr(), so Vulkan was never re-probed
        w = window
        w._vulkan_probed = False
        monkeypatch.setattr(m.HWDeviceProber, "start", lambda self: None)
        monkeypatch.setattr(m.HWDecoderChecker, "start", lambda self: None)
        monkeypatch.setattr(m.VulkanDeviceProber, "start", lambda self: None)
        w.probe_device("/dev/dri/renderD129")
        assert w._vulkan_probed is True
        assert w.vulkan_prober is not None
        assert w.hw_device_prober is not None
        assert w._device_probe_started is True

    def test_retired_prober_kept_referenced_until_finished(self, window, qapp):
        # regression: replaced probers lost their last Python reference while
        # still running, which can crash the process (QThread destroyed mid-run)
        import time
        w = window

        class Sleepy(QThread):
            def run(self):
                time.sleep(0.05)

        old = Sleepy()
        old.start()
        w._retire_prober(old)
        assert old in w._retired_probers
        assert old.wait(2000)
        deadline = time.time() + 2
        while old in w._retired_probers and time.time() < deadline:
            qapp.processEvents()
            time.sleep(0.01)
        assert old not in w._retired_probers

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
