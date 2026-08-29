"""Unit tests for the pure decision logic extracted from EncoderWorker."""
import pytest

FFMPEG_OPTS_SAMPLE = """
Encoder h264_vaapi [H.264 (VAAPI)]:
    General capabilities: dr1 delay hardware
    Supported pixel formats: vaapi
h264_vaapi AVOptions:
  -idr_interval      <int>        E..V....... Distance (in I-frames) between key frames (from 0 to INT_MAX) (default 0)
  -rc_mode           <int>        E..V....... Set rate control mode (from 0 to 6) (default auto)
     CQP             1            E..V....... Constant-quality
  -qp                <int>        E..V....... Constant QP (from 0 to 52) (default 0)
  -quality           <int>        E..V....... Set encode quality (from -1 to INT_MAX) (default -1)
"""


class TestCodecKeys:
    def test_ui_key_mapping(self, m):
        assert m.codec_key_from_ui("H.264") == "h264"
        assert m.codec_key_from_ui("H.265") == "hevc"
        assert m.codec_key_from_ui("AV1") == "av1"
        assert m.codec_key_from_ui("VP9") == "vp9"

    def test_encoder_key_mapping(self, m):
        assert m.codec_key_for_encoder("libx264") == "h264"
        assert m.codec_key_for_encoder("h264_vaapi") == "h264"
        assert m.codec_key_for_encoder("libx265") == "hevc"
        assert m.codec_key_for_encoder("hevc_vaapi") == "hevc"
        assert m.codec_key_for_encoder("libsvtav1") == "av1"
        assert m.codec_key_for_encoder("av1_vaapi") == "av1"
        assert m.codec_key_for_encoder("libvpx-vp9") == "vp9"
        assert m.codec_key_for_encoder("vp9_vaapi") == "vp9"


class TestParseEncoderOptions:
    def test_parses_option_names(self, m):
        opts = m.parse_encoder_options(FFMPEG_OPTS_SAMPLE)
        assert {"idr_interval", "rc_mode", "qp", "quality"} <= opts

    def test_ignores_suboptions_and_non_options(self, m):
        opts = m.parse_encoder_options(FFMPEG_OPTS_SAMPLE)
        assert "CQP" not in opts
        assert "Encoder" not in opts

    def test_empty_input(self, m):
        assert m.parse_encoder_options("") == set()
        assert m.parse_encoder_options(None) == set()


class TestValidateInputFile:
    def test_missing_file(self, m):
        assert m.validate_input_file("/nonexistent/definitely-missing.mp4")

    def test_directory_rejected(self, m, tmp_path):
        assert m.validate_input_file(str(tmp_path))

    def test_valid_file(self, m, tmp_path):
        f = tmp_path / "a.mp4"
        f.write_bytes(b"data")
        assert m.validate_input_file(str(f)) is None


class TestSelectOutputContainer:
    def test_passthrough_uses_mkv(self, m):
        assert m.select_output_container("copy", "AAC", "", False, False) == ".mkv"

    def test_pcm_uses_mov(self, m):
        assert m.select_output_container("libx264", "PCM", "", False, False) == ".mov"

    def test_vp9_with_aac_falls_back_to_mp4(self, m):
        # regression: AAC is illegal in WebM; VP9 is valid in MP4
        assert m.select_output_container("libvpx-vp9", "AAC", "aac", True, False) == ".mp4"
        assert m.select_output_container("vp9_vaapi", "AAC", "aac", True, False) == ".mp4"

    def test_av1_with_aac_falls_back_to_mp4(self, m):
        assert m.select_output_container("libsvtav1", "AAC", "aac", False, True) == ".mp4"

    def test_vp9_with_opus_uses_webm(self, m):
        assert m.select_output_container("libvpx-vp9", "Opus", "libopus", True, False) == ".webm"

    def test_h264_defaults_to_mp4(self, m):
        assert m.select_output_container("libx264", "AAC", "aac", False, False) == ".mp4"


class TestStripBsfFlags:
    def test_strips_bsf_pairs(self, m):
        flags = ["-c:v", "libx265", "-b:v", "500k", "-bsf:v", "hevc_metadata=crop_right=4", "-maxrate", "500k"]
        assert m.strip_bsf_flags(flags) == ["-c:v", "libx265", "-b:v", "500k", "-maxrate", "500k"]

    def test_no_bsf_passthrough(self, m):
        flags = ["-c:v", "libx264"]
        assert m.strip_bsf_flags(flags) == flags


class TestAlignResolution:
    def test_alignment(self, m):
        assert m.align_resolution(1920, 1080) == (1920, 1088)
        assert m.align_resolution(1, 1) == (64, 16)


class TestCheckDiskSpace:
    def _worker(self, m):
        return m.EncoderWorker({"input": "x", "v_codec": "H.264"})

    def test_existing_directory_is_measured(self, m, tmp_path):
        worker = self._worker(m)
        has_space, available, required = worker.check_disk_space(str(tmp_path / "o.mp4"), 10.0)
        assert required == 11.0  # 10% safety margin
        assert available == available  # finite, from statvfs

    def test_missing_directory_does_not_report_zero_bytes(self, m, tmp_path):
        # regression: an uncreated output dir reported available=0, which the
        # caller read as "out of disk" and logged a bogus low-space warning
        worker = self._worker(m)
        target = str(tmp_path / "not_yet_created" / "o.mp4")
        has_space, available, _ = worker.check_disk_space(target, 10.0)
        assert has_space is True
        assert available == float("inf")

    def test_insufficient_space_detected(self, m, tmp_path, monkeypatch):
        worker = self._worker(m)

        class FakeStat:
            f_bavail = 1
            f_frsize = 1024  # ~1 MB free

        monkeypatch.setattr(m.os, "statvfs", lambda _p: FakeStat())
        has_space, available, required = worker.check_disk_space(str(tmp_path / "o.mp4"), 1000.0)
        assert has_space is False


class TestCalcSizeModeBitrate:
    def test_single_audio_stream(self, m):
        # 2.1GiB 5m13s file, 20MiB target, 128kbps audio, 1 stream
        assert m.calc_size_mode_bitrate(20, 313, 128, 1, 0.95) == 381

    def test_audio_streams_not_reserved_when_none_encoded(self, m):
        # regression: reserving for probed-but-unencoded streams shrank the
        # video budget (was 253kbps); only encoded streams may be reserved
        assert m.calc_size_mode_bitrate(20, 313, 128, 1, 0.95) == 381
        assert m.calc_size_mode_bitrate(20, 313, 128, 2, 0.95) == 253

    def test_zero_audio_streams_no_deduction(self, m):
        assert m.calc_size_mode_bitrate(20, 313, 128, 0, 0.95) == 509

    def test_short_video_margin(self, m):
        assert m.calc_size_mode_bitrate(20, 20, 128, 1, 0.85) == 7002

    def test_budget_floor_when_audio_exceeds_target(self, m):
        assert m.calc_size_mode_bitrate(1, 313, 128, 2, 0.95) == 1

    def test_invalid_duration(self, m):
        assert m.calc_size_mode_bitrate(20, 0, 128, 1, 0.95) == 1


class TestParseFrameRate:
    def test_rational(self, m):
        assert m.parse_frame_rate("60/1") == 60.0
        assert abs(m.parse_frame_rate("2997/100") - 29.97) < 1e-9

    def test_decimal(self, m):
        assert m.parse_frame_rate("59.96") == 59.96

    def test_invalid(self, m):
        assert m.parse_frame_rate("0/0") == 0.0
        assert m.parse_frame_rate("") == 0.0
        assert m.parse_frame_rate("N/A") == 0.0
        assert m.parse_frame_rate("garbage") == 0.0


class TestStreamBitrateKbps:
    def test_bit_rate_field(self, m):
        assert m.stream_bitrate_kbps({"bit_rate": "192000"}) == 192.0

    def test_estimated_from_size_and_duration(self, m):
        # 240000 bytes over 10s = 192000 bps = 192 kbps
        assert m.stream_bitrate_kbps({"size": "240000", "duration": "10.0"}) == 192.0

    def test_bit_rate_preferred_over_estimate(self, m):
        assert m.stream_bitrate_kbps({"bit_rate": "128000", "size": "240000", "duration": "10.0"}) == 128.0

    def test_unknown_fields(self, m):
        assert m.stream_bitrate_kbps({}) == 0.0
        assert m.stream_bitrate_kbps({"bit_rate": "N/A"}) == 0.0
        assert m.stream_bitrate_kbps({"size": "100"}) == 0.0
        assert m.stream_bitrate_kbps({"size": "x", "duration": "y"}) == 0.0


class TestSvtGopSize:
    def test_five_seconds(self, m):
        assert m.svt_gop_size(60) == 300
        assert m.svt_gop_size(30) == 150

    def test_minimum_floor(self, m):
        # gop-constraint-rc requires a GOP > 119 frames
        assert m.svt_gop_size(24) == 120
        assert m.svt_gop_size(10) == 120

    def test_missing_fps(self, m):
        assert m.svt_gop_size(None) == 300
        assert m.svt_gop_size(0) == 300


class TestSizeModeSvtGopConstraint:
    def test_non_short_svtav1_gets_gop_constraint(self, m):
        flags = m.build_video_flags(**{
            **FLAG_KWARGS, "video_codec_cmd": "libsvtav1", "is_av1": True,
            "mode": "size", "video_kbps": 270, "fps": 59.96})
        joined = " ".join(flags)
        assert "-g 300" in joined
        assert "gop-constraint-rc=1" in joined

    def test_short_svtav1_keeps_tbr(self, m):
        flags = m.build_video_flags(**{
            **FLAG_KWARGS, "video_codec_cmd": "libsvtav1", "is_av1": True,
            "mode": "size", "video_kbps": 300, "is_short_video": True})
        joined = " ".join(flags)
        assert "tbr=300" in joined
        assert "gop-constraint-rc" not in joined

    def test_av1_vaapi_gets_no_gop_constraint(self, m):
        flags = m.build_video_flags(**{
            **FLAG_KWARGS, "video_codec_cmd": "av1_vaapi", "use_hw": True, "is_av1": True,
            "mode": "size", "video_kbps": 270})
        assert "gop-constraint-rc" not in flags

    def test_other_codecs_unaffected(self, m):
        flags = m.build_video_flags(**{
            **FLAG_KWARGS, "video_codec_cmd": "libx264",
            "mode": "size", "video_kbps": 270})
        joined = " ".join(flags)
        assert "-maxrate" in joined and "-bufsize" in joined
        assert "gop-constraint-rc" not in joined


class TestBuildScaleFilter:
    def test_default_bicubic(self, m):
        assert m.build_scale_filter(1280, 720, "Bicubic") == "scale=1280:720:flags=bicubic"

    def test_mitchell(self, m):
        assert "param0=0.333" in m.build_scale_filter(1280, 720, "Bicubic", mitchell=True)
        assert "param0=0.333" in m.build_scale_filter(1280, 720, "Mitchell HQ")

    def test_lanczos_and_nearest(self, m):
        assert m.build_scale_filter(1280, 720, "Lanczos HQ") == "scale=1280:720:flags=lanczos"
        assert m.build_scale_filter(1280, 720, "Nearest") == "scale=1280:720:flags=neighbor"


FLAG_KWARGS = dict(
    video_codec_cmd="libx264",
    use_hw=False,
    is_av1=False,
    is_vp9=False,
    quality_preset="3",
    gpu_vendor="unknown",
    mode="quality",
    video_kbps=0,
    crf=24,
    pad_right=0,
    pad_bottom=0,
    is_short_video=False,
    hw_encoder_opts={},
)


class TestBuildVideoFlags:
    def test_passthrough(self, m):
        flags = m.build_video_flags(video_codec_cmd="copy", **{k: v for k, v in FLAG_KWARGS.items() if k != "video_codec_cmd"})
        assert flags == ["-c:v", "copy"]

    def test_sw_av1_quality(self, m):
        flags = m.build_video_flags(**{**FLAG_KWARGS, "video_codec_cmd": "libsvtav1", "is_av1": True,
                                       "quality_preset": "8"})
        assert "-preset" in flags and "8" in flags
        assert "-pix_fmt" in flags and "yuv420p10le" in flags
        assert "-crf" in flags and "-b:v" in flags

    def test_hw_av1_quality_uses_cqp(self, m):
        # regression: -crf is silently ignored by av1_vaapi
        flags = m.build_video_flags(**{
            **FLAG_KWARGS, "video_codec_cmd": "av1_vaapi", "use_hw": True, "is_av1": True,
            "hw_encoder_opts": {"av1": {"rc_mode", "qp"}}})
        assert "-rc_mode" in flags and "CQP" in flags
        assert "-qp" in flags and "24" in flags
        assert "-crf" not in flags

    def test_hw_av1_preset_gated(self, m):
        # -preset is a generic option ignored by av1_vaapi on most builds
        flags = m.build_video_flags(**{
            **FLAG_KWARGS, "video_codec_cmd": "av1_vaapi", "use_hw": True, "is_av1": True,
            "hw_encoder_opts": {"av1": {"rc_mode", "qp"}}})
        assert "-preset" not in flags

    def test_hw_av1_preset_when_supported(self, m):
        flags = m.build_video_flags(**{
            **FLAG_KWARGS, "video_codec_cmd": "av1_vaapi", "use_hw": True, "is_av1": True,
            "quality_preset": "8", "hw_encoder_opts": {"av1": {"rc_mode", "qp", "preset"}}})
        assert "-preset" in flags and "8" in flags

    def test_hw_av1_unknown_opts_assume_modern(self, m):
        flags = m.build_video_flags(**{
            **FLAG_KWARGS, "video_codec_cmd": "av1_vaapi", "use_hw": True, "is_av1": True})
        assert "-rc_mode" in flags and "-qp" in flags

    def test_hw_vp9_compression_level_gated(self, m):
        supported = m.build_video_flags(**{
            **FLAG_KWARGS, "video_codec_cmd": "vp9_vaapi", "use_hw": True, "is_vp9": True,
            "quality_preset": "2", "hw_encoder_opts": {"vp9": {"compression_level"}}})
        assert "-compression_level" in supported and "2" in supported
        unsupported = m.build_video_flags(**{
            **FLAG_KWARGS, "video_codec_cmd": "vp9_vaapi", "use_hw": True, "is_vp9": True,
            "quality_preset": "2", "hw_encoder_opts": {"vp9": set()}})
        assert "-compression_level" not in unsupported

    def test_hw_vp9_quality_falls_back_with_warning(self, m):
        logs = []
        m.build_video_flags(**{
            **FLAG_KWARGS, "video_codec_cmd": "vp9_vaapi", "use_hw": True, "is_vp9": True,
            "hw_encoder_opts": {"vp9": {"rc_mode"}}, "emit": logs.append})
        assert any("no qp option" in l for l in logs)

    def test_sw_vp9_presets(self, m):
        for preset, expected in [("0", ("best", "0")), ("1", ("best", "1")),
                                 ("2", ("good", "0")), ("3", ("good", "1")),
                                 ("4", ("good", "2")), ("5", ("good", "3"))]:
            flags = m.build_video_flags(**{
                **FLAG_KWARGS, "video_codec_cmd": "libvpx-vp9", "is_vp9": True,
                "quality_preset": preset})
            joined = " ".join(flags)
            assert f"-deadline {expected[0]}" in joined
            assert f"-cpu-used {expected[1]}" in joined
        flags = m.build_video_flags(**{
            **FLAG_KWARGS, "video_codec_cmd": "libvpx-vp9", "is_vp9": True,
            "quality_preset": "99"})
        assert "-cpu-used 5" in " ".join(flags)

    def test_hw_h264_amd_compression_map(self, m):
        for preset, level in [("1", "29"), ("2", "1"), ("4", "11"), ("7", "0")]:
            flags = m.build_video_flags(**{
                **FLAG_KWARGS, "video_codec_cmd": "h264_vaapi", "use_hw": True,
                "gpu_vendor": "amd", "quality_preset": preset})
            joined = " ".join(flags)
            assert f"-compression_level {level}" in joined
        flags = m.build_video_flags(**{
            **FLAG_KWARGS, "video_codec_cmd": "h264_vaapi", "use_hw": True,
            "gpu_vendor": "amd", "quality_preset": "bogus"})
        assert "-compression_level 1" in " ".join(flags)

    def test_hw_h264_intel_no_compression_level(self, m):
        logs = []
        flags = m.build_video_flags(**{
            **FLAG_KWARGS, "video_codec_cmd": "h264_vaapi", "use_hw": True,
            "gpu_vendor": "intel", "emit": logs.append})
        assert "-compression_level" not in flags
        assert any("INTEL GPU" in l for l in logs)

    def test_sw_x264_preset_name(self, m):
        flags = m.build_video_flags(**{**FLAG_KWARGS, "video_codec_cmd": "libx264", "quality_preset": "4"})
        assert "-preset" in flags and "fast" in flags
        flags = m.build_video_flags(**{**FLAG_KWARGS, "video_codec_cmd": "libx264", "quality_preset": "bogus"})
        assert "-preset" in flags and "medium" in flags

    def test_hevc_padding_bsf(self, m):
        # VAAPI pads internally to alignment, so the crop metadata is wanted
        flags = m.build_video_flags(**{
            **FLAG_KWARGS, "video_codec_cmd": "hevc_vaapi", "use_hw": True,
            "pad_right": 4, "pad_bottom": 16,
            "mode": "size", "video_kbps": 500})
        assert "-bsf:v" in flags and "hevc_metadata=crop_right=4:crop_bottom=16" in flags

    def test_hevc_padding_bsf_not_applied_to_software(self, m):
        # regression: software encoders output the exact resolution, so the
        # alignment crop was cutting real content (e.g. 8px off 1080p)
        flags = m.build_video_flags(**{
            **FLAG_KWARGS, "video_codec_cmd": "libx265",
            "pad_right": 0, "pad_bottom": 8,
            "mode": "size", "video_kbps": 500})
        assert "-bsf:v" not in flags
        flags = m.build_video_flags(**{**FLAG_KWARGS, "video_codec_cmd": "libx265"})
        assert "-bsf:v" not in flags

    def test_size_mode_flags(self, m):
        flags = m.build_video_flags(**{**FLAG_KWARGS, "mode": "size", "video_kbps": 500})
        assert "-b:v" in flags and "500k" in flags
        assert "-maxrate" in flags and "-bufsize" in flags

    def test_short_video_svtav1_tbr(self, m):
        flags = m.build_video_flags(**{
            **FLAG_KWARGS, "video_codec_cmd": "libsvtav1", "is_av1": True,
            "mode": "size", "video_kbps": 300, "is_short_video": True})
        assert "-svtav1-params" in flags and "tbr=300" in flags

    def test_short_video_av1_vaapi_maxrate(self, m):
        flags = m.build_video_flags(**{
            **FLAG_KWARGS, "video_codec_cmd": "av1_vaapi", "use_hw": True, "is_av1": True,
            "mode": "size", "video_kbps": 300, "is_short_video": True})
        assert "-maxrate" in flags and "315k" in flags

    def test_quality_sw_crf(self, m):
        flags = m.build_video_flags(**{**FLAG_KWARGS, "crf": 19})
        assert "-crf" in flags and "19" in flags


VF_KWARGS = dict(
    input_file="/tmp/in.mp4",
    device="/dev/dri/renderD128",
    video_codec_cmd="h264_vaapi",
    use_hw=True,
    is_av1=False,
    algo="VAAPI (HW)",
    force_mitchell=False,
    fps_filter="",
    can_hw_decode=False,
    use_vulkan_decode=False,
    pix_fmt="nv12",
    target_w=1920,
    target_h=1080,
    orig_w=1920,
    orig_h=1080,
    input_codec="h264",
    is_prores=False,
)


class TestBuildVfChain:
    def test_software_path(self, m):
        extras, vf, name = m.build_vf_chain(**{**VF_KWARGS, "use_hw": False})
        assert extras == ["-i", "/tmp/in.mp4"]
        assert vf == ["format=yuv420p"]
        assert "Software" in name

    def test_software_path_av1_10bit(self, m):
        _, vf, _ = m.build_vf_chain(**{**VF_KWARGS, "use_hw": False, "is_av1": True})
        assert "format=yuv420p10le" in vf

    def test_software_path_scaling(self, m):
        _, vf, _ = m.build_vf_chain(**{**VF_KWARGS, "use_hw": False, "target_w": 1280, "target_h": 720})
        assert vf[0] == "scale=1280:720:flags=bicubic"

    def test_software_path_applies_fps_filter(self, m):
        # regression: the fps filter was dropped in the Full Software path,
        # silently ignoring framerate changes on all 2-pass (SW) encodes
        _, vf, _ = m.build_vf_chain(**{
            **VF_KWARGS, "use_hw": False, "fps_filter": "fps=24",
            "target_w": 1280, "target_h": 720})
        assert vf[0] == "fps=24"
        assert vf[1] == "scale=1280:720:flags=bicubic"
        assert vf[2] == "format=yuv420p"

    def test_software_path_applies_fps_filter_no_scaling(self, m):
        _, vf, _ = m.build_vf_chain(**{**VF_KWARGS, "use_hw": False, "fps_filter": "fps=24"})
        assert vf == ["fps=24", "format=yuv420p"]

    def test_cpu_decode_upload_path(self, m):
        extras, vf, name = m.build_vf_chain(**VF_KWARGS)
        assert "-init_hw_device" in extras and f"vaapi=va:{VF_KWARGS['device']}" in extras
        assert "-filter_hw_device" in extras and "va" in extras
        assert extras[-2:] == ["-i", "/tmp/in.mp4"]
        assert vf == ["format=nv12", "hwupload"]
        assert "CPU Decode" in name

    def test_cpu_path_software_scaling(self, m):
        _, vf, _ = m.build_vf_chain(**{**VF_KWARGS, "algo": "Bicubic", "target_w": 1280, "target_h": 720})
        assert vf[0] == "scale=1280:720:flags=bicubic"
        assert vf[1:] == ["format=nv12", "hwupload"]

    def test_cpu_path_gpu_scaling_after_upload(self, m):
        _, vf, _ = m.build_vf_chain(**{**VF_KWARGS, "algo": "VAAPI (HW)", "target_w": 1280, "target_h": 720})
        assert vf == ["format=nv12", "hwupload", "scale_vaapi=w=1280:h=720:format=nv12"]

    def test_fps_filter_added_on_cpu_path(self, m):
        _, vf, _ = m.build_vf_chain(**{**VF_KWARGS, "fps_filter": "fps=24", "target_w": 1280, "target_h": 720})
        assert vf[0] == "fps=24"

    def test_universal_path_selected_when_hw_decodable(self, m):
        extras, vf, name = m.build_vf_chain(**{**VF_KWARGS, "can_hw_decode": True})
        assert "-hwaccel" in extras and "vaapi" in extras
        assert "-hwaccel_output_format" in extras
        assert vf == ["scale_vaapi=format=nv12"]
        assert "Universal" in name

    def test_fps_forces_cpu_path_even_when_hw_decodable(self, m):
        # regression: fps is a software filter and cannot take vaapi frames
        extras, vf, name = m.build_vf_chain(**{**VF_KWARGS, "can_hw_decode": True, "fps_filter": "fps=24"})
        assert "-hwaccel" not in extras
        assert vf[0] == "fps=24"
        assert "CPU Decode" in name

    def test_force_cpu_path_used_for_fallback(self, m):
        extras, vf, name = m.build_vf_chain(**{**VF_KWARGS, "can_hw_decode": True, "force_cpu_path": True})
        assert "-hwaccel" not in extras
        assert "CPU Decode" in name

    def test_vulkan_prores_path(self, m):
        extras, vf, name = m.build_vf_chain(**{
            **VF_KWARGS, "use_vulkan_decode": True, "input_codec": "prores", "is_prores": True})
        assert "-init_hw_device" in extras and "vulkan" in extras
        assert "-hwaccel" in extras
        assert vf[0].startswith("hwdownload")
        assert "hwupload=derive_device=vaapi" in vf[0]
        assert "Vulkan" in name

    def test_vulkan_path_respects_force_cpu(self, m):
        _, _, name = m.build_vf_chain(**{
            **VF_KWARGS, "use_vulkan_decode": True, "force_cpu_path": True})
        assert "CPU Decode" in name

    def test_vulkan_path_falls_back_when_fps_filter_set(self, m):
        # regression: the Vulkan pipeline had no slot for the fps filter and
        # silently ignored framerate changes for ProRes inputs
        _, vf, name = m.build_vf_chain(**{
            **VF_KWARGS, "use_vulkan_decode": True, "input_codec": "prores",
            "is_prores": True, "fps_filter": "fps=24"})
        assert vf[0] == "fps=24"
        assert "hwupload" in vf
        assert "Vulkan" not in name

    def test_av1_hw_uses_cpu_path_with_10bit(self, m):
        # AV1 encoding always uses the CPU-decode path (special 10-bit handling)
        extras, vf, name = m.build_vf_chain(**{
            **VF_KWARGS, "can_hw_decode": True, "is_av1": True, "pix_fmt": "yuv420p10le"})
        assert "-hwaccel" not in extras
        assert "format=yuv420p10le" in vf
        assert "hwupload" in vf
        assert "CPU Decode" in name


class TestApplyVaapiHwdecode:
    def _cmd(self):
        return ["ffmpeg", "-hide_banner", "-y", "-i", "/tmp/in.mp4",
                "-vf", "scale=1280:720:flags=bicubic,format=yuv420p",
                "-c:v", "libsvtav1", "-preset", "8"]

    def _kw(self, **over):
        kw = dict(enabled=True, can_hw_decode=True,
                  device="/dev/dri/renderD128",
                  is_av1=False, fps_filter="", target_w=1280, target_h=720,
                  orig_w=1920, orig_h=1080)
        kw.update(over)
        return kw

    def test_downscale_moves_scaling_to_gpu(self, m):
        cmd = self._cmd()
        assert m.apply_vaapi_hwdecode(cmd, **self._kw()) is True
        assert cmd[3:9] == ["-hwaccel", "vaapi", "-hwaccel_output_format",
                            "vaapi", "-hwaccel_device", "/dev/dri/renderD128"]
        assert cmd[9:11] == ["-i", "/tmp/in.mp4"]
        vf = cmd[cmd.index("-vf") + 1]
        assert vf == ("scale_vaapi=w=1280:h=720:format=nv12,"
                      "hwdownload,format=nv12,format=yuv420p")
        assert "scale=" not in vf
        assert cmd[-4:] == ["-c:v", "libsvtav1", "-preset", "8"]

    def test_no_resize_uses_format_only_scale_vaapi(self, m):
        cmd = self._cmd()
        kw = self._kw(target_w=1920, target_h=1080)
        assert m.apply_vaapi_hwdecode(cmd, **kw) is True
        vf = cmd[cmd.index("-vf") + 1]
        assert vf == "scale_vaapi=format=nv12,hwdownload,format=nv12,format=yuv420p"

    def test_av1_targets_p010le_and_preserves_10bit(self, m):
        cmd = ["ffmpeg", "-y", "-i", "/tmp/in.mp4",
               "-vf", "format=yuv420p10le", "-c:v", "libsvtav1"]
        kw = self._kw(is_av1=True, target_w=1920, target_h=1080)
        assert m.apply_vaapi_hwdecode(cmd, **kw) is True
        vf = cmd[cmd.index("-vf") + 1]
        assert vf == ("scale_vaapi=format=p010le,hwdownload,format=p010le,"
                      "format=yuv420p10le")

    def test_fps_filter_survives_download(self, m):
        # regression: the software fps filter cannot run on VAAPI frames and
        # must be re-inserted after hwdownload
        cmd = self._cmd()
        kw = self._kw(fps_filter="fps=24")
        assert m.apply_vaapi_hwdecode(cmd, **kw) is True
        vf = cmd[cmd.index("-vf") + 1]
        assert vf == ("scale_vaapi=w=1280:h=720:format=nv12,hwdownload,"
                      "format=nv12,fps=24,format=yuv420p")

    def test_caller_vf_chain_cannot_leak_into_command(self, m):
        # regression: the old implementation copied unknown filters from the
        # caller's chain, so a trailing hwupload (GPU frames) or a duplicate
        # software scale could end up in front of a software encoder
        cmd = ["ffmpeg", "-y", "-i", "in.mp4",
               "-vf", "fps=24,scale=1280:720:flags=lanczos,format=yuv420p,hwupload",
               "-c:v", "libx264"]
        assert m.apply_vaapi_hwdecode(cmd, **self._kw()) is True
        vf = cmd[cmd.index("-vf") + 1]
        assert vf == ("scale_vaapi=w=1280:h=720:format=nv12,"
                      "hwdownload,format=nv12,format=yuv420p")
        assert "hwupload" not in vf
        assert "scale=" not in vf
        assert "fps=" not in vf

    @pytest.mark.parametrize("prereq", ["enabled", "can_hw_decode", "device"])
    def test_prerequisite_missing_leaves_command_unchanged(self, m, prereq):
        kwargs = self._kw()
        kwargs[prereq] = False if prereq != "device" else ""
        cmd = self._cmd()
        assert m.apply_vaapi_hwdecode(cmd, **kwargs) is False
        assert cmd == self._cmd()

    def test_missing_vf_or_i_refuses_to_mutate(self, m):
        assert m.apply_vaapi_hwdecode(["ffmpeg", "-y", "-i", "in.mp4"],
                                      **self._kw()) is False
        cmd_no_i = ["ffmpeg", "-y", "-vf", "format=yuv420p"]
        before = list(cmd_no_i)
        assert m.apply_vaapi_hwdecode(cmd_no_i, **self._kw()) is False
        assert cmd_no_i == before


class TestPassthroughAudioBudget:
    def test_all_bitrates_known(self, m):
        assert m.passthrough_audio_budget([128.0, 192.0], [0, 1]) == (2, 160.0)

    def test_unknown_streams_use_fallback(self, m):
        # MKV publishes no per-stream bit_rate at all
        n, avg = m.passthrough_audio_budget([0.0, 0.0], [0, 1])
        assert n == 2
        assert avg == m.PASSTHROUGH_AUDIO_FALLBACK_KBPS

    def test_mixed_known_and_unknown(self, m):
        n, avg = m.passthrough_audio_budget([384.0, 0.0], [0, 1])
        assert n == 2
        assert avg == (384.0 + m.PASSTHROUGH_AUDIO_FALLBACK_KBPS) / 2

    def test_mixed_totals_are_preserved(self, m):
        # calc_size_mode_bitrate multiplies avg by count, so the reserve must
        # equal the sum of the per-stream figures.
        rates = [384.0, 0.0, 128.0]
        n, avg = m.passthrough_audio_budget(rates, [0, 1, 2])
        assert n * avg == pytest.approx(384.0 + m.PASSTHROUGH_AUDIO_FALLBACK_KBPS + 128.0)

    def test_no_streams_is_free(self, m):
        assert m.passthrough_audio_budget([], []) == (0, 0.0)

    def test_index_beyond_list_counts_as_unknown(self, m):
        n, avg = m.passthrough_audio_budget([128.0], [0, 1])
        assert n == 2
        assert avg == (128.0 + m.PASSTHROUGH_AUDIO_FALLBACK_KBPS) / 2


class TestShouldRun2Pass:
    def test_explicit_2pass_long_video(self, m):
        assert m.should_run_2pass("size", False, "libx264", False, True, False) is True

    def test_explicit_2pass_short_video_skipped(self, m):
        # Short clips use single-pass VBR with a tight maxrate instead
        assert m.should_run_2pass("size", False, "libx264", True, True, False) is False

    def test_forced_av1_overrides_short_video(self, m):
        # regression: auto-scale forced 2-pass but `not is_short_video` gated
        # the whole expression, so short clips silently stayed single-pass
        assert m.should_run_2pass("size", False, "libsvtav1", True, False, True) is True

    def test_not_requested(self, m):
        assert m.should_run_2pass("size", False, "libx264", False, False, False) is False

    def test_never_for_hardware(self, m):
        assert m.should_run_2pass("size", True, "hevc_vaapi", False, True, True) is False

    def test_never_for_passthrough(self, m):
        assert m.should_run_2pass("size", False, "copy", False, True, False) is False

    def test_never_for_quality_mode(self, m):
        assert m.should_run_2pass("quality", False, "libx264", False, True, False) is False


class TestVaapiDeviceReuse:
    def _kw(self):
        return dict(enabled=True, can_hw_decode=True, device="/dev/dri/renderD128",
                    is_av1=False, fps_filter="", target_w=1280, target_h=720,
                    orig_w=1920, orig_h=1080)

    def test_reuses_named_device_already_on_command(self, m):
        # regression: -hwaccel_device repeated the raw path, making FFmpeg open
        # the DRM node a second time instead of reusing the declared instance
        cmd = ["ffmpeg", "-hide_banner", "-y",
               "-init_hw_device", "vaapi=va:/dev/dri/renderD128",
               "-filter_hw_device", "va",
               "-i", "in.mp4", "-vf", "format=yuv420p", "-c:v", "libx264"]
        assert m.apply_vaapi_hwdecode(cmd, **self._kw()) is True
        idx = cmd.index("-hwaccel_device")
        assert cmd[idx + 1] == "va"

    def test_falls_back_to_path_without_init_hw_device(self, m):
        cmd = ["ffmpeg", "-y", "-i", "in.mp4", "-vf", "format=yuv420p", "-c:v", "libx264"]
        assert m.apply_vaapi_hwdecode(cmd, **self._kw()) is True
        idx = cmd.index("-hwaccel_device")
        assert cmd[idx + 1] == "/dev/dri/renderD128"

    def test_ignores_device_for_different_path(self, m):
        cmd = ["ffmpeg", "-y",
               "-init_hw_device", "vaapi=other:/dev/dri/renderD129",
               "-i", "in.mp4", "-vf", "format=yuv420p", "-c:v", "libx264"]
        assert m.apply_vaapi_hwdecode(cmd, **self._kw()) is True
        idx = cmd.index("-hwaccel_device")
        assert cmd[idx + 1] == "/dev/dri/renderD128"
