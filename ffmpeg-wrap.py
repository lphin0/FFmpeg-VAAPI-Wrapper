"""
FFmpeg VAAPI Wrapper - Hardware-accelerated video transcoding GUI

A PySide6-based GUI application for transcoding videos using FFmpeg with
VAAPI hardware acceleration support for AMD/Intel GPUs on Linux.

Features:
- VAAPI hardware encoding (H.264, HEVC, AV1, VP9)
- Vulkan hardware decoding for ProRes
- Automatic GPU vendor detection (AMD/Intel)
- Container/codec compatibility validation
- Multi-file batch processing queue
- Real-time progress and ETA tracking
- 2-pass encoding for optimal quality at target sizes
- Auto-downscale for low bitrate scenarios

Requirements:
- Python 3.7+
- PySide6
- FFmpeg with VAAPI support
- FFprobe

Usage:
    python3 ffmpeg-wrap.py

Author: lphin
License: See LICENSE file
"""

from __future__ import annotations

import sys
import os
import subprocess
import glob
import math
import tempfile
import json
import threading
import uuid
import re
import array
from dataclasses import dataclass
from typing import List, Optional, Tuple, Dict, Any
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                                QHBoxLayout, QLabel, QLineEdit, QPushButton,
                                QComboBox, QTextEdit, QFormLayout, QMessageBox,
                                QFileDialog, QCheckBox, QGroupBox, QListWidget,
                                QListWidgetItem, QSlider, QSpinBox, QStackedWidget,
                                QAbstractItemView, QSizePolicy, QProgressBar)
from PySide6.QtCore import Qt, QThread, Signal, QTimer
import time
from concurrent.futures import ThreadPoolExecutor
from PySide6.QtGui import QFont


@dataclass
class VideoInfo:
    """Container for video file probe results"""
    duration: float
    width: int
    height: int
    codec: str
    audio_count: int
    audio_codec: str
    audio_codecs: List[str]
    decodable_audio_indices: List[int]
    is_prores: bool
    has_alpha: bool
    fps: float = 0.0
    
    @property
    def resolution(self) -> Tuple[int, int]:
        return (self.width, self.height)
    
    @property
    def is_valid(self) -> bool:
        return self.width > 0 and self.height > 0


def detect_gpu_vendor(device_path: str) -> str:
    """Detect GPU vendor from DRI device path.
    
    Returns: 'amd', 'intel', or 'unknown'
    """
    try:
        device_num = device_path.split('renderD')[-1]
        vendor_path = f"/sys/class/drm/renderD{device_num}/device/vendor"
        
        if os.path.exists(vendor_path):
            with open(vendor_path, 'r') as f:
                vendor_id = f.read().strip()
                if vendor_id.lower() == '0x8086':
                    return 'intel'
                elif vendor_id.lower() in ['0x1002', '0x1022']:
                    return 'amd'
        
        card_path = f"/sys/class/drm/renderD{device_num}"
        if os.path.exists(card_path):
            device_link = os.path.realpath(f"{card_path}/device")
            if 'i915' in device_link.lower():
                return 'intel'
            elif 'amdgpu' in device_link.lower() or 'radeon' in device_link.lower():
                return 'amd'
        
        return 'unknown'
    except Exception:
        return 'unknown'


# Container compatibility matrices for validation
# MKV imposes no restrictions and is intentionally absent from these dicts
VIDEO_CODEC_CONTAINER_COMPAT = {
    'MP4': {'h264', 'hevc', 'vp9', 'av1', 'mpeg4', 'prores'},
    'WEBM': {'vp8', 'vp9', 'av1'},
    'MOV': {'prores', 'h264', 'hevc', 'dnxhd', 'mpeg4', 'jpeg'},
}

AUDIO_CODEC_CONTAINER_COMPAT = {
    'MP4': {'aac', 'mp3', 'opus', 'alac', 'ac3', 'eac3', 'flac'},
    'WEBM': {'opus', 'vorbis'},
    'MOV': {'pcm_s16le', 'pcm_s24le', 'pcm_s32le', 'pcm_f32le', 'aac', 'alac', 'mp3'},
}


def codec_key_from_ui(codec_text: str) -> str:
    """Map UI codec display text to a canonical codec key."""
    key = codec_text.lower().replace('.', '')
    if key in ('h264', 'h265', 'av1', 'vp9'):
        return 'hevc' if key == 'h265' else key
    return key


def validate_input_file(filepath: str) -> Optional[str]:
    """Validate that an input file exists, is a regular file, and is readable.

    Returns: error message if invalid, or None if the file is valid.
    """
    if not os.path.exists(filepath):
        return f"Input file does not exist: {filepath}"
    if not os.path.isfile(filepath):
        return f"Input path is not a file: {filepath}"
    if not os.access(filepath, os.R_OK):
        return f"No read permission for input file: {filepath}"
    return None


def parse_encoder_options(help_output: str) -> set:
    """Parse `ffmpeg -h encoder=X` output for supported private option names.

    Option availability varies across FFmpeg builds (e.g. `-compression_level`
    on vp9_vaapi), so flags are only emitted when the encoder actually exposes
    them. Returns a set of option names without the leading dash.
    """
    options = set()
    for raw in (help_output or '').split('\n'):
        line = raw.strip()
        if line.startswith('-') and len(line) > 1 and line[1] != ' ':
            name = line[1:].split(None, 1)[0]
            if re.match(r'^[A-Za-z_][A-Za-z0-9_]*$', name):
                options.add(name)
    return options


def codec_key_for_encoder(video_codec_cmd: str) -> str:
    """Map an FFmpeg encoder name to a canonical codec key (h264/hevc/av1/vp9)."""
    enc = video_codec_cmd.lower()
    if 'av1' in enc:
        return 'av1'
    if 'vp9' in enc or 'vpx-vp9' in enc:
        return 'vp9'
    if 'hevc' in enc or 'x265' in enc:
        return 'hevc'
    if 'h264' in enc or 'x264' in enc:
        return 'h264'
    return enc


def build_scale_filter(target_w: int, target_h: int, algo: str, mitchell: bool = False) -> str:
    """Build the software scale filter string for the given algorithm."""
    if mitchell or "Mitchell" in algo:
        return f"scale={target_w}:{target_h}:flags=bicubic:param0=0.333:param1=0.333"
    if "Lanczos" in algo:
        return f"scale={target_w}:{target_h}:flags=lanczos"
    if "Nearest" in algo:
        return f"scale={target_w}:{target_h}:flags=neighbor"
    return f"scale={target_w}:{target_h}:flags=bicubic"


def align_resolution(width: int, height: int, align_w: int = 64, align_h: int = 16) -> Tuple[int, int]:
    """Round resolution up to VAAPI-friendly alignment."""
    new_w = math.ceil(width / align_w) * align_w
    new_h = math.ceil(height / align_h) * align_h
    return new_w, new_h


def select_output_container(video_codec_cmd: str, a_codec: str, audio_enc_name: str,
                            is_vp9: bool, is_av1: bool) -> str:
    """Pick an output extension for 'Auto' container mode.

    WebM only accepts VP8/VP9/AV1 video and Opus/Vorbis audio, so VP9/AV1 with
    non-Opus audio falls back to MP4 (both codecs are valid in MP4).
    """
    if video_codec_cmd == "copy":
        return ".mkv"
    if a_codec == "PCM":
        return ".mov"
    if is_vp9 or is_av1:
        if "opus" in audio_enc_name or "vorbis" in audio_enc_name:
            return ".webm"
        return ".mp4"
    return ".mp4"


def strip_bsf_flags(flags: List[str]) -> List[str]:
    """Remove -bsf:v entries (and their values) from an encoder flag list."""
    result = []
    skip_next = False
    for flag in flags:
        if skip_next:
            skip_next = False
            continue
        if flag == '-bsf:v':
            skip_next = True
            continue
        result.append(flag)
    return result


def parse_frame_rate(rate_str: str) -> float:
    """Parse an ffprobe frame rate string ('60/1', '2997/100') into a float."""
    if not rate_str or rate_str in ('0/0', 'N/A'):
        return 0.0
    try:
        if '/' in rate_str:
            num, den = rate_str.split('/', 1)
            num_f, den_f = float(num), float(den)
            if den_f > 0 and num_f > 0:
                return num_f / den_f
        else:
            val = float(rate_str)
            if val > 0:
                return val
    except (ValueError, TypeError):
        pass
    return 0.0


def svt_gop_size(fps: Optional[float] = None) -> int:
    """GOP length (frames) for SVT-AV1 size-mode rate control.

    gop-constraint-rc requires a GOP larger than 119 frames; target ~5
    seconds like SVT-AV1's own default, and clamp to a safe floor.
    """
    if fps and fps > 0:
        return max(120, int(round(fps * 5)))
    return 300


def calc_size_mode_bitrate(target_mb: float, duration: float, audio_bitrate: float,
                           num_audio_streams: int, margin: float) -> int:
    """Calculate the target video bitrate (kbps) for size-mode encoding.

    Reserves bitrate for exactly the audio streams that will be encoded
    (num_audio_streams), then splits the remaining bits across the video
    duration. Returns at least 1 kbps.
    """
    if duration <= 0:
        return 1
    effective_mb = target_mb * margin
    target_bits = effective_mb * 8 * 1024 * 1024
    audio_total = audio_bitrate * 1000 * duration * num_audio_streams
    video_bits = target_bits - audio_total
    return max(1, int((video_bits / duration) / 1000))


def build_video_flags(*, video_codec_cmd: str, use_hw: bool, is_av1: bool, is_vp9: bool,
                      quality_preset: str, gpu_vendor: str, mode: str, video_kbps: int,
                      crf: int, pad_right: int, pad_bottom: int, is_short_video: bool,
                      hw_encoder_opts: Optional[Dict[str, set]] = None,
                      fps: Optional[float] = None,
                      emit=None) -> List[str]:
    """Build the video encoder flags for the selected codec/mode combination.

    Hardware encoders ignore generic `-preset`/`-crf` options (they print a
    "has not been used" warning), so HW quality mode uses `-rc_mode CQP -qp`,
    and HW-only options like vp9_vaapi's `-compression_level` are only emitted
    when the probed encoder actually supports them.
    """
    emit = emit or (lambda m: None)
    hw_encoder_opts = hw_encoder_opts or {}
    video_flags = ["-c:v", video_codec_cmd]

    if video_codec_cmd == "copy":
        return video_flags

    if is_av1:
        if not use_hw:
            video_flags.extend(["-preset", quality_preset])
            video_flags.extend(["-pix_fmt", "yuv420p10le"])
        elif 'preset' in hw_encoder_opts.get('av1', set()):
            video_flags.extend(["-preset", quality_preset])
        else:
            emit("Notice: This FFmpeg build's av1_vaapi has no -preset option, skipping speed preset.")
    elif is_vp9:
        if use_hw:
            if 'compression_level' in hw_encoder_opts.get('vp9', set()):
                video_flags.extend(["-compression_level", quality_preset])
            else:
                emit("Notice: This FFmpeg build's vp9_vaapi has no -compression_level option, using default.")
        else:
            try:
                vp9_quality = int(quality_preset)
            except (ValueError, TypeError):
                vp9_quality = 4  # Default to "Default" preset
                emit(f"Warning: Invalid quality preset '{quality_preset}', using default.")
            if vp9_quality == 0:
                video_flags.extend(["-deadline", "best", "-cpu-used", "0"])
            elif vp9_quality == 1:
                video_flags.extend(["-deadline", "best", "-cpu-used", "1"])
            elif vp9_quality == 2:
                video_flags.extend(["-deadline", "good", "-cpu-used", "0"])
            elif vp9_quality == 3:
                video_flags.extend(["-deadline", "good", "-cpu-used", "1"])
            elif vp9_quality == 4:
                video_flags.extend(["-deadline", "good", "-cpu-used", "2"])
            elif vp9_quality == 5:
                video_flags.extend(["-deadline", "good", "-cpu-used", "3"])
            else:
                video_flags.extend(["-deadline", "good", "-cpu-used", str(min(vp9_quality, 5))])
            video_flags.extend(["-row-mt", "1", "-pix_fmt", "yuv420p"])
    else:
        if use_hw:
            # Check GPU vendor - VCN compression levels are AMDGPU-specific
            if "h264_vaapi" in video_codec_cmd or "hevc_vaapi" in video_codec_cmd:
                if gpu_vendor == 'amd':
                    # VCN (Video Core Next) compression level mapping for AMDGPU
                    # Binary flags: VBAQ=16, pre-encode=8, quality=4, balanced=2, speed=0, validity=1
                    compression_map = {
                        1: 29,  # Quality (Best): quality + pre-encode + VBAQ (4+8+16+1)
                        2: 1,   # Balanced (Default): balanced + pre-encode + VBAQ (AMD recommended)
                        4: 11,  # Speed (Fast): balanced + pre-encode (2+8+1)
                        7: 0    # Max Speed: speed preset only
                    }
                    try:
                        preset_val = int(quality_preset)
                        compression_level = compression_map.get(preset_val, 1)  # Default to balanced (1)
                        emit(f"HW Preset: {preset_val} -> VCN compression_level={compression_level}")
                        video_flags.extend(["-compression_level", str(compression_level)])
                    except (ValueError, TypeError):
                        emit(f"Warning: Invalid quality preset '{quality_preset}', using default")
                        video_flags.extend(["-compression_level", "1"])
                else:
                    # Intel or other GPU - skip VCN compression levels
                    emit(f"Notice: {gpu_vendor.upper()} GPU detected - VCN compression levels not applied")
        else:
            presets = ["veryslow", "slower", "slow", "medium", "fast", "faster", "veryfast", "superfast", "ultrafast"]
            try:
                p_idx = int(quality_preset)
            except (ValueError, TypeError):
                p_idx = 3  # Default to "medium"
                emit(f"Warning: Invalid quality preset '{quality_preset}', using medium.")
            if 0 <= p_idx < len(presets):
                video_flags.extend(["-preset", presets[p_idx]])

        if "hevc" in video_codec_cmd or "x265" in video_codec_cmd:
            if pad_right > 0 or pad_bottom > 0:
                emit(f"Metadata: Cropping padding (Right:{pad_right}, Bottom:{pad_bottom}) for HEVC.")
                video_flags.extend(["-bsf:v", f"hevc_metadata=crop_right={pad_right}:crop_bottom={pad_bottom}"])
        elif use_hw and (pad_right > 0 or pad_bottom > 0):
            emit(f"Warning: Resolution {pad_right}px padding detected for HW encode; encoder may add internal padding.")

    # --- Mode-specific rate control ---
    if mode == 'size':
        video_flags.extend(["-b:v", f"{video_kbps}k"])
        # For short videos, use VBR with tight maxrate (~10% higher)
        # to prevent overshoot while allowing some rate control flexibility
        if is_short_video:
            if is_av1:
                if video_codec_cmd == "av1_vaapi":
                    av1_maxrate = int(video_kbps * 1.05)
                    video_flags.extend(["-maxrate", f"{av1_maxrate}k", "-bufsize", f"{av1_maxrate}k"])
                else:
                    video_flags.extend(["-svtav1-params", f"tbr={video_kbps}"])
            else:
                maxrate = int(video_kbps * 1.1)
                video_flags.extend(["-maxrate", f"{maxrate}k", "-bufsize", f"{maxrate}k"])
        elif is_av1 and video_codec_cmd == "libsvtav1":
            # SVT-AV1 VBR drifts below the target bitrate on long encodes;
            # gop-constraint-rc pins every GOP to the target rate so the
            # final size matches. Requires a GOP larger than 119 frames.
            gop = svt_gop_size(fps)
            video_flags.extend(["-g", str(gop), "-svtav1-params", "gop-constraint-rc=1"])
        elif not is_av1 and not is_vp9:
            video_flags.extend(["-maxrate", f"{video_kbps}k", "-bufsize", f"{video_kbps*2}k"])
    else:
        crf_val = str(crf)
        if use_hw:
            enc_opts = hw_encoder_opts.get(codec_key_for_encoder(video_codec_cmd), set())
            has_rc = 'rc_mode' in enc_opts
            has_qp = 'qp' in enc_opts
            if has_rc and has_qp:
                video_flags.extend(["-rc_mode", "CQP", "-qp", crf_val])
            elif has_qp:
                video_flags.extend(["-qp", crf_val])
            elif not enc_opts:
                # Option support unknown (probe failed/encoder absent): assume a
                # modern build; -qp is at worst ignored, not fatal
                video_flags.extend(["-rc_mode", "CQP", "-qp", crf_val])
            else:
                emit("Warning: This FFmpeg build's hardware encoder exposes no qp option; quality setting may not be applied.")
                video_flags.extend(["-crf", crf_val, "-b:v", "0"])
        else:
            video_flags.extend(["-crf", crf_val])
            if is_av1 or is_vp9:
                video_flags.extend(["-b:v", "0"])

    return video_flags


def build_vf_chain(*, input_file: str, device: str, video_codec_cmd: str, use_hw: bool,
                   is_av1: bool, algo: str, force_mitchell: bool, fps_filter: str,
                   can_hw_decode: bool, use_vulkan_decode: bool, pix_fmt: str,
                   target_w: int, target_h: int, orig_w: int, orig_h: int,
                   input_codec: str, is_prores: bool,
                   force_cpu_path: bool = False, emit=None) -> Tuple[List[str], List[str], str]:
    """Build the ffmpeg command prefix and filter chain for the chosen pipeline.

    Returns: (base_cmd_extras, vf_chain, pipeline_name)
    """
    emit = emit or (lambda m: None)
    vf_chain = []

    if not use_hw:
        extras = ["-i", input_file]
        if target_w != orig_w or target_h != orig_h:
            vf_chain.append(build_scale_filter(target_w, target_h, algo, force_mitchell or "Mitchell" in algo))
        vf_chain.append("format=yuv420p10le" if is_av1 else "format=yuv420p")
        return extras, vf_chain, "Full Software (CPU)"

    if use_vulkan_decode and not force_cpu_path:
        extras = [
            "-init_hw_device", "vulkan",
            "-init_hw_device", f"vaapi=va:{device}",
            "-filter_hw_device", "va",
            "-hwaccel", "vulkan", "-i", input_file,
        ]
        scale_filters = ["hwdownload"]
        if target_w != orig_w or target_h != orig_h:
            scale_filters.append(f"scale={target_w}:{target_h}:flags=bicubic")
        scale_filters.append(f"format={pix_fmt}")
        scale_filters.append("hwupload=derive_device=vaapi")
        vf_chain.append(",".join(scale_filters))
        return extras, vf_chain, "ProRes Vulkan HW Decode -> VAAPI Encode"

    # The fps filter operates on software frames, so any framerate change
    # requires the CPU decode path (fps -> hw frames fails at filter negotiation).
    use_software_scaler = (
        is_av1 or
        algo != "VAAPI (HW)" or
        force_mitchell or
        bool(fps_filter) or
        force_cpu_path
    )

    if use_software_scaler or not can_hw_decode:
        if not can_hw_decode:
            if is_prores:
                emit("Notice: ProRes uses CPU decode (VAAPI doesn't support ProRes decode)")
            else:
                emit(f"Notice: Codec '{input_codec}' not HW-decodable. Using CPU Decode.")

        extras = [
            "-init_hw_device", f"vaapi=va:{device}",
            "-filter_hw_device", "va",
            "-i", input_file,
        ]
        if fps_filter:
            vf_chain.append(fps_filter)
        if target_w != orig_w or target_h != orig_h:
            if algo == "VAAPI (HW)" and not use_software_scaler:
                # User wants VAAPI scaling but couldn't use HW decode path:
                # scale on GPU after upload
                vf_chain.append(f"format={pix_fmt}")
                vf_chain.append("hwupload")
                vf_chain.append(f"scale_vaapi=w={target_w}:h={target_h}:format={pix_fmt}")
            else:
                # Software scaling before upload
                vf_chain.append(build_scale_filter(target_w, target_h, algo, force_mitchell))
                vf_chain.append(f"format={pix_fmt}")
                vf_chain.append("hwupload")
        else:
            # No scaling needed, just format and upload
            vf_chain.append(f"format={pix_fmt}")
            vf_chain.append("hwupload")
        return extras, vf_chain, "CPU Decode -> VAAPI Upload -> Encode"

    # Universal VAAPI pipeline (Wiki Method #3)
    # Handles both HW-decodable and SW-decodable inputs automatically
    extras = [
        "-init_hw_device", f"vaapi=va:{device}",
        "-hwaccel", "vaapi",
        "-hwaccel_output_format", "vaapi",
        "-hwaccel_device", "va",
        "-i", input_file,
        "-filter_hw_device", "va",
    ]

    # Use scale_vaapi for both scaling and format conversion
    if target_w != orig_w or target_h != orig_h:
        if is_av1:
            # AV1 10-bit hw encode: use p010le for hardware surface
            vf_chain.append(f"scale_vaapi=w={target_w}:h={target_h}:format=p010le")
        else:
            vf_chain.append(f"scale_vaapi=w={target_w}:h={target_h}:format={pix_fmt}")
    else:
        if is_av1:
            vf_chain.append("scale_vaapi=format=p010le")
        else:
            vf_chain.append(f"scale_vaapi=format={pix_fmt}")
    return extras, vf_chain, "Universal VAAPI HW/SW Decode -> Encode"


class EncoderOutputReader:
    """Reads an ffmpeg output stream without ever stopping.

    The previous implementation stopped reading after N matched log lines,
    which let the pipe fill up and block ffmpeg indefinitely. This reader
    keeps draining the stream and only throttles what it forwards to the UI.
    """

    TIME_PATTERN = re.compile(r'time=(\d{2}):(\d{2}):(\d{2}\.\d+)')
    KEYWORD_PATTERN = re.compile(r'frame=|Error|Stream #|kb/s|kB time=|error|Invalid argument', re.IGNORECASE)
    MAX_EMIT = 10000
    SUPPRESSION_NOTICE_EVERY = 1000

    def __init__(self, stream, video_duration: float, encode_start_time: Optional[float],
                 log_cb, progress_cb, eta_cb, is_cancelled):
        self.stream = stream
        self.video_duration = float(video_duration or 0.0)
        self.encode_start_time = encode_start_time
        self.log_cb = log_cb
        self.progress_cb = progress_cb
        self.eta_cb = eta_cb
        self.is_cancelled = is_cancelled

    def read_loop(self) -> int:
        """Read until EOF or cancellation. Returns the number of suppressed lines."""
        emitted = 0
        suppressed = 0
        while True:
            if self.is_cancelled():
                break
            line = self.stream.readline()
            if not line:
                break  # EOF
            line = line.strip()
            if line:
                self._parse_progress(line)
                if self.KEYWORD_PATTERN.search(line):
                    if emitted < self.MAX_EMIT:
                        self.log_cb(line)
                        emitted += 1
                    else:
                        suppressed += 1
                        if suppressed % self.SUPPRESSION_NOTICE_EVERY == 0:
                            self.log_cb(f"(suppressing further log output, {suppressed} lines dropped)")
        if suppressed:
            self.log_cb(f"Log output suppressed for {suppressed} lines (UI limit reached).")
        return suppressed

    def _parse_progress(self, line: str):
        if self.video_duration <= 0:
            return
        match = self.TIME_PATTERN.search(line)
        if not match:
            return
        try:
            hours = int(match.group(1))
            minutes = int(match.group(2))
            seconds = float(match.group(3))
            current_time = hours * 3600 + minutes * 60 + seconds
            progress = int((current_time / self.video_duration) * 100)
            self.progress_cb(max(0, min(100, progress)))

            if self.encode_start_time and current_time > 0:
                elapsed = time.time() - self.encode_start_time
                if elapsed > 0:
                    rate = current_time / elapsed  # seconds of video per second of encode
                    if rate > 0:
                        eta_seconds = int((self.video_duration - current_time) / rate)
                        self.eta_cb(eta_seconds)
        except (ValueError, ZeroDivisionError):
            pass


class ProbingCoordinator:
    """Coordinates log output from multiple prober threads for consistent ordering"""
    
    PROBER_ORDER = ['sw_encoders', 'audio_encoders', 'hw_encoders', 'hw_decoders', 'vulkan']
    DEFAULT_TIMEOUT = 30.0
    
    def __init__(self):
        self.lock = threading.Lock()
        self.pending_probers = set()
        self.completed_logs = {}
        self.callback = None
        self.timeout_callback = None
        self.start_time = None
        self.timeout = self.DEFAULT_TIMEOUT
    
    def reset(self, prober_ids: list, timeout: float = 30.0):
        """Register additional probers without wiping previously registered ones.

        Multiple probe groups (software encoders, audio encoders, hardware
        encoders/decoders, vulkan) are started in sequence; accumulated state
        lets the flush fire once all of them have reported in.
        """
        with self.lock:
            self.pending_probers.update(prober_ids)
            self.start_time = time.time()
            self.timeout = timeout if timeout else self.DEFAULT_TIMEOUT
    
    def set_timeout_callback(self, callback):
        self.timeout_callback = callback
    
    def check_timeout(self) -> bool:
        if self.start_time is None:
            return False
        
        elapsed = time.time() - self.start_time
        if elapsed > self.timeout:
            with self.lock:
                if self.pending_probers:
                    for prober_id in list(self.pending_probers):
                        self.completed_logs[prober_id] = f"{prober_id}: TIMED OUT"
                    self.pending_probers.clear()
                    
                    if self.callback:
                        ordered_logs = []
                        for prober_id in self.PROBER_ORDER:
                            if prober_id in self.completed_logs:
                                ordered_logs.append(self.completed_logs[prober_id])
                        self.callback(ordered_logs)
                    
                    if self.timeout_callback:
                        self.timeout_callback(elapsed)
                    return True
        return False
    
    def submit_logs(self, prober_id: str, logs: str, callback) -> bool:
        with self.lock:
            self.completed_logs[prober_id] = logs
            self.pending_probers.discard(prober_id)
            self.callback = callback
            if len(self.pending_probers) == 0:
                if callback:
                    ordered_logs = []
                    for prober_id in self.PROBER_ORDER:
                        if prober_id in self.completed_logs:
                            ordered_logs.append(self.completed_logs[prober_id])
                    callback(ordered_logs)
                self.completed_logs = {}
                return True
        return False

class EncoderWorker(QThread):
    log_signal = Signal(str)
    scale_notification_signal = Signal(str)
    finished_signal = Signal(bool)
    compatibility_warning_signal = Signal(str)
    progress_signal = Signal(int)  # Progress percentage (0-100)
    eta_signal = Signal(int)  # Estimated seconds remaining

    def __init__(self, params):
        super().__init__()
        self.params = params
        self.process = None
        self.is_cancelled = False
        self.ffmpeg_path = params.get('ffmpeg_path', 'ffmpeg')
        self.ffprobe_path = params.get('ffprobe_path', 'ffprobe')
        self._process_lock = threading.Lock()
        self._output_lock = threading.Lock()
        self.video_duration = 0.0  # Store video duration for progress calculation
        self.output_file = None  # Track output file for cleanup
        self.encode_start_time = None  # Track encode start time for ETA
    
    def validate_device_available(self, device_path: str) -> Tuple[bool, Optional[str]]:
        """Validate hardware device is still available.
        
        Returns: (is_available: bool, error_message: Optional[str])
        """
        if not device_path:
            return True, None  # Software mode, no device needed
        
        if not os.path.exists(device_path):
            return False, f"Hardware device {device_path} not found"
        
        if not os.access(device_path, os.R_OK | os.W_OK):
            return False, f"No read/write permission for device {device_path}"
        
        return True, None
    
    def set_output_file(self, filepath: str):
        """Set the output file path for cleanup on cancellation"""
        with self._output_lock:
            self.output_file = filepath
    
    def check_disk_space(self, output_path: str, estimated_mb: float) -> Tuple[bool, float, float]:
        """Check if there's enough disk space for encoding.
        
        Args:
            output_path: Path to output file
            estimated_mb: Estimated output size in MB
        
        Returns: (has_space: bool, available_mb: float, required_mb: float)
        """
        try:
            output_dir = os.path.dirname(output_path)
            if not output_dir:
                output_dir = '.'
            
            if not os.path.exists(output_dir):
                return True, 0, estimated_mb  # Directory doesn't exist yet, can't check
            
            stat = os.statvfs(output_dir)
            available_mb = (stat.f_bavail * stat.f_frsize) / (1024 * 1024)
            
            # Add 10% safety margin
            required_mb = estimated_mb * 1.1
            
            return (available_mb >= required_mb, available_mb, required_mb)
        except OSError:
            return True, 0, estimated_mb  # On error, allow encoding to proceed
    
    def estimate_output_size(self, params: Dict[str, Any], video_info: VideoInfo) -> float:
        """Estimate output file size in MB.
        
        Returns: Estimated size in MB
        """
        mode = params.get('mode', 'quality')
        
        if mode == 'size':
            # Target size mode - use specified size
            try:
                return float(params.get('size', 10))
            except (ValueError, TypeError):
                return 10.0
        
        # Quality mode - estimate based on bitrate
        # Use a conservative estimate: CRF typically produces files
        # that are 50-80% of the original for similar quality
        try:
            # Get input file size
            input_size_mb = os.path.getsize(params['input']) / (1024 * 1024)
            
            # For CRF encoding, estimate based on resolution and CRF
            crf = params.get('crf', 24)
            # Lower CRF = larger file, higher CRF = smaller file
            # CRF 23 is "neutral", adjust factor accordingly
            crf_factor = 1.0 - ((crf - 23) * 0.05)  # 5% change per CRF step
            crf_factor = max(0.3, min(2.0, crf_factor))  # Clamp between 0.3x and 2x
            
            estimated_mb = input_size_mb * crf_factor * 0.7  # Conservative estimate
            return max(estimated_mb, 1.0)  # At least 1 MB
        except (OSError, TypeError):
            return 50.0  # Default estimate if input size unknown

    def _clamp_bitrate(self, video_kbps: int) -> int:
        """Clamp bitrate to valid range with warning for high values."""
        if video_kbps < 1:
            return 1
        elif video_kbps > 100000:
            self.log_signal.emit(f"Warning: Calculated bitrate {video_kbps}k is very high, capping at 100000k.")
            return 100000
        return video_kbps

    def cancel(self):
        self.is_cancelled = True
        with self._process_lock:
            if self.process:
                self.log_signal.emit("--- CANCELLING ENCODE... ---")
                try:
                    self.process.terminate()
                    self.process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    try:
                        self.process.kill()
                        self.process.wait(timeout=5)
                    except (subprocess.TimeoutExpired, OSError, Exception) as e:
                        self.log_signal.emit(f"Warning: Process termination incomplete: {e}")
                except (OSError, Exception) as e:
                    self.log_signal.emit(f"Warning: Error terminating process: {e}")
                self.process = None
        
        # Clean up partial output file
        with self._output_lock:
            if self.output_file and os.path.exists(self.output_file):
                try:
                    os.remove(self.output_file)
                    self.log_signal.emit(f"Cleaned up partial file: {os.path.basename(self.output_file)}")
                except (OSError, PermissionError) as e:
                    self.log_signal.emit(f"Warning: Could not clean up partial file: {e}")

    def get_video_info(self, filepath: str) -> Optional[VideoInfo]:
        if self.is_cancelled:
            return None

        cmd = [
            self.ffprobe_path,
            "-v", "error",
            "-show_streams",
            "-show_entries", "format=duration",
            "-of", "json",
            filepath
        ]

        try:
            output = subprocess.check_output(cmd, stderr=subprocess.STDOUT).decode().strip()
            data = json.loads(output)

            if not data.get('streams'):
                raise ValueError("No streams found.")

            video_stream = next((s for s in data.get('streams', []) if s.get('codec_type') == 'video'), None)
            if not video_stream:
                raise ValueError("No video stream found.")

            w = int(video_stream.get('width', 0) or 0)
            h = int(video_stream.get('height', 0) or 0)
            codec = video_stream.get('codec_name', 'unknown') or 'unknown'
            fps = parse_frame_rate(video_stream.get('avg_frame_rate', '') or video_stream.get('r_frame_rate', ''))

            audio_streams = [s for s in data.get('streams', []) if s.get('codec_type') == 'audio']
            audio_count = len(audio_streams)
            decodable_audio_indices = []
            audio_codec = ""
            audio_codecs = []

            for i, s in enumerate(audio_streams):
                c_name = (s.get('codec_name') or '').strip().lower()
                audio_codecs.append(c_name if c_name else 'unknown')
                if c_name and c_name not in ('unknown', 'none'):
                    decodable_audio_indices.append(i)
                    if not audio_codec:
                        audio_codec = s.get('codec_name', '') or ''
                else:
                    self.log_signal.emit(f"Notice: Audio stream {i} has no codec name, skipping")

            dur = None
            if 'format' in data and 'duration' in data['format']:
                try:
                    dur = float(data['format']['duration'])
                except (ValueError, TypeError):
                    pass

            if dur is None and 'duration' in video_stream:
                try:
                    dur = float(video_stream['duration'])
                except (ValueError, TypeError):
                    pass

            if dur is None:
                self.log_signal.emit("Warning: Duration metadata missing.")
                dur = 0.0

            is_prores = codec in ('prores', 'prores_aw', 'prores_ks')
            has_alpha = False
            if is_prores and codec == 'prores_ks':
                pix_fmt = video_stream.get('pix_fmt', '')
                if 'yuva' in pix_fmt or 'ya' in pix_fmt or '4444' in pix_fmt:
                    has_alpha = True

            return VideoInfo(
                duration=dur,
                width=w,
                height=h,
                codec=codec,
                audio_count=audio_count,
                audio_codec=audio_codec,
                audio_codecs=audio_codecs,
                decodable_audio_indices=decodable_audio_indices,
                is_prores=is_prores,
                has_alpha=has_alpha,
                fps=fps,
            )

        except subprocess.CalledProcessError as e:
            self.log_signal.emit(f"FFprobe Error: {e.output.decode().strip()}")
            return None
        except Exception as e:
            self.log_signal.emit(f"Probe Parse Exception: {str(e)}")
            return None

    def run_ffmpeg_process(self, cmd, pass_name=""):
        if self.is_cancelled: return False
        if pass_name: self.log_signal.emit(f"--- STARTING {pass_name} ---")

        self.encode_start_time = time.time()  # Track start time for ETA

        process = None
        try:
            # Replace 'ffmpeg' with custom path if provided
            if cmd[0] == "ffmpeg":
                cmd[0] = self.ffmpeg_path
            
            # Log the actual command being executed
            self.log_signal.emit(f"Command: {' '.join(cmd)}")
            
            process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
            with self._process_lock:
                self.process = process

            # Read output to EOF without ever stopping (prevents pipe deadlock).
            # Log lines are throttled for the UI, but the stream is always drained.
            reader = EncoderOutputReader(
                stream=process.stdout,
                video_duration=self.video_duration,
                encode_start_time=self.encode_start_time,
                log_cb=self.log_signal.emit,
                progress_cb=self.progress_signal.emit,
                eta_cb=self.eta_signal.emit,
                is_cancelled=lambda: self.is_cancelled,
            )
            reader.read_loop()

            if self.is_cancelled:
                try:
                    process.terminate()
                except OSError:
                    pass

            process.wait(timeout=3600)  # 1 hour timeout
            returncode = process.returncode
            with self._process_lock:
                self.process = None
            return returncode == 0 and not self.is_cancelled
        except subprocess.TimeoutExpired as e:
            if not self.is_cancelled:
                self.log_signal.emit(f"Process timeout after 1 hour: {e}")
            # Never leave an orphaned ffmpeg running
            if process is not None:
                try:
                    process.kill()
                    process.wait(timeout=5)
                except (subprocess.TimeoutExpired, OSError, Exception):
                    pass
            with self._process_lock:
                self.process = None
            return False
        except Exception as e:
            if not self.is_cancelled:
                self.log_signal.emit(f"Process Error: {e}")
            if process is not None and process.poll() is None:
                try:
                    process.kill()
                except (OSError, Exception):
                    pass
            with self._process_lock:
                self.process = None
            return False

    def run(self):
        try:
            self._run_unsafe()
        except Exception as e:
            self.log_signal.emit(f"Critical Worker Error: {e}")
            self.finished_signal.emit(False)

    def _run_unsafe(self):
        p = self.params
        input_file = p['input']
        
        # Validate input file exists and is accessible
        input_error = validate_input_file(input_file)
        if input_error:
            self.log_signal.emit(f"Error: {input_error}")
            self.finished_signal.emit(False)
            return
        
        mode = p['mode']
        copy_data = p.get('copy_data', True)
        container_mode = p.get('container', 'MP4')
        auto_scale = p.get('auto_scale', True)
        use_hw = p.get('use_hw', True)
        selected_codec = p['v_codec']

        video_info = self.get_video_info(input_file)
        if video_info is None:
            self.finished_signal.emit(False)
            return
        
        self.video_duration = video_info.duration if video_info.duration else 0.0

        if video_info.width == 0 or video_info.height == 0:
            self.log_signal.emit("Error: Could not determine video resolution.")
            self.finished_signal.emit(False)
            return

        self.log_signal.emit(f"=== Processing: {os.path.basename(input_file)} ({video_info.width}x{video_info.height} {video_info.codec}) ===")

        orig_w = video_info.width
        orig_h = video_info.height
        input_codec = video_info.codec
        
        # Validate hardware device availability if using HW encoding
        if use_hw:
            device = p.get('device', '/dev/dri/renderD128')
            available, error = self.validate_device_available(device)
            if not available:
                self.log_signal.emit(f"Error: {error}")
                self.finished_signal.emit(False)
                return
        audio_count = video_info.audio_count
        audio_codecs = video_info.audio_codecs
        decodable_audio_indices = video_info.decodable_audio_indices
        is_prores = video_info.is_prores
        duration = video_info.duration

        # --- Video Passthrough Check ---
        if p['v_codec'] == "Passthrough":
            video_compat = VIDEO_CODEC_CONTAINER_COMPAT.get(container_mode)
            if video_compat is not None and input_codec not in video_compat:
                self.compatibility_warning_signal.emit(f"Video codec '{input_codec}' is not compatible with {container_mode} container. Please select a different container or use a different video codec.")
                self.finished_signal.emit(False)
                return
            
            video_codec_cmd = "copy"
            v_tag = "_passthrough"
            is_av1 = False
            is_vp9 = False
            use_hw = False
        else:
            # --- Determine Codec Command ---
            is_av1 = "AV1" in selected_codec
            is_vp9 = "VP9" in selected_codec

            v_tag = "_enc"
            video_codec_cmd = "libx264"  # Default fallback

            # Hardware vs Software Codec Selection
            if is_av1:
                v_tag = "_av1"
                if use_hw:
                    video_codec_cmd = "av1_vaapi"
                else:
                    video_codec_cmd = "libsvtav1"
            elif "H.265" in selected_codec:
                v_tag = "_hevc"
                if use_hw:
                    video_codec_cmd = "hevc_vaapi"
                else:
                    video_codec_cmd = "libx265"
            elif "VP9" in selected_codec:
                v_tag = "_vp9"
                if use_hw:
                    video_codec_cmd = "vp9_vaapi"
                else:
                    video_codec_cmd = "libvpx-vp9"
            else: # H.264
                v_tag = "_h264"
                if use_hw:
                    video_codec_cmd = "h264_vaapi"
                else:
                    video_codec_cmd = "libx264"

        quality_preset = str(p.get('quality_preset', ''))
        if not quality_preset:
            self.log_signal.emit("Error: Quality preset not specified.")
            self.finished_signal.emit(False)
            return
        device = p.get('device', '/dev/dri/renderD128')

        # --- Audio & Container Logic ---
        a_codec = p.get('a_codec', 'AAC')
        if a_codec == "Passthrough":
            # Only copy audio if there are decodable audio streams
            if audio_count > 0:
                # Validate ALL audio codecs against container compatibility
                compat_list = AUDIO_CODEC_CONTAINER_COMPAT.get(container_mode)
                incompatible = []
                for i in decodable_audio_indices:
                    if i < len(audio_codecs):
                        stream_codec = audio_codecs[i]
                        if compat_list is not None and stream_codec not in compat_list:
                            incompatible.append((i, stream_codec))
                if incompatible:
                    details = ", ".join(f"stream {i}: {c}" for i, c in incompatible)
                    self.compatibility_warning_signal.emit(
                        f"Audio codec(s) not compatible with {container_mode} container: {details}. "
                        f"Please select a different container or use a different audio codec."
                    )
                    self.finished_signal.emit(False)
                    return
                audio_flags = ["-c:a", "copy"]
            else:
                audio_flags = []
            audio_bitrate = 0.0
        elif a_codec == "PCM":
            if container_mode in ("MP4", "WEBM"):
                self.compatibility_warning_signal.emit(
                    f"PCM audio is not supported in the {container_mode} container. "
                    f"Select MOV or MKV (or use a different audio codec)."
                )
                self.finished_signal.emit(False)
                return
            audio_flags = ["-c:a", "pcm_s16le", "-ar", "48000", "-ac", "2"]
            audio_bitrate = 1536.0
        else:
            # Select audio encoder based on codec preference and availability
            audio_encoders = p.get('audio_encoders', {})
            
            if a_codec == "Opus":
                # Use libopus if available, otherwise fallback to native opus
                if audio_encoders.get('opus', False):
                    enc = "libopus"
                    self.log_signal.emit("Audio Encoder: Using libopus (high quality)")
                else:
                    enc = "opus"  # Native opus
                    self.log_signal.emit("Audio Encoder: Using native opus")
            else:  # AAC
                # Use libfdk-aac if available, otherwise fallback to native aac
                if audio_encoders.get('aac', False):
                    enc = "libfdk_aac"
                    self.log_signal.emit("Audio Encoder: Using libfdk_aac (high quality)")
                else:
                    enc = "aac"  # Native aac
                    self.log_signal.emit("Audio Encoder: Using native aac")
            
            try:
                raw_bitrate = float(p.get('a_bitrate', 128))
                if raw_bitrate < 8: raw_bitrate = 128.0
            except (ValueError, TypeError):
                raw_bitrate = 128.0

            audio_bitrate = raw_bitrate
            audio_flags = ["-c:a", enc, "-b:a", f"{int(audio_bitrate)}k", "-ar", "48000", "-ac", "2"]
            if enc == "libopus":
                audio_flags.extend(["-sample_fmt", "flt"])

        # --- Resolve Output Container ---
        if container_mode == "MP4": ext = ".mp4"
        elif container_mode == "MKV": ext = ".mkv"
        elif container_mode == "MOV": ext = ".mov"
        elif container_mode == "WEBM": ext = ".webm"
        else:
            # Auto-select container based on both video and audio codecs for best compatibility
            audio_enc_name = enc if a_codec not in ("Passthrough", "PCM") else ""
            ext = select_output_container(video_codec_cmd, a_codec, audio_enc_name, is_vp9, is_av1)
            self.log_signal.emit(f"Auto container selected: {ext} (video: {video_codec_cmd}, audio: {a_codec})")

        # --- Output Path ---
        out_folder = p.get('output_folder', '')
        input_filename_no_ext = os.path.splitext(os.path.basename(input_file))[0]
        
        # Validate and sanitize output folder path
        if out_folder:
            # Normalize path and resolve any relative components
            out_folder = os.path.abspath(out_folder)
            # Check for path traversal attempts
            if not os.path.isdir(out_folder):
                self.log_signal.emit(f"Error: Invalid output directory: {out_folder}")
                self.finished_signal.emit(False)
                return
            # Check write permission
            if not os.access(out_folder, os.W_OK):
                self.log_signal.emit(f"Error: No write permission for output directory: {out_folder}")
                self.finished_signal.emit(False)
                return
            output_base = os.path.join(out_folder, input_filename_no_ext)
        else:
            output_base = os.path.splitext(input_file)[0]

        # --- Resolution Target ---
        res_map = {
            "2160p (4K)": 2160, "1440p": 1440, "1080p": 1080,
            "720p": 720, "540p": 540, "480p": 480, "360p": 360,
            "240p": 240, "144p": 144
        }
        res_str = p.get('res_choice', 'Original')
        target_h = res_map.get(res_str, orig_h if orig_h else 1080) if orig_h else 1080
        ar_str = p.get('ar_choice', 'Original')

        # --- Bitrate & Scaling Logic ---
        video_kbps = 0
        forced_av1 = False
        force_mitchell = False
        safety_margin = 0.98 if not use_hw else 0.95  # Software encoders track target size more tightly

        if mode == 'size':
            if duration <= 0:
                self.log_signal.emit("Error: Unknown duration, cannot calculate target size.")
                self.finished_signal.emit(False)
                return

            try:
                target_mb = float(p['size'])

                # Apply tighter margin for short AV1 videos to prevent overshoot
                # AV1 rate control needs many frames to converge; on short clips
                # it consistently overshoots. Prefer smaller files over overshooting.
                if is_av1 and duration < 30:
                    safety_margin = 0.85
                num_audio_streams = len(decodable_audio_indices) if a_codec != "Passthrough" else 0
                video_kbps = calc_size_mode_bitrate(target_mb, duration, audio_bitrate, num_audio_streams, safety_margin)
                video_kbps = self._clamp_bitrate(video_kbps)

                # === SCALING & AUTO-SWITCH ===
                if auto_scale and not is_av1:
                    if video_kbps < 200 and target_h > 240:
                        target_h = 240
                        self.scale_notification_signal.emit(f"240p")
                        force_mitchell = True
                        self.log_signal.emit(f"⚠ Auto-Scale: Forcing 240p (Bitrate {video_kbps}k)")
                        forced_av1 = True
                        quality_preset = "3"
                    elif video_kbps < 500 and target_h > 480:
                        target_h = 480
                        self.scale_notification_signal.emit(f"480p")
                        force_mitchell = True
                        self.log_signal.emit(f"⚠ Auto-Scale: Forcing 480p (Bitrate {video_kbps}k)")
                        forced_av1 = True
                        quality_preset = "6"

                if forced_av1:
                    is_av1 = True
                    video_codec_cmd = "libsvtav1"
                    use_hw = False
                    v_tag = "_av1"
                    forced_av1_margin = 0.85 if duration < 30 else 0.98
                    video_kbps = calc_size_mode_bitrate(target_mb, duration, audio_bitrate, num_audio_streams, forced_av1_margin)
                    video_kbps = self._clamp_bitrate(video_kbps)
                    self.log_signal.emit(f"⚠ Auto-Scale: Forcing 2-pass encoding for optimal quality at low bitrate (margin: {forced_av1_margin:.0%})")
                else:
                    if is_av1 and duration < 30:
                        self.log_signal.emit(f"Target: {target_mb}MB | Video: {video_kbps}k | Codec: {video_codec_cmd} | Short video: {safety_margin:.0%} margin")
                    else:
                        self.log_signal.emit(f"Target: {target_mb}MB | Video: {video_kbps}k | Codec: {video_codec_cmd}")

            except ValueError:
                self.finished_signal.emit(False)
                return
        else:
            crf_value = p.get('crf', 24)
            self.log_signal.emit(f"Mode: CQP {crf_value}")

        # Block explicitly-selected containers that cannot carry the chosen
        # video codec (the UI only shows a note; MKV/Auto impose no restrictions)
        if video_codec_cmd != "copy":
            compat = VIDEO_CODEC_CONTAINER_COMPAT.get(container_mode)
            if compat is not None and codec_key_for_encoder(video_codec_cmd) not in compat:
                self.compatibility_warning_signal.emit(
                    f"Video codec '{selected_codec}' is not compatible with the {container_mode} container. "
                    f"Please select a different container or video codec."
                )
                self.finished_signal.emit(False)
                return

        # --- Width Calculation ---
        target_w = target_h  # Initialize target_w with target_h as fallback
        if ar_str in ["Original", "Auto"]:
            if orig_h == 0:
                self.log_signal.emit("Error: Invalid original height for aspect ratio calculation.")
                self.finished_signal.emit(False)
                return
            if orig_w == 0:
                self.log_signal.emit("Error: Invalid original width for aspect ratio calculation.")
                self.finished_signal.emit(False)
                return
            ratio = orig_w / orig_h
            target_w = int(target_h * ratio)
        else:
            nums = ar_str.split(':')
            if len(nums) != 2:
                self.log_signal.emit(f"Error: Invalid aspect ratio format: {ar_str}")
                self.finished_signal.emit(False)
                return
            try:
                num_width = float(nums[0])
                num_height = float(nums[1])
                if num_height == 0:
                    self.log_signal.emit("Error: Aspect ratio denominator cannot be zero.")
                    self.finished_signal.emit(False)
                    return
                if num_width == 0:
                    self.log_signal.emit("Error: Aspect ratio numerator cannot be zero.")
                    self.finished_signal.emit(False)
                    return
                target_w = int(target_h * (num_width / num_height))
            except (ValueError, ZeroDivisionError) as e:
                self.log_signal.emit(f"Error: Invalid aspect ratio values: {e}")
                self.finished_signal.emit(False)
                return

        target_w = (target_w // 2) * 2
        target_h = (target_h // 2) * 2

        # --- Framerate Handling ---
        fps_choice = p.get('fps_choice', 'Original')
        fps_filter = ""
        if fps_choice != "Original":
            try:
                fps_val = float(fps_choice)
                fps_filter = f"fps={fps_val}"
            except ValueError:
                pass  # If invalid, use original

        aligned_w, aligned_h = align_resolution(target_w, target_h, 64, 16)
        pad_right = aligned_w - target_w
        pad_bottom = aligned_h - target_h

        output_file = f"{output_base}{v_tag}{ext}"
        self.set_output_file(output_file)
        
        # Check disk space before encoding
        estimated_size = self.estimate_output_size(p, video_info)
        has_space, available, required = self.check_disk_space(output_file, estimated_size)
        
        if not has_space:
            self.log_signal.emit(
                f"Error: Insufficient disk space. "
                f"Required: {required:.1f}MB, Available: {available:.1f}MB"
            )
            self.finished_signal.emit(False)
            return
        elif available < required * 1.5:
            self.log_signal.emit(
                f"Warning: Low disk space. Required: {required:.1f}MB, Available: {available:.1f}MB"
            )
        
        # Validate output directory exists and is writable (only for non-custom output folders)
        output_dir = os.path.dirname(output_file)
        if output_dir and not os.path.exists(output_dir):
            try:
                os.makedirs(output_dir, exist_ok=True)
            except OSError as e:
                self.log_signal.emit(f"Error: Cannot create output directory {output_dir}: {e}")
                self.finished_signal.emit(False)
                return
        
        if output_dir and not os.access(output_dir, os.W_OK):
            self.log_signal.emit(f"Error: No write permission for output directory: {output_dir}")
            self.finished_signal.emit(False)
            return

        # --- Build Pipeline ---
        base_cmd = ["ffmpeg", "-hide_banner", "-y"]
        map_flags = ["-map", "0:v:0"]
        for idx in decodable_audio_indices:
            map_flags.extend(["-map", f"0:a:{idx}"])
        meta_subs_flags = []
        is_mp4_container = (ext == ".mp4")

        if copy_data:
            if is_mp4_container:
                self.log_signal.emit("Notice: MP4 selected. Converting subtitles to text, skipping fonts/data.")
                map_flags.extend(["-map", "0:s?", "-map_metadata", "0"])
                meta_subs_flags = ["-c:s", "mov_text"]
            elif ext == ".webm":
                # WebM only supports WebVTT subtitles; convert instead of copying
                map_flags.extend(["-map", "0:s?", "-map_metadata", "0"])
                meta_subs_flags = ["-c:s", "webvtt"]
            else:
                map_flags.extend(["-map", "0:s?", "-map", "0:d?", "-map", "0:t?", "-map_metadata", "0"])
                meta_subs_flags = ["-c:s", "copy", "-c:d", "copy", "-c:t", "copy"]
        else:
            map_flags.extend(["-map_metadata", "-1"])

        algo = p.get('algo', 'Bicubic')

        # --- PIPELINE CONSTRUCTION (SW vs HW) ---
        pipeline_name = ""
        vf_kwargs = {}
        if video_codec_cmd == "copy":
            # Video passthrough - no scaling or encoding, no framerate changes
            self.log_signal.emit("Pipeline: Video Passthrough (No re-encoding)")
            base_cmd.extend(["-i", input_file])
            vf_chain = []
            video_flags = ["-c:v", "copy"]
            pipeline_name = "Video Passthrough (No re-encoding)"
        else:
            hw_decoder_caps = p.get('hw_decoder_caps', {})
            vulkan_available = p.get('vulkan_available', False)
            use_hw_decode = p.get('use_hw_decode', False)
            can_hw_decode = hw_decoder_caps.get(input_codec, False)
            use_vulkan_decode = is_prores and use_hw_decode and vulkan_available
            pix_fmt = "yuv420p10le" if (is_av1 and "av1_vaapi" in video_codec_cmd) else "nv12"

            vf_kwargs = dict(
                input_file=input_file,
                device=device,
                video_codec_cmd=video_codec_cmd,
                use_hw=use_hw,
                is_av1=is_av1,
                algo=algo,
                force_mitchell=force_mitchell,
                fps_filter=fps_filter,
                can_hw_decode=can_hw_decode,
                use_vulkan_decode=use_vulkan_decode,
                pix_fmt=pix_fmt,
                target_w=target_w,
                target_h=target_h,
                orig_w=orig_w,
                orig_h=orig_h,
                input_codec=input_codec,
                is_prores=is_prores,
                emit=self.log_signal.emit,
            )
            extras, vf_chain, pipeline_name = build_vf_chain(**vf_kwargs)
            base_cmd.extend(extras)
            self.log_signal.emit(f"Pipeline: {pipeline_name}")
            video_flags = build_video_flags(
                video_codec_cmd=video_codec_cmd,
                use_hw=use_hw,
                is_av1=is_av1,
                is_vp9=is_vp9,
                quality_preset=quality_preset,
                gpu_vendor=hw_decoder_caps.get('gpu_vendor', 'unknown'),
                mode=mode,
                video_kbps=video_kbps,
                crf=p.get('crf', 24),
                pad_right=pad_right,
                pad_bottom=pad_bottom,
                is_short_video=video_info.duration < 30,
                hw_encoder_opts=p.get('hw_encoder_opts', {}),
                fps=video_info.fps,
                emit=self.log_signal.emit,
            )

        if vf_chain:
            base_cmd.extend(["-vf", ",".join(vf_chain)])

        is_short_video = video_info.duration < 30

        # --- Execute ---
        # VP9 supports 2-pass encoding
        # Force 2-pass when auto downscale is triggered (forced_av1)
        # For short videos (<30s), use VBR with tight maxrate for better accuracy
        run_2pass = (mode == 'size' and not use_hw and video_codec_cmd != "copy" and not is_short_video) and (p.get('two_pass', False) or forced_av1)

        if run_2pass:
            temp_dir = tempfile.gettempdir()
            # Use a more specific filename with a unique identifier
            unique_id = str(uuid.uuid4())[:8]
            pass_log_base = os.path.join(temp_dir, f"ffmpeg_2pass_log_{unique_id}")

            cmd_p1 = base_cmd + ["-map", "0:v:0"]

            # First pass: AV1/VP9 can use faster presets, x264/x265 must match pass 2
            if is_av1:
                cmd_p1.extend(["-c:v", video_codec_cmd, "-b:v", f"{video_kbps}k", "-preset", "11", "-pix_fmt", "yuv420p10le"])
                if video_codec_cmd == "libsvtav1":
                    # Match pass 2's rate control so the 2-pass stats stay valid
                    cmd_p1.extend(["-g", str(svt_gop_size(video_info.fps)), "-svtav1-params", "gop-constraint-rc=1"])
            elif is_vp9:
                cmd_p1.extend(["-c:v", video_codec_cmd, "-b:v", f"{video_kbps}k", "-deadline", "realtime", "-cpu-used", "7", "-row-mt", "1"])
            elif "libx" in video_codec_cmd:
                # x264/x265: use video_flags (already has -c:v, -preset, -b:v, -maxrate, -bufsize)
                # Strip -bsf:v entries (bitstream filter not needed for null output pass 1)
                cmd_p1.extend(strip_bsf_flags(video_flags))

            cmd_p1.extend(["-pass", "1", "-passlogfile", pass_log_base, "-an", "-f", "null", os.devnull])

            if not self.run_ffmpeg_process(cmd_p1, "PASS 1"):
                self.finished_signal.emit(False)
                return

            # Second pass: use the actual selected preset (video_flags already contains it)
            cmd_p2 = base_cmd + map_flags + video_flags + ["-pass", "2", "-passlogfile", pass_log_base] + audio_flags + meta_subs_flags + [output_file]
            success = self.run_ffmpeg_process(cmd_p2, "PASS 2")

            try:
                # Use glob to find all pass log files (handles variations)
                log_pattern = f"{pass_log_base}*"
                
                for f in glob.glob(log_pattern):
                    try:
                        os.remove(f)
                    except (OSError, PermissionError) as e:
                        self.log_signal.emit(f"Warning: Could not remove {os.path.basename(f)}: {e}")
                
                # Also clean up any temporary files in the same directory
                temp_dir = os.path.dirname(pass_log_base)
                base_name = os.path.basename(pass_log_base)
                
                # FFmpeg sometimes creates files with different patterns
                extra_patterns = [
                    os.path.join(temp_dir, f"{base_name}*.log*"),
                    os.path.join(temp_dir, f"{base_name}*.mbtree*"),
                    os.path.join(temp_dir, f"ffmpeg2pass-{base_name}*")
                ]
                
                for pattern in extra_patterns:
                    for f in glob.glob(pattern):
                        try:
                            os.remove(f)
                        except (OSError, PermissionError):
                            pass  # Silently ignore cleanup failures for extra patterns
                                
            except Exception as e:
                self.log_signal.emit(f"Warning: Pass log cleanup error: {e}")
        else:
            cmd = base_cmd + map_flags + video_flags + audio_flags + meta_subs_flags + [output_file]
            success = self.run_ffmpeg_process(cmd, "ENCODE")

            # Some GPU/driver combinations cannot share VAAPI surfaces between
            # decoder and encoder (the universal/Vulkan pipelines fail at filter
            # negotiation). Retry once with CPU decode -> VAAPI upload, which is
            # the most compatible path.
            if not success and not self.is_cancelled and vf_kwargs:
                if pipeline_name in ("Universal VAAPI HW/SW Decode -> Encode",
                                     "ProRes Vulkan HW Decode -> VAAPI Encode"):
                    self.log_signal.emit("HW decode pipeline failed; retrying with CPU decode...")
                    retry_kwargs = dict(vf_kwargs)
                    retry_kwargs['force_cpu_path'] = True
                    retry_extras, retry_vf, retry_name = build_vf_chain(**retry_kwargs)
                    retry_base = ["ffmpeg", "-hide_banner", "-y"] + retry_extras
                    if retry_vf:
                        retry_base.extend(["-vf", ",".join(retry_vf)])
                    retry_cmd = retry_base + map_flags + video_flags + audio_flags + meta_subs_flags + [output_file]
                    self.log_signal.emit(f"Pipeline (retry): {retry_name}")
                    success = self.run_ffmpeg_process(retry_cmd, "ENCODE (CPU DECODE RETRY)")

        self.finished_signal.emit(success)

class SWEncoderChecker(QThread):
    """Thread-safe checker for software video encoders"""
    log_signal = Signal(str)
    finished_signal = Signal(dict)
    warning_signal = Signal(str, str)
    
    def __init__(self, ffmpeg_path):
        super().__init__()
        self.ffmpeg_path = ffmpeg_path if ffmpeg_path else 'ffmpeg'
        self.available_codecs = {'h264': False, 'hevc': False}
        self.log_messages = []
        self.prober_id = 'sw_encoders'
    
    def _probe_codec(self, codec_info):
        key, codec, display_name = codec_info
        cmd = [
            self.ffmpeg_path, "-y", "-hide_banner",
            "-f", "lavfi", "-i", "nullsrc=duration=1:size=320x240:rate=1",
            "-c:v", codec,
            "-f", "null", "-"
        ]
        try:
            result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=5)
            return key, display_name, result.returncode == 0, None
        except FileNotFoundError:
            return key, display_name, False, "ffmpeg_not_found"
        except subprocess.TimeoutExpired:
            return key, display_name, False, "timeout"
        except Exception as e:
            return key, display_name, False, str(e)
    
    def run(self):
        codecs_to_check = [
            ('h264', 'libx264', 'H.264'),
            ('hevc', 'libx265', 'H.265')
        ]
        
        self.log_messages.append("Checking software video encoders...")
        ffmpeg_not_found = False
        unavailable_codecs = []
        
        try:
            with ThreadPoolExecutor(max_workers=2) as executor:
                results = list(executor.map(self._probe_codec, codecs_to_check))
            
            for key, display_name, success, error in results:
                if error == "ffmpeg_not_found":
                    ffmpeg_not_found = True
                    break
                elif error == "timeout":
                    self.available_codecs[key] = False
                    self.log_messages.append(f"Error: Timeout checking {display_name}. Encoder may not be responsive.")
                    unavailable_codecs.append(display_name)
                elif success:
                    self.available_codecs[key] = True
                    self.log_messages.append(f"SW Encoder: {display_name} - Available")
                else:
                    self.available_codecs[key] = False
                    self.log_messages.append(f"SW Encoder: {display_name} - Not Available")
                    unavailable_codecs.append(display_name)
                    
        except Exception as e:
            self.log_messages.append(f"Error checking software encoders: {str(e)}")
        
        if ffmpeg_not_found:
            self.log_messages.append("Error: FFmpeg not found. Cannot verify software encoder capabilities.")
            self.warning_signal.emit("FFmpeg Not Found",
                                    "FFmpeg executable not found on your system!\n\n"
                                    "Please install FFmpeg to use this application.\n"
                                    "On Fedora: sudo dnf install ffmpeg\n"
                                    "On Ubuntu: sudo apt install ffmpeg")
            self.available_codecs = {'h264': True, 'hevc': True}
        elif unavailable_codecs:
            codec_list = ", ".join(unavailable_codecs)
            self.warning_signal.emit("Missing Codec Support",
                                    f"Your FFmpeg build does not support the following codecs:\n\n"
                                    f"{codec_list}\n\n"
                                    f"These codecs will be grayed out in the dropdown.\n"
                                    f"Consider installing a full FFmpeg build with all codecs enabled.")
        
        self.log_signal.emit('\n'.join(self.log_messages))
        self.finished_signal.emit(self.available_codecs)


class AudioEncoderChecker(QThread):
    """Thread-safe checker for audio encoders"""
    log_signal = Signal(str)
    finished_signal = Signal(dict)
    
    def __init__(self, ffmpeg_path):
        super().__init__()
        self.ffmpeg_path = ffmpeg_path if ffmpeg_path else 'ffmpeg'
        self.available_encoders = {'opus': False, 'aac': False}
        self.log_messages = []
        self.prober_id = 'audio_encoders'
    
    def run(self):
        self.log_messages.append("Checking audio encoder availability...")
        
        try:
            cmd = [self.ffmpeg_path, "-hide_banner", "-encoders"]
            result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=10)
            if result.returncode == 0:
                encoders_output = result.stdout.decode()
                if "libopus" in encoders_output:
                    self.available_encoders['opus'] = True
                    self.log_messages.append("Audio Encoder: libopus - Available")
                else:
                    self.available_encoders['opus'] = False
                    self.log_messages.append("Audio Encoder: libopus - Not Available")
                
                if "libfdk_aac" in encoders_output:
                    self.available_encoders['aac'] = True
                    self.log_messages.append("Audio Encoder: libfdk_aac - Available")
                else:
                    self.available_encoders['aac'] = False
                    self.log_messages.append("Audio Encoder: libfdk_aac - Not Available (will use native aac)")
            else:
                self.available_encoders['opus'] = False
                self.available_encoders['aac'] = False
                self.log_messages.append("Audio Encoder: Could not check encoders, assuming native implementations only")
        except FileNotFoundError:
            self.log_messages.append("Error: FFmpeg not found. Cannot verify audio encoder capabilities.")
            self.available_encoders['opus'] = False
            self.available_encoders['aac'] = False
        except subprocess.TimeoutExpired:
            self.log_messages.append("Error: Timeout checking audio encoders.")
            self.available_encoders['opus'] = False
            self.available_encoders['aac'] = False
        except Exception as e:
            self.log_messages.append(f"Error checking audio encoders: {str(e)}")
            self.available_encoders['opus'] = False
            self.available_encoders['aac'] = False
        
        self.log_messages.append("Audio Encoder: native opus and aac - Available (fallback)")
        
        # Emit all log messages as one consolidated message
        self.log_signal.emit('\n'.join(self.log_messages))
        self.finished_signal.emit(self.available_encoders)


class HWDeviceProber(QThread):
    """Thread-safe hardware device encoder capability prober"""
    log_signal = Signal(str)
    finished_signal = Signal(str, dict)
    warning_signal = Signal(str)
    
    def __init__(self, ffmpeg_path, device_path):
        super().__init__()
        self.ffmpeg_path = ffmpeg_path if ffmpeg_path else 'ffmpeg'
        self.device_path = device_path
        self.capabilities: dict = {'av1': False, 'h264': False, 'hevc': False, 'vp9': False, 'gpu_vendor': 'unknown', 'encoder_options': {}}
        self.log_messages = []
        self.prober_id = 'hw_encoders'
    
    def _probe_codec(self, codec_info):
        key, codec = codec_info
        cmd = [
            self.ffmpeg_path, "-y", "-hide_banner",
            "-init_hw_device", f"vaapi=dev:{self.device_path}",
            "-filter_hw_device", "dev",
            "-f", "lavfi", "-i", "nullsrc=duration=1:size=320x240:rate=1",
            "-vf", "format=nv12,hwupload",
            "-c:v", codec,
            "-f", "null", "-"
        ]
        try:
            result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=5)
            return key, result.returncode == 0, None
        except subprocess.TimeoutExpired:
            return key, False, "timeout"
        except FileNotFoundError:
            return key, False, "ffmpeg_not_found"
        except Exception as e:
            return key, False, str(e)
    
    def _query_encoder_options(self, codec_name: str) -> set:
        """Query which private options an encoder exposes (varies by build)."""
        try:
            result = subprocess.run(
                [self.ffmpeg_path, "-hide_banner", "-h", f"encoder={codec_name}"],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=5)
            if result.returncode == 0:
                return parse_encoder_options(result.stdout.decode(errors='replace'))
        except (subprocess.TimeoutExpired, FileNotFoundError, Exception):
            pass
        return set()
    
    def run(self):
        gpu_vendor = detect_gpu_vendor(self.device_path)
        self.log_messages.append(f"Hardware Device: {self.device_path} (GPU: {gpu_vendor.upper()})")
        
        codecs_to_check = [
            ('h264', 'h264_vaapi'),
            ('hevc', 'hevc_vaapi'),
            ('av1', 'av1_vaapi'),
            ('vp9', 'vp9_vaapi')
        ]
        
        ffmpeg_not_found = False
        device_error = None
        
        try:
            with ThreadPoolExecutor(max_workers=len(codecs_to_check)) as executor:
                results = list(executor.map(self._probe_codec, codecs_to_check))
            
            for key, success, error in results:
                if error == "ffmpeg_not_found":
                    ffmpeg_not_found = True
                    break
                elif error == "timeout":
                    self.capabilities[key] = False
                    self.log_messages.append(f"  -> {key.upper()} Encoder: Not Supported (Timeout)")
                elif success:
                    self.capabilities[key] = True
                    self.log_messages.append(f"  -> {key.upper()} Encoder: Supported")
                else:
                    self.capabilities[key] = False
                    self.log_messages.append(f"  -> {key.upper()} Encoder: Not Supported")
                    
        except Exception as e:
            self.log_messages.append(f"Error probing device: {str(e)}")
            self.warning_signal.emit(f"Could not verify capabilities for {self.device_path}. All codecs enabled (Use at your own risk).")
            self.capabilities = {'av1': True, 'h264': True, 'hevc': True, 'vp9': True, 'gpu_vendor': 'unknown', 'encoder_options': {}}
            self.log_signal.emit('\n'.join(self.log_messages))
            self.finished_signal.emit(self.device_path, self.capabilities)
            return
        
        if ffmpeg_not_found:
            self.log_messages.append("Error: FFmpeg not found. Cannot verify hardware capabilities.")
            self.warning_signal.emit("FFmpeg executable not found. All codec options enabled (Use at your own risk).")
            self.capabilities = {'av1': True, 'h264': True, 'hevc': True, 'vp9': True, 'gpu_vendor': 'unknown', 'encoder_options': {}}
        else:
            self.capabilities['gpu_vendor'] = gpu_vendor
            # Capture per-encoder option support (e.g. vp9_vaapi compression_level,
            # av1_vaapi preset) so the encoder flags can be gated at runtime
            encoder_options = {}
            for key, codec_name in codecs_to_check:
                if self.capabilities.get(key):
                    encoder_options[key] = self._query_encoder_options(codec_name)
            self.capabilities['encoder_options'] = encoder_options
        
        self.log_signal.emit('\n'.join(self.log_messages))
        self.finished_signal.emit(self.device_path, self.capabilities)


class HWDecoderChecker(QThread):
    """Thread-safe hardware device decoder capability prober.

    On Linux, VAAPI hardware decode happens through the standard
    software decoders accelerated via ``-hwaccel vaapi`` — there are
    no separate ``*_vaapi`` decoder entries in ``ffmpeg -decoders``.
    This checker parses ``ffmpeg -codecs`` output instead: if a
    codec family has a VAAPI **encoder** registered (e.g.
    ``h264_vaapi``), the GPU supports that codec and VAAPI-hwaccel
    decode will work for it.
    """
    log_signal = Signal(str)
    finished_signal = Signal(str, dict)
    warning_signal = Signal(str)

    VAAPI_ENCODER_TO_CODEC = {
        'h264_vaapi': 'h264',
        'hevc_vaapi': 'hevc',
        'vp8_vaapi': 'vp8',
        'vp9_vaapi': 'vp9',
        'av1_vaapi': 'av1',
        'mpeg2_vaapi': 'mpeg2video',
    }

    def __init__(self, ffmpeg_path, device_path):
        super().__init__()
        self.ffmpeg_path = ffmpeg_path if ffmpeg_path else 'ffmpeg'
        self.device_path = device_path
        self.decoder_caps = {'h264': False, 'hevc': False, 'vp8': False, 'vp9': False, 'av1': False, 'mpeg2video': False, 'gpu_vendor': 'unknown'}
        self.log_messages = []
        self.prober_id = 'hw_decoders'

    def run(self):
        gpu_vendor = detect_gpu_vendor(self.device_path)
        self.decoder_caps['gpu_vendor'] = gpu_vendor
        self.log_messages.append(f"  Hardware Device: {self.device_path} (GPU: {gpu_vendor.upper()})")
        self.log_messages.append(f"  Hardware decoder support (VAAPI):")

        try:
            cmd = [self.ffmpeg_path, "-hide_banner", "-codecs"]
            result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=10)

            if result.returncode != 0:
                self.log_messages.append("  Error: Could not query FFmpeg codecs")
                self.warning_signal.emit("Could not query FFmpeg codecs. All decode options enabled (Use at your own risk).")
                for key in self.decoder_caps:
                    if key != 'gpu_vendor':
                        self.decoder_caps[key] = True
                self.log_signal.emit('\n'.join(self.log_messages))
                self.finished_signal.emit(self.device_path, self.decoder_caps)
                return

            codecs_output = result.stdout.decode(errors='replace')
            supported_codecs = set()

            for line in codecs_output.split('\n'):
                for encoder_name, codec_key in self.VAAPI_ENCODER_TO_CODEC.items():
                    if encoder_name in line:
                        supported_codecs.add(codec_key)

            decoder_codecs = ['h264', 'hevc', 'vp8', 'vp9', 'av1', 'mpeg2video']
            for codec in decoder_codecs:
                if codec in supported_codecs:
                    self.decoder_caps[codec] = True
                    self.log_messages.append(f"  -> {codec.upper()} Decoder: Supported")
                else:
                    self.decoder_caps[codec] = False
                    self.log_messages.append(f"  -> {codec.upper()} Decoder: Not Supported")

        except FileNotFoundError:
            self.log_messages.append("Error: FFmpeg not found. Cannot verify hardware decoder capabilities.")
            self.warning_signal.emit("FFmpeg executable not found. All codec options enabled (Use at your own risk).")
            for key in self.decoder_caps:
                if key != 'gpu_vendor':
                    self.decoder_caps[key] = True
        except subprocess.TimeoutExpired:
            self.log_messages.append("Error: Timeout checking FFmpeg codecs.")
            self.warning_signal.emit("Could not verify decoder capabilities (timeout). All decode options enabled.")
            for key in self.decoder_caps:
                if key != 'gpu_vendor':
                    self.decoder_caps[key] = True
        except Exception as e:
            self.log_messages.append(f"Error probing decoder caps: {str(e)}")
            self.warning_signal.emit(f"Could not verify capabilities for {self.device_path}. All codecs enabled (Use at your own risk).")
            for key in self.decoder_caps:
                if key != 'gpu_vendor':
                    self.decoder_caps[key] = True

        self.log_signal.emit('\n'.join(self.log_messages))
        self.finished_signal.emit(self.device_path, self.decoder_caps)


class VulkanDeviceProber(QThread):
    """Thread-safe Vulkan device capability prober for ProRes hwaccel"""
    log_signal = Signal(str)
    finished_signal = Signal(bool, dict)
    warning_signal = Signal(str)
    
    def __init__(self, ffmpeg_path):
        super().__init__()
        self.ffmpeg_path = ffmpeg_path if ffmpeg_path else 'ffmpeg'
        self.vulkan_capabilities = {'vulkan_available': False, 'prores_vulkan': False}
        self.log_messages = []
        self.prober_id = 'vulkan'
    
    def run(self):
        try:
            # First check if Vulkan hwaccel is available at all
            cmd = [
                self.ffmpeg_path, "-hide_banner",
                "-init_hw_device", "vulkan",
                "-f", "lavfi", "-i", "nullsrc=duration=0.1:size=320x240:rate=1",
                "-frames:v", "1",
                "-f", "null", "-"
            ]
            
            result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=10)
            
            if result.returncode == 0:
                self.vulkan_capabilities['vulkan_available'] = True
                
                # Check for prores_vulkan decoder (requires patched FFmpeg)
                decoders_cmd = [self.ffmpeg_path, "-decoders"]
                decoders_result = subprocess.run(decoders_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=5)
                
                if decoders_result.returncode == 0:
                    decoders_output = decoders_result.stdout.decode()
                    # Look for prores_vulkan decoder
                    if 'prores_vulkan' in decoders_output:
                        self.vulkan_capabilities['prores_vulkan'] = True
                        self.log_messages.append("  -> ProRes Vulkan decode: Supported")
                    else:
                        self.log_messages.append("  -> ProRes Vulkan decode: Not available")
            else:
                self.vulkan_capabilities['vulkan_available'] = False
                self.vulkan_capabilities['prores_vulkan'] = False
        
        except FileNotFoundError:
            self.vulkan_capabilities['vulkan_available'] = False
            self.vulkan_capabilities['prores_vulkan'] = False
            self.warning_signal.emit("FFmpeg executable not found.")
        except subprocess.TimeoutExpired:
            self.vulkan_capabilities['vulkan_available'] = False
            self.vulkan_capabilities['prores_vulkan'] = False
        except Exception:
            self.vulkan_capabilities['vulkan_available'] = False
            self.vulkan_capabilities['prores_vulkan'] = False
        
        self.log_signal.emit('\n'.join(self.log_messages) if self.log_messages else "")
        self.finished_signal.emit(
            self.vulkan_capabilities.get('prores_vulkan', False),
            self.vulkan_capabilities
        )


class NotificationManager:
    """Generate pleasant tones without any external files"""
    
    def __init__(self):
        # Check for pre-installed tools
        self.has_notify_send = self._check_command('notify-send')
        self.has_paplay = self._check_command('paplay')
        self.has_aplay = self._check_command('aplay')
    
    def _check_command(self, command):
        """Check if a command is available in PATH"""
        try:
            return subprocess.call(['which', command],
                                 stdout=subprocess.DEVNULL,
                                 stderr=subprocess.DEVNULL) == 0
        except:
            return False
    
    def _generate_tone(self, frequency, duration, volume=0.5):
        """Generate a sine wave tone as PCM data"""
        sample_rate = 44100
        samples = []
        
        # Generate sine wave with fade in/out for smooth sound
        fade_duration = 0.02  # 20ms fade
        fade_samples = int(fade_duration * sample_rate)
        total_samples = int(duration * sample_rate)
        
        for i in range(total_samples):
            t = i / sample_rate
            # Sine wave formula
            raw_value = math.sin(2 * math.pi * frequency * t)
            
            # Apply fade in/out to avoid clicking
            if i < fade_samples:
                # Fade in
                factor = i / fade_samples
            elif i > total_samples - fade_samples:
                # Fade out
                factor = (total_samples - i) / fade_samples
            else:
                factor = 1.0
            
            value = int(32767 * volume * factor * raw_value)
            samples.append(value)
        
        # Convert to bytes (little-endian 16-bit)
        return array.array('h', samples).tobytes()
    
    def _play_pcm(self, audio_data: bytes) -> bool:
        """Play raw signed 16-bit little-endian mono 44.1kHz PCM audio."""
        if self.has_paplay:
            proc = subprocess.Popen([
                'paplay', '--raw', '--format=s16le',
                '--channels=1', '--rate=44100'
            ], stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        elif self.has_aplay:
            proc = subprocess.Popen([
                'aplay', '-q', '-f', 'cd', '-c', '1'
            ], stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        else:
            return False
        try:
            proc.communicate(input=audio_data)
            return True
        except Exception:
            return False

    def _play_chime(self):
        """Play a pleasant two-tone chime"""
        try:
            # Generate two tones: A5 (880Hz) then C#6 (1108.73Hz)
            tone1 = self._generate_tone(880, 0.1, 0.25)    # A5
            tone2 = self._generate_tone(1108.73, 0.15, 0.25)  # C#6
            return self._play_pcm(tone1 + tone2)
        except Exception:
            return False

    def _play_failure_chime(self):
        try:
            tone1 = self._generate_tone(1108.73, 0.1, 0.25)
            tone2 = self._generate_tone(1108.73, 0.1, 0.25)
            tone3 = self._generate_tone(440, 0.2, 0.25)
            return self._play_pcm(tone1 + tone2 + tone3)
        except Exception:
            return False
    
    def notify_failure(self):
        if self._play_failure_chime():
            return
        print('\a', end='', flush=True)
    
    def notify_completion(self, message="Encoding completed successfully", show_popup=True, play_sound=True):
        """Play pleasant chime and show notification"""
        success = False
        
        # Method 1: Desktop notification (if enabled)
        if show_popup and self.has_notify_send:
            try:
                subprocess.run([
                    'notify-send',
                    '-i', 'video-x-generic',
                    '-t', '3000',
                    'FFMPEG Encoder',
                    message
                ], check=True, timeout=2, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                success = True
            except (subprocess.CalledProcessError, subprocess.TimeoutExpired):
                pass
        
        # Method 2: Play generated chime (if enabled)
        if play_sound:
            if self._play_chime():
                success = True
            # Method 3: Terminal bell (last resort, if sound enabled)
            if not success:
                print('\a', end='', flush=True)
        
        return success

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Linux FFMPEG VAAPI Wrapper")
        self.setFixedSize(472, 660)
        self.setAcceptDrops(True)
        self.batch_params = {}
        self.sleep_inhibitor = None
        self._queue_paths = set()

        self.device_capabilities = {}
        self.warning_shown = False
        self.available_sw_codecs = {'h264': False, 'hevc': False}
        self.available_audio_encoders = {'opus': False, 'aac': False}
        self.hw_decoder_capabilities = {}
        self.notification_manager = NotificationManager()
        self._queue_paths_lock = threading.Lock()
        
        self._probe_lock = threading.Lock()
        
        self.vulkan_available = False
        self.vulkan_capabilities = {}
        self.vulkan_prober = None
        
        self.sw_encoder_checker = None
        self.audio_encoder_checker = None
        self.hw_device_prober = None
        self.hw_decoder_checker = None
        self.hw_encoder_check_complete = False
        self._probing_active = True
        self._initial_probing_done = False
        self._probe_generation = 0  # Incremented on ffmpeg path change; stale prober results are dropped
        self._failed_count = 0
        
        self.probing_coordinator = ProbingCoordinator()

        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        layout.setContentsMargins(2, 2, 2, 2)
        layout.setSpacing(3)

        # Set font sizes
        default_font = QApplication.font()
        default_font.setPointSizeF(default_font.pointSizeF() - 1)
        central.setFont(default_font)

        # --- QUEUE ---
        queue_container = QWidget()
        queue_container.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        queue_layout = QHBoxLayout(queue_container)
        queue_layout.setContentsMargins(2, 2, 2, 2)

        self.queue_list = QListWidget()
        self.queue_list.setSelectionMode(QAbstractItemView.ExtendedSelection)
        self.queue_list.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        # Reduce queue list font size by 2 points
        queue_font = QApplication.font()
        queue_font.setPointSizeF(queue_font.pointSizeF() - 2)
        self.queue_list.setFont(queue_font)

        # Create widget to hold list and placeholder
        queue_list_widget = QWidget()
        queue_list_layout = QVBoxLayout(queue_list_widget)
        queue_list_layout.setContentsMargins(0, 0, 0, 0)
        queue_list_layout.setSpacing(0)
        queue_list_layout.addWidget(self.queue_list)

        # Placeholder label (overlay on top of list)
        self.queue_placeholder = QLabel("Drag files here...")
        placeholder_font = QApplication.font()
        placeholder_font.setPointSizeF(placeholder_font.pointSizeF() + 1)
        self.queue_placeholder.setFont(placeholder_font)
        self.queue_placeholder.setStyleSheet("color: #999; font-style: italic;")
        self.queue_placeholder.setAlignment(Qt.AlignCenter)
        self.queue_placeholder.setParent(self.queue_list)
        self.queue_placeholder.raise_()

        h_btn_q = QVBoxLayout()
        self.btn_add_queue = QPushButton("Add File(s)")
        self.btn_add_queue.setStyleSheet("background-color: #1976D2; color: white; font-weight: bold;")
        self.btn_add_queue.clicked.connect(self.add_files_dialog)

        self.btn_rem_queue = QPushButton("Remove Selected")
        self.btn_rem_queue.clicked.connect(self.remove_from_queue)

        self.btn_clear_queue = QPushButton("Clear Queue")
        self.btn_clear_queue.clicked.connect(self.clear_queue)

        h_btn_q.addWidget(self.btn_add_queue)
        h_btn_q.addWidget(self.btn_rem_queue)
        h_btn_q.addWidget(self.btn_clear_queue)
        h_btn_q.addStretch()

        queue_layout.addWidget(queue_list_widget)
        queue_layout.addLayout(h_btn_q)
        layout.addWidget(queue_container)

        # --- VIDEO SETTINGS ---
        vid_grp = QGroupBox("Video Settings")
        vid_grp.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)
        vid_layout = QFormLayout(vid_grp)
        vid_layout.setLabelAlignment(Qt.AlignLeft)
        vid_layout.setContentsMargins(2, 2, 2, 2)

        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["Target Size", "Quality", "Passthrough"])
        self.mode_combo.currentTextChanged.connect(self.toggle_mode)
        vid_layout.addRow("Encoding Mode:", self.mode_combo)

        self.target_label = QLabel("Target Size:")

        self.size_widget = QWidget()
        size_layout = QHBoxLayout(self.size_widget)
        size_layout.setContentsMargins(0,0,0,0)
        self.target_size = QLineEdit("20")
        self.target_size.editingFinished.connect(self.validate_target_size)
        size_layout.addWidget(self.target_size)
        size_layout.addWidget(QLabel("MB"))

        self.chk_2pass = QCheckBox("2-Pass")
        self.chk_2pass.toggled.connect(self.on_2pass_toggled)
        size_layout.addWidget(self.chk_2pass)

        size_layout.addStretch()

        self.quality_widget = QWidget()
        q_layout = QHBoxLayout(self.quality_widget)
        q_layout.setContentsMargins(0,0,0,0)
        self.q_spin = QSpinBox()
        self.q_spin.setRange(0, 51)
        self.q_spin.setValue(24)
        q_layout.addWidget(QLabel("(0 = Lossless):"))
        q_layout.addWidget(self.q_spin)
        q_layout.addStretch()

        self.mode_stack = QStackedWidget()
        self.mode_stack.addWidget(self.size_widget)
        self.mode_stack.addWidget(self.quality_widget)

        vid_layout.addRow(self.target_label, self.mode_stack)

        c_layout = QHBoxLayout()
        self.v_codec = QComboBox()
        self.v_codec.addItems(["H.264", "H.265", "AV1"])
        self.v_codec.currentTextChanged.connect(self.update_codec_ui)

        self.chk_hw = QCheckBox("HW")
        self.chk_hw.setChecked(True)
        self.chk_hw.setToolTip("Use VAAPI Hardware Acceleration if available")
        self.chk_hw.toggled.connect(self.on_hw_toggled)

        c_layout.addWidget(self.v_codec)
        c_layout.addWidget(self.chk_hw)
        vid_layout.addRow("Video Codec:", c_layout)

        self.quality_combo = QComboBox()
        # Removed setMinimumWidth(160) - will be synced later
        vid_layout.addRow("Speed Preset:", self.quality_combo)

        # --- VAAPI DEVICE DETECTION ---
        self.device_combo = QComboBox()
        self.device_combo.setMinimumWidth(150)
        vid_layout.addRow("VAAPI Device:", self.device_combo)

        self.detect_devices()

        # --- OUTPUT SETTINGS ---
        out_layout = QHBoxLayout()
        out_layout.setContentsMargins(2, 2, 2, 2)

        self.container_combo = QComboBox()
        self.container_combo.addItems(["MP4", "WEBM", "MKV", "MOV", "Auto"])
        self.container_combo.setToolTip("MP4 recommended for compatibility. WEBM for web. MKV for maximum codec support.")

        self.output_path = QLineEdit()
        self.output_path.setPlaceholderText("Default (Same as Source)")
        output_font = QApplication.font()
        output_font.setPointSizeF(output_font.pointSizeF() - 2)
        self.output_path.setFont(output_font)
        self.btn_out_browse = QPushButton("Browse")
        self.btn_out_browse.clicked.connect(self.browse_output_folder)

        out_layout.addWidget(QLabel("Format:"))
        out_layout.addWidget(self.container_combo)
        out_layout.addWidget(QLabel("Destination:"))
        out_layout.addWidget(self.output_path)
        out_layout.addWidget(self.btn_out_browse)
        layout.addLayout(out_layout)

        # --- CREATE TWO-COLUMN LAYOUT ---
        columns_layout = QHBoxLayout()
        
        # --- LEFT COLUMN: VIDEO SETTINGS ---
        left_column = QVBoxLayout()
        left_column.addWidget(vid_grp)

        # --- MISC SETTINGS ---
        misc_grp = QGroupBox("Miscellaneous")
        misc_grp.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)
        misc_layout = QVBoxLayout(misc_grp)
        misc_layout.setContentsMargins(2, 2, 2, 2)

        # FFmpeg path selector
        ffmpeg_layout = QHBoxLayout()
        ffmpeg_layout.setContentsMargins(0, 0, 0, 0)
        ffmpeg_layout.addWidget(QLabel("FFmpeg:"))
        
        self.ffmpeg_path = QLineEdit()
        self.ffmpeg_path.setPlaceholderText("System default")
        ffmpeg_path_font = QApplication.font()
        ffmpeg_path_font.setPointSizeF(ffmpeg_path_font.pointSizeF() - 2)
        self.ffmpeg_path.setFont(ffmpeg_path_font)
        self.ffmpeg_path.setToolTip("Leave empty to use system default FFmpeg")
        self.ffmpeg_path.editingFinished.connect(self.on_ffmpeg_path_changed)
        
        self.ffmpeg_browse_btn = QPushButton("Browse")
        self.ffmpeg_browse_btn.clicked.connect(self.browse_ffmpeg)
        
        ffmpeg_layout.addWidget(self.ffmpeg_path)
        ffmpeg_layout.addWidget(self.ffmpeg_browse_btn)
        misc_layout.addLayout(ffmpeg_layout)

        self.chk_copy_data = QCheckBox("Copy Subtitles and Metadata")
        self.chk_copy_data.setChecked(True)
        misc_layout.addWidget(self.chk_copy_data)

        left_column.addWidget(misc_grp)

        # --- RIGHT COLUMN: SCALING AND AUDIO ---
        right_column = QVBoxLayout()

        # --- SCALER ---
        scale_grp = QGroupBox("Resolution and Framerate")
        scale_grp.setSizePolicy(QSizePolicy.Minimum, QSizePolicy.Fixed)
        scale_layout = QFormLayout(scale_grp)
        scale_layout.setLabelAlignment(Qt.AlignLeft)
        scale_layout.setContentsMargins(2, 2, 2, 2)

        self.res_combo = QComboBox()
        self.res_options = ["1080p", "Original", "2160p (4K)", "1440p", "720p", "540p", "480p", "360p", "240p", "144p"]
        self.res_combo.addItems(self.res_options)
        self.res_combo.setCurrentText("720p")
        scale_layout.addRow("Resolution:", self.res_combo)

        self.ar_combo = QComboBox()
        self.ar_combo.addItems(["16:9", "Original", "Auto", "4:3", "1:1", "9:16", "21:9"])
        self.ar_combo.setCurrentText("Original") # Default to Original
        scale_layout.addRow("Aspect Ratio:", self.ar_combo)

        # Create widget to hold framerate dropdown and custom input
        fps_widget = QWidget()
        fps_layout = QHBoxLayout(fps_widget)
        fps_layout.setContentsMargins(0, 0, 0, 0)

        self.fps_combo = QComboBox()
        self.fps_combo.addItems(["Original", "23.976", "24", "25", "29.97", "30", "48", "50", "59.94", "60", "Custom"])
        self.fps_combo.setCurrentText("30")

        self.fps_custom_input = QLineEdit("30")
        self.fps_custom_input.setMaximumWidth(35)
        self.fps_custom_input.hide()
        self.fps_custom_input.editingFinished.connect(lambda: self.validate_custom_fps())

        fps_layout.addWidget(self.fps_combo)
        fps_layout.addWidget(self.fps_custom_input)
        fps_layout.addStretch()

        scale_layout.addRow("Framerate:", fps_widget)

        # Show/hide custom input based on framerate selection
        self.fps_combo.currentTextChanged.connect(self.toggle_custom_fps)

        self.algo_combo = QComboBox()
        self.algo_combo.addItems(["VAAPI (HW)", "Bicubic", "Bilinear", "Nearest", "Lanczos HQ", "Mitchell HQ"])
        self.algo_combo.setCurrentIndex(0)
        scale_layout.addRow("Scaling:", self.algo_combo)

        self.chk_auto_scale = QCheckBox("Auto Downscale")
        self.chk_auto_scale.setChecked(False) # Unchecked by default
        self.chk_auto_scale.setToolTip("Automatically lowers resolution and switches to SW AV1 if bitrate is very low (HW encoders struggle with low bitrates).")
        scale_layout.addRow("", self.chk_auto_scale)

        right_column.addWidget(scale_grp)

        # --- AUDIO ---
        aud_grp = QGroupBox("Audio Settings")
        aud_grp.setSizePolicy(QSizePolicy.Minimum, QSizePolicy.Fixed)
        aud_layout = QFormLayout(aud_grp)
        aud_layout.setContentsMargins(2, 2, 2, 2)

        self.a_codec = QComboBox()
        self.a_codec.addItems(["Opus", "AAC", "PCM", "Passthrough"])
        self.a_codec.currentTextChanged.connect(self.toggle_audio)
        aud_layout.addRow("Audio Codec:", self.a_codec)

        # Create container for bitrate input and label
        self.bitrate_widget = QWidget()
        bitrate_layout = QHBoxLayout(self.bitrate_widget)
        bitrate_layout.setContentsMargins(0, 0, 0, 0)
        
        self.a_bitrate = QLineEdit("128")
        self.a_bitrate.editingFinished.connect(self.validate_audio_bitrate)
        bitrate_layout.addWidget(self.a_bitrate)
        bitrate_layout.addWidget(QLabel("kbps"))
        
        aud_layout.addRow("Audio Bitrate:", self.bitrate_widget)
        right_column.addWidget(aud_grp)
        
        right_column.addStretch()
        
        # Add columns to split layout
        columns_layout.addLayout(left_column)
        columns_layout.addLayout(right_column)
        
        layout.addLayout(columns_layout)

        # --- LOG ---
        log_font = QFont("monospace")
        log_font.setPointSizeF(QApplication.font().pointSizeF() - 3)
        self.log = QTextEdit()
        self.log.setReadOnly(True)
        self.log.setStyleSheet("background-color: #222; color: #eee; font-family: monospace;")
        self.log.setFont(log_font)
        self.log.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        layout.addWidget(self.log)

        # --- PROGRESS BAR ---
        progress_container = QWidget()
        progress_layout = QHBoxLayout(progress_container)
        progress_layout.setContentsMargins(2, 2, 2, 2)
        
        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(0)
        self.progress_bar.setTextVisible(True)
        self.progress_bar.setStyleSheet("""
            QProgressBar {
                border: 2px solid grey;
                border-radius: 5px;
                text-align: center;
                background-color: #333;
                color: white;
            }
            QProgressBar::chunk {
                background-color: #4CAF50;
                border-radius: 3px;
            }
        """)
        
        self.time_label = QLabel("ETA: --:--:--")
        self.time_label.setStyleSheet("color: white; font-family: monospace; font-weight: bold;")
        self.time_label.setMinimumWidth(90)
        self.time_label.setCursor(Qt.PointingHandCursor)
        self.time_label.setToolTip("Click to toggle ETA/Elapsed time")
        self._show_eta = True  # Default to showing ETA
        self.time_label.mousePressEvent = self._toggle_time_display
        
        progress_layout.addWidget(self.progress_bar)
        progress_layout.addWidget(self.time_label)
        layout.addWidget(progress_container)
        
        # --- BUTTONS ---
        btn_row = QHBoxLayout()
        self.btn_start = QPushButton("Start Queue")
        self.btn_start.clicked.connect(self.start_queue)
        self.btn_start.setStyleSheet("background-color: #2e7d32; color: white; font-weight: bold; padding: 10px;")

        self.chk_no_popup = QCheckBox("No Popup")
        self.chk_no_popup.setChecked(True)
        self.chk_no_sound = QCheckBox("No Sound")
        
        self.chk_hw_decode = QCheckBox("HW Decode")
        self.chk_hw_decode.setChecked(True)
        self.chk_hw_decode.setToolTip("Use Vulkan hardware decoding for ProRes inputs")

        self.btn_cancel = QPushButton("Cancel")
        self.btn_cancel.clicked.connect(self.cancel)
        self.btn_cancel.setStyleSheet("background-color: #c62828; color: white; font-weight: bold; padding: 10px;")
        self.btn_cancel.setEnabled(False)

        btn_row.addWidget(self.btn_start)
        btn_row.addWidget(self.chk_no_popup)
        btn_row.addWidget(self.chk_no_sound)
        btn_row.addWidget(self.chk_hw_decode)
        btn_row.addStretch()
        btn_row.addWidget(self.btn_cancel)
        layout.addLayout(btn_row)

        # Connect signals AFTER widgets are created
        self.device_combo.currentIndexChanged.connect(self.on_device_changed)
        
        # Connect validation signals for container/codec compatibility
        self.container_combo.currentTextChanged.connect(self.validate_container_codec)
        self.v_codec.currentTextChanged.connect(self.validate_container_codec)
        self.a_codec.currentTextChanged.connect(self.validate_container_codec)

        # Initialize UI State with defaults
        self.update_codec_ui(self.v_codec.currentText())

        # Apply preferred defaults
        self.v_codec.setCurrentText("AV1")
        self.chk_2pass.setChecked(True)
        self.update_codec_ui(self.v_codec.currentText())

        # Sync widget sizes based on other widgets
        # Set Target Size widget width to fit input (4 digits), MB label, and 2-Pass checkbox
        self.size_widget.setFixedWidth(150)
        # Set Speed Preset to fit "Balanced (Default)" and match Encoding Mode to it
        self.quality_combo.setMinimumWidth(140)
        self.mode_combo.setMinimumWidth(140)
        # Match Audio Bitrate widget width to Audio Codec width
        self.bitrate_widget.setFixedWidth(self.a_codec.sizeHint().width())
        self.res_combo.setFixedWidth(self.algo_combo.sizeHint().width())
        self.ar_combo.setFixedWidth(self.algo_combo.sizeHint().width())

        self._set_ui_enabled(False)
        
        QTimer.singleShot(100, self.check_sw_encoders)
        QTimer.singleShot(150, self.check_audio_encoders)
        QTimer.singleShot(200, self.initial_probe)
        QTimer.singleShot(0, self.update_queue_placeholder)
        
        self.probe_timeout_timer = None
        self.probing_coordinator.set_timeout_callback(self._on_probe_timeout)
        
        self.worker = None
    
    def _on_probe_timeout(self, elapsed: float):
        """Handle probing timeout"""
        self.log.append(f"Warning: Probing timed out after {elapsed:.1f}s. Some capabilities may be incomplete.")
        if self.probe_timeout_timer:
            self.probe_timeout_timer.stop()
        # Unblock the UI even if a prober thread is stuck
        self._probing_active = False
        self._set_ui_enabled(True)
        if not self._initial_probing_done:
            self._initial_probing_done = True
            self.log.append("System probing complete (with timeouts).")
    
    def _start_probe_timeout_timer(self):
        """Start (or restart) the single timeout timer for probing"""
        if self.probe_timeout_timer is None:
            self.probe_timeout_timer = QTimer()
            self.probe_timeout_timer.timeout.connect(self._check_probe_timeout)
        self.probe_timeout_timer.start(1000)  # Check every second
    
    def _check_probe_timeout(self):
        """Check if probing has timed out"""
        if self.probing_coordinator.check_timeout():
            if self.probe_timeout_timer:
                self.probe_timeout_timer.stop()
            self._check_probing_complete()

    # --- CHECKBOX HANDLERS ---
    def on_hw_toggled(self, checked):
        if checked:
            self.chk_2pass.setEnabled(False)
            self.chk_2pass.setChecked(False)
            # Enable GPU scaling option and set it as default
            gpu_idx = self.algo_combo.findText("VAAPI (HW)")
            if gpu_idx >= 0:
                self.algo_combo.model().item(gpu_idx).setEnabled(True)
            self.algo_combo.setCurrentText("VAAPI (HW)")
        else:
            # Switching to software encoding
            if self.mode_combo.currentText() == "Target Size":
                self.chk_2pass.setEnabled(True)
                self.chk_2pass.setChecked(True)  # Enable and check 2-pass by default for SW
            else:
                self.chk_2pass.setEnabled(False)
                self.chk_2pass.setChecked(False)
            # Disable GPU scaling option and switch to Bicubic
            gpu_idx = self.algo_combo.findText("VAAPI (HW)")
            if gpu_idx >= 0:
                self.algo_combo.model().item(gpu_idx).setEnabled(False)
            self.algo_combo.setCurrentText("Bicubic")

        self.update_preset_options()

    def on_2pass_toggled(self, checked):
        if checked:
            self.chk_hw.setEnabled(False)
            self.chk_hw.setChecked(False)
        else:
            self.chk_hw.setEnabled(True)

        self.update_preset_options()

    def toggle_mode(self, text):
        if text == "Target Size":
            self.mode_stack.setCurrentIndex(0)
            self.target_label.setText("Target Size (MB):")
            # If HW is off, enable 2-pass. If HW is on, keep 2-pass off.
            if not self.chk_hw.isChecked():
                self.chk_2pass.setEnabled(True)
                # Default to checking it if we switch to size mode and are in SW
                self.chk_2pass.setChecked(True)
            else:
                self.chk_2pass.setEnabled(False)
                self.chk_2pass.setChecked(False)
            # Enable video-related options
            self.v_codec.setEnabled(True)
            self.chk_hw.setEnabled(True)
            self.quality_combo.setEnabled(True)
            self.chk_copy_data.setEnabled(True)
            self.res_combo.setEnabled(True)
            self.ar_combo.setEnabled(True)
            self.algo_combo.setEnabled(True)
            self.chk_auto_scale.setEnabled(True)
            self.mode_stack.setEnabled(True)
        elif text == "Quality":
            self.mode_stack.setCurrentIndex(1)
            self.target_label.setText("Target Quality:")
            self.chk_2pass.setEnabled(False)
            self.chk_2pass.setChecked(False)
            # Enable video-related options
            self.v_codec.setEnabled(True)
            self.chk_hw.setEnabled(True)
            self.quality_combo.setEnabled(True)
            self.chk_copy_data.setEnabled(True)
            self.res_combo.setEnabled(True)
            self.ar_combo.setEnabled(True)
            self.algo_combo.setEnabled(True)
            self.chk_auto_scale.setEnabled(True)
            self.mode_stack.setEnabled(True)
        elif text == "Passthrough":
            # Disable all video-related options
            self.v_codec.setEnabled(False)
            self.chk_hw.setEnabled(False)
            self.chk_2pass.setEnabled(False)
            self.quality_combo.setEnabled(False)
            self.chk_copy_data.setEnabled(True)  # Keep copy data enabled
            self.res_combo.setEnabled(False)
            self.ar_combo.setEnabled(False)
            self.fps_combo.setEnabled(False)
            self.fps_custom_input.setEnabled(False)
            self.algo_combo.setEnabled(False)
            self.chk_auto_scale.setEnabled(False)
            self.mode_stack.setEnabled(False)

    def validate_container_codec(self):
        """Validate container and codec compatibility in real-time"""
        if self.mode_combo.currentText() == "Passthrough":
            # Passthrough validation handled separately
            return
        
        container = self.container_combo.currentText()
        if container == "Auto":
            return  # Auto will pick compatible container
        
        v_codec_key = codec_key_from_ui(self.v_codec.currentText())
        
        compatible_codecs = VIDEO_CODEC_CONTAINER_COMPAT.get(container, set())
        
        if compatible_codecs and v_codec_key not in compatible_codecs:
            # Show inline warning in log
            self.log.append(f"Note: {v_codec_key.upper()} may not be optimal for {container}")
            # Set tooltip on container combo
            self.container_combo.setToolTip(
                f"Note: {v_codec_key.upper()} is not supported in {container}\n"
                f"Compatible codecs: {', '.join(sorted(compatible_codecs))}"
            )
        else:
            self.container_combo.setToolTip("")
        
        # Validate audio codec too
        a_codec = self.a_codec.currentText()
        if a_codec not in ["Passthrough", "PCM"]:
            a_codec_key = a_codec.lower()
            a_compatible = AUDIO_CODEC_CONTAINER_COMPAT.get(container, set())
            
            if a_compatible and a_codec_key not in a_compatible:
                if container == "WEBM":
                    self.log.append(f"Note: {a_codec} audio is not supported in WEBM (only Opus/Vorbis); encoding will fail if unchanged.")
                else:
                    self.log.append(f"Note: {a_codec} audio may not be supported in {container}")

    # --- SOFTWARE ENCODER DETECTION ---
    
    def on_sw_encoders_found(self, available_codecs, generation=None):
        """Handle completion of software encoder checking"""
        if generation is not None and generation != self._probe_generation:
            return  # stale result from a previous ffmpeg path
        self.available_sw_codecs = available_codecs
        self.update_codec_options()
        with self._probe_lock:
            self.sw_encoder_checker = None
        self._check_probing_complete()
    
    def on_audio_encoders_found(self, available_encoders, generation=None):
        """Handle completion of audio encoder checking"""
        if generation is not None and generation != self._probe_generation:
            return
        self.available_audio_encoders = available_encoders
        with self._probe_lock:
            self.audio_encoder_checker = None
        self._check_probing_complete()
    
    def on_device_encoders_found(self, device_path, capabilities, generation=None):
        """Handle completion of hardware encoder probing"""
        if generation is not None and generation != self._probe_generation:
            return
        self.device_capabilities[device_path] = capabilities
        with self._probe_lock:
            self.hw_device_prober = None
        self.hw_encoder_check_complete = True
        self._check_probing_complete()
        self._refresh_codec_ui_if_probing_done()
    
    def on_device_decoders_found(self, device_path, decoder_caps, generation=None):
        """Handle completion of hardware decoder probing"""
        if generation is not None and generation != self._probe_generation:
            return
        self.hw_decoder_capabilities[device_path] = decoder_caps
        with self._probe_lock:
            self.hw_decoder_checker = None
        self._check_probing_complete()
        self._refresh_codec_ui_if_probing_done()

    def _refresh_codec_ui_if_probing_done(self):
        """Refresh codec UI once both HW encoder and decoder probing complete."""
        if self.hw_encoder_check_complete and self.hw_decoder_checker is None:
            self.update_codec_ui(self.v_codec.currentText())
    
    def on_vulkan_found(self, has_vulkan, capabilities, generation=None):
        """Handle completion of Vulkan hwaccel probing"""
        if generation is not None and generation != self._probe_generation:
            return
        self.vulkan_available = has_vulkan
        self.vulkan_capabilities = capabilities
        with self._probe_lock:
            self.vulkan_prober = None
        self._check_probing_complete()
        
        if has_vulkan:
            self.log.append("ProRes Vulkan hardware decode available")
            self.chk_hw_decode.setToolTip("Use Vulkan hardware decoding for ProRes inputs")
        else:
            self.chk_hw_decode.setToolTip("Vulkan ProRes decode not available - will use CPU decode for ProRes")
    
    def _set_ui_enabled(self, enabled, encoding=False):
        """Enable or disable UI during probing/encoding.

        When encoding=True, the cancel button is enabled instead of the
        queue controls, and the system sleep lock is released on re-enable.
        """
        self.btn_start.setEnabled(enabled)
        self.btn_add_queue.setEnabled(enabled)
        self.btn_rem_queue.setEnabled(enabled)
        self.btn_clear_queue.setEnabled(enabled)
        self.queue_list.setEnabled(enabled)
        self.btn_cancel.setEnabled(encoding and not enabled)
        self.chk_hw_decode.setEnabled(enabled)
        self.mode_combo.setEnabled(enabled)
        self.v_codec.setEnabled(enabled)
        self.chk_hw.setEnabled(enabled)
        self.chk_2pass.setEnabled(enabled)
        self.quality_combo.setEnabled(enabled)
        self.container_combo.setEnabled(enabled)
        self.output_path.setEnabled(enabled)
        self.btn_out_browse.setEnabled(enabled)
        self.ffmpeg_path.setEnabled(enabled)
        self.ffmpeg_browse_btn.setEnabled(enabled)
        self.chk_copy_data.setEnabled(enabled)
        self.res_combo.setEnabled(enabled)
        self.ar_combo.setEnabled(enabled)
        self.fps_combo.setEnabled(enabled)
        self.fps_custom_input.setEnabled(enabled)
        self.algo_combo.setEnabled(enabled)
        self.chk_auto_scale.setEnabled(enabled)
        self.a_codec.setEnabled(enabled)
        self.a_bitrate.setEnabled(enabled)
        self.target_size.setEnabled(enabled)
        self.q_spin.setEnabled(enabled)
        if hasattr(self, 'device_combo'):
            self.device_combo.setEnabled(enabled and len(glob.glob("/dev/dri/renderD*")) > 1)
        
        if encoding and enabled:
            self.release_sleep()
    
    def _check_probing_complete(self):
        """Check if all probing is complete and re-enable UI"""
        if not self._probing_active:
            return
        
        with self._probe_lock:
            sw_done = self.sw_encoder_checker is None
            audio_done = self.audio_encoder_checker is None
            hw_enc_done = self.hw_device_prober is None
            hw_dec_done = self.hw_decoder_checker is None
            vulkan_done = self.vulkan_prober is None or not hasattr(self, '_vulkan_probed')
        
        if sw_done and audio_done and hw_enc_done and hw_dec_done and vulkan_done:
            self._probing_active = False
            if self.probe_timeout_timer:
                self.probe_timeout_timer.stop()
            self._set_ui_enabled(True)
            if not self._initial_probing_done:
                self._initial_probing_done = True
                self.log.append("System probing complete.")
    
    def on_encoder_warning(self, title, message):
        """Show warning message from encoder checker"""
        QMessageBox.warning(self, title, message)
    
    def on_device_warning(self, message):
        """Show warning message from device prober"""
        if not self.warning_shown:
            QMessageBox.warning(self, "Capability Detection Warning",
                              f"{message}\n\nThe application will attempt to proceed, but encoding may fail if the hardware does not support the selected codec.")
            self.warning_shown = True
    
    def on_encoder_log(self, message):
        """Handle log message from prober threads directly (immediate output)"""
        for line in message.split('\n'):
            if line.strip():
                self.log.append(line)
    
    def _flush_coordinated_logs(self, ordered_logs):
        """Flush logs from coordinator in order"""
        for logs in ordered_logs:
            for line in logs.split('\n'):
                if line.strip():
                    self.log.append(line)
    
    def _on_prober_log(self, prober_id, message):
        """Handle log message from prober threads via coordinator for ordered output"""
        self.probing_coordinator.submit_logs(prober_id, message, self._flush_coordinated_logs)
    
    def _current_ffmpeg_cmd(self) -> str:
        """Return the current FFmpeg command path, or 'ffmpeg' if unset."""
        ffmpeg_path = self.ffmpeg_path.text().strip() if hasattr(self, 'ffmpeg_path') else ''
        return ffmpeg_path if ffmpeg_path else 'ffmpeg'

    def check_sw_encoders(self):
        """Check which software encoders are available in the FFmpeg build (threaded)"""
        ffmpeg_cmd = self._current_ffmpeg_cmd()
        gen = self._probe_generation
        
        self.probing_coordinator.reset(['sw_encoders'])
        self._start_probe_timeout_timer()
        
        self.sw_encoder_checker = SWEncoderChecker(ffmpeg_cmd)
        self.sw_encoder_checker.log_signal.connect(lambda msg: self._on_prober_log('sw_encoders', msg))
        self.sw_encoder_checker.finished_signal.connect(lambda caps, g=gen: self.on_sw_encoders_found(caps, g))
        self.sw_encoder_checker.warning_signal.connect(self.on_encoder_warning)
        self.sw_encoder_checker.start()

    def check_audio_encoders(self):
        """Check which audio encoders are available in the FFmpeg build (threaded)"""
        ffmpeg_cmd = self._current_ffmpeg_cmd()
        gen = self._probe_generation
        
        self.probing_coordinator.reset(['audio_encoders'])
        self._start_probe_timeout_timer()
        
        self.audio_encoder_checker = AudioEncoderChecker(ffmpeg_cmd)
        self.audio_encoder_checker.log_signal.connect(lambda msg: self._on_prober_log('audio_encoders', msg))
        self.audio_encoder_checker.finished_signal.connect(lambda caps, g=gen: self.on_audio_encoders_found(caps, g))
        self.audio_encoder_checker.start()

    def update_codec_options(self):
        """Update the video codec dropdown based on available encoders"""
        self.v_codec.blockSignals(True)
        current_text = self.v_codec.currentText()
        self.v_codec.clear()

        # Always add AV1 (libsvtav1 is usually available)
        self.v_codec.addItem("AV1")

        # Add H.265 (may be grayed out if not available)
        self.v_codec.addItem("H.265")
        if not self.available_sw_codecs.get('hevc', False):
            hevc_idx = self.v_codec.findText("H.265")
            if hevc_idx >= 0:
                self.v_codec.model().item(hevc_idx).setEnabled(False)

        # Add H.264 (may be grayed out if not available)
        self.v_codec.addItem("H.264")
        if not self.available_sw_codecs.get('h264', False):
            h264_idx = self.v_codec.findText("H.264")
            if h264_idx >= 0:
                self.v_codec.model().item(h264_idx).setEnabled(False)

        # Add VP9 - always available in FFmpeg
        self.v_codec.addItem("VP9")

        # Restore selection if possible
        if self.v_codec.findText(current_text) >= 0 and self.v_codec.findText(current_text) < self.v_codec.count():
            selected_idx = self.v_codec.findText(current_text)
            if self.v_codec.model().item(selected_idx).isEnabled():
                self.v_codec.setCurrentText(current_text)
            else:
                # Select first available codec
                for i in range(self.v_codec.count()):
                    if self.v_codec.model().item(i).isEnabled():
                        self.v_codec.setCurrentIndex(i)
                        break
        else:
            # Select first available codec
            for i in range(self.v_codec.count()):
                if self.v_codec.model().item(i).isEnabled():
                    self.v_codec.setCurrentIndex(i)
                    break

        self.v_codec.blockSignals(False)

    # --- DEVICE DETECTION LOGIC ---
    def detect_devices(self):
        self.device_combo.blockSignals(True)
        self.device_combo.clear()
        devices = glob.glob("/dev/dri/renderD*")

        if not devices:
            devices = ["/dev/dri/renderD128"]
            self.log.append("Warning: No render devices found in /dev/dri/, using default fallback.")

        devices.sort()
        self.device_combo.addItems(devices)

        if len(devices) <= 1:
            self.device_combo.setEnabled(False)
        else:
            self.device_combo.setEnabled(True)

        self.device_combo.blockSignals(False)

    def initial_probe(self):
        if self.device_combo.count() > 0:
            self.probe_device(self.device_combo.currentText())

    def on_device_changed(self, index):
        if index < 0: return
        device = self.device_combo.currentText()
        if device:
            self.probe_device(device)

    def probe_device(self, device_path):
        """Probe device capabilities (threaded)"""
        if device_path in self.device_capabilities:
            self.update_codec_ui(self.v_codec.currentText())
            return

        ffmpeg_cmd = self._current_ffmpeg_cmd()
        gen = self._probe_generation

        prober_ids = ['hw_encoders', 'hw_decoders']
        if not hasattr(self, '_vulkan_probed'):
            prober_ids.append('vulkan')
            self._vulkan_probed = True
        self.probing_coordinator.reset(prober_ids)

        self.hw_device_prober = HWDeviceProber(ffmpeg_cmd, device_path)
        self.hw_device_prober.log_signal.connect(lambda msg: self._on_prober_log('hw_encoders', msg))
        self.hw_device_prober.finished_signal.connect(lambda dev, caps, g=gen: self.on_device_encoders_found(dev, caps, g))
        self.hw_device_prober.warning_signal.connect(self.on_device_warning)
        self.hw_device_prober.start()
        
        self.hw_decoder_checker = HWDecoderChecker(ffmpeg_cmd, device_path)
        self.hw_decoder_checker.log_signal.connect(lambda msg: self._on_prober_log('hw_decoders', msg))
        self.hw_decoder_checker.finished_signal.connect(lambda dev, caps, g=gen: self.on_device_decoders_found(dev, caps, g))
        self.hw_decoder_checker.warning_signal.connect(self.on_device_warning)
        self.hw_decoder_checker.start()
        
        if 'vulkan' in prober_ids:
            self.vulkan_prober = VulkanDeviceProber(ffmpeg_cmd)
            self.vulkan_prober.log_signal.connect(lambda msg: self._on_prober_log('vulkan', msg))
            self.vulkan_prober.finished_signal.connect(lambda hv, caps, g=gen: self.on_vulkan_found(hv, caps, g))
            self.vulkan_prober.start()

    # --- UI UPDATERS ---
    def update_codec_ui(self, codec_text):
        is_av1 = "AV1" in codec_text
        is_h264 = "H.264" in codec_text
        is_hevc = "H.265" in codec_text
        is_vp9 = "VP9" in codec_text

        use_hw = self.chk_hw.isChecked()
        current_device = self.device_combo.currentText()

        caps = self.device_capabilities.get(current_device, {'av1': False, 'h264': False, 'hevc': False, 'vp9': False})

        hw_supported = False
        if is_av1:
            hw_supported = caps['av1']
        elif is_h264:
            hw_supported = caps['h264']
        elif is_hevc:
            hw_supported = caps['hevc']
        elif is_vp9:
            hw_supported = caps.get('vp9', False)

        self.chk_hw.blockSignals(True)
        self.chk_2pass.blockSignals(True)

        is_size_mode = self.mode_combo.currentText() == "Target Size"

        # Default Logic:
        # 1. If HW is supported, Check HW, Uncheck 2-Pass.
        # 2. If HW is NOT supported, Uncheck HW, Enable & Check 2-Pass (if in Size Mode).

        if hw_supported:
            self.chk_hw.setEnabled(True)
            self.chk_hw.setChecked(True) # Check HW by default

            # If HW is active, disable 2-pass
            self.chk_2pass.setEnabled(False)
            self.chk_2pass.setChecked(False)
            # Enable GPU scaling option and set it as default
            gpu_idx = self.algo_combo.findText("VAAPI (HW)")
            if gpu_idx >= 0:
                self.algo_combo.model().item(gpu_idx).setEnabled(True)
            self.algo_combo.setCurrentText("VAAPI (HW)")
        else:
            # HW not supported
            self.chk_hw.setEnabled(False)
            self.chk_hw.setChecked(False)

            # We are in SW mode.
            # If Size mode, default 2-pass ON.
            if is_size_mode:
                self.chk_2pass.setEnabled(True)
                self.chk_2pass.setChecked(True) # Check 2-pass by default for SW
            else:
                self.chk_2pass.setEnabled(False)
                self.chk_2pass.setChecked(False)
            # Disable GPU scaling option and switch to Bicubic
            gpu_idx = self.algo_combo.findText("VAAPI (HW)")
            if gpu_idx >= 0:
                self.algo_combo.model().item(gpu_idx).setEnabled(False)
            self.algo_combo.setCurrentText("Bicubic")

        self.chk_hw.blockSignals(False)
        self.chk_2pass.blockSignals(False)

        self.update_preset_options()
    
    def validate_custom_fps(self):
        fps_text = self.fps_custom_input.text().strip()
        if not fps_text:
            self.fps_custom_input.setText("30")
            return
        try:
            value = float(fps_text)
            if value <= 0 or value > 240:  # Reasonable FPS range
                self.fps_custom_input.setText("30")
        except ValueError:
            self.fps_custom_input.setText("30")

    def update_preset_options(self):
        codec = self.v_codec.currentText()
        use_hw = self.chk_hw.isChecked()
        is_av1 = "AV1" in codec
        is_vp9 = "VP9" in codec
        
        # Check GPU vendor for AMD-specific VCN compression levels
        current_device = self.device_combo.currentText()
        hw_caps = self.hw_decoder_capabilities.get(current_device, {})
        gpu_vendor = hw_caps.get('gpu_vendor', 'unknown')
        is_amd = gpu_vendor == 'amd'

        self.quality_combo.clear()

        if is_av1 and not use_hw:
            for i in range(14):
                desc = str(i)
                if i==0: desc += " (Slowest)"
                if i==7: desc += " (Default)"
                if i==8: desc += " (Recommended)"
                if i==12: desc += " (Fast)"
                if i==13: desc += " (Realtime)"
                self.quality_combo.addItem(desc, i)
            self.quality_combo.setCurrentIndex(7)
        elif is_av1 and use_hw:
            # av1_vaapi -preset (where supported) mirrors the SVT-AV1 0-13 scale
            for i in range(14):
                desc = str(i)
                if i==0: desc += " (Slowest)"
                if i==7: desc += " (Default)"
                if i==8: desc += " (Recommended)"
                if i==12: desc += " (Fast)"
                if i==13: desc += " (Realtime)"
                self.quality_combo.addItem(desc, i)
            self.quality_combo.setCurrentIndex(7)
        elif is_vp9 and not use_hw:
            # VP9 speed presets
            # Best: best deadline, cpu-used 0 (slowest, best quality)
            # High: best deadline, cpu-used 1
            # Good: good deadline, cpu-used 0
            # Balanced: good deadline, cpu-used 1
            # Default: good deadline, cpu-used 2
            # Fast: good deadline, cpu-used 3
            presets = [
                ("Best", 0),
                ("High", 1),
                ("Good", 2),
                ("Balanced", 3),
                ("Default", 4),
                ("Fast", 5)
            ]
            for name, value in presets:
                desc = name
                if value == 0: desc += " (Slowest)"
                if value == 4: desc += " (Default)"
                if value == 5: desc += " (Fastest)"
                self.quality_combo.addItem(desc, value)
            self.quality_combo.setCurrentIndex(4)
        elif is_vp9 and use_hw:
            # VP9 VAAPI uses compression_level: higher = faster
            presets = [
                ("Quality (Slowest)", 0),
                ("Balanced (Default)", 2),
                ("Speed", 4),
                ("Max Speed", 7)
            ]
            for name, value in presets:
                self.quality_combo.addItem(name, value)
            self.quality_combo.setCurrentIndex(1)
            self.quality_combo.setEnabled(True)
        elif use_hw:
            # VCN compression levels are AMD-specific
            # For non-AMD GPUs, disable speed preset options
            if is_amd:
                self.quality_combo.addItem("Quality (Best)", 1)
                self.quality_combo.addItem("Balanced (Default)", 2)
                self.quality_combo.addItem("Speed (Fast)", 4)
                self.quality_combo.addItem("Max Speed", 7)
                self.quality_combo.setCurrentIndex(0)
                self.quality_combo.setEnabled(True)
            else:
                # Non-AMD GPU: VCN compression levels not supported
                # Use balanced default, disable the combo
                self.quality_combo.addItem("Balanced (Default)", 2)
                self.quality_combo.setCurrentIndex(0)
                self.quality_combo.setEnabled(False)
        else:
            presets = [
                ("veryslow", 0), ("slower", 1), ("slow", 2), ("medium", 3),
                ("fast", 4), ("faster", 5), ("veryfast", 6), ("superfast", 7), ("ultrafast", 8)
            ]
            for p_name, p_idx in presets:
                self.quality_combo.addItem(f"{p_name.capitalize()}", p_idx)
                if p_name == "medium":
                    self.quality_combo.setCurrentIndex(self.quality_combo.count() - 1)

    # --- SLEEP INHIBITION ---
    def inhibit_sleep(self):
        try:
            cmd = ["systemd-inhibit", "--what=idle:sleep", "--who=VAAPI-Transcoder", "--why=Encoding", "--mode=block", "sleep", "infinity"]
            self.sleep_inhibitor = subprocess.Popen(cmd)
            self.log.append("System sleep inhibited.")
        except Exception as e:
            self.log.append(f"Warning: Could not inhibit system sleep ({e})")

    def release_sleep(self):
        if self.sleep_inhibitor:
            try:
                self.sleep_inhibitor.terminate()
                self.sleep_inhibitor.wait(timeout=5)
                self.log.append("System sleep lock released.")
            except (subprocess.TimeoutExpired, OSError, Exception) as e:
                self.log.append(f"Warning: Error releasing sleep lock: {e}")
            self.sleep_inhibitor = None

    def toggle_audio(self, txt):
        if txt in ["PCM", "Passthrough"]:
            # Remember the last numeric bitrate before switching to N/A
            text = self.a_bitrate.text().strip()
            try:
                if float(text) > 0:
                    self._last_audio_bitrate = text
            except ValueError:
                pass
            self.a_bitrate.setEnabled(False)
            self.a_bitrate.setText("N/A")
        else:
            self.a_bitrate.setEnabled(True)
            if self.a_bitrate.text() == "N/A":
                self.a_bitrate.setText(getattr(self, '_last_audio_bitrate', "128"))
        
        # Auto-switch container based on audio codec
        if txt == "PCM":
            if self.container_combo.currentText() != "MOV":
                QMessageBox.warning(self, "Container Switch", "Warning: Switching to MOV for PCM!")
            self.container_combo.setCurrentText("MOV")

    def validate_target_size(self):
        text = self.target_size.text().strip()
        if not text:
            self.target_size.setText("10")
            return
        try:
            value = float(text)
            if value < 1:
                self.target_size.setText("10")
            elif value > 100000:  # Reasonable upper limit (100GB)
                self.target_size.setText("100000")
        except ValueError:
            self.target_size.setText("10")

    def validate_audio_bitrate(self):
        text = self.a_bitrate.text().strip()
        if not text:
            self.a_bitrate.setText("128")
            return
        try:
            value = float(text)
            if value < 8:
                self.a_bitrate.setText("8")
            elif value > 1536:  # Reasonable upper limit (1536 kbps)
                self.a_bitrate.setText("1536")
        except ValueError:
            self.a_bitrate.setText("128")

    def toggle_custom_fps(self, text):
        if text == "Custom":
            self.fps_custom_input.show()
        else:
            self.fps_custom_input.hide()
            # Validate custom FPS value when hidden
            fps_text = self.fps_custom_input.text().strip()
            if fps_text:
                try:
                    value = float(fps_text)
                    if value <= 0 or value > 240:  # Reasonable FPS range
                        self.fps_custom_input.setText("30")
                except ValueError:
                    self.fps_custom_input.setText("30")

    def show_scale_notification(self, res_text):
        if not self.chk_no_sound.isChecked():
            QMessageBox.information(self, "Auto-Scale", f"Bitrate too low.\nResolution set to {res_text}.")
    
    def show_compatibility_warning(self, message):
        QMessageBox.warning(self, "Compatibility Error", message)
    
    def update_progress(self, value):
        """Update the progress bar with the current value (0-100)"""
        self.progress_bar.setValue(value)

    def dragEnterEvent(self, e):
        if e.mimeData().hasUrls(): e.accept()
        else: e.ignore()

    def dropEvent(self, e):
        urls = e.mimeData().urls()
        files = [u.toLocalFile() for u in urls]
        for f in files: self.add_path_to_queue(f)

    def browse_output_folder(self):
        folder = QFileDialog.getExistingDirectory(self, "Select Output Folder")
        if folder: self.output_path.setText(folder)

    def browse_ffmpeg(self):
        file, _ = QFileDialog.getOpenFileName(self, "Select FFmpeg Binary", "", "FFmpeg (ffmpeg)")
        if file and self.validate_ffmpeg_path(file):
            self.ffmpeg_path.setText(file)
            self.on_ffmpeg_path_changed()

    def validate_ffmpeg_path(self, path):
        """Validate that the FFmpeg path exists and is executable"""
        if not path:
            return True  # Empty path means use system default
        
        # Normalize the path
        path = os.path.abspath(path)
        
        # Check if path exists
        if not os.path.exists(path):
            QMessageBox.critical(self, "Invalid FFmpeg Path",
                                f"FFmpeg executable not found at:\n{path}")
            return False
        
        # Check if it's a file
        if not os.path.isfile(path):
            QMessageBox.critical(self, "Invalid FFmpeg Path",
                                f"Path is not a file:\n{path}")
            return False
        
        # Check if it's executable
        if not os.access(path, os.X_OK):
            QMessageBox.critical(self, "Invalid FFmpeg Path",
                                f"File is not executable:\n{path}")
            return False
        
        return True

    def on_ffmpeg_path_changed(self):
        """Handle FFmpeg path changes - re-check encoders and device capabilities"""
        ffmpeg_path = self.ffmpeg_path.text().strip()
        
        # Validate the path before proceeding
        if ffmpeg_path and not self.validate_ffmpeg_path(ffmpeg_path):
            self.ffmpeg_path.setText("")  # Clear invalid path
            return
        
        self.log.append("FFmpeg path changed. Re-detecting capabilities...")
        # Bump the generation counter: results from any still-running prober
        # threads are dropped, so we don't block the UI waiting on them
        self._probe_generation += 1
        self._probing_active = True
        self.hw_encoder_check_complete = False
        self.available_sw_codecs = {'h264': False, 'hevc': False}
        self.available_audio_encoders = {'opus': False, 'aac': False}
        self.device_capabilities = {}
        self.hw_decoder_capabilities = {}
        self.vulkan_available = False
        self.vulkan_capabilities = {}
        self._vulkan_probed = False  # Reset Vulkan probe flag
        self.update_codec_options()
        
        # Start new checks (stale prober results are ignored via generation)
        self.check_sw_encoders()
        self.check_audio_encoders()
        if self.device_combo.count() > 0:
            self.probe_device(self.device_combo.currentText())

    def add_files_dialog(self):
        files, _ = QFileDialog.getOpenFileNames(self, "Select Videos", "", "Videos (*.mp4 *.mkv *.mov *.avi *.webm)")
        if files:
            for f in files: self.add_path_to_queue(f)

    def add_path_to_queue(self, fpath):
        # Reserve the path atomically to prevent duplicate adds from concurrent calls
        with self._queue_paths_lock:
            if fpath in self._queue_paths:
                return
            self._queue_paths.add(fpath)
        
        input_error = validate_input_file(fpath)
        if input_error:
            self.log.append(f"Error: {input_error}")
            with self._queue_paths_lock:
                self._queue_paths.discard(fpath)
            return
        
        item = QListWidgetItem(os.path.basename(fpath))
        item.setData(Qt.UserRole, fpath)
        self.queue_list.addItem(item)
        self.log.append(f"Added: {os.path.basename(fpath)}")
        self.update_queue_placeholder()

    def remove_from_queue(self):
        with self._queue_paths_lock:
            for item in self.queue_list.selectedItems():
                fpath = item.data(Qt.UserRole)
                self.queue_list.takeItem(self.queue_list.row(item))
                self._queue_paths.discard(fpath)
        self.update_queue_placeholder()

    def clear_queue(self):
        with self._queue_paths_lock:
            self.queue_list.clear()
            self._queue_paths.clear()
        self.update_queue_placeholder()

    def update_queue_placeholder(self):
        """Toggle placeholder visibility based on queue contents."""
        if self.queue_list.count() == 0:
            self.queue_placeholder.setGeometry(0, 0, self.queue_list.width(), self.queue_list.height())
            self.queue_placeholder.show()
        else:
            self.queue_placeholder.hide()

    def start_queue(self):
        if self.queue_list.count() == 0:
            QMessageBox.information(self, "Queue", "Queue is empty.")
            return

        self._total_jobs = self.queue_list.count()
        self._failed_count = 0

        # Validate and reset empty values to defaults
        self.validate_target_size()
        self.validate_audio_bitrate()
        
        # Validate custom FPS if selected
        if self.fps_combo.currentText() == "Custom":
            self.validate_custom_fps()

        # Capture audio bitrate before the encoding UI is disabled
        abit = "0"
        if self.a_bitrate.isEnabled(): abit = self.a_bitrate.text()

        self._set_ui_enabled(False, encoding=True)
        self.inhibit_sleep()

        mode_txt = self.mode_combo.currentText()
        mode_code = 'size' if "Size" in mode_txt else 'quality'
        
        # Handle Passthrough mode
        if mode_txt == "Passthrough":
            v_codec = "Passthrough"
        else:
            v_codec = self.v_codec.currentText()
            # Check if selected codec is available
            codec_key = codec_key_from_ui(v_codec)
            # VP9 is always available, no check needed
            
            if v_codec != "VP9" and not self.available_sw_codecs.get(codec_key, True):
                QMessageBox.critical(self, "Codec Not Available",
                                    f"The selected codec '{v_codec}' is not available in your FFmpeg build.\n\n"
                                    f"Please select a different codec or install a full FFmpeg build.")
                self.reset_ui()
                return

        container_code = self.container_combo.currentText().split()[0] # Auto/MKV/MP4

        # Get framerate choice - use custom input value if "Custom" is selected
        fps_choice = self.fps_combo.currentText()
        if fps_choice == "Custom":
            fps_text = self.fps_custom_input.text().strip()
            try:
                fps_value = float(fps_text)
                if fps_value <= 0 or fps_value > 240:
                    fps_value = 30.0
                    self.fps_custom_input.setText("30")
                fps_choice = str(fps_value)
            except ValueError:
                fps_choice = "30"
                self.fps_custom_input.setText("30")

        # Get FFmpeg and FFprobe paths
        ffmpeg_path = self.ffmpeg_path.text().strip()
        if ffmpeg_path:
            # Validate FFmpeg path before proceeding
            if not self.validate_ffmpeg_path(ffmpeg_path):
                self.reset_ui()
                return
            
            # Derive ffprobe path from ffmpeg path (same directory)
            ffmpeg_dir = os.path.dirname(ffmpeg_path)
            ffprobe_path = os.path.join(ffmpeg_dir, 'ffprobe') if ffmpeg_dir else 'ffprobe'
        else:
            ffmpeg_path = 'ffmpeg'
            ffprobe_path = 'ffprobe'
        
        current_device = self.device_combo.currentText()
        qp_data = self.quality_combo.currentData()
        self.batch_params = {
            'mode': mode_code,
            'size': self.target_size.text(),
            'crf': self.q_spin.value(),
            'v_codec': v_codec,
            'use_hw': self.chk_hw.isChecked(),
            'use_hw_decode': self.chk_hw_decode.isChecked(),
            'vulkan_available': self.vulkan_available,
            # currentData() returns None for unset items; 0 is a valid preset
            'quality_preset': qp_data if qp_data is not None else 3,
            'two_pass': self.chk_2pass.isChecked(),
            'res_choice': self.res_combo.currentText(),
            'ar_choice': self.ar_combo.currentText(),
            'fps_choice': fps_choice,
            'algo': self.algo_combo.currentText(),
            'a_codec': self.a_codec.currentText(),
            'a_bitrate': abit,
            'output_folder': self.output_path.text(),
            'device': current_device,
            'copy_data': self.chk_copy_data.isChecked(),
            'container': container_code,
            'auto_scale': self.chk_auto_scale.isChecked(),
            'hw_decoder_caps': self.hw_decoder_capabilities.get(current_device, {}),
            'hw_encoder_opts': self.device_capabilities.get(current_device, {}).get('encoder_options', {}),
            'audio_encoders': self.available_audio_encoders,
            'ffmpeg_path': ffmpeg_path,
            'ffprobe_path': ffprobe_path
        }
        self.process_next_job()

    def process_next_job(self):
        if self.queue_list.count() == 0:
            self.all_finished()
            return

        item = self.queue_list.item(0)
        item.setSelected(True)
        fpath = item.data(Qt.UserRole)

        current_job_params = self.batch_params.copy()
        current_job_params['input'] = fpath

        # Reset progress bar for new job
        self.progress_bar.setValue(0)
        
        # Start timer for this job
        self.job_start_time = time.time()
        self.time_label.setText("ETA: --:--:--")
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_elapsed_time)
        self.timer.start(1000)  # Update every second

        self.worker = EncoderWorker(current_job_params)
        self.worker.log_signal.connect(self.log.append)
        self.worker.scale_notification_signal.connect(self.show_scale_notification)
        self.worker.finished_signal.connect(self.job_finished)
        self.worker.compatibility_warning_signal.connect(self.show_compatibility_warning)
        self.worker.progress_signal.connect(self.update_progress)
        self.worker.eta_signal.connect(self.update_eta)
        self.worker.start()
    
    @staticmethod
    def _format_time(seconds) -> str:
        """Format seconds as HH:MM:SS."""
        hours = int(seconds // 3600)
        minutes = int((seconds % 3600) // 60)
        secs = int(seconds % 60)
        return f"{hours:02d}:{minutes:02d}:{secs:02d}"

    def update_elapsed_time(self):
        """Update the elapsed time display"""
        if not self._show_eta and hasattr(self, 'job_start_time'):
            elapsed = time.time() - self.job_start_time
            self.time_label.setText(f"Elapsed: {self._format_time(elapsed)}")

    def update_eta(self, eta_seconds: int):
        """Update the ETA display"""
        if self._show_eta:
            if eta_seconds >= 0:
                self.time_label.setText(f"ETA: {self._format_time(eta_seconds)}")
            else:
                self.time_label.setText("ETA: --:--:--")

    def _toggle_time_display(self, event):
        """Toggle between ETA and Elapsed time display"""
        self._show_eta = not self._show_eta
        # Immediately update the display
        if hasattr(self, 'job_start_time'):
            if self._show_eta:
                self.time_label.setText("ETA: --:--:--")
            else:
                elapsed = time.time() - self.job_start_time
                self.time_label.setText(f"Elapsed: {self._format_time(elapsed)}")

    def job_finished(self, success):
        if success: self.log.append(">>> JOB FINISHED SUCCESSFULLY")
        else: self.log.append(">>> JOB FAILED or CANCELLED")

        # Stop the timer
        if hasattr(self, 'timer'):
            self.timer.stop()

        # Access is_cancelled with proper thread synchronization
        with self.worker._process_lock:
            was_cancelled = self.worker.is_cancelled
        
        if not success and not was_cancelled:
            self._failed_count += 1
            self.log.append(f"ERROR: Job failed: {os.path.basename(self.worker.params.get('input', ''))}")
            if not self.chk_no_sound.isChecked():
                self.notification_manager.notify_failure()
        
        if not was_cancelled:
            # Get the file path before removing the item
            item = self.queue_list.item(0)
            if item:
                fpath = item.data(Qt.UserRole)
                self.queue_list.takeItem(0)
                # Remove from _queue_paths to allow re-adding the same file
                self._queue_paths.discard(fpath)
                # Update placeholder visibility when queue becomes empty
                self.update_queue_placeholder()
                # Re-enable add button to allow adding new files after each job completes
                self.btn_add_queue.setEnabled(True)
                self.process_next_job()
        else:
            self.reset_ui()
            self.log.append("Queue Cancelled.")

    def all_finished(self):
        self.reset_ui()
        job_count = getattr(self, '_total_jobs', 0)
        failed = getattr(self, '_failed_count', 0)
        if failed:
            message = f"Queue finished: {failed}/{job_count} job(s) failed"
        else:
            message = f"All queue items processed successfully ({job_count} files)"

        # Show popup only if "No Popup" is not checked
        if not self.chk_no_popup.isChecked():
            QMessageBox.information(self, "Done", message)
        else:
            self.log.append("All queue items processed (No Popup mode).")

        # Play sound only if "No Sound" is not checked
        if not self.chk_no_sound.isChecked():
            self.notification_manager.notify_completion(message, show_popup=not self.chk_no_popup.isChecked(), play_sound=True)
        elif not self.chk_no_popup.isChecked():
            # If sound is disabled but popup is enabled, still show desktop notification
            self.notification_manager.notify_completion(message, show_popup=True, play_sound=False)

    def reset_ui(self):
        self._set_ui_enabled(True, encoding=True)
        self.progress_bar.setValue(0)

    def cancel(self):
        if self.worker and self.worker.isRunning():
            reply = QMessageBox.question(self, "Cancel", "Stop transcoding?", QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if reply == QMessageBox.Yes:
                self.worker.cancel()
                self.btn_cancel.setEnabled(False)

    def closeEvent(self, event):
        try:
            if self.worker and self.worker.isRunning():
                reply = QMessageBox.question(self, "Exit", "Transcoding in progress. Exit?", QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
                if reply == QMessageBox.Yes:
                    self.worker.cancel()
                    event.accept()
                else:
                    event.ignore()
            else:
                event.accept()
        finally:
            self.release_sleep()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.update_queue_placeholder()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec())
