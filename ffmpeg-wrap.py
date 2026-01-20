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
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                                QHBoxLayout, QLabel, QLineEdit, QPushButton,
                                QComboBox, QTextEdit, QFormLayout, QMessageBox,
                                QFileDialog, QCheckBox, QGroupBox, QListWidget,
                                QListWidgetItem, QSlider, QSpinBox, QStackedWidget,
                                QAbstractItemView, QSizePolicy, QProgressBar)
from PySide6.QtCore import Qt, QThread, Signal, QTimer
from PySide6.QtGui import QFont

class EncoderWorker(QThread):
    log_signal = Signal(str)
    scale_notification_signal = Signal(str)
    finished_signal = Signal(bool)
    compatibility_warning_signal = Signal(str)
    progress_signal = Signal(int)  # Progress percentage (0-100)
    
    # Pre-compiled regex pattern for efficient log line filtering
    LOG_KEYWORD_PATTERN = re.compile(r'frame=|Error|Stream #|kb/s|kB time=|error|Invalid argument', re.IGNORECASE)
    
    # Pattern to extract time from FFmpeg progress output
    TIME_PATTERN = re.compile(r'time=(\d{2}):(\d{2}):(\d{2}\.\d+)')

    def __init__(self, params):
        super().__init__()
        self.params = params
        self.process = None
        self.is_cancelled = False
        self.ffmpeg_path = params.get('ffmpeg_path', 'ffmpeg')
        self.ffprobe_path = params.get('ffprobe_path', 'ffprobe')
        self._process_lock = threading.Lock()
        self.video_duration = 0.0  # Store video duration for progress calculation

    def cancel(self):
        self.is_cancelled = True
        with self._process_lock:
            if self.process:
                self.log_signal.emit("--- CANCELLING ENCODE... ---")
                try:
                    self.process.terminate()
                    self.process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    try:
                        self.process.kill()
                        self.process.wait(timeout=2)
                    except (subprocess.TimeoutExpired, OSError, Exception) as e:
                        self.log_signal.emit(f"Warning: Process termination incomplete: {e}")
                except (OSError, Exception) as e:
                    self.log_signal.emit(f"Warning: Error terminating process: {e}")
            # Process is now None or terminated within the lock
            self.process = None

    def get_video_info(self, filepath):
        if self.is_cancelled: return None, 0, 0, "", 0, ""

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
            audio_count = sum(1 for s in data.get('streams', []) if s.get('codec_type') == 'audio')
            
            # Get first audio codec name
            audio_codec = ""
            audio_streams = [s for s in data.get('streams', []) if s.get('codec_type') == 'audio']
            if audio_streams:
                audio_codec = audio_streams[0].get('codec_name', 'unknown') or 'unknown'

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

            return dur, w, h, codec, audio_count, audio_codec

        except subprocess.CalledProcessError as e:
            self.log_signal.emit(f"FFprobe Error: {e.output.decode().strip()}")
            return None, 0, 0, "", 0, ""
        except Exception as e:
            self.log_signal.emit(f"Probe Parse Exception: {str(e)}")
            return None, 0, 0, "", 0, ""

    def run_ffmpeg_process(self, cmd, pass_name=""):
        if self.is_cancelled: return False
        if pass_name: self.log_signal.emit(f"--- STARTING {pass_name} ---")

        try:
            # Replace 'ffmpeg' with custom path if provided
            if cmd[0] == "ffmpeg":
                cmd[0] = self.ffmpeg_path
            
            # Log the actual command being executed
            self.log_signal.emit(f"Command: {' '.join(cmd)}")
            
            self.process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)

            line_count = 0
            max_lines = 10000  # Prevent memory exhaustion from excessive log output
            
            while True:
                if self.is_cancelled:
                    with self._process_lock:
                        if self.process:
                            try:
                                self.process.terminate()
                            except (OSError, Exception):
                                pass
                    break

                line = self.process.stdout.readline()
                if not line and self.process.poll() is not None:
                    break

                if line:
                    line = line.strip()
                    # Use pre-compiled regex pattern for faster matching
                    if self.LOG_KEYWORD_PATTERN.search(line):
                        self.log_signal.emit(line)
                        line_count += 1
                        if line_count >= max_lines:
                            self.log_signal.emit("Warning: Log output limit reached, suppressing further messages.")
                            break
                    
                    # Parse progress from FFmpeg output
                    if self.video_duration > 0:
                        time_match = self.TIME_PATTERN.search(line)
                        if time_match:
                            try:
                                hours = int(time_match.group(1))
                                minutes = int(time_match.group(2))
                                seconds = float(time_match.group(3))
                                current_time = hours * 3600 + minutes * 60 + seconds
                                progress = int((current_time / self.video_duration) * 100)
                                progress = max(0, min(100, progress))  # Clamp between 0-100
                                self.progress_signal.emit(progress)
                            except (ValueError, ZeroDivisionError):
                                pass

            self.process.wait(timeout=3600)  # 1 hour timeout
            # Capture returncode before cleaning up process reference
            returncode = self.process.returncode
            # Clean up process reference after completion
            with self._process_lock:
                self.process = None
            return returncode == 0 and not self.is_cancelled
        except subprocess.TimeoutExpired as e:
            if not self.is_cancelled:
                self.log_signal.emit(f"Process timeout after 1 hour: {e}")
            with self._process_lock:
                self.process = None
            return False
        except Exception as e:
            if not self.is_cancelled:
                self.log_signal.emit(f"Process Error: {e}")
            with self._process_lock:
                self.process = None
            return False

    def align_resolution(self, width, height, align_w=64, align_h=16):
        new_w = math.ceil(width / align_w) * align_w
        new_h = math.ceil(height / align_h) * align_h
        return new_w, new_h

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
        if not os.path.exists(input_file):
            self.log_signal.emit(f"Error: Input file does not exist: {input_file}")
            self.finished_signal.emit(False)
            return
        
        if not os.path.isfile(input_file):
            self.log_signal.emit(f"Error: Input path is not a file: {input_file}")
            self.finished_signal.emit(False)
            return
        
        if not os.access(input_file, os.R_OK):
            self.log_signal.emit(f"Error: No read permission for input file: {input_file}")
            self.finished_signal.emit(False)
            return
        
        mode = p['mode']
        copy_data = p.get('copy_data', True)
        container_mode = p.get('container', 'MP4')
        auto_scale = p.get('auto_scale', True)
        use_hw = p.get('use_hw', True)
        selected_codec = p['v_codec']

        duration, orig_w, orig_h, input_codec, audio_count, audio_codec = self.get_video_info(input_file)
        self.video_duration = duration if duration else 0.0

        if orig_w == 0 or orig_h == 0:
            self.log_signal.emit("Error: Could not determine video resolution.")
            self.finished_signal.emit(False)
            return

        self.log_signal.emit(f"=== Processing: {os.path.basename(input_file)} ({orig_w}x{orig_h} {input_codec}) ===")

        # --- Video Passthrough Check ---
        if p['v_codec'] == "Passthrough":
            # Container compatibility for video passthrough
            mp4_compatible_video = ['h264', 'hevc', 'vp9', 'av1', 'mpeg4']
            webm_compatible_video = ['vp8', 'vp9', 'av1']
            mov_compatible_video = ['prores', 'h264', 'hevc', 'dnxhd', 'mpeg4']
            
            if container_mode == "MP4" and input_codec not in mp4_compatible_video:
                self.compatibility_warning_signal.emit(f"Video codec '{input_codec}' is not compatible with MP4 container. Please select a different container or use a different video codec.")
                self.finished_signal.emit(False)
                return
            elif container_mode == "WEBM" and input_codec not in webm_compatible_video:
                self.compatibility_warning_signal.emit(f"Video codec '{input_codec}' is not compatible with WEBM container. Please select a different container or use a different video codec.")
                self.finished_signal.emit(False)
                return
            elif container_mode == "MOV" and input_codec not in mov_compatible_video:
                self.compatibility_warning_signal.emit(f"Video codec '{input_codec}' is not compatible with MOV container. Please select a different container or use a different video codec.")
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
            safety_margin = 0.95

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
        audio_ext_hint = ".webm"
        
        # MP4 compatible audio codecs for passthrough
        mp4_compatible_codecs = ['aac', 'mp3', 'opus', 'alac', 'ac3', 'eac3', 'flac']
        
        # WebM compatible audio codecs for passthrough
        webm_compatible_codecs = ['opus', 'vorbis']
        
        # MOV compatible audio codecs for passthrough
        mov_compatible_codecs = ['pcm_s16le', 'pcm_s24le', 'pcm_s32le', 'pcm_f32le', 'aac', 'alac', 'mp3']

        a_codec = p.get('a_codec', 'AAC')
        if a_codec == "Passthrough":
            audio_flags = ["-c:a", "copy"]
            audio_bitrate = 0.0
            
            # Check if audio codec is compatible with selected container
            if container_mode == "MP4" and audio_codec not in mp4_compatible_codecs:
                self.compatibility_warning_signal.emit(f"Audio codec '{audio_codec}' is not compatible with MP4 container. Please select a different container or use a different audio codec.")
                self.finished_signal.emit(False)
                return
            elif container_mode == "WEBM" and audio_codec not in webm_compatible_codecs:
                self.compatibility_warning_signal.emit(f"Audio codec '{audio_codec}' is not compatible with WEBM container. Please select a different container or use a different audio codec.")
                self.finished_signal.emit(False)
                return
            elif container_mode == "MOV" and audio_codec not in mov_compatible_codecs:
                self.compatibility_warning_signal.emit(f"Audio codec '{audio_codec}' is not compatible with MOV container. Please select a different container or use a different audio codec.")
                self.finished_signal.emit(False)
                return
            else:
                audio_ext_hint = ".mkv"
        elif a_codec == "PCM":
            audio_flags = ["-c:a", "pcm_s16le", "-ar", "48000", "-ac", "2"]
            audio_bitrate = 1536.0
            audio_ext_hint = ".mov"
        else:
            enc = "libopus" if a_codec == "Opus" else "aac"
            try:
                raw_bitrate = float(p.get('a_bitrate', 128))
                if raw_bitrate < 8: raw_bitrate = 128.0
            except (ValueError, TypeError):
                raw_bitrate = 128.0

            audio_bitrate = raw_bitrate
            audio_flags = ["-c:a", enc, "-b:a", f"{int(audio_bitrate)}k", "-ar", "48000", "-ac", "2"]
            if enc == "libopus":
                audio_flags.extend(["-sample_fmt", "flt"])

            audio_ext_hint = ".webm" if enc == "libopus" else ".mp4"

        # --- Resolve Output Container ---
        if container_mode == "MP4": ext = ".mp4"
        elif container_mode == "MKV": ext = ".mkv"
        elif container_mode == "MOV": ext = ".mov"
        elif container_mode == "WEBM": ext = ".webm"
        else: ext = audio_ext_hint

        # --- Output Path ---
        out_folder = p.get('output_folder', '')
        input_filename_no_ext = os.path.splitext(os.path.basename(input_file))[0]
        
        # Validate and sanitize output folder path
        if out_folder:
            # Normalize path and resolve any relative components
            out_folder = os.path.abspath(out_folder)
            # Check for path traversal attempts
            if '..' in out_folder or not os.path.isdir(out_folder):
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
        target_h = res_map.get(res_str, orig_h)
        ar_str = p.get('ar_choice', 'Original')

        # --- Bitrate & Scaling Logic ---
        video_kbps = 0
        forced_av1 = False
        force_mitchell = False

        if mode == 'size':
            if duration <= 0:
                self.log_signal.emit("Error: Unknown duration, cannot calculate target size.")
                self.finished_signal.emit(False)
                return

            try:
                target_mb = float(p['size'])
                effective_mb = target_mb * safety_margin
                target_bits = effective_mb * 8 * 1024 * 1024

                total_audio_bitrate = audio_bitrate * max(1, audio_count)
                audio_total = total_audio_bitrate * 1000 * duration

                video_bits = target_bits - audio_total

                if video_bits <= 0:
                    self.log_signal.emit("Error: Target size too small for audio tracks.")
                    self.finished_signal.emit(False)
                    return

                video_kbps = int((video_bits / duration) / 1000)
                # Add bounds checking to prevent overflow
                if video_kbps < 1:
                    video_kbps = 1
                elif video_kbps > 100000:  # Reasonable upper limit (100 Mbps)
                    self.log_signal.emit(f"Warning: Calculated bitrate {video_kbps}k is very high, capping at 100000k.")
                    video_kbps = 100000

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
                    effective_mb = target_mb * 0.95
                    target_bits = effective_mb * 8 * 1024 * 1024
                    video_bits = target_bits - audio_total
                    video_kbps = int((video_bits / duration) / 1000)
                    # Add bounds checking for forced AV1 case
                    if video_kbps < 1:
                        video_kbps = 1
                    elif video_kbps > 100000:
                        self.log_signal.emit(f"Warning: Calculated bitrate {video_kbps}k is very high, capping at 100000k.")
                        video_kbps = 100000
                    self.log_signal.emit("⚠ Auto-Scale: Forcing 2-pass encoding for optimal quality at low bitrate")
                else:
                    self.log_signal.emit(f"Target: {target_mb}MB | Video: {video_kbps}k | Codec: {video_codec_cmd}")

            except ValueError:
                self.finished_signal.emit(False)
                return
        else:
            crf_value = p.get('crf', 24)
            self.log_signal.emit(f"Mode: CQP {crf_value}")

        # --- Width Calculation ---
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

        aligned_w, aligned_h = self.align_resolution(target_w, target_h, 64, 16)
        pad_right = aligned_w - target_w
        pad_bottom = aligned_h - target_h

        output_file = f"{output_base}{v_tag}{ext}"
        
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
        map_flags = ["-map", "0:v:0", "-map", "0:a"]
        meta_subs_flags = []
        is_mp4_container = (ext == ".mp4")

        if copy_data:
            if is_mp4_container:
                self.log_signal.emit("Notice: MP4 selected. Converting subtitles to text, skipping fonts/data.")
                map_flags.extend(["-map", "0:s?", "-map_metadata", "0"])
                meta_subs_flags = ["-c:s", "mov_text"]
            else:
                map_flags.extend(["-map", "0:s?", "-map", "0:d?", "-map", "0:t?", "-map_metadata", "0"])
                meta_subs_flags = ["-c:s", "copy", "-c:d", "copy", "-c:t", "copy"]
        else:
            map_flags.extend(["-map_metadata", "-1"])

        vf_chain = []
        algo = p.get('algo', 'Bicubic')

        # Add framerate filter if specified (before scaling)
        if fps_filter:
            vf_chain.append(fps_filter)

        # --- PIPELINE CONSTRUCTION (SW vs HW) ---
        if video_codec_cmd == "copy":
            # Video passthrough - no scaling or encoding
            self.log_signal.emit("Pipeline: Video Passthrough (No re-encoding)")
            base_cmd.extend(["-i", input_file])
            vf_chain = []
            video_flags = ["-c:v", "copy"]
        elif use_hw:
            # Check if input codec can be hardware decoded
            device = p.get('device', '/dev/dri/renderD128')
            hw_decoder_caps = p.get('hw_decoder_caps', {})
            can_hw_decode = hw_decoder_caps.get(input_codec, False)

            if not can_hw_decode:
                self.log_signal.emit(f"Notice: Codec '{input_codec}' not HW-decodable. Using CPU Decode.")

            if is_av1 and "av1_vaapi" in video_codec_cmd:
                 pix_fmt = "yuv420p10le"
            else:
                 pix_fmt = "nv12"

            if is_av1 or "VAAPI" not in algo or force_mitchell:
                use_software_scaler = True
            else:
                use_software_scaler = False

            if use_software_scaler:
                self.log_signal.emit(f"Pipeline: CPU Scale -> GPU Encode")
                base_cmd.extend(["-init_hw_device", f"vaapi=va:{device}", "-filter_hw_device", "va", "-i", input_file])

                scale_filter_str = f"scale={target_w}:{target_h}:flags=bicubic"
                if force_mitchell:
                    scale_filter_str = f"scale={target_w}:{target_h}:flags=bicubic:param0=0.333:param1=0.333"
                elif "Lanczos" in algo:
                    scale_filter_str = f"scale={target_w}:{target_h}:flags=lanczos"
                elif "Nearest" in algo:
                    scale_filter_str = f"scale={target_w}:{target_h}:flags=neighbor"

                vf_chain.append(scale_filter_str)
                vf_chain.append(f"format={pix_fmt},hwupload")
            else:
                if can_hw_decode:
                    self.log_signal.emit(f"Pipeline: Full Hardware (VAAPI)")
                    base_cmd.extend(["-hwaccel", "vaapi", "-hwaccel_device", device, "-hwaccel_output_format", "vaapi", "-i", input_file])
                    vf_chain.append(f"scale_vaapi=w={target_w}:h={target_h}:format={pix_fmt}")
                else:
                    self.log_signal.emit(f"Pipeline: CPU Decode -> GPU Scale/Encode")
                    base_cmd.extend(["-init_hw_device", f"vaapi=va:{device}", "-filter_hw_device", "va", "-i", input_file])
                    vf_chain.append(f"format={pix_fmt},hwupload")
                    vf_chain.append(f"scale_vaapi=w={target_w}:h={target_h}:format={pix_fmt}")

        else:
            self.log_signal.emit("Pipeline: Full Software (CPU)")
            base_cmd.extend(["-i", input_file])

            scale_filter_str = f"scale={target_w}:{target_h}:flags=bicubic"
            if force_mitchell or "Mitchell" in algo:
                 scale_filter_str = f"scale={target_w}:{target_h}:flags=bicubic:param0=0.333:param1=0.333"
            elif "Lanczos" in algo:
                 scale_filter_str = f"scale={target_w}:{target_h}:flags=lanczos"
            elif "Nearest" in algo:
                 scale_filter_str = f"scale={target_w}:{target_h}:flags=neighbor"

            vf_chain.append(scale_filter_str)
            if is_av1:
                 vf_chain.append("format=yuv420p10le")
            else:
                 vf_chain.append("format=yuv420p")

        if vf_chain:
            base_cmd.extend(["-vf", ",".join(vf_chain)])

        video_flags = ["-c:v", video_codec_cmd]

        # --- Encoding Parameters ---
        if video_codec_cmd == "copy":
            # No encoding parameters for passthrough
            pass
        elif is_av1:
            video_flags.extend(["-preset", quality_preset])
            if not use_hw:
                video_flags.extend(["-pix_fmt", "yuv420p10le"])
        elif is_vp9:
            # VP9 encoding parameters
            if use_hw:
                video_flags.extend(["-compression_level", quality_preset])
            else:
                # VP9 uses proper parameters according to FFmpeg wiki
                # -deadline: good (default), best (slower), realtime (fastest)
                # -cpu-used: 0-5 with good/best, 0-8 with realtime
                # -row-mt 1: enables multi-threading for better performance
                try:
                    vp9_quality = int(quality_preset)
                except (ValueError, TypeError):
                    vp9_quality = 4  # Default to "Default" preset if conversion fails
                    self.log_signal.emit(f"Warning: Invalid quality preset '{quality_preset}', using default.")
                
                if vp9_quality == 0:
                    # Best quality: best deadline
                    video_flags.extend(["-deadline", "best", "-cpu-used", "0"])
                elif vp9_quality == 1:
                    # High quality: best deadline with some speed tradeoff
                    video_flags.extend(["-deadline", "best", "-cpu-used", "1"])
                elif vp9_quality == 2:
                    # Good quality: good deadline with low cpu-used
                    video_flags.extend(["-deadline", "good", "-cpu-used", "0"])
                elif vp9_quality == 3:
                    # Balanced: good deadline with moderate cpu-used
                    video_flags.extend(["-deadline", "good", "-cpu-used", "1"])
                elif vp9_quality == 4:
                    # Default: good deadline with higher cpu-used
                    video_flags.extend(["-deadline", "good", "-cpu-used", "2"])
                elif vp9_quality == 5:
                    # Fast: good deadline with high cpu-used
                    video_flags.extend(["-deadline", "good", "-cpu-used", "3"])
                else:
                    # Very fast: good deadline with max cpu-used
                    video_flags.extend(["-deadline", "good", "-cpu-used", str(min(vp9_quality, 5))])
                video_flags.extend(["-row-mt", "1", "-pix_fmt", "yuv420p"])
        else:
            if use_hw:
                # Check GPU vendor - VCN compression levels are AMDGPU-specific
                device = p.get('device', '/dev/dri/renderD128')
                gpu_vendor = p.get('hw_decoder_caps', {}).get('gpu_vendor', 'unknown')
                
                if "h264_vaapi" in video_codec_cmd or "hevc_vaapi" in video_codec_cmd:
                    if gpu_vendor == 'amd':
                        # VCN (Video Core Next) compression level mapping for AMDGPU
                        # Binary flags: VBAQ=16, pre-encode=8, quality=4, balanced=2, speed=0, validity=1
                        # Map UI preset values to correct VCN compression levels
                        compression_map = {
                            1: 29,  # Quality (Best): quality + pre-encode + VBAQ (4+8+16+1)
                            2: 1,   # Balanced (Default): balanced + pre-encode + VBAQ (AMD recommended)
                            4: 11,  # Speed (Fast): balanced + pre-encode (2+8+1)
                            7: 0    # Max Speed: speed preset only
                        }
                        try:
                            preset_val = int(quality_preset)
                            compression_level = compression_map.get(preset_val, 1)  # Default to balanced (1)
                            self.log_signal.emit(f"HW Preset: {preset_val} -> VCN compression_level={compression_level}")
                            video_flags.extend(["-compression_level", str(compression_level)])
                        except (ValueError, TypeError):
                            self.log_signal.emit(f"Warning: Invalid quality preset '{quality_preset}', using default")
                            video_flags.extend(["-compression_level", "1"])
                    else:
                        # Intel or other GPU - skip VCN compression levels
                        # Intel Quick Sync doesn't use the same compression_level parameter
                        self.log_signal.emit(f"Notice: {gpu_vendor.upper()} GPU detected - VCN compression levels not applied")
            else:
                presets = ["veryslow", "slower", "slow", "medium", "fast", "faster", "veryfast", "superfast", "ultrafast"]
                try:
                    p_idx = int(quality_preset)
                except (ValueError, TypeError):
                    p_idx = 3  # Default to "medium" if conversion fails
                    self.log_signal.emit(f"Warning: Invalid quality preset '{quality_preset}', using medium.")
                
                if 0 <= p_idx < len(presets):
                    video_flags.extend(["-preset", presets[p_idx]])

            if "hevc" in video_codec_cmd:
                if pad_right > 0 or pad_bottom > 0:
                    self.log_signal.emit(f"Metadata: Cropping padding (Right:{pad_right}, Bottom:{pad_bottom}) for HEVC.")
                    video_flags.extend(["-bsf:v", f"hevc_metadata=crop_right={pad_right}:crop_bottom={pad_bottom}"])

        if mode == 'size':
            video_flags.extend(["-b:v", f"{video_kbps}k"])
            if not is_av1 and not is_vp9:
                video_flags.extend(["-maxrate", f"{video_kbps}k", "-bufsize", f"{video_kbps*2}k"])
        else:
            crf_val = str(p.get('crf', 24))
            if is_av1:
                video_flags.extend(["-crf", crf_val, "-b:v", "0"])
            elif is_vp9:
                video_flags.extend(["-crf", crf_val, "-b:v", "0"])
            else:
                if use_hw:
                    video_flags.extend(["-rc_mode", "CQP", "-qp", crf_val])
                else:
                    video_flags.extend(["-crf", crf_val])

        # --- Execute ---
        # VP9 supports 2-pass encoding
        # Force 2-pass when auto downscale is triggered (forced_av1)
        run_2pass = (mode == 'size' and not use_hw and video_codec_cmd != "copy") and (p.get('two_pass', False) or forced_av1)

        if run_2pass:
            temp_dir = tempfile.gettempdir()
            # Use a more specific filename with a unique identifier
            unique_id = str(uuid.uuid4())[:8]
            pass_log_base = os.path.join(temp_dir, f"ffmpeg_2pass_log_{unique_id}")

            cmd_p1 = base_cmd + ["-map", "0:v:0", "-c:v", video_codec_cmd, "-b:v", f"{video_kbps}k"]

            # First pass: always use fastest preset possible
            if is_av1:
                cmd_p1.extend(["-preset", "12", "-pix_fmt", "yuv420p10le"])  # Fast AV1 preset for pass 1
            elif is_vp9:
                cmd_p1.extend(["-deadline", "realtime", "-cpu-used", "8", "-row-mt", "1"])  # Fastest VP9 for pass 1
            elif "libx" in video_codec_cmd:
                cmd_p1.extend(["-preset", "ultrafast"])  # Fastest libx264/libx265 preset

            cmd_p1.extend(["-pass", "1", "-passlogfile", pass_log_base, "-an", "-f", "null", os.devnull])

            if not self.run_ffmpeg_process(cmd_p1, "PASS 1"):
                self.finished_signal.emit(False)
                return

            # Second pass: use the actual selected preset (video_flags already contains it)
            cmd_p2 = base_cmd + map_flags + video_flags + ["-pass", "2", "-passlogfile", pass_log_base] + audio_flags + meta_subs_flags + [output_file]
            success = self.run_ffmpeg_process(cmd_p2, "PASS 2")

            try:
                # More specific pattern matching with validation
                expected_files = [
                    f"{pass_log_base}-0.log",
                    f"{pass_log_base}-0.log.mbtree"
                ]
                for f in expected_files:
                    if os.path.exists(f):
                        try:
                            os.remove(f)
                        except (OSError, PermissionError) as e:
                            self.log_signal.emit(f"Warning: Could not remove {f}: {e}")
            except (OSError, PermissionError) as e:
                self.log_signal.emit(f"Warning: Could not clean up pass log files: {e}")
        else:
            cmd = base_cmd + map_flags + video_flags + audio_flags + meta_subs_flags + [output_file]
            success = self.run_ffmpeg_process(cmd, "ENCODE")

        self.finished_signal.emit(success)

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
    
    def _play_chime(self):
        """Play a pleasant two-tone chime"""
        try:
            # Generate two tones: A5 (880Hz) then C#6 (1108.73Hz)
            tone1 = self._generate_tone(880, 0.1, 0.25)    # A5
            tone2 = self._generate_tone(1108.73, 0.15, 0.25)  # C#6
            
            audio_data = tone1 + tone2
            
            # Play using paplay (PipeWire/PulseAudio)
            if self.has_paplay:
                proc = subprocess.Popen([
                    'paplay', '--raw', '--format=s16le',
                    '--channels=1', '--rate=44100'
                ], stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                proc.communicate(input=audio_data)
                return True
            
            # Fallback to aplay (ALSA)
            elif self.has_aplay:
                proc = subprocess.Popen([
                    'aplay', '-q', '-f', 'cd', '-c', '1'
                ], stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                proc.communicate(input=audio_data)
                return True
            
        except Exception:
            pass
        
        return False
    
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
        self._queue_paths = set()  # Set for O(1) duplicate checking

        self.device_capabilities = {}
        self.warning_shown = False
        self.available_sw_codecs = {'h264': False, 'hevc': False}
        self.hw_decoder_capabilities = {}  # Store hardware decoder capabilities
        self.notification_manager = NotificationManager()

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
        self.target_size = QLineEdit("10")
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
        btn_out_browse = QPushButton("Browse")
        btn_out_browse.clicked.connect(self.browse_output_folder)

        out_layout.addWidget(QLabel("Format:"))
        out_layout.addWidget(self.container_combo)
        out_layout.addWidget(QLabel("Destination:"))
        out_layout.addWidget(self.output_path)
        out_layout.addWidget(btn_out_browse)
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
        
        ffmpeg_browse_btn = QPushButton("Browse")
        ffmpeg_browse_btn.clicked.connect(self.browse_ffmpeg)
        
        ffmpeg_layout.addWidget(self.ffmpeg_path)
        ffmpeg_layout.addWidget(ffmpeg_browse_btn)
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
        self.res_combo.setCurrentText("Original")
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
        self.fps_combo.setCurrentText("Original")

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
        
        progress_layout.addWidget(self.progress_bar)
        layout.addWidget(progress_container)
        
        # --- BUTTONS ---
        btn_row = QHBoxLayout()
        self.btn_start = QPushButton("Start Queue")
        self.btn_start.clicked.connect(self.start_queue)
        self.btn_start.setStyleSheet("background-color: #2e7d32; color: white; font-weight: bold; padding: 10px;")

        self.chk_no_popup = QCheckBox("No Popup")
        self.chk_no_sound = QCheckBox("No Sound")

        self.btn_cancel = QPushButton("Cancel")
        self.btn_cancel.clicked.connect(self.cancel)
        self.btn_cancel.setStyleSheet("background-color: #c62828; color: white; font-weight: bold; padding: 10px;")
        self.btn_cancel.setEnabled(False)

        btn_row.addWidget(self.btn_start)
        btn_row.addWidget(self.chk_no_popup)
        btn_row.addWidget(self.chk_no_sound)
        btn_row.addStretch()
        btn_row.addWidget(self.btn_cancel)
        layout.addLayout(btn_row)

        # Connect signals AFTER widgets are created
        self.device_combo.currentIndexChanged.connect(self.on_device_changed)

        # Initialize UI State with defaults
        self.update_codec_ui(self.v_codec.currentText())

        # Sync widget sizes based on other widgets
        # Set Target Size widget width to fit input (4 digits), MB label, and 2-Pass checkbox
        self.size_widget.setFixedWidth(150)
        # Set Speed Preset to fit "Balanced (Default)" and match Encoding Mode to it
        self.quality_combo.setMinimumWidth(140)
        self.mode_combo.setMinimumWidth(140)
        # Match Audio Bitrate widget width to Audio Codec width
        self.bitrate_widget.setFixedWidth(self.a_codec.sizeHint().width())
        # Match Resolution and Aspect Ratio dropdowns to Scaling dropdown width
        self.res_combo.setFixedWidth(self.algo_combo.sizeHint().width())
        self.ar_combo.setFixedWidth(self.algo_combo.sizeHint().width())

        # Schedule background initialization tasks after window is shown
        # This prevents UI blocking during device probing and encoder checking
        QTimer.singleShot(100, self.check_sw_encoders)
        QTimer.singleShot(150, self.initial_probe)
        QTimer.singleShot(0, self.update_queue_placeholder)  # Initialize placeholder visibility
        
        self.worker = None

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
            self.algo_combo.setEnabled(False)
            self.chk_auto_scale.setEnabled(False)
            self.mode_stack.setEnabled(False)

    # --- SOFTWARE ENCODER DETECTION ---
    def check_sw_encoders(self):
        """Check which software encoders are available in the FFmpeg build"""
        # Get custom FFmpeg path if provided
        ffmpeg_path = self.ffmpeg_path.text().strip() if hasattr(self, 'ffmpeg_path') else ''
        ffmpeg_cmd = ffmpeg_path if ffmpeg_path else 'ffmpeg'
        
        ffmpeg_not_found = False
        unavailable_codecs = []
        codecs_to_check = [
            ('h264', 'libx264', 'H.264'),
            ('hevc', 'libx265', 'H.265')
        ]

        for key, codec, display_name in codecs_to_check:
            try:
                cmd = [
                    ffmpeg_cmd, "-y", "-hide_banner",
                    "-f", "lavfi", "-i", "nullsrc=duration=1:size=320x240:rate=1",
                    "-c:v", codec,
                    "-f", "null", "-"
                ]
                result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=5)
                if result.returncode == 0:
                    self.available_sw_codecs[key] = True
                    self.log.append(f"SW Encoder: {display_name} - Available")
                else:
                    self.available_sw_codecs[key] = False
                    self.log.append(f"SW Encoder: {display_name} - Not Available")
                    unavailable_codecs.append(display_name)
            except FileNotFoundError:
                self.log.append("Error: FFmpeg not found. Cannot verify software encoder capabilities.")
                ffmpeg_not_found = True
                break
            except subprocess.TimeoutExpired:
                self.log.append(f"Error: Timeout checking {codec}. Encoder may not be responsive.")
                self.available_sw_codecs[key] = False
                unavailable_codecs.append(display_name)
            except Exception as e:
                self.log.append(f"Error checking {codec}: {str(e)}")
                self.available_sw_codecs[key] = False
                unavailable_codecs.append(display_name)

        # Show popup if FFmpeg is not found
        if ffmpeg_not_found:
            QMessageBox.critical(self, "FFmpeg Not Found",
                                "FFmpeg executable not found on your system!\n\n"
                                "Please install FFmpeg to use this application.\n"
                                "On Fedora: sudo dnf install ffmpeg\n"
                                "On Ubuntu: sudo apt install ffmpeg")
            # Assume all are available if FFmpeg is not found (for testing purposes)
            self.available_sw_codecs = {'h264': True, 'hevc': True}
        # Show popup if some codecs are unavailable
        elif unavailable_codecs:
            codec_list = ", ".join(unavailable_codecs)
            QMessageBox.warning(self, "Missing Codec Support",
                                f"Your FFmpeg build does not support the following codecs:\n\n"
                                f"{codec_list}\n\n"
                                f"These codecs will be grayed out in the dropdown.\n"
                                f"Consider installing a full FFmpeg build with all codecs enabled.")

        # Update codec dropdown based on availability
        self.update_codec_options()

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

    def detect_gpu_vendor(self, device_path):
        """Detect GPU vendor from DRI device path"""
        try:
            # Extract device number from path (e.g., /dev/dri/renderD128 -> 128)
            device_num = device_path.split('renderD')[-1]
            
            # Try to read vendor from sysfs
            vendor_path = f"/sys/class/drm/renderD{device_num}/device/vendor"
            if os.path.exists(vendor_path):
                with open(vendor_path, 'r') as f:
                    vendor_id = f.read().strip()
                    # Intel vendor ID is 0x8086
                    if vendor_id.lower() == '0x8086':
                        return 'intel'
                    # AMD vendor IDs: 0x1002 (ATI) or 0x1022
                    elif vendor_id.lower() in ['0x1002', '0x1022']:
                        return 'amd'
            
            # Fallback: try to parse from device name
            card_path = f"/sys/class/drm/renderD{device_num}"
            if os.path.exists(card_path):
                device_link = os.path.realpath(f"{card_path}/device")
                if 'i915' in device_link.lower():
                    return 'intel'
                elif 'amdgpu' in device_link.lower() or 'radeon' in device_link.lower():
                    return 'amd'
            
            return 'unknown'
        except Exception as e:
            self.log.append(f"Warning: Could not detect GPU vendor: {e}")
            return 'unknown'

    def probe_device(self, device_path):
        if device_path in self.device_capabilities:
            self.update_codec_ui(self.v_codec.currentText())
            return

        # Get custom FFmpeg path if provided
        ffmpeg_path = self.ffmpeg_path.text().strip() if hasattr(self, 'ffmpeg_path') else ''
        ffmpeg_cmd = ffmpeg_path if ffmpeg_path else 'ffmpeg'

        self.log.append(f"Probing device: {device_path} ...")
        
        # Detect GPU vendor
        gpu_vendor = self.detect_gpu_vendor(device_path)
        self.log.append(f"  -> GPU Vendor: {gpu_vendor.upper()}")
        capabilities = {'av1': False, 'h264': False, 'hevc': False}
        decoder_caps = {'h264': False, 'hevc': False, 'vp8': False, 'vp9': False, 'av1': False, 'mpeg2video': False}
        error_occurred = False

        # Probe hardware encoders
        codecs_to_check = [
            ('h264', 'h264_vaapi'),
            ('hevc', 'hevc_vaapi'),
            ('av1', 'av1_vaapi')
        ]

        try:
            for key, codec in codecs_to_check:
                cmd = [
                    ffmpeg_cmd, "-y", "-hide_banner",
                    "-init_hw_device", f"vaapi=dev:{device_path}",
                    "-filter_hw_device", "dev",
                    "-f", "lavfi", "-i", "nullsrc=duration=1:size=320x240:rate=1",
                    "-vf", "format=nv12,hwupload",
                    "-c:v", codec,
                    "-f", "null", "-"
                ]

                try:
                    result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=5)
                except subprocess.TimeoutExpired:
                    self.log.append(f"  -> {codec.upper()} Encoder: Not Supported (Timeout)")
                    capabilities[key] = False
                    continue

                if result.returncode == 0:
                    capabilities[key] = True
                    self.log.append(f"  -> {codec.upper()} Encoder: Supported")
                else:
                    stderr_text = result.stderr.decode()
                    if "No such device" in stderr_text or "Permission denied" in stderr_text:
                        raise Exception(f"Cannot access device {device_path}: {stderr_text.strip()}")

                    self.log.append(f"  -> {codec.upper()} Encoder: Not Supported")

            self.device_capabilities[device_path] = capabilities

            # Probe hardware decoders
            self.log.append("  Probing hardware decoder support...")
            decoder_codecs = ['h264', 'hevc', 'vp8', 'vp9', 'av1', 'mpeg2video']
            
            for dec_codec in decoder_codecs:
                cmd = [
                    ffmpeg_cmd, "-y", "-hide_banner",
                    "-hwaccel", "vaapi",
                    "-hwaccel_device", device_path,
                    "-f", "lavfi", "-i", "nullsrc=duration=1:size=320x240:rate=1",
                    "-c:v", "rawvideo",
                    "-f", "null", "-"
                ]
                
                try:
                    result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=5)
                except subprocess.TimeoutExpired:
                    decoder_caps[dec_codec] = False
                    self.log.append(f"  -> {dec_codec.upper()} Decoder: Not Supported (Timeout)")
                    continue
                
                if result.returncode == 0:
                    decoder_caps[dec_codec] = True
                    self.log.append(f"  -> {dec_codec.upper()} Decoder: Supported")
                else:
                    decoder_caps[dec_codec] = False
                    self.log.append(f"  -> {dec_codec.upper()} Decoder: Not Supported")
            
            self.hw_decoder_capabilities[device_path] = decoder_caps

        except FileNotFoundError:
            self.log.append("Error: FFmpeg not found. Cannot verify hardware capabilities.")
            self.capabilities_warning("FFmpeg executable not found. All codec options enabled (Use at your own risk).")
            error_occurred = True
        except Exception as e:
            self.log.append(f"Error probing device: {str(e)}")
            self.capabilities_warning(f"Could not verify capabilities for {device_path}. All codecs enabled (Use at your own risk).")
            error_occurred = True

        if error_occurred:
            self.device_capabilities[device_path] = {'av1': True, 'h264': True, 'hevc': True, 'gpu_vendor': 'unknown'}
            self.hw_decoder_capabilities[device_path] = {'h264': True, 'hevc': True, 'vp8': True, 'vp9': True, 'av1': True, 'mpeg2video': True, 'gpu_vendor': 'unknown'}
        else:
            # Store GPU vendor in both capabilities dictionaries
            self.device_capabilities[device_path]['gpu_vendor'] = gpu_vendor
            self.hw_decoder_capabilities[device_path]['gpu_vendor'] = gpu_vendor

        self.update_codec_ui(self.v_codec.currentText())

    def capabilities_warning(self, message):
        if not self.warning_shown:
            QMessageBox.warning(self, "Capability Detection Warning",
                                f"{message}\n\nThe application will attempt to proceed, but encoding may fail if the hardware does not support the selected codec.")
            self.warning_shown = True

    # --- UI UPDATERS ---
    def update_codec_ui(self, codec_text):
        is_av1 = "AV1" in codec_text
        is_h264 = "H.264" in codec_text
        is_hevc = "H.265" in codec_text
        is_vp9 = "VP9" in codec_text

        use_hw = self.chk_hw.isChecked()
        current_device = self.device_combo.currentText()

        caps = self.device_capabilities.get(current_device, {'av1': False, 'h264': False, 'hevc': False})

        hw_supported = False
        if is_av1:
            hw_supported = caps['av1']
        elif is_h264:
            hw_supported = caps['h264']
        elif is_hevc:
            hw_supported = caps['hevc']
        # VP9 doesn't have hardware support (software only)

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

        self.quality_combo.clear()

        if is_av1 and not use_hw:
            for i in range(14):
                desc = str(i)
                if i==0: desc += " (Slowest)"
                if i==8: desc += " (Default)"
                if i==12: desc += " (Fast)"
                if i==13: desc += " (Realtime)"
                self.quality_combo.addItem(desc, i)
            self.quality_combo.setCurrentIndex(8)
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
        elif use_hw:
            self.quality_combo.addItem("Quality (Best)", 1)
            self.quality_combo.addItem("Balanced (Default)", 2)
            self.quality_combo.addItem("Speed (Fast)", 4)
            self.quality_combo.addItem("Max Speed", 7)
            self.quality_combo.setCurrentIndex(1)
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
        self.a_bitrate.setEnabled(txt not in ["PCM", "Passthrough"])
        if txt in ["PCM", "Passthrough"]: self.a_bitrate.setText("N/A")
        elif self.a_bitrate.text() == "N/A": self.a_bitrate.setText("128")
        
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
            if value <= 0:
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
        if not self.chk_silent.isChecked():
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
        self.available_sw_codecs = {'h264': False, 'hevc': False}
        self.device_capabilities = {}
        self.update_codec_options()
        self.check_sw_encoders()
        if self.device_combo.count() > 0:
            self.probe_device(self.device_combo.currentText())

    def add_files_dialog(self):
        files, _ = QFileDialog.getOpenFileNames(self, "Select Videos", "", "Videos (*.mp4 *.mkv *.mov *.avi *.webm)")
        if files:
            for f in files: self.add_path_to_queue(f)

    def add_path_to_queue(self, fpath):
        if fpath in self._queue_paths: return
        
        # Validate file exists and is accessible before adding to queue
        if not os.path.exists(fpath):
            self.log.append(f"Error: File does not exist: {os.path.basename(fpath)}")
            return
        
        if not os.path.isfile(fpath):
            self.log.append(f"Error: Path is not a file: {os.path.basename(fpath)}")
            return
        
        if not os.access(fpath, os.R_OK):
            self.log.append(f"Error: No read permission for file: {os.path.basename(fpath)}")
            return
        
        item = QListWidgetItem(os.path.basename(fpath))
        item.setData(Qt.UserRole, fpath)
        self.queue_list.addItem(item)
        self._queue_paths.add(fpath)
        self.log.append(f"Added: {os.path.basename(fpath)}")
        self.update_queue_placeholder()

    def remove_from_queue(self):
        for item in self.queue_list.selectedItems():
            fpath = item.data(Qt.UserRole)
            self.queue_list.takeItem(self.queue_list.row(item))
            self._queue_paths.discard(fpath)
        self.update_queue_placeholder()

    def clear_queue(self):
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

        self.btn_start.setEnabled(False)
        self.btn_cancel.setEnabled(True)
        self.btn_add_queue.setEnabled(False)
        self.btn_rem_queue.setEnabled(False)
        self.btn_clear_queue.setEnabled(False)
        self.queue_list.setEnabled(False)

        self.inhibit_sleep()

        # Validate and reset empty values to defaults
        self.validate_target_size()
        self.validate_audio_bitrate()
        
        # Validate custom FPS if selected
        if self.fps_combo.currentText() == "Custom":
            self.validate_custom_fps()

        abit = "0"
        if self.a_bitrate.isEnabled(): abit = self.a_bitrate.text()

        mode_txt = self.mode_combo.currentText()
        mode_code = 'size' if "Size" in mode_txt else 'quality'
        
        # Handle Passthrough mode
        if mode_txt == "Passthrough":
            v_codec = "Passthrough"
        else:
            v_codec = self.v_codec.currentText()
            # Check if selected codec is available
            codec_key = v_codec.lower().replace('.', '')
            if v_codec == "H.264":
                codec_key = 'h264'
            elif v_codec == "H.265":
                codec_key = 'hevc'
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
            ffmpeg_name = os.path.basename(ffmpeg_path)
            ffprobe_name = ffmpeg_name.replace('ffmpeg', 'ffprobe')
            ffprobe_path = os.path.join(ffmpeg_dir, ffprobe_name) if ffmpeg_dir else ffprobe_name
        else:
            ffmpeg_path = 'ffmpeg'
            ffprobe_path = 'ffprobe'
        
        # Additional validation for numeric fields
        try:
            crf_val = self.q_spin.value()
            if crf_val < 0 or crf_val > 51:
                self.q_spin.setValue(24)
        except Exception:
            self.q_spin.setValue(24)

        self.batch_params = {
            'mode': mode_code,
            'size': self.target_size.text(),
            'crf': self.q_spin.value(),
            'v_codec': v_codec,
            'use_hw': self.chk_hw.isChecked(),
            'quality_preset': self.quality_combo.currentData(),
            'two_pass': self.chk_2pass.isChecked(),
            'res_choice': self.res_combo.currentText(),
            'ar_choice': self.ar_combo.currentText(),
            'fps_choice': fps_choice,
            'algo': self.algo_combo.currentText(),
            'a_codec': self.a_codec.currentText(),
            'a_bitrate': abit,
            'output_folder': self.output_path.text(),
            'device': self.device_combo.currentText(),
            'copy_data': self.chk_copy_data.isChecked(),
            'container': container_code,
            'auto_scale': self.chk_auto_scale.isChecked(),
            'hw_decoder_caps': self.hw_decoder_capabilities.get(self.device_combo.currentText(), {}),
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

        self.worker = EncoderWorker(current_job_params)
        self.worker.log_signal.connect(self.log.append)
        self.worker.scale_notification_signal.connect(self.show_scale_notification)
        self.worker.finished_signal.connect(self.job_finished)
        self.worker.compatibility_warning_signal.connect(self.show_compatibility_warning)
        self.worker.progress_signal.connect(self.update_progress)
        self.worker.start()

    def job_finished(self, success):
        if success: self.log.append(">>> JOB FINISHED SUCCESSFULLY")
        else: self.log.append(">>> JOB FAILED or CANCELLED")

        # Access is_cancelled with proper thread synchronization
        with self.worker._process_lock:
            was_cancelled = self.worker.is_cancelled
        
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
        job_count = self.queue_list.count()
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
        self.release_sleep()
        self.btn_start.setEnabled(True)
        self.btn_cancel.setEnabled(False)
        self.btn_add_queue.setEnabled(True)
        self.btn_rem_queue.setEnabled(True)
        self.btn_clear_queue.setEnabled(True)
        self.queue_list.setEnabled(True)
        self.progress_bar.setValue(0)  # Reset progress bar

    def cancel(self):
        if self.worker and self.worker.isRunning():
            reply = QMessageBox.question(self, "Cancel", "Stop transcoding?", QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if reply == QMessageBox.Yes:
                self.worker.cancel()
                self.btn_cancel.setEnabled(False)

    def closeEvent(self, event):
        if self.worker and self.worker.isRunning():
            reply = QMessageBox.question(self, "Exit", "Transcoding in progress. Exit?", QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if reply == QMessageBox.Yes:
                self.worker.cancel()
                event.accept()
            else:
                event.ignore()
        else:
            self.release_sleep()
            event.accept()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.update_queue_placeholder()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec())
