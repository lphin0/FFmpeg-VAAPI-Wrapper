# FFmpeg VAAPI Wrapper

I vibecoded this for personal use because Shutter Encoder and Handbrake refuse to add VAAPI support, which is the only way to do hardware encoding on the open source AMD drivers. Nvidia users and Windows users should just use those programs instead. This may work for Intel users, but it is untested!

The auto downscale feature is something made for fun to compress large videos down to a size that fits in Discord. It checks the final bitrate and downscales the video, forcing AV1 software encoding. This is because AMD hardware encoders struggle with extremely low bitrates, and you want as high encoding quality as possible at those low bitrates anyway. It only takes effect when the bitrate is below 500kbps.

I'm not likely to fix bugs often, only when something appears that affects my workflow.

# Requirements

## Python Dependencies
- Python 3.6 or higher
- PySide6 (Qt6 bindings)

```bash
pip install PySide6
```

## System Dependencies (Linux)

### Required
- FFmpeg
- FFprobe

### Optional (for sound and desktop notifications)
- `notify-send` (libnotify) - for desktop notifications
- PipeWire/PulseAudio - for sound playback
- ALSA - for sound playback (fallback)

### H.264/H.265 Encoder and Decoder Support

Fedora users can find software encoders (libx264, libx265) and hardware encoders/decoders (VA API, mesa-freeworld) at [RPM Fusion](https://rpmfusion.org/Configuration)

## Running the Script
```bash
python3 ffmpeg-wrap.py