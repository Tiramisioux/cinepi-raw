![cp_raw_banner](https://github.com/cinepi/cinepi-raw/assets/25234407/71591abc-f9b2-467e-806f-30557bcd1491)

*fork of rpicam-apps that builds upon the rpicam-raw app, offering cinema dng recording capabillities and integration with REDIS offering an abstract "API" like layer for custom integrations / controls.*

License
-------

The source code is made available under the simplified [BSD 2-Clause license](https://spdx.org/licenses/BSD-2-Clause.html).

# How to install

## 0 . Prerequisites

If you run Raspberry Pi OS Lite, begin by installing the following packages:

```bash
sudo apt install -y python-pip git python3-jinja2 ffmpeg
````

## Install libcamera

```shell
git clone https://github.com/raspberrypi/libcamera && \
sudo find ~/libcamera -type f \( -name '*.py' -o -name '*.sh' \) -exec chmod +x {} \; && \
cd libcamera && \
sudo meson setup build --buildtype=release \
  -Dpipelines=rpi/vc4,rpi/pisp \
  -Dipas=rpi/vc4,rpi/pisp \
  -Dv4l2=true \
  -Dgstreamer=enabled \
  -Dtest=false \
  -Dlc-compliance=disabled \
  -Dcam=disabled \
  -Dqcam=disabled \
  -Ddocumentation=disabled \
  -Dpycamera=enabled && \
sudo ninja -C build install && \
cd
```

```shell
cd ~/libcamera/utils && sudo chmod +x *.py *.sh && sudo chmod +x ~/libcamera/src/ipa/ipa-sign.sh && cd ~/libcamera && sudo ninja -C build install
```

```shell
sudo apt-get install --reinstall libtiff5-dev && sudo ln -sf $(find /usr/lib -name "libtiff.so" | head -n 1) /usr/lib/aarch64-linux-gnu/libtiff.so.5 && export LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH && sudo ldconfig
```

```shell
sudo apt install -y python3-pip git python3-jinja2 libboost-dev libgnutls28-dev openssl pybind11-dev qtbase5-dev libqt5core5a meson cmake python3-yaml python3-ply libglib2.0-dev libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev libavdevice59
```

## Install cpp-mjpeg streamer

```shell
sudo apt install -y libspdlog-dev libjsoncpp-dev && cd /home/pi && git clone https://github.com/tiramisioux/cpp-mjpeg-streamer.git --branch cinemate && cd cpp-mjpeg-streamer && mkdir build && cd build && cmake .. && make && sudo make install && cd
```

## Install cinepi-raw dependencies

`ffmpeg` is required on the Pi for WAV BEXT/iXML timecode metadata writes.

```shell
sudo apt install -y cmake libepoxy-dev libavdevice-dev build-essential cmake libboost-program-options-dev libdrm-dev libexif-dev libcamera-dev libjpeg-dev libtiff5-dev libpng-dev redis-server libhiredis-dev libasound2-dev libjsoncpp-dev libpng-dev meson ninja-build libavcodec-dev libavdevice-dev libavformat-dev libswresample-dev ffmpeg && sudo apt-get install libjsoncpp-dev && cd ~ && git clone https://github.com/sewenew/redis-plus-plus.git && cd redis-plus-plus && mkdir build && cd build && cmake .. && make && sudo make install && cd ~
```

```shell
sudo ldconfig
```

## Install cinepi-raw 

```shell
git clone https://github.com/Tiramisioux/cinepi-raw.git && cd cinepi-raw && mkdir build && cd build && sudo meson setup && sudo ninja && cd ../.. && sudo meson install -C cinepi-raw/build && sudo ldconfig
```
### for pi 4:

```shell
sudo echo "/home/pi/cinepi-raw/build
/usr/lib/aarch64-linux-gnu
/usr/local/lib/aarch64-linux-gnu" | sudo tee /etc/ld.so.conf.d/cinepi-raw.conf && sudo ldconfig && echo 'export LD_LIBRARY_PATH=/home/pi/cinepi-raw/build:/usr/lib/aarch64-linux-gnu:/usr/local/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH' >> ~/.bashrc && source ~/.bashrc
```

## Set redis key

```shell
redis-cli <<EOF
SET cg_rb 2.5,2.2
PUBLISH cp_controls cg_rb
EOF
```

## Quick usage

Here is an example using IMX477 camera connected to cam0 while forcing the preview onto the second HDMI socket.

In `/boot/firmware/config.txt`, set

`dtoverlay=imx477,cam0`

reboot

then

```bash
cinepi-raw --mode 2028:1080:12:U --width 2028 --height 1080 --lores-width 1280 --lores-height 720 --shutter 20000 --awbgains "2.5,2.0" --awb auto --tuning-file ~/libcamera/src/ipa/rpi/pisp/data/imx477.json --hdmi-port 1 --cam-port cam0 
```
# CineMate fork

_Adapted to libcamera 0.5 / rpicam-apps 1.0.7._

## Additional flags

The following flags extend the base `rpicam-apps` functionality with CinePi-raw–specific features:

| Flag                      | Default | Description |
|---------------------------|---------|-------------|
| `--cam-port <string>`     | `""`    | Physical camera port to use (e.g. `cam0` or `cam1`). |
| `--hdmi-port <int>`       | `-1`    | Choose a specific HDMI connector for the DRM preview:<br>`0` = HDMI-0, `1` = HDMI-1, `-1` = automatic. |
| `--same-hdmi`             | `false` | Force both CinePi apps (capture & controller) to share the same HDMI output. |
| `--keep16`                | `false` | Write full 16-bit DNG files; **disable** 12-bit packing of 16-bit streams. |
| `--encode-workers <n>`    | `2`     | Number of DNG encode worker threads to spawn (min. `1`). |
| `--disk-workers <n>`      | `8`     | Number of disk writer threads used for flushing DNGs (min. `1`). |
| `--encode-affinity <list>`| `auto`  | Pin encode workers to a CPU list (e.g. `4,5` or `2-5`). |
| `--disk-affinity <list>`  | `auto`  | Pin disk workers to the specified CPU list. |
| `--encode-nice <int>`     | `auto`  | Nice level for encode workers (`-20` = highest priority, `19` = lowest). |
| `--disk-nice <int>`       | `auto`  | Nice level applied to disk workers. |
| `--plain-arecord-timecode-offset-frames <int>` | `0` | Frame offset added to the 16-bit plain `arecord` WAV metadata timecode. PCM is not shifted. |
| `--audio-timecode-offset-frames <int>` | `0` | Frame offset added to the 24-bit USB-capture WAV metadata timecode. PCM is not shifted. |
| `--unique-camera-model <string>` | `"cinepi"` | Override the `UniqueCameraModel` DNG tag embedded in recorded frames. Changing to `Blackmagic Pocket Cinema Camera 4K` enables ISO settings to clips in DaVinci Resolve. |

### WAV timecode offset (`--audio-timecode-offset-frames`)

A USB capture path can land a fixed number of frames early or late relative to video even after clock correction (analog/buffering latency that is constant across takes). `--audio-timecode-offset-frames` nudges the **WAV metadata timecode** by a whole number of frames to compensate. Only the embedded BWF/iXML timecode is shifted — the PCM samples are never moved.

- This flag covers the **24-bit USB capture (helper) path**. The 16-bit plain-`arecord` path has its own `--plain-arecord-timecode-offset-frames`.
- **Sign convention:** a **positive** offset moves the timecode later, so audio lands later on the NLE timeline — use a positive value when the sound is *early*. A negative value moves it earlier.
- Independent of clock correction; both can be active at once.
- Like the clock-correction flag, Cinemate sets this automatically from `audio.timecode_offset_frames` in `settings.json`; pass it manually only when running `cinepi-raw` directly.

When non-zero, `cinepi-raw` logs after each take:

```
Applied 24-bit USB capture WAV metadata timecode offset: +2 frames; PCM timing unchanged
```

## Frame-rate phase lock

Off by default (`fps_phase_lock` Redis key). A closed-loop servo that holds the
recorded frame cadence on the operator's nominal fps, so audio and video stay in
sync across long takes.

**How it differs from stock cinepi-raw:** stock cinepi-raw sets one
`FrameDurationLimits` per fps change and lets the sensor free-run, so a small
fixed quantisation/crystal offset between the requested rate and what the sensor
actually delivers accumulates over a take. The phase lock measures and corrects
every frame instead.

**How it works:** each frame in `process()` it compares the accumulated frame
phase (from the monotonic `SensorTimestamp`) against the ideal `n / fps` and trims
`FrameDurationLimits` with a PI servo. The integer-VBLANK quantisation downstream
is dithered (first-order sigma-delta) so the *average* rate is exact. It is
VBLANK-only (never touches line length) and pre-converges during preview, so a
clip is locked from the first frame.

**What it means for sync:** the video cadence tracks the Pi clock — the same clock
the audio is captured against — so A/V no longer drift apart over long takes; the
residual is a bounded sub-frame offset, not an accumulating drift.

Gains are tunable live via `pll_kp` / `pll_ki` / `pll_deadband_us`. On a
multi-camera `--sync` genlock rig it is safe to leave enabled: cinepi-raw infers
its role from `--sync` and runs the absolute Pi-clock discipline only on the
master (`--sync` off or `server`). The `--sync` client self-suppresses the lock
so libcamera's rpi.sync owns that sensor's VBLANK and holds the relative A→B
genlock — the lock never shares a sensor's VBLANK with rpi.sync, which is the
conflict the client gate prevents. If you would rather keep the master strictly
constant-rate, disable the lock and discipline the sync server's rate instead.

## Manual DNG encoder

- Manual writing of DNG tags. 

- Frames are written uncompressed, for simple I/O.

- Packs the 16 bit files to 12 bit, unless `--keep16` is used.

- Supports both IMX 585 color and mono variants.

### Worker pool tuning examples

- **Cooler operation:** Limit the encoder and disk workers if you want to reduce thermal load. For example:

  ```bash
  cinepi-raw --encode-workers 4 --disk-workers 4 [other options]
  ```

- **Steer workloads to specific CPUs:** Combine affinity and nice controls to keep background threads off the CPU cores you care about:

  ```bash
  cinepi-raw --encode-workers 4 --encode-affinity 4-5 --encode-nice -5 \
             --disk-workers 2 --disk-affinity 0-3 --disk-nice 8 [other options]
  ```

  This example pins encode workers to CPUs 4–5 with a higher priority while leaving disk flush threads on the little cores with a lower scheduling priority.
  
## Audio recording

- Places WAV output alongside  DNG take (`media/RAW/<folder>.wav`).

- Compatible with USB 16 bit mono and RODE Videomic 24bit stereo microphone using `dsnoop`.

### .asoundrc Setup

For `dsnoop` support, create a `~/etc/asound.conf`:

```bash

    sudo tee /etc/asound.conf >/dev/null <<'EOF'
# --- Hardware handle (use stable card name; change "NTG" if your card shows a different name in `arecord -l`)
pcm.mic_hw {
  type hw
  card "NTG"
  device 0
}

# --- One shared dsnoop backend pinned to the mic's native mode (RØDE NTG: S24_3LE @ 48k, stereo)
pcm.mic_dsnoop {
  type dsnoop
  ipc_key 5978
  ipc_perm 0666
  ipc_key_add_uid false
  slave {
    pcm "hw:CARD=NTG,DEV=0"
    format S24_3LE
    rate 48000
    channels 2
  }
  bindings.0 0
  bindings.1 1
}

# --- Front-ends: let plug adapt whatever the app asks for (stereo 24-bit or mono 16-bit)
pcm.mic_24bit {
  type plug
  slave.pcm "mic_dsnoop"
}

pcm.mic_16bit {
  type plug
  slave.pcm "mic_dsnoop"
}
EOF

```

Exit nano editor using ctrl+x.

## Controlling recording via Redis

CinePi-raw listens for recording commands through a **single string key**  `is_recording` and the **`cp_controls` pub-sub channel**.  

The mechanism in the CineMate fork is edge-driven: only **transitions** 0 → 1 or 1 → 0 start or stop a take; duplicate writes are ignored.

| Transition | What CinePi-raw does |
|------------|----------------------|
| **0 → 1**  | Creates a new take folder under `/media/RAW/YYYYMMDD/clip_###/` and starts writing CinemaDNG frames (plus WAV if audio is enabled). |
| **1 → 0**  | Closes the current take and stops encoding. |
| **0 → 0** or **1 → 1** | No effect (debounce). |

```bash
# Start recording
redis-cli SET is_recording 1
redis-cli PUBLISH cp_controls is_recording    # triggers 0 → 1 edge

# Stop recording
redis-cli SET is_recording 0
redis-cli PUBLISH cp_controls is_recording    # triggers 1 → 0 edge
```

## Live digital punch-in (center-crop preview)

Via Redis you can punch-in the HDMI preview while leaving the RAW recording untouched – good for C-mount lenses that don’t cover the whole
sensor.

```bash
SET zoom 1.5
PUBLISH cp_controls zoom
```
CinemaDNGs always contain the entire sensor.
