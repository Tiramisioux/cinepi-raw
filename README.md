![cp_raw_banner](https://github.com/cinepi/cinepi-raw/assets/25234407/71591abc-f9b2-467e-806f-30557bcd1491)

*fork of rpicam-apps that builds upon the rpicam-raw app, offering cinema dng recording capabillities and integration with REDIS offering an abstract "API" like layer for custom integrations / controls.*

License
-------

The source code is made available under the simplified [BSD 2-Clause license](https://spdx.org/licenses/BSD-2-Clause.html).

# How to install

## 0 . Prerequisites

If you run Raspberry Pi OS Lite, begin by installing the following packages:

```bash
sudo apt install -y python-pip git python3-jinja2
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

```shell
sudo apt install -y cmake libepoxy-dev libavdevice-dev build-essential cmake libboost-program-options-dev libdrm-dev libexif-dev libcamera-dev libjpeg-dev libtiff5-dev libpng-dev redis-server libhiredis-dev libasound2-dev libjsoncpp-dev libpng-dev meson ninja-build libavcodec-dev libavdevice-dev libavformat-dev libswresample-dev && sudo apt-get install libjsoncpp-dev && cd ~ && git clone https://github.com/sewenew/redis-plus-plus.git && cd redis-plus-plus && mkdir build && cd build && cmake .. && make && sudo make install && cd ~
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
| `--latency-sample-interval <n>` | `10` | Sample encode/disk latency metrics every N frames for `cp_stats`. |
| `--per-frame-logs[=bool]` | `false` | Enable verbose per-frame encoder/disk DEBUG logs (`true/false`, `1/0`). |

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
# RODE NTG path (24-bit stereo)
pcm.mic_dsnoop_24 {
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

# Cheap USB path (16-bit mono)
pcm.mic_dsnoop_16 {
  type dsnoop
  ipc_key 5979
  ipc_perm 0666
  ipc_key_add_uid false
  slave {
    pcm "hw:CARD=Device,DEV=0"
    format S16_LE
    rate 48000
    channels 1
  }
  bindings.0 0
}

pcm.mic_24bit { type plug; slave.pcm "mic_dsnoop_24" }
pcm.mic_16bit { type plug; slave.pcm "mic_dsnoop_16" }

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

### cp_stats observability fields

`cp_stats` now publishes additional fields for Cinemate v2/analyzer compatibility while preserving existing keys:

- Per-frame fields: `sensorTimestamp`, `stats_seq`, `encode_queue_size`, `disk_queue_size`, `ram_buffers`.
- Sampled fields: `encode_latency_ms`, `disk_latency_ms` (sampled every `--latency-sample-interval` frames, default `10`).
- Sampling behavior: sampled latency keys are repeated with the last sampled value until the next sample refresh.

High-frequency per-frame encode/disk logs are now DEBUG-gated and disabled by default; startup/configuration logs remain INFO/WARN/ERROR.
