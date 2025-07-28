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
git clone https://github.com/Tiramisioux/cinepi-raw.git --branch rpicam-apps_1.7_custom_encoder && cd cinepi-raw && mkdir build && cd build && sudo meson setup && sudo ninja && cd ../.. && sudo meson install -C cinepi-raw/build && sudo ldconfig
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

| Flag                    | Default           | Description                                                                                          |
|-------------------------|-------------------|------------------------------------------------------------------------------------------------------|
| `--cam-port <string>`   | `""`              | Physical camera port to use (e.g. `cam0` or `cam1`).                                         |
| `--hdmi-port <int>`     | `-1`              | Choose a specific HDMI connector for the DRM preview:<br>`0` = HDMI-0, `1` = HDMI-1, `-1` = automatic. |
| `--same-hdmi`           | `false`           | Force both CinePi apps (capture & controller) to share the same HDMI output.                        |
| `--keep16`              | `false`           | Write full 16-bit DNG files; **disable** 12-bit packing of 16-bit streams.                            |
| `--sync-fps <float>`    | `30.0`            | Frame rate for generated sync pulses. When using `--sync client` and `--framerate` is omitted, this value becomes the camera framerate. |

When `--sync client` is active, omitting `--framerate` means the value of `--sync-fps` will be used as the camera's framerate. Without the client flag, the framerate is controlled via Redis.

## Manual DNG encoder

- Manual writing of DNG tags. 

- Frames are written uncompressed, for simple I/O.

- Packs the 16 bit files to 12 bit, unless `--keep16` is used.

- Supports both IMX 585 color and mono variants.
  
## Audio recording

- Places WAV output alongside  DNG take (`media/RAW/<folder>.wav`).

- Compatible with USB 16 bit mono and RODE Videomic 24bit stereo microphone using `dsnoop`.

### .asoundrc Setup

For `dsnoop` support, create a `~/.asoundrc` in home directory:

```bash
nano ~/.asoundrc
```

```bash

    pcm.dsnoop_24bit {
        type dsnoop
        ipc_key 2048
        slave {
            pcm "hw:Device,0"
            channels 2
            rate 48000
            format S24_3LE
            period_size 1024
            buffer_size 4096
        }
    }

    pcm.dsnoop_16bit {
        type dsnoop
        ipc_key 2049
        slave {
            pcm "hw:Device,0"
            channels 1
            rate 48000
            format S16_LE
            period_size 1024
            buffer_size 4096
        }
    }

    pcm.mic_24bit {
        type plug
        slave.pcm "dsnoop_24bit"
    }

    pcm.mic_16bit {
        type plug
        slave.pcm "dsnoop_16bit"
    }

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
