![cp_raw_banner](https://github.com/cinepi/cinepi-raw/assets/25234407/71591abc-f9b2-467e-806f-30557bcd1491)

*fork of rpicam-apps that builds upon the rpicam-raw app, offering cinema dng recording capabillities and integration with REDIS offering an abstract "API" like layer for custom integrations / controls.*

Requirements
-----
Please install the below requirements before continuing with the rest of the build process:

[Redis](https://github.com/redis/redis)

[Hiredis](https://github.com/redis/hiredis)

[Redis++](https://github.com/sewenew/redis-plus-plus)

Build
-----
For usage and build instructions, see the [below.](https://github.com/Tiramisioux/cinepi-raw/edit/rpicam-apps_1.7_custom_encoder/README.md#build--install)

License
-------

The source code is made available under the simplified [BSD 2-Clause license](https://spdx.org/licenses/BSD-2-Clause.html).

---

# CineMate fork

- Adapted to libcamera 0.5 / rpicam-apps 1.0.7.

## Additional flags

The following flags extend the base `rpicam-apps` functionality with CinePi-raw–specific features:

| Flag                    | Default           | Description                                                                                          |
|-------------------------|-------------------|------------------------------------------------------------------------------------------------------|
| `--cam-port <string>`   | `""`              | Physical camera port to use (e.g. `cam0` or `cam1`).                                         |
| `--hdmi-port <int>`     | `-1`              | Choose a specific HDMI connector for the DRM preview:<br>`0` = HDMI-0, `1` = HDMI-1, `-1` = automatic. |
| `--same-hdmi`           | `false`           | Force both CinePi apps (capture & controller) to share the same HDMI output.                        |
| `--keep16`              | `false`           | Write full 16-bit DNG files; **disable** 12-bit packing of 16-bit streams.                            |
| `--zoom <float>`       | `1.0`             | Centre-crop digital zoom for streams **0** (viewfinder/encode) and **2** (lo-res).<br>`0.5` = zoom-out, `2.0` = 200 % punch-in. If `--scaler-crops` is present it takes precedence. |


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

---

## Build & Install

### 0 . Prerequisites

If you run Raspberry Pi OS Lite, begin by installing the following packages:

```bash
sudo apt install -y python-pip git python3-jinja2
````

```bash
sudo apt install -y libboost-dev
sudo apt install -y libgnutls28-dev openssl libtiff-dev pybind11-dev
sudo apt install -y qtbase5-dev libqt5core5a libqt5widgets
sudo apt install -y meson cmake
sudo apt install -y python3-yaml python3-ply
sudo apt install -y libglib2.0-dev libgstreamer-plugins-base1.0-dev
```

### 1 . Build & install libcamera

    git clone https://github.com/raspberrypi/libcamera
    sudo meson setup build --buildtype=release -Dpipelines=rpi/vc4,rpi/pisp -Dipas=rpi/vc4,rpi/pisp -Dv4l2=true -Dgstreamer=enabled -Dtest=false -Dlc-compliance=disabled -Dcam=disabled -Dqcam=disabled -Ddocumentation=disabled -Dpycamera=enabled
    ninja -C build install
    sudo ldconfig

### 2 . Install redis-plus-plus (C++ Redis client)
    git clone https://github.com/sewenew/redis-plus-plus.git
    cd redis-plus-plus && mkdir build && cd build
    cmake .. && make -j$(nproc)
    sudo make install
    cd ../..
    sudo ldconfig

### 3 . Clone, build & install cinepi‑raw
    git clone https://github.com/Tiramisioux/cinepi-raw.git --rpicam-apps_1.7_custom_encoder
    cd cinepi-raw
    sudo meson setup build --buildtype=release     
    ninja -C build                            
    sudo meson install -C build
    sudo ldconfig

---

## Quick usage

Here is an example using IMX477 camera connected to cam0 while forcing the preview onto the second HDMI socket.

In `/boot/firmware/config.txt`, set

`dtoverlay=imx477,cam0`

reboot

then

```bash
cinepi-raw --mode 2028:1080:12:U --width 2028 --height 1080 --lores-width 1280 --lores-height 720 --shutter 20000 --awbgains "2.5,2.0" --awb auto --tuning-file ~/libcamera/src/ipa/rpi/pisp/data/imx477.json --hdmi-port 1 --cam-port cam0 
```

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

## Live digital zoom via Redis

```bash
SET zoom 1.8
PUBLISH cp_controls zoom
```

The value is a simple float matching the --zoom CLI flag. CinePi-raw applies the new crop in the very next frame — no restart needed.


