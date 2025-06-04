# cinepi-raw

Small, **libcamera‑based** utilities for driving Raspberry Pi cameras with CinePi

> **Heads‑up**  
> The upstream tools were renamed from `libcamera-*` ➜ `rpicam-*`.  
> This fork goes one step further and re‑brands the whole tree as **cinepi‑raw**, while adding features required by the CinePi project.  

## What’s new in in CineMate fork

### Multi‑instance multi‑HDMI routing

CinePi can <em>several</em> `cinepi-raw` instances in parallel – for example, two IMX585 sensors on a Raspberry Pi 5, each with its own monitor.  

ith the new <code>--hdmi-port</code> / <code>--same-hdmi</code> logic every instance can now claim a **specific** HDMI socket (0 = left, 1 = right) and keep its preview there, regardless of the order in which the programs start.

If you give only the first instance a port number and add <code>--same-hdmi</code> to the rest, they’ll automatically follow that choice.
</details>

| Feature | Flag | Notes |
|---------|------|-------|
| Force preview to a specific HDMI socket | `--hdmi-port <0\|1>` | Pi 4 / Pi 5 with dual HDMI; **‑1 or omit** ➜ let KMS choose |
| Keep *all* apps on the same HDMI output | `--same-hdmi` | First app decides, the rest follow |

The new flags are parsed by **`CinePiOptions`**, translated to a DRM *connector‑id* in `hdmi_utils.cpp`, and enforced by the revamped `drm_preview` backend.  
Everything else (stills, video, raw pipelines) is untouched rpicam‑apps code.

## Quick usage – IMX477 example
    cinepi-raw \
    --mode 2028:1080:12:U \
    --width 2028 --height 1080 \
    --lores-width 1280 --lores-height 720 \
    -p "0,30,1920,1020" \
    --shutter 20000 \
    --awbgains "2.5,2.0" \
    --awb auto \
    --tuning-file ~/libcamera/src/ipa/rpi/pisp/data/imx477.json \
    --hdmi-port 1

_Captures RAW DNG while forcing the preview onto the second HDMI socket._

## Build & Install

### 0 . Prerequisites

    sudo apt update
    sudo apt install -y python3-pip git python3-jinja2 python3-ply python3-yaml \
                    libboost-dev libgnutls28-dev openssl libtiff-dev pybind11-dev \
                    qtbase5-dev libqt5core5a libqt5gui5 libqt5widgets5 \
                    meson ninja-build cmake libglib2.0-dev \
                    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
                    libavdevice59 libavdevice-dev libdrm-dev libexif-dev \
                    libjpeg-dev libpng-dev libtiff5-dev \
                    build-essential redis-server libhiredis-dev libjsoncpp-dev
    sudo ldconfig

### 1 . Build & install libcamera

    git clone https://github.com/Tiramisioux/libcamera.git --branch cinepi-sdk-002
    sudo find libcamera -type f \( -name '*.py' -o -name '*.sh' \) -exec chmod +x {} \;
    cd libcamera
    sudo meson setup build --buildtype=release \
        -Dpipelines=rpi/vc4,rpi/pisp \
        -Dipas=rpi/vc4,rpi/pisp \
        -Dv4l2=true -Dgstreamer=enabled \
        -Dtest=false -Dlc-compliance=disabled -Dcam=disabled -Dqcam=disabled \
        -Ddocumentation=disabled -Dpycamera=enabled
    sudo ninja -C build install
    cd ..

Tiff dev quirk – on some images one header is missing a soname symlink:

    sudo ln -sf $(ldconfig -p | grep libtiff.so | head -n1 | awk '{print $4}') \
           /usr/lib/aarch64-linux-gnu/libtiff.so.5
    sudo ldconfig

### 2 . Install redis-plus-plus (C++ Redis client)
    git clone https://github.com/sewenew/redis-plus-plus.git
    cd redis-plus-plus && mkdir build && cd build
    cmake .. && make -j$(nproc)
    sudo make install
    cd ../..
    sudo ldconfig

### 3 . Clone, build & install cinepi‑raw
    git clone https://github.com/Tiramisioux/cinepi-raw.git --branch cinepi-sdk-002_cinemate-v3
    cd cinepi-raw
    meson setup build --buildtype=release     # no sudo – build in user space
    ninja -C build                            # ⏳
    sudo meson install -C build
    sudo ldconfig


### Redis bootstrap (optional)

If you use the CinePi controller’s Redis interface, pre‑seed a few keys:

    redis-cli --pipe <<'EOF'
    SET fps_actual          24
    PUBLISH cp_controls     fps
    SET shutter_a           180
    PUBLISH cp_controls     shutter_a
    SET shutter_a_nom       180
    PUBLISH cp_controls     shutter_a_nom
    SET is_recording        0
    PUBLISH cp_controls     is_recording
    SET is_writing          0
    PUBLISH cp_controls     is_writing
    SET sensor              imx477
    PUBLISH cp_controls     sensor
    SET sensor_mode         0
    PUBLISH cp_controls     sensor_mode
    SET is_writing_buf      0
    SET current_sensor_mode 0
    PUBLISH cp_controls     current_sensor_mode
    SET fps_max             50
    PUBLISH cp_controls     fps_max
    SET trigger_mode        0
    PUBLISH cp_controls     trigger_mode
    SET is_buffering        0
    PUBLISH cp_controls     is_buffering
    SET fps_user            24
    PUBLISH fps_user        0
    SET fps_last            24
    PUBLISH fps_user        0
    EOF


