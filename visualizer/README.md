# RealSense Dual Stream Visualizer (D435i + T265)

A high-performance visualizer built with OpenCV and Librealsense 2.53.1 featuring a fixed high-resolution canvas (**2560 × 1520**), 1:1 pixel rendering with black letterboxing, a dedicated **Pop-Out Controller Window**, and full resolution / FPS table matrix controls for both cameras.

## Architecture

The application runs two synchronized OpenCV windows:
1. **Stream Visualizer Window (`2560 × 1520`)**:
   - Total Width: **2560** ($1280 \times 2$, Left and Right camera viewports)
   - Total Height: **1520** ($720$ D435i top + $800$ T265 bottom)
   - Pure 1:1 pixel rendering: all streams are centered in their respective $1280 \times 720$ (D435i) or $1280 \times 800$ (T265) viewports with clean black bars around them if they don't fill the slot.
2. **Stream Controller Window (`520 × 700`)**:
   - Pop-out control panel housing the master record button, stream toggles, exposure/gain sliders, and the interactive resolution & FPS matrix tables.

---

## 1. Stream Visualizer Window (2560 × 1520)

- **Top Row (D435i Infrared Stereo, 2560 × 720 container)**:
  - **Left IR Slot (x: 0..1280, y: 0..720)** | **Right IR Slot (x: 1280..2560, y: 0..720)**
  - Every resolution is centered inside its $1280 \times 720$ slot with surrounding black bars:
    - **`848 x 100` (300 FPS Fast Mode)**: Centered at $x_{off} = 216$, $y_{off} = 310$ with 310 px top & bottom black bars.
    - **`848 x 480`**: Centered at $x_{off} = 216$, $y_{off} = 120$.
    - **`1280 x 720`**: Fills the entire $1280 \times 720$ slot completely ($x_{off} = 0$, $y_{off} = 0$).
    - **`640 x 480`**, **`640 x 360`**, **`480 x 270`**, **`424 x 240`**: Centered with symmetric black borders.
  - **Badges**: Located at the bottom-left of each slot showing real-time measured FPS, resolution, target FPS, and emitter status.
- **Bottom Row (T265 Fisheye Stereo, 2560 × 800 container)**:
  - **Left Fisheye Slot (x: 0..1280, y: 720..1520)** | **Right Fisheye Slot (x: 1280..2560, y: 720..1520)**
  - Native $848 \times 800$ frame centered horizontally at $x_{off} = (1280 - 848) / 2 = 216$ pixels with 216 px black bars left and right.
  - **Badges**: Located at the bottom-left of each slot showing real-time measured FPS, active target FPS mode (30, 60, or 90), exposure, gain, and auto-exposure status.

---

## 2. Pop-Out Controller Window (520 × 700)

### Top Section: Master Record Button
- **`[ ● RECORD 2s BENCHMARK (R) ]`**:
  - Buffers 2.0 seconds of uncompressed frames directly to RAM.
  - Automatically dumps high-speed JPEG images to `captures_d435i/` and `captures_t265/`.
  - Generates `benchmark_summary.txt` with exact duration, resolution, frame counts, and measured FPS.

### D435i Section
- **Stream Toggle**: `[ 🔘 D435i Stream: ON/OFF ]`
- **IR Emitter Toggle**: `[ 🔘 IR Emitter: ON/OFF ]`
- **Interactive Resolution & FPS Matrix Table**:
  - Click any available FPS cell to hot-switch the camera stream profile dynamically.
  - Currently active mode is highlighted in **bright emerald green**.
  - Unavailable FPS modes are **greyed out** (`-`).

| Resolution | 6 FPS | 15 FPS | 30 FPS | 60 FPS | 90 FPS | 300 FPS |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **848 x 100** | - | - | - | - | - | **`[ 300 ]`** |
| **1280 x 720** | **`[ 6 ]`** | **`[ 15 ]`** | **`[ 30 ]`** | - | - | - |
| **848 x 480** | **`[ 6 ]`** | **`[ 15 ]`** | **`[ 30 ]`** | **`[ 60 ]`** | **`[ 90 ]`** | - |
| **640 x 480** | **`[ 6 ]`** | **`[ 15 ]`** | **`[ 30 ]`** | **`[ 60 ]`** | **`[ 90 ]`** | - |
| **640 x 360** | **`[ 6 ]`** | **`[ 15 ]`** | **`[ 30 ]`** | **`[ 60 ]`** | **`[ 90 ]`** | - |
| **480 x 270** | **`[ 6 ]`** | **`[ 15 ]`** | **`[ 30 ]`** | **`[ 60 ]`** | **`[ 90 ]`** | - |
| **424 x 240** | **`[ 6 ]`** | **`[ 15 ]`** | **`[ 30 ]`** | **`[ 60 ]`** | **`[ 90 ]`** | - |

### T265 Section
- **Stream Toggle**: `[ 🔘 T265 Stream: ON/OFF ]`
- **Resolution & Reflash Table**:
  - Click **`[ 30 ]`**, **`[ 60 ]`**, or **`[ 90 ]`** to trigger an automatic background firmware reflash and pipeline reboot into that frame rate mode.
  - Active mode highlighted in **cyan/teal**; pulses amber during firmware transfer.

| Resolution | FPS Options (Dynamic Bootloader Reflash) |
| :--- | :--- |
| **848 x 800** | **`[ 30 ]`** &nbsp;&nbsp; **`[ 60 ]`** &nbsp;&nbsp; **`[ 90 ]`** |

- **Exposure & Gain Sliders**:
  - Click or drag on the tracks: Exposure ($200$ to $16,000\,\mu\text{s}$, automatically clamped to $10,000\,\mu\text{s}$ in 90 FPS mode) and Gain ($1.0\times$ to $10.0\times$).

---

## Keyboard Hotkeys

Works in either active window:

| Key | Action |
| :--- | :--- |
| **`R`** / **`r`** / **Space** | Trigger 2-Second Capture Benchmark |
| **`1`** | Toggle D435i Stream (`ON` / `OFF`) |
| **`2`** | Toggle T265 Stream (`ON` / `OFF`) |
| **`E`** / **`e`** | Toggle D435i Laser Emitter (`ON` / `OFF`) |
| **`3`** | Switch T265 to **30 FPS** |
| **`6`** | Switch T265 to **60 FPS** |
| **`9`** | Switch T265 to **90 FPS** |
| **`Q`** / **ESC** | Clean exit & sensor shutdown |

---

## Building & Running

```bash
cd visualizer
make
./run.sh
```
